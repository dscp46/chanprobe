/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Signal generation (A, G, L, tune tone) and tone/sequence detection.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "signal.h"

/* ------------------------------------------------------ generation --- */

static float *sig_grow(sigbuf_t *b, size_t n)
{
    if (b->n + n > b->cap) {
        size_t nc = b->cap ? b->cap : 65536;
        while (nc < b->n + n)
            nc *= 2;
        float *nx = realloc(b->x, nc * sizeof(float));
        if (!nx)
            return NULL;
        b->x = nx;
        b->cap = nc;
    }
    float *p = b->x + b->n;
    b->n += n;
    return p;
}

static void fade(float *x, size_t n, int fs)
{
    size_t nf = (size_t)(FADE_MS * fs / 1000);
    if (2 * nf > n)
        nf = n / 2;
    for (size_t i = 0; i < nf; i++) {
        float g = (float)(0.5 - 0.5 * cos(M_PI * i / nf));
        x[i] *= g;
        x[n - 1 - i] *= g;
    }
}

int sig_tone(sigbuf_t *b, int fs, double f, double ms, double amp)
{
    size_t n = (size_t)llround(ms * fs / 1000.0);
    float *p = sig_grow(b, n);
    if (!p)
        return -1;
    for (size_t i = 0; i < n; i++)
        p[i] = (float)(amp * sin(2 * M_PI * f * (double)i / fs));
    fade(p, n, fs);
    return 0;
}

int sig_probe(sigbuf_t *b, int fs, double ms, double gain)
{
    int np;
    float *per = probe_make_period(fs, &np, L_PEAK_DBFS);
    if (!per)
        return -1;
    size_t n = (size_t)llround(ms * fs / 1000.0);
    float *p = sig_grow(b, n);
    if (!p) {
        free(per);
        return -1;
    }
    for (size_t i = 0; i < n; i++)
        p[i] = (float)(per[i % np] * gain);
    fade(p, n, fs);
    free(per);
    return 0;
}

int sig_glg(sigbuf_t *b, int fs)   /* G, L1, G, L2, G */
{
    double a = db2lin(TONE_DBFS);
    if (sig_tone(b, fs, TONE_G_HZ, DUR_G_MS, a) ||
        sig_probe(b, fs, DUR_L_MS, 1.0) ||
        sig_tone(b, fs, TONE_G_HZ, DUR_G_MS, a) ||
        sig_probe(b, fs, DUR_L_MS, L2_GAIN) ||
        sig_tone(b, fs, TONE_G_HZ, DUR_G_MS, a))
        return -1;
    return 0;
}

void sig_free(sigbuf_t *b)
{
    free(b->x);
    memset(b, 0, sizeof(*b));
}

/* ------------------------------------------------------- detection --- */

double tone_ratio(const float *x, int n, double f, int fs)
{
    double c = 2 * cos(2 * M_PI * f / fs), s1 = 0, s2 = 0, e = 0;
    for (int i = 0; i < n; i++) {
        double s0 = x[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
        e += (double)x[i] * x[i];
    }
    double p = s1 * s1 + s2 * s2 - c * s1 * s2;
    if (e < 1e-12)
        return 0;
    return 2 * p / (n * e);
}

#define BLK(fs) ((fs) / 100)    /* 10 ms analysis blocks */

static int rd(source_t *s, float *b, int n)
{
    if (g_stop)
        return -1;
    int r = s->read(s->ctx, b, n);
    return r == n ? 0 : -1;
}

int wait_tone(source_t *s, double f, int min_ms, int timeout_ms)
{
    int B = BLK(s->fs), need = min_ms / 10, cnt = 0;
    long blocks = 0;
    float *b = malloc(sizeof(float) * B);
    if (!b)
        return -1;
    for (;;) {
        if (rd(s, b, B)) {
            free(b);
            return -1;
        }
        if (tone_ratio(b, B, f, s->fs) > 0.5)
            cnt++;
        else
            cnt = 0;
        if (cnt >= need) {
            free(b);
            return 0;
        }
        if (timeout_ms > 0 && ++blocks * 10 >= timeout_ms) {
            free(b);
            return 1;
        }
    }
}

int wait_tone_end(source_t *s, double f, int timeout_ms)
{
    int B = BLK(s->fs), cnt = 0;
    long blocks = 0;
    float *b = malloc(sizeof(float) * B);
    if (!b)
        return -1;
    for (;;) {
        if (rd(s, b, B)) {
            free(b);
            return -1;
        }
        if (tone_ratio(b, B, f, s->fs) < 0.2)
            cnt++;
        else
            cnt = 0;
        if (cnt >= 2) {
            free(b);
            return 0;
        }
        if (timeout_ms > 0 && ++blocks * 10 >= timeout_ms) {
            free(b);
            return 1;
        }
    }
}

/* Record one probe level: b holds its first block; read until want samples.
 * Guard-tone blocks in the last 100 ms (the next guard starting early) are
 * counted in *g. */
static int record_level(source_t *s, float *b, int B, sigbuf_t *rec, size_t want, int *g)
{
    const int fs = s->fs;
    for (;;) {
        float *p = sig_grow(rec, B);
        if (!p)
            return -1;
        memcpy(p, b, sizeof(float) * B);
        if (rec->n + fs / 10 > want && tone_ratio(b, B, TONE_G_HZ, fs) > 0.5)
            (*g)++;
        if (rec->n >= want)
            return 0;
        if (rd(s, b, B))
            return -1;
    }
}

/* Pass a guard tone (present, then gone) within max_ms. *g counts its
 * blocks. Returns 0 with b holding the first block after it, 1 if no guard
 * tone was found, <0 on error. */
static int pass_guard(source_t *s, float *b, int B, int max_ms, int *g)
{
    int seen = 0;
    for (int blocks = 0; blocks * 10 < max_ms; blocks++) {
        if (rd(s, b, B))
            return -1;
        double t = tone_ratio(b, B, TONE_G_HZ, s->fs);
        if (t > 0.5) {
            seen = 1;
            (*g)++;
        } else if (seen && t < 0.2) {
            return 0;
        }
    }
    return 1;
}

int rx_probe(source_t *s, int timeout_ms, probe_result_t *res)
{
    const int fs = s->fs, B = BLK(fs), N = fs / BASE_HZ;
    const size_t want = (size_t)fs * DUR_L_MS / 1000;
    sigbuf_t rec1 = {0}, rec2 = {0};
    float *b = malloc(sizeof(float) * B);
    int ret = -1, have2 = 0;
    long blocks;
    unsigned long xr0;
    int lead = 5, mid = 0, trail = 0;        /* wait_tone() consumed 5 blocks */

    memset(res, 0, sizeof(*res));
    res->fs = fs;
    if (!b)
        return -1;

    /* 1. Leading guard tone, >= 50 ms. */
    int r = wait_tone(s, TONE_G_HZ, 50, timeout_ms);
    if (r) {
        ret = r;
        goto out;
    }
    dbg("rx_probe: guard tone detected\n");
    /* 2. End of the leading guard tone = start of L1. */
    for (blocks = 0;; blocks++) {
        if (rd(s, b, B))
            goto out;
        double t = tone_ratio(b, B, TONE_G_HZ, fs);
        if (t > 0.5)
            lead++;
        if (t < 0.2)
            break;
        if (blocks > 100) {     /* 1 s of guard tone: not our sequence */
            ret = 2;
            goto out;
        }
    }
    xr0 = s->xruns ? *s->xruns : 0;
    /* 3. L1, then the middle guard tone, whose end starts L2. */
    if (record_level(s, b, B, &rec1, want, &mid))
        goto out;
    r = pass_guard(s, b, B, DUR_G_MS + 300, &mid);
    if (r < 0)
        goto out;
    if (r == 0) {
        /* L2 must follow, 6 dB below L1: if the first block after the guard
         * tone is more than 12 dB down, the transmission has ended. */
        double e1 = 0, e2 = 0;
        for (size_t i = 0; i < rec1.n; i++)
            e1 += (double)rec1.x[i] * rec1.x[i];
        for (int i = 0; i < B; i++)
            e2 += (double)b[i] * b[i];
        if (e2 / B < e1 / rec1.n * db2lin(-24))      /* power ratio of -12 dB */
            r = 1;
    }
    if (r == 0) {
        /* 4. L2, then the trailing guard tone. */
        if (record_level(s, b, B, &rec2, want, &trail))
            goto out;
        have2 = 1;
        r = pass_guard(s, b, B, 1000, &trail);
        if (r < 0)
            goto out;
    }
    dbg("rx_probe: guard tones %d / %d / %d ms, L2 %s\n", lead * 10, mid * 10, trail * 10,
        have2 ? "received" : "missing");
    if (s->xruns && *s->xruns != xr0)
        logmsg("warning: capture overrun during the probe signal\n");

    size_t off = (size_t)AN_SKIP_PERIODS * N;
    ret = probe_analyze(rec1.x + off, rec1.n - off, fs, res) ? 2 : 0;
    if (!ret) {
        static probe_result_t r2;
        if (have2 && !probe_analyze(rec2.x + off, rec2.n - off, fs, &r2))
            probe_combine(res, &r2);
        else
            logmsg("warning: L2 (second probe level) %s\n",
                   have2 ? "could not be analysed" : "not received");
    }
    res->lead_g_ms = lead * 10;
    res->mid_g_ms = mid * 10;
    res->trail_g_ms = (have2 ? trail : mid) * 10;
out:
    free(b);
    sig_free(&rec1);
    sig_free(&rec2);
    return ret;
}
