/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Offline self test: simulated channel between two sides running at
 * 44100 / 48000 Hz with sample clock offset, filtering, delay and noise.
 */
#include <complex.h>
#include <pthread.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/common.h"
#include "../src/modem.h"
#include "../src/probe.h"
#include "../src/report.h"
#include "../src/session.h"
#include "../src/signal.h"

volatile sig_atomic_t g_stop;
int g_verbose;
void logmsg(const char *fmt, ...) { if (!g_verbose) return; va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap); }
void dbg(const char *fmt, ...) { if (!g_verbose) return; va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap); }

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

/* ---- simulated channel ---------------------------------------------- */

typedef struct { double b0, b1, b2, a1, a2; } biquad_t;

static biquad_t lowpass(double fc, double q, int fs)
{
    double w = 2 * M_PI * fc / fs, al = sin(w) / (2 * q), c = cos(w), a0 = 1 + al;
    biquad_t b = {(1 - c) / 2 / a0, (1 - c) / a0, (1 - c) / 2 / a0, -2 * c / a0, (1 - al) / a0};
    return b;
}

static double complex bq_resp(const biquad_t *b, double f, int fs)
{
    double complex z = cexp(-I * 2 * M_PI * f / fs);
    return (b->b0 + b->b1 * z + b->b2 * z * z) / (1 + b->a1 * z + b->a2 * z * z);
}

static void bq_run(const biquad_t *b, float *x, size_t n)
{
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (size_t i = 0; i < n; i++) {
        double y = b->b0 * x[i] + b->b1 * x1 + b->b2 * x2 - b->a1 * y1 - b->a2 * y2;
        x2 = x1; x1 = x[i]; y2 = y1; y1 = y;
        x[i] = (float)y;
    }
}

static double gauss(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

/* Windowed-sinc resampler from fs_in to fs_out; the sender's clock is
 * (1 + eps) fast; delay in input samples. */
static float *resample(const float *x, size_t n, int fs_in, int fs_out, double eps,
                       double delay, size_t *nout)
{
    const int T = 32;
    double step = (double)fs_in / fs_out * (1 + eps);
    double fc = 0.45 * (fs_in < fs_out ? fs_in : fs_out) / fs_in;
    size_t m = (size_t)((n - delay) / step);
    float *y = malloc(sizeof(float) * m);
    for (size_t k = 0; k < m; k++) {
        double u = k * step - delay;
        long i0 = (long)floor(u);
        double s = 0;
        for (long i = i0 - T + 1; i <= i0 + T; i++) {
            if (i < 0 || i >= (long)n)
                continue;
            double d = u - i;
            double w = 0.42 + 0.5 * cos(M_PI * d / T) + 0.08 * cos(2 * M_PI * d / T);
            double sn = fabs(d) < 1e-9 ? 2 * fc : sin(2 * M_PI * fc * d) / (M_PI * d);
            s += x[i] * sn * w;
        }
        y[k] = (float)s;
    }
    *nout = m;
    return y;
}

typedef struct { const float *x; size_t n, pos; } memsrc_t;

static int memread(void *ctx, float *b, int n)
{
    memsrc_t *m = ctx;
    if (m->pos + n > m->n)
        return -1;
    memcpy(b, m->x + m->pos, sizeof(float) * n);
    m->pos += n;
    return n;
}

static float *pad(const float *x, size_t n, int fs, double lead_s, double tail_s,
                  double noise_db, size_t *nout)
{
    size_t a = (size_t)(lead_s * fs), b = (size_t)(tail_s * fs), N = a + n + b;
    float *y = calloc(N, sizeof(float));
    memcpy(y + a, x, sizeof(float) * n);
    double s = db2lin(noise_db);
    for (size_t i = 0; i < N; i++)
        y[i] += (float)(s * gauss());
    *nout = N;
    return y;
}

/* ---- tests ------------------------------------------------------------ */

static void test_crest(int fs)
{
    int np;
    float *p = probe_make_period(fs, &np, L_PEAK_DBFS);
    double pk = 0, ms = 0;
    for (int i = 0; i < np; i++) {
        pk = fmax(pk, fabs(p[i]));
        ms += p[i] * p[i];
    }
    double cf = 20 * log10(pk / sqrt(ms / np));
    printf("crest factor @%d: period %d samples, sampled peak %.2f dBFS, CF %.2f dB, RMS %.2f dBFS\n",
           fs, np, 20 * log10(pk), cf, 10 * log10(2 * ms / np));
    CHECK(np * BASE_HZ == fs, "period length");
    CHECK(20 * log10(pk) <= L_PEAK_DBFS + 0.01, "peak above -3 dBFS");
    CHECK(cf < 5.8, "crest factor too high (%.2f)", cf);
    free(p);
}

static void test_probe(int fs_tx, int fs_rx, double ppm, int filt, double noise_db)
{
    sigbuf_t b = {0};
    sig_glg(&b, fs_tx);
    biquad_t lp = lowpass(7000, 0.707, fs_tx);
    if (filt)
        bq_run(&lp, b.x, b.n);
    size_t n1, n2;
    float *y = resample(b.x, b.n, fs_tx, fs_rx, ppm * 1e-6, 37.3, &n1);
    float *z = pad(y, n1, fs_rx, 0.4, 0.4, noise_db, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs_rx, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r);

    printf("probe %d -> %d Hz, %+.0f ppm, %s, noise %.0f dBFS: rc %d, est %+.2f ppm\n",
           fs_tx, fs_rx, ppm, filt ? "LP 7 kHz" : "flat", noise_db, rc, r.ppm);
    CHECK(rc == 0 && r.valid, "probe not analysed");
    CHECK(r.has_l2 && fabs(r.rms_dbfs - r.rms2_dbfs - L2_STEP_DB) < 0.05, "L2 not received");
    CHECK(fabs(r.ppm - ppm) < 1.0, "ppm estimate %.2f vs %.2f", r.ppm, ppm);
    double maxa = 0, maxp = 0, minsnr = 1e9;
    double href = cabs(bq_resp(&lp, 1050, fs_tx));
    for (int h = 1; h <= NBINS; h++) {
        if (!probe_is_tone(h) || h * BASE_HZ > 9500)
            continue;
        double exp = filt ? 20 * log10(cabs(bq_resp(&lp, h * BASE_HZ, fs_tx)) / href) : 0;
        maxa = fmax(maxa, fabs(r.amp_db[h] - exp));
        if (!filt)
            maxp = fmax(maxp, fabs(r.phase_deg[h]));
        minsnr = fmin(minsnr, r.snr_db[h]);
    }
    double floor_db = -1e9;
    for (int h = 1; h <= NBINS; h++)
        if (!probe_is_tone(h))
            floor_db = fmax(floor_db, r.amp_db[h]);
    printf("  max amp error %.3f dB, max phase error %.2f deg, min SNR %.1f dB, "
           "worst silent bin %.1f dBc\n  level %.2f dBFS RMS, guard tones %d / %d ms\n",
           maxa, maxp, minsnr, floor_db, r.rms_dbfs, r.lead_g_ms, r.trail_g_ms);
    CHECK(abs(r.lead_g_ms - DUR_G_MS) <= 20 && abs(r.trail_g_ms - DUR_G_MS) <= 20, "guard durations");
    if (g_verbose)
        for (int h = 1; h <= NBINS; h++)
            printf("    %5d %+7.2f %+7.2f %6.1f\n", h * BASE_HZ, r.amp_db[h], r.phase_deg[h], r.snr_db[h]);
    CHECK(maxa < 0.3, "amplitude error %.3f dB", maxa);
    CHECK(maxp < 2.0, "phase error %.2f deg", maxp);
    sig_free(&b);
    free(y);
    free(z);
}

static void test_modem(int fs_tx, int fs_rx, double ppm, double noise_db)
{
    probe_result_t r = {0}, back;
    r.valid = 1;
    r.fs = fs_tx;
    r.ppm = -12.345;
    for (int h = 1; h <= NBINS; h++) {
        r.amp_db[h] = -0.37 * h + 0.5 * sin(h);
        r.phase_deg[h] = probe_is_tone(h) ? 170 * sin(0.3 * h) : 0;
        r.snr_db[h] = probe_is_tone(h) ? 30 + h % 7 : 0;
    }
    uint8_t msg[REPORT_LEN], out[1024];
    size_t len = report_pack(&r, msg), olen = 0, n1, n2;
    float *x;
    size_t n = modem_build(fs_tx, msg, len, &x);
    float *y = resample(x, n, fs_tx, fs_rx, ppm * 1e-6, 11.7, &n1);
    float *z = pad(y, n1, fs_rx, 0.3, 0.5, noise_db, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs_rx, NULL};
    int heard = 0;
    int rc = rx_modem(&s, 2000, out, sizeof(out), &olen, &heard);
    int ok = rc == 0 && olen >= len && !memcmp(out, msg, len) && !report_unpack(out, olen, &back) &&
             fabs(back.ppm - r.ppm) < 0.006;
    printf("modem %d -> %d Hz, %+.0f ppm, noise %.0f dBFS: %zu of %zu bytes, burst %.2f s, "
           "sync held %d ms, rc %d, %s\n", fs_tx, fs_rx, ppm, noise_db, len, modem_payload_max(),
           (double)n / fs_tx, heard, rc, ok ? "OK" : "BAD");
    CHECK(ok, "modem frame not recovered");
    free(x);
    free(y);
    free(z);
}


/* A +20 deg anomaly on one carrier must show on that carrier only (the
 * phase is not referenced to 1050 Hz). */
static void test_phase_anomaly(int fs, int hbad, double deg)
{
    int np = fs / BASE_HZ;
    float *per = probe_make_period(fs, &np, L_PEAK_DBFS);
    /* Amplitude of one tone in this period (all tones are equal). */
    double complex c = 0;
    for (int k = 0; k < np; k++)
        c += per[k] * cexp(-I * 2 * M_PI * REF_BIN * k / np);
    double a = 2 * cabs(c) / np, ph = probe_phase_deg(hbad) * M_PI / 180;
    for (int k = 0; k < np; k++) {
        double t = 2 * M_PI * hbad * k / np;
        per[k] += (float)(a * (cos(t + ph + deg * M_PI / 180) - cos(t + ph)));
    }
    sigbuf_t b = {0};
    double ga = db2lin(TONE_DBFS);
    for (int lv = 0; lv < 2; lv++) {         /* G, L1, G, L2, G with the altered L */
        sig_tone(&b, fs, TONE_G_HZ, DUR_G_MS, ga);
        size_t l0 = b.n;
        sig_probe(&b, fs, DUR_L_MS, 1.0);
        for (size_t i = l0; i < b.n; i++)
            b.x[i] = per[(i - l0) % np] * (lv ? L2_GAIN : 1);
    }
    sig_tone(&b, fs, TONE_G_HZ, DUR_G_MS, ga);
    size_t n2;
    float *z = pad(b.x, b.n, fs, 0.4, 0.4, -70, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r);
    double other = 0;
    for (int h = 1; h <= NBINS; h++)
        if (probe_is_tone(h) && h != hbad)
            other = fmax(other, fabs(r.phase_deg[h]));
    printf("phase anomaly %+.0f deg on %d Hz: measured %+.2f deg there, others within %.2f deg\n",
           deg, hbad * BASE_HZ, r.phase_deg[hbad], other);
    CHECK(rc == 0 && fabs(r.phase_deg[hbad] - deg) < 2.0 && other < 2.0, "phase anomaly");
    free(per);
    free(z);
    sig_free(&b);
}

/* Voice-band noise (300..3000 Hz), as an FM receiver with open squelch. */
static void vb_noise(float *x, size_t n, double db, unsigned seed)
{
    biquad_t hp, lp;
    srand(seed);
    for (size_t i = 0; i < n; i++)
        x[i] = (float)gauss();
    /* 2nd order high-pass at 300 Hz and two 2nd order low-pass at 3 kHz */
    double w = 2 * M_PI * 300 / 48000.0, al = sin(w) / (2 * 0.707), c = cos(w), a0 = 1 + al;
    hp = (biquad_t){(1 + c) / 2 / a0, -(1 + c) / a0, (1 + c) / 2 / a0, -2 * c / a0, (1 - al) / a0};
    lp = lowpass(3000, 0.707, 48000);
    bq_run(&hp, x, n);
    bq_run(&lp, x, n);
    bq_run(&lp, x, n);
    double ms = 0;
    for (size_t i = 0; i < n; i++)
        ms += x[i] * x[i];
    double g = db2lin(db) / sqrt(ms / n);
    for (size_t i = 0; i < n; i++)
        x[i] *= (float)g;
}

/* Results burst preceded by a 150 ms squelch-noise blip, with the receiver
 * output otherwise quiet, or with open-squelch noise all along. */
static void test_modem_noise(int open_squelch)
{
    const int fs = 48000;
    probe_result_t q = {0};
    q.valid = 1;
    uint8_t msg[REPORT_LEN], out[1024];
    size_t len = report_pack(&q, msg), olen = 0;
    float *x;
    size_t n = modem_build(fs, msg, len, &x);
    size_t lead = fs * 6 / 10, tail = fs / 2, N = lead + n + tail;
    float *z = calloc(N, sizeof(float)), *nz = malloc(sizeof(float) * N);
    vb_noise(nz, N, -25, 7);
    for (size_t i = 0; i < N; i++) {
        int burst = i >= lead && i < lead + n;
        int blip = i >= fs / 10 && i < fs / 4;          /* 100..250 ms */
        if (open_squelch || blip)
            z[i] = nz[i];
        else
            z[i] = 0.001f * nz[i];
        if (burst)
            z[i] += x[i - lead];
    }
    memsrc_t m = {z, N, 0};
    source_t s = {memread, &m, fs, NULL};
    int heard = 0;
    int rc = rx_modem(&s, 2000, out, sizeof(out), &olen, &heard);
    printf("modem after a squelch blip%s: rc %d, returned at %.2f s (burst ends at %.2f s)\n",
           open_squelch ? ", squelch open" : "", rc, (double)m.pos / fs, (double)(lead + n) / fs);
    CHECK(rc == 0 && !memcmp(out, msg, len), "modem lost after noise blip");

    /* Noise only: must time out, not hang nor report a burst. */
    vb_noise(nz, N, -16, 9);
    for (size_t i = 0; i < N; i++)
        z[i] = open_squelch ? nz[i] : ((i >= fs / 10 && i < fs / 4) ? nz[i] : 0.001f * nz[i]);
    memsrc_t m2 = {z, N, 0};
    source_t s2 = {memread, &m2, fs, NULL};
    rc = rx_modem(&s2, 3000, out, sizeof(out), &olen, &heard);
    printf("modem, noise only%s: rc %d (1 = timeout), returned at %.2f s\n",
           open_squelch ? ", squelch open" : "", rc, (double)m2.pos / fs);
    CHECK(rc == 1 && m2.pos <= (size_t)fs * 31 / 10, "noise only must time out after 3 s");
    free(x);
    free(z);
    free(nz);
}

/* Audio continuity: one sample dropped, or a 20 ms dropout, inside L. */
static void test_continuity(int fs, int kind)
{
    sigbuf_t b = {0};
    sig_glg(&b, fs);
    size_t mid = (size_t)fs * (DUR_G_MS + 500) / 1000, n2;
    if (kind == 1) {                     /* sample slip */
        memmove(b.x + mid, b.x + mid + 1, sizeof(float) * (b.n - mid - 1));
        b.n--;
    } else if (kind == 2) {              /* 20 ms dropout */
        memset(b.x + mid, 0, sizeof(float) * (fs / 50));
    }
    float *z = pad(b.x, b.n, fs, 0.4, 0.4, -50, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r);
    const char *what[] = {"clean", "one sample slipped", "20 ms dropout"};
    printf("continuity @%d, %s: lowest period %+.1f dB, %d jump(s), largest step %.1f us\n", fs,
           what[kind], r.min_level_db, r.jumps, r.max_jump_us);
    CHECK(rc == 0, "probe");
    if (kind == 0)
        CHECK(r.jumps == 0 && r.min_level_db > -1, "false glitch");
    if (kind == 1)
        CHECK(r.jumps >= 1 && fabs(r.max_jump_us - 1e6 / fs) < 0.3e6 / fs, "slip not seen");
    if (kind == 2)
        CHECK(r.min_level_db < -3, "dropout not seen");
    free(z);
    sig_free(&b);
}

/* Memoryless nonlinearity in the channel: y = x + a2 x^2 + a3 x^3. The mute
 * bins must show the distortion and its order; a3 < 0 compresses. */
static void test_distortion(int fs, double a2, double a3, int expect)
{
    sigbuf_t b = {0};
    sig_glg(&b, fs);
    for (size_t i = 0; i < b.n; i++) {
        double x = b.x[i];
        b.x[i] = (float)(x + a2 * x * x + a3 * x * x * x);
    }
    size_t n2;
    float *z = pad(b.x, b.n, fs, 0.4, 0.4, -80, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r);
    int good = 0, inband = 0, nt = 0;
    double cm = 0;
    for (int h = 1; h <= NBINS; h++)
        if (probe_is_tone(h)) {
            cm += r.comp_db[h];
            nt++;
        }
    cm /= nt;
    printf("distortion a2 %.2f a3 %.2f: rc %d, L2 %s, level step %.2f dB, mean comp %+.2f dB\n   ",
           a2, a3, rc, r.has_l2 ? "ok" : "missing", r.rms_dbfs - r.rms2_dbfs, cm);
    for (int h = 1; h <= NBINS; h++) {
        if (probe_is_tone(h))
            continue;
        int k = probe_dist_order(&r, h);
        printf(" %d Hz: %+.1f/%+.1f dBc n=%d;", h * BASE_HZ, r.dist_dbc[h], r.dist2_dbc[h], k);
        inband++;
        good += k == expect;
    }
    printf("\n");
    CHECK(rc == 0 && r.has_l2, "two-level probe");
    CHECK(good >= inband - 1, "order %d not recognised (%d of %d)", expect, good, inband);
    if (a3 < 0)
        CHECK(cm < -0.2 && r.rms_dbfs - r.rms2_dbfs < L2_STEP_DB - 0.3, "compression not seen");
    else
        CHECK(fabs(cm) < 0.2, "spurious compression");
    free(z);
    sig_free(&b);
}

/* Clean channel: no distortion reported, no compression. */
static void test_no_distortion(int fs)
{
    sigbuf_t b = {0};
    sig_glg(&b, fs);
    size_t n2;
    float *z = pad(b.x, b.n, fs, 0.4, 0.4, -50, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r), seen = 0;
    double cmax = 0;
    for (int h = 1; h <= NBINS; h++) {
        if (probe_is_tone(h))
            cmax = fmax(cmax, fabs(r.comp_db[h]));
        else
            seen += r.dist_ok[h];
    }
    printf("no distortion @%d: rc %d, %d mute bin(s) with distortion, max |comp| %.3f dB\n", fs,
           rc, seen, cmax);
    CHECK(rc == 0 && r.has_l2 && seen == 0 && cmax < 0.1, "clean channel");
    free(z);
    sig_free(&b);
}

/* Transmission stops after the middle guard tone: L1 results, no L2. */
static void test_l2_missing(int fs)
{
    sigbuf_t b = {0};
    double ga = db2lin(TONE_DBFS);
    sig_tone(&b, fs, TONE_G_HZ, DUR_G_MS, ga);
    sig_probe(&b, fs, DUR_L_MS, 1.0);
    sig_tone(&b, fs, TONE_G_HZ, DUR_G_MS, ga);
    size_t n2;
    float *z = pad(b.x, b.n, fs, 0.4, 0.6, -60, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r);
    printf("L2 missing @%d: rc %d, valid %d, has_l2 %d, guard tones %d / %d / %d ms\n", fs, rc,
           r.valid, r.has_l2, r.lead_g_ms, r.mid_g_ms, r.trail_g_ms);
    CHECK(rc == 0 && r.valid && !r.has_l2 && abs(r.trail_g_ms - DUR_G_MS) <= 20, "L2 missing");
    free(z);
    sig_free(&b);
}

/* The transmitter is keyed off before the end of the audio (latency in the
 * radio's audio path): the receiver must see it, and the data must survive. */
static void test_clipped_tail(int fs, int cut_ms)
{
    sigbuf_t b = {0};
    sig_glg(&b, fs);
    size_t cut = (size_t)cut_ms * fs / 1000, n2;
    float *z = pad(b.x, b.n - cut, fs, 0.4, 0.4, -60, &n2);
    memsrc_t m = {z, n2, 0};
    source_t s = {memread, &m, fs, NULL};
    probe_result_t r;
    int rc = rx_probe(&s, 2000, &r);
    printf("probe, last %d ms cut: rc %d, guard tones %d / %d ms\n", cut_ms, rc, r.lead_g_ms,
           r.trail_g_ms);
    CHECK(rc == 0 && abs(r.trail_g_ms - (DUR_G_MS - cut_ms)) <= 20, "trailing guard measure");
    free(z);
    sig_free(&b);

    probe_result_t q = {0};
    q.valid = 1;
    uint8_t msg[REPORT_LEN], out[1024];
    size_t len = report_pack(&q, msg), olen = 0;
    float *x;
    size_t n = modem_build(fs, msg, len, &x);
    z = pad(x, n - cut, fs, 0.3, 0.5, -40, &n2);
    memsrc_t m2 = {z, n2, 0};
    source_t s2 = {memread, &m2, fs, NULL};
    int heard;
    rc = rx_modem(&s2, 2000, out, sizeof(out), &olen, &heard);
    printf("modem, last %d ms cut: rc %d\n", cut_ms, rc);
    CHECK(rc == 0 && !memcmp(out, msg, len), "clipped modem burst not decoded");
    free(x);
    free(z);
}

/* ---- full session over a simulated half-duplex link ------------------ */
/* Each side has its own sample rate and clock error; both run in their own
 * thread on a shared, simulated time line (no real-time waits). */

typedef struct sim sim_t;
typedef struct {
    sim_t *sim;
    int id, fs, txdelay, txtail, nf;
    double eps, noise;
    long k, pend_end;            /* own sample clock */
    int reading;
    biquad_t f[3];               /* receiver-side filter chain */
    double st[3][4];
    uint64_t rng;
} sside_t;
typedef struct { double t0, rate; float *x; size_t n; } sburst_t;
struct sim {
    pthread_mutex_t m;
    pthread_cond_t c;
    sside_t s[2];
    sburst_t b[2][8];
    int nb[2], done[2];
};

static double stime(const sside_t *s, long k) { return k / (s->fs * (1 + s->eps)); }

static double bound(sim_t *m, int id)
{
    const sside_t *o = &m->s[id];
    if (m->done[id])
        return 1e30;
    long k = o->reading && o->pend_end > o->k ? o->pend_end : o->k;
    return stime(o, k);
}

static double urand(sside_t *s)
{
    s->rng = s->rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return ((s->rng >> 11) + 0.5) / 9007199254740992.0;
}

static int sim_read(void *ctx, float *buf, int n)
{
    sside_t *s = ctx;
    sim_t *m = s->sim;
    int o = !s->id;
    pthread_mutex_lock(&m->m);
    s->pend_end = s->k + n;
    pthread_cond_broadcast(&m->c);
    while (bound(m, o) < stime(s, s->k + n))
        pthread_cond_wait(&m->c, &m->m);
    const int T = 16;
    for (int i = 0; i < n; i++) {
        double t = stime(s, s->k + i), v = 0;
        for (int j = 0; j < m->nb[o]; j++) {
            sburst_t *b = &m->b[o][j];
            double u = (t - b->t0) * b->rate;
            if (u < -T || u > b->n + T)
                continue;
            long i0 = (long)floor(u);
            for (long q = i0 - T + 1; q <= i0 + T; q++) {
                if (q < 0 || q >= (long)b->n)
                    continue;
                double d = u - q;
                double w = 0.42 + 0.5 * cos(M_PI * d / T) + 0.08 * cos(2 * M_PI * d / T);
                v += b->x[q] * (fabs(d) < 1e-12 ? 1.0 : sin(M_PI * d) / (M_PI * d)) * w;
            }
        }
        double g = sqrt(-2 * log(urand(s))) * cos(2 * M_PI * urand(s));
        v += s->noise * g;
        for (int q = 0; q < s->nf; q++) {
            biquad_t *f = &s->f[q];
            double *z = s->st[q];
            double y = f->b0 * v + f->b1 * z[0] + f->b2 * z[1] - f->a1 * z[2] - f->a2 * z[3];
            z[1] = z[0]; z[0] = v; z[3] = z[2]; z[2] = y;
            v = y;
        }
        buf[i] = (float)v;
    }
    s->k += n;
    pthread_cond_broadcast(&m->c);
    pthread_mutex_unlock(&m->m);
    return n;
}

static int sim_tx(void *ctx, const float *x, size_t n)
{
    sside_t *s = ctx;
    sim_t *m = s->sim;
    pthread_mutex_lock(&m->m);
    long d = (long)s->txdelay * s->fs / 1000;
    sburst_t *b = &m->b[s->id][m->nb[s->id]++];
    b->t0 = stime(s, s->k + d);
    b->rate = s->fs * (1 + s->eps);
    b->x = malloc(sizeof(float) * n);
    memcpy(b->x, x, sizeof(float) * n);
    b->n = n;
    s->reading = 0;
    s->k += d + (long)n + (long)s->txtail * s->fs / 1000;
    pthread_cond_broadcast(&m->c);
    pthread_mutex_unlock(&m->m);
    return 0;
}

static int sim_rx_start(void *ctx)
{
    sside_t *s = ctx;
    pthread_mutex_lock(&s->sim->m);
    s->reading = 1;
    s->pend_end = s->k;
    pthread_cond_broadcast(&s->sim->c);
    pthread_mutex_unlock(&s->sim->m);
    return 0;
}

static void sim_wait(void *ctx, int ms)
{
    sside_t *s = ctx;
    pthread_mutex_lock(&s->sim->m);
    s->reading = 0;
    s->k += (long)ms * s->fs / 1000;
    pthread_cond_broadcast(&s->sim->c);
    pthread_mutex_unlock(&s->sim->m);
}

typedef struct { sside_t *s; session_result_t r; int rc; } sjob_t;

static void *sim_thread(void *arg)
{
    sjob_t *j = arg;
    sside_t *s = j->s;
    link_t l = {.fs = s->fs, .tw_ms = 100, .timeout_ms = 10000, .transmit = sim_tx,
                .rx_start = sim_rx_start, .wait_ms = sim_wait,
                .src = {sim_read, s, s->fs, NULL}, .ctx = s};
    j->rc = s->id == 0 ? session_initiator(&l, &j->r) : session_responder(&l, &j->r);
    pthread_mutex_lock(&s->sim->m);
    s->sim->done[s->id] = 1;
    pthread_cond_broadcast(&s->sim->c);
    pthread_mutex_unlock(&s->sim->m);
    return NULL;
}

static biquad_t highpass(double fc, double q, int fs)
{
    double w = 2 * M_PI * fc / fs, al = sin(w) / (2 * q), c = cos(w), a0 = 1 + al;
    biquad_t b = {(1 + c) / 2 / a0, -(1 + c) / a0, (1 + c) / 2 / a0, -2 * c / a0, (1 - al) / a0};
    return b;
}

static void test_session(int fsA, double ppmA, int fsB, double ppmB, int voiceband)
{
    sim_t m;
    memset(&m, 0, sizeof(m));
    pthread_mutex_init(&m.m, NULL);
    pthread_cond_init(&m.c, NULL);
    for (int i = 0; i < 2; i++) {
        sside_t *s = &m.s[i];
        s->sim = &m;
        s->id = i;
        s->fs = i ? fsB : fsA;
        s->eps = (i ? ppmB : ppmA) * 1e-6;
        s->txdelay = 100;
        s->txtail = 30;
        s->noise = db2lin(voiceband ? -25 : -60);
        s->rng = 12345 + i;
        if (voiceband) {             /* 300..3000 Hz, open squelch noise */
            s->nf = 3;
            s->f[0] = highpass(300, 0.707, s->fs);
            s->f[1] = lowpass(3000, 0.707, s->fs);
            s->f[2] = lowpass(3000, 0.707, s->fs);
        }
    }
    if (!voiceband) {                /* A -> B path: 7 kHz lowpass at B */
        m.s[1].nf = 1;
        m.s[1].f[0] = lowpass(7000, 0.707, fsB);
    }

    sjob_t ja = {.s = &m.s[0]}, jb = {.s = &m.s[1]};
    pthread_t ta, tb;
    pthread_create(&tb, NULL, sim_thread, &jb);
    pthread_create(&ta, NULL, sim_thread, &ja);
    pthread_join(ta, NULL);
    pthread_join(tb, NULL);

    double dA = (ppmB - ppmA), dB = -dA;   /* expected clock offsets seen */
    printf("session%s A %d Hz %+.0f ppm <-> B %d Hz %+.0f ppm: rc %d/%d, "
           "A rx %+.2f ppm (exp %+.0f), B rx %+.2f ppm (exp %+.0f), sim time %.1f s\n",
           voiceband ? " (300-3000 Hz, open squelch)" : "", fsA, ppmA, fsB, ppmB, ja.rc, jb.rc, ja.r.rx.ppm, dA, jb.r.rx.ppm, dB,
           stime(&m.s[0], m.s[0].k));
    CHECK(ja.rc == 0 && jb.rc == 0, "session failed");
    CHECK(ja.r.have_rx && ja.r.have_tx && jb.r.have_rx && jb.r.have_tx, "missing results");
    CHECK(fabs(ja.r.rx.ppm - dA) < 1 && fabs(jb.r.rx.ppm - dB) < 1, "ppm");
    double maxd = 0, maxa = 0, href = cabs(bq_resp(&m.s[1].f[0], 1050, fsB));
    for (int h = 1; h <= NBINS; h++) {
        /* What B measured must reach A unchanged (to the 0.01 unit) */
        maxd = fmax(maxd, fabs(ja.r.tx.amp_db[h] - jb.r.rx.amp_db[h]));
        maxd = fmax(maxd, fabs(jb.r.tx.amp_db[h] - ja.r.rx.amp_db[h]));
        if (!voiceband && probe_is_tone(h) && h * BASE_HZ <= 9500) {
            double e = 20 * log10(cabs(bq_resp(&m.s[1].f[0], h * BASE_HZ, fsB)) / href);
            maxa = fmax(maxa, fabs(jb.r.rx.amp_db[h] - e));
            maxa = fmax(maxa, fabs(ja.r.rx.amp_db[h]));
        }
    }
    printf("  report transfer max diff %.3f dB, max amplitude error %.3f dB\n", maxd, maxa);
    CHECK(maxd <= 0.006, "report transfer");
    CHECK(maxa < 0.3, "amplitude error %.3f", maxa);
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < m.nb[i]; j++)
            free(m.b[i][j].x);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "-v"))
        g_verbose = 1;
    srand(1);
    test_crest(44100);
    test_crest(48000);
    test_probe(48000, 48000, 0, 0, -70);
    test_probe(48000, 44100, +80, 0, -60);
    test_probe(44100, 48000, -50, 1, -60);
    test_probe(44100, 44100, +20, 1, -50);
    test_modem(48000, 44100, +80, -40);
    test_modem(44100, 48000, -80, -40);
    test_modem(48000, 48000, 0, -30);
    test_modem(48000, 44100, +40, -25);
    test_phase_anomaly(48000, REF_BIN, 20);
    test_phase_anomaly(44100, 17, -30);
    test_l2_missing(48000);
    test_no_distortion(48000);
    test_no_distortion(44100);
    test_distortion(48000, 0.10, 0, 2);
    test_distortion(48000, 0, -0.30, 3);
    test_modem_noise(0);
    test_modem_noise(1);
    test_continuity(48000, 0);
    test_continuity(48000, 1);
    test_continuity(44100, 1);
    test_continuity(48000, 2);
    test_clipped_tail(48000, 80);
    test_clipped_tail(44100, 60);
    test_session(48000, +30, 44100, -40, 0);
    test_session(44100, -25, 48000, +50, 0);
    test_session(48000, +15, 48000, -25, 1);
    test_session(44100, -10, 48000, +30, 1);
    printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
