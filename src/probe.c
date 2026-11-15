/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Probe signal L: 60 cosines on a 150 Hz grid, 150..10500 Hz.
 * 150..3750 Hz: ITU-T V.34 Table 17 phases (900/1200/1800/2400 Hz silent,
 * and 3600 Hz silent as well, unlike V.34).
 * 3900..10500 Hz: 0/180 deg pattern jointly optimised for crest factor
 * (4800/6000/7200/8400/9600 Hz silent). Crest factor ~5.7 dB.
 */
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "probe.h"

static const short phase_tab[NBINS] = {
    /*  150 */ 0, 180, 0, 0, 0, -1, 0, -1,
    /* 1350 */ 0, 0, 180, -1, 0, 0, 180, -1,
    /* 2550 */ 0, 180, 0, 180, 180, 180, 180, -1,   /* 3600 Hz: mute */
    /* 3750 */ 0, 180, 0, 180, 0, 0, 180, -1,
    /* 4950 */ 0, 0, 0, 180, 180, 0, 0, -1,
    /* 6150 */ 0, 0, 180, 180, 0, 0, 180, -1,
    /* 7350 */ 180, 180, 180, 0, 180, 0, 180, -1,
    /* 8550 */ 0, 180, 0, 0, 0, 180, 180, -1,
    /* 9750 */ 180, 180, 180, 180, 0, 0,
};

int probe_phase_deg(int h)
{
    if (h < 1 || h > NBINS)
        return -1;
    return phase_tab[h - 1];
}

float *probe_make_period(int fs, int *np, double peak_dbfs)
{
    int N = fs / BASE_HZ;
    float *p = malloc(sizeof(float) * N);
    double *acc = calloc(N, sizeof(double));
    if (!p || !acc) {
        free(p);
        free(acc);
        return NULL;
    }
    /* True peak estimated on a 16x oversampled grid. */
    const int OS = 16;
    double peak = 0;
    for (int m = 0; m < N * OS; m++) {
        double t = (double)m / (N * OS), s = 0;
        for (int h = 1; h <= NBINS; h++)
            if (probe_is_tone(h))
                s += cos(2 * M_PI * h * t + probe_phase_deg(h) * M_PI / 180);
        if (fabs(s) > peak)
            peak = fabs(s);
        if (m % OS == 0)
            acc[m / OS] = s;
    }
    double g = db2lin(peak_dbfs) / peak;
    for (int n = 0; n < N; n++)
        p[n] = (float)(acc[n] * g);
    free(acc);
    *np = N;
    return p;
}

static double wrap(double a)
{
    a = fmod(a + M_PI, 2 * M_PI);
    if (a < 0)
        a += 2 * M_PI;
    return a - M_PI;
}

static void dft_periods(const float *x, int N, int P, const double complex *tw,
                        double complex *Y)
{
    for (int p = 0; p < P; p++) {
        const float *xp = x + (size_t)p * N;
        for (int h = 1; h <= NBINS; h++) {
            double complex s = 0;
            int idx = 0;
            for (int k = 0; k < N; k++) {
                s += xp[k] * tw[idx];
                idx += h;
                if (idx >= N)
                    idx -= N;
            }
            Y[h * P + p] = s * (2.0 / N);
        }
    }
}

/* Relative clock offset: tone h rotates by 2*pi*h*eps per period. Phase
 * change over `lag` periods; a long lag is more precise but only unambiguous
 * while 70 * eps * lag < 0.5 (lag 1: up to 7000 ppm; lag 64: 110 ppm). */
static double est_eps(const double complex *Y, int P, int lag)
{
    double num = 0, den = 0;
    for (int h = 1; h <= NBINS; h++) {
        if (!probe_is_tone(h))
            continue;
        double complex R = 0;
        for (int p = 0; p + lag < P; p++)
            R += Y[h * P + p + lag] * conj(Y[h * P + p]);
        double w = cabs(R);
        num += w * h * carg(R);
        den += w * (double)h * h;
    }
    return den > 0 ? num / (2 * M_PI * den * lag) : 0;
}

/* y[k] = x(T + k / (1 + eps)), Blackman-windowed sinc, 2T taps. */
#define RS_T 16
static void resample_eps(const float *x, float *y, size_t m, double eps)
{
    double step = 1.0 / (1.0 + eps);
    for (size_t k = 0; k < m; k++) {
        double u = RS_T + k * step;
        long i0 = (long)floor(u);
        double fr = u - i0, s = 0;
        /* sin(pi (j - fr)) = (-1)^(j+1) sin(pi fr) */
        double sfr = sin(M_PI * fr);
        for (int j = -RS_T + 1; j <= RS_T; j++) {
            double d = j - fr;   /* tap position relative to u */
            double c = cos(M_PI * d / RS_T);
            double w = 0.42 + 0.5 * c + 0.08 * (2 * c * c - 1);
            double sn = fabs(d) < 1e-9 ? 1.0 : ((j & 1) ? sfr : -sfr) / (M_PI * d);
            s += x[i0 + j] * sn * w;
        }
        y[k] = (float)s;
    }
}

int probe_analyze(const float *x, size_t n, int fs, probe_result_t *r)
{
    const int N = fs / BASE_HZ, P = AN_PERIODS;
    const size_t m = (size_t)N * P;
    memset(r, 0, sizeof(*r));
    r->fs = fs;
    if (n < m)
        return -1;
    double ms = 0;
    for (size_t i = 0; i < m; i++)
        ms += (double)x[i] * x[i];
    r->rms_dbfs = 10 * log10(2 * ms / m + 1e-30);

    double complex *Y = malloc(sizeof(double complex) * NBINS_PLUS1 * P);
    double complex *tw = malloc(sizeof(double complex) * N);
    float *y = malloc(sizeof(float) * m);
    if (!Y || !tw || !y) {
        free(Y);
        free(tw);
        free(y);
        return -1;
    }
    for (int k = 0; k < N; k++)
        tw[k] = cexp(-I * 2 * M_PI * k / N);

    /* Pass 1: coarse clock offset. */
    dft_periods(x, N, P, tw, Y);
    double eps1 = est_eps(Y, P, 1), eps = eps1;
    /* Pass 2: resample to cancel it (avoids inter-tone leakage), re-measure. */
    if (fabs(eps1) < 0.01 && RS_T + (m + 1) / (1.0 + eps1) + RS_T < (double)n) {
        resample_eps(x, y, m, eps1);
        dft_periods(y, N, P, tw, Y);
        eps = est_eps(Y, P, 1);              /* residual, a fraction of a ppm */
        if (fabs(eps) < 20e-6)
            eps = est_eps(Y, P, P / 2);      /* refine on a long baseline */
        r->ppm = ((1 + eps1) * (1 + eps) - 1) * 1e6;
    } else {
        r->ppm = eps1 * 1e6;
    }
    free(y);

    /* Derotate, average, measure variance. */
    double complex M[NBINS_PLUS1];
    double var[NBINS_PLUS1], pw[NBINS_PLUS1];
    for (int h = 1; h <= NBINS; h++) {
        double complex m = 0;
        double c = (P - 1) / 2.0;
        for (int p = 0; p < P; p++) {
            Y[h * P + p] *= cexp(-I * 2 * M_PI * h * eps * (p - c));
            m += Y[h * P + p];
        }
        m /= P;
        double v = 0, q = 0;
        for (int p = 0; p < P; p++) {
            double complex d = Y[h * P + p] - m;
            v += creal(d * conj(d));
            q += creal(Y[h * P + p] * conj(Y[h * P + p]));
        }
        M[h] = m;
        var[h] = v / (P - 1);
        pw[h] = q / P;
    }

    /* Continuity: level and timing of each period against the average.
     * A dropout lowers the level of a period; a slipped or repeated sample
     * steps the timing (phase slope across the tones) between periods. */
    double *tp = malloc(sizeof(double) * P), *dd = malloc(sizeof(double) * P);
    r->min_level_db = 0;
    r->jumps = 0;
    r->max_jump_us = 0;
    if (tp && dd) {
        for (int p = 0; p < P; p++) {
            double complex C = 0;
            double E = 0, Sw = 0, Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
            for (int h = 1; h <= NBINS; h++) {
                if (!probe_is_tone(h))
                    continue;
                double complex z = Y[h * P + p] * conj(M[h]);
                double ww = creal(M[h] * conj(M[h]));
                C += z;
                E += ww;
                double y = carg(z);
                Sw += ww; Sx += ww * h; Sy += ww * y; Sxx += ww * h * h; Sxy += ww * h * y;
            }
            double g = E > 0 ? cabs(C) / E : 0;
            double lv = 20 * log10(g + 1e-9);
            if (lv < r->min_level_db)
                r->min_level_db = lv;
            double b = (Sw * Sxy - Sx * Sy) / (Sw * Sxx - Sx * Sx + 1e-30);
            tp[p] = b * N / (2 * M_PI);            /* samples */
        }
        /* Robust spread of the period-to-period steps (median absolute
         * deviation), so that only steps well above the noise count. */
        int nd = P - 1;
        for (int p = 0; p < nd; p++)
            dd[p] = fabs(tp[p + 1] - tp[p]);
        double *srt = malloc(sizeof(double) * nd);
        double sig = 0;
        if (srt) {
            memcpy(srt, dd, sizeof(double) * nd);
            for (int a = 1; a < nd; a++)        /* insertion sort, nd = 127 */
                for (int c = a; c > 0 && srt[c - 1] > srt[c]; c--) {
                    double t = srt[c]; srt[c] = srt[c - 1]; srt[c - 1] = t;
                }
            sig = 1.4826 * srt[nd / 2];
            free(srt);
        }
        double thr = fmax(0.3, 6 * sig);
        for (int p = 0; p < nd; p++) {
            if (dd[p] > thr)
                r->jumps++;
            if (dd[p] * 1e6 / fs > r->max_jump_us)
                r->max_jump_us = dd[p] * 1e6 / fs;
        }
    }
    free(tp);
    free(dd);
    free(Y);
    free(tw);

    double ref = cabs(M[REF_BIN]);
    if (ref < 1e-5 || 20 * log10(ref * ref / (var[REF_BIN] + 1e-30)) < 0) {
        r->valid = 0;
        return -1;
    }

    /* Phase relative to the transmitted one. */
    double th[NBINS_PLUS1], w[NBINS_PLUS1];
    for (int h = 1; h <= NBINS; h++) {
        if (!probe_is_tone(h))
            continue;
        th[h] = carg(M[h]) - probe_phase_deg(h) * M_PI / 180;
        w[h] = cabs(M[h]);
    }
    /* Coarse bulk delay (in samples, modulo one period). */
    double best = -1, tau = 0;
    for (int i = 0; i < 4 * N; i++) {
        double t = i / 4.0;
        double complex s = 0;
        for (int h = 1; h <= NBINS; h++)
            if (probe_is_tone(h))
                s += w[h] * cexp(I * (th[h] + 2 * M_PI * h * t / N));
        if (cabs(s) > best) {
            best = cabs(s);
            tau = t;
        }
    }
    /* Weighted LS fit of a straight line (constant + bulk delay) through the
     * received-minus-transmitted phases. The phase anomaly of each carrier is
     * its deviation from that line. */
    double complex s0 = 0;
    for (int h = 1; h <= NBINS; h++)
        if (probe_is_tone(h))
            s0 += w[h] * cexp(I * (th[h] + 2 * M_PI * h * tau / N));
    double c0 = carg(s0);
    double yp[NBINS_PLUS1];
    double Sw = 0, Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
    for (int h = 1; h <= NBINS; h++) {
        if (!probe_is_tone(h))
            continue;
        yp[h] = wrap(th[h] + 2 * M_PI * h * tau / N - c0);
        double ww = w[h] * w[h];
        Sw += ww; Sx += ww * h; Sy += ww * yp[h]; Sxx += ww * h * h; Sxy += ww * h * yp[h];
    }
    double b = (Sw * Sxy - Sx * Sy) / (Sw * Sxx - Sx * Sx);
    double a = (Sy - b * Sx) / Sw;

    for (int h = 1; h <= NBINS; h++) {
        if (probe_is_tone(h)) {
            double m = cabs(M[h]);
            r->amp_db[h] = 20 * log10(m / ref + 1e-12);
            r->snr_db[h] = 10 * log10(m * m / (var[h] + 1e-30));
            r->phase_deg[h] = wrap(yp[h] - a - b * h) * 180 / M_PI;
            r->abs_db[h] = 20 * log10(m + 1e-12);
        } else {
            /* |mean|^2 still holds var / P of noise: remove it. Distortion is
             * measurable when it stands 10 dB above that residual (noise alone
             * gets there with a probability of 5e-5). */
            double floor = var[h] / P;
            double coh = creal(M[h] * conj(M[h])) - floor;
            r->noise_dbc[h] = 10 * log10(var[h] / (ref * ref) + 1e-30);
            r->dist_ok[h] = coh > 10 * floor;
            r->dist_dbc[h] = 10 * log10((r->dist_ok[h] ? coh : 10 * floor) / (ref * ref) + 1e-30);
            r->amp_db[h] = r->noise_dbc[h];
            r->snr_db[h] = 0;
            r->phase_deg[h] = 0;
        }
    }
    (void)pw;
    r->valid = 1;
    return 0;
}

void probe_combine(probe_result_t *r1, const probe_result_t *r2)
{
    if (!r1->valid || !r2->valid)
        return;
    r1->has_l2 = 1;
    r1->rms2_dbfs = r2->rms_dbfs;
    for (int h = 1; h <= NBINS; h++) {
        if (probe_is_tone(h)) {
            r1->comp_db[h] = (r1->abs_db[h] - r2->abs_db[h]) - L2_STEP_DB;
            r1->ampm_deg[h] = wrap((r1->phase_deg[h] - r2->phase_deg[h]) * M_PI / 180) * 180 / M_PI;
        } else {
            r1->dist2_dbc[h] = r2->dist_dbc[h];
            r1->dist2_ok[h] = r2->dist_ok[h];
            r1->noise2_dbc[h] = r2->noise_dbc[h];
        }
    }
    /* Continuity: the worse of the two windows. */
    if (r2->min_level_db < r1->min_level_db)
        r1->min_level_db = r2->min_level_db;
    r1->jumps += r2->jumps;
    if (r2->max_jump_us > r1->max_jump_us)
        r1->max_jump_us = r2->max_jump_us;
}

int probe_dist_order(const probe_result_t *r, int h)
{
    if (!r->valid || !r->has_l2 || probe_is_tone(h) || !r->dist_ok[h] || !r->dist2_ok[h])
        return 0;
    double n = 1 + (r->dist_dbc[h] - r->dist2_dbc[h]) / L2_STEP_DB;
    long k = lround(n);
    if (k < 2 || fabs(n - k) > 0.4)
        return 0;
    return (int)k;
}
