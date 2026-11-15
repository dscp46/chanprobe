/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_PROBE_H
#define CHANPROBE_PROBE_H

#include <stddef.h>
#include "common.h"

/* Phase of harmonic h (1..NBINS) in degrees, or -1 if the bin is silent. */
int probe_phase_deg(int h);
static inline int probe_is_tone(int h) { return probe_phase_deg(h) >= 0; }

/* One period (fs/150 samples) of L, scaled so that the true (oversampled)
 * peak equals peak_dbfs. Returns malloc'd buffer, *np = period length. */
float *probe_make_period(int fs, int *np, double peak_dbfs);

typedef struct {
    int    valid;
    int    fs;                    /* sample rate of the measuring side */
    double ppm;                   /* remote TX clock vs local RX clock */
    double amp_db[NBINS_PLUS1];   /* tone: dB re 1050 Hz; silent: level dBc */
    double phase_deg[NBINS_PLUS1];/* tone: phase anomaly, i.e. received minus sent
                                   * phase, minus the best linear fit (delay + const) */
    double snr_db[NBINS_PLUS1];   /* tone: per-period SNR in its 150 Hz bin */
    /* Silent (mute) bins, separated into the part that repeats with the
     * probe (distortion products) and the random part (noise), dBc: */
    double noise_dbc[NBINS_PLUS1];
    double dist_dbc[NBINS_PLUS1]; /* upper bound when !dist_ok */
    int    dist_ok[NBINS_PLUS1];  /* distortion measurable above the noise */
    double abs_db[NBINS_PLUS1];   /* local only: absolute tone level, dBFS-ish */
    /* Level dependence, L1 versus L2 (filled by probe_combine): */
    int    has_l2;
    double rms2_dbfs;             /* received level of L2 */
    double comp_db[NBINS_PLUS1];  /* tone: gain change beyond the 6.02 dB step */
    double ampm_deg[NBINS_PLUS1]; /* tone: phase change L1 vs L2 */
    double dist2_dbc[NBINS_PLUS1], noise2_dbc[NBINS_PLUS1];
    int    dist2_ok[NBINS_PLUS1];
    double rms_dbfs;              /* received level of L (full-scale sine = 0) */
    int lead_g_ms, trail_g_ms;    /* how long the guard tones were heard */
    int mid_g_ms;                 /* local only: guard tone between L1 and L2 */
    /* Continuity of the audio over the analysed 128 periods (853 ms): */
    double min_level_db;          /* lowest period level re the average (dropouts) */
    int jumps;                    /* sudden timing steps between periods (slips) */
    double max_jump_us;           /* largest timing step between two periods */
} probe_result_t;

/* Analyse AN_PERIODS periods of L starting at x[0].
 * n must be >= AN_PERIODS * fs/150. Returns 0 on success. */
int probe_analyze(const float *x, size_t n, int fs, probe_result_t *r);


/* Merge the analysis of L2 (r2) into that of L1 (r1). */
void probe_combine(probe_result_t *r1, const probe_result_t *r2);

/* Distortion order from how a mute bin's distortion scales between L1 and
 * L2 (dBc rises by (n - 1) x 6.02 dB). 0 when it cannot be told. */
int probe_dist_order(const probe_result_t *r, int h);

#endif
