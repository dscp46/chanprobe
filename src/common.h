/* chanprobe - audio channel prober
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CHANPROBE_COMMON_H
#define CHANPROBE_COMMON_H

#include <signal.h>
#include <stddef.h>

#define CP_VERSION "0.4.2"

/* Probe comb: harmonics h = 1..NBINS of BASE_HZ (150 .. 10500 Hz). */
#define BASE_HZ   150
#define NBINS     70
#define NBINS_PLUS1 (NBINS + 1)
#define REF_BIN   7          /* 1050 Hz: amplitude / phase reference */

/* Levels (dBFS; a full-scale sine is 0 dBFS, AES17). */
#define L_PEAK_DBFS     (-3.0)
#define TONE_DBFS       (-10.0)  /* A, G and tune tone; data burst RMS */
#define L_RMS_DBFS      (-5.7)   /* resulting RMS level of L1 */
/* L2 follows L1 at exactly half the amplitude (V.34 style two-level probe). */
#define L2_GAIN         0.5
#define L2_STEP_DB      6.0206   /* 20 log10(2) */

#define TONE_A_HZ   900.0
#define TONE_G_HZ   1200.0
#define TUNE_HZ     1000.0

#define DUR_A_MS    300
#define DUR_G_MS    200
#define DUR_L_MS    1000      /* per level: 150 periods of 6.667 ms */
#define FADE_MS     3.0

/* Analysis window: skip the first periods of L, average the next ones. */
#define AN_SKIP_PERIODS  11
#define AN_PERIODS       128

extern volatile sig_atomic_t g_stop;
extern int g_verbose;

void logmsg(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void dbg(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static inline double db2lin(double db) { return __builtin_pow(10.0, db / 20.0); }

/* Generic blocking sample source (ALSA capture, or a memory buffer in tests).
 * read() returns the number of mono float samples delivered (== n), or <0. */
typedef struct {
    int (*read)(void *ctx, float *buf, int n);
    void *ctx;
    int fs;
    unsigned long *xruns;    /* optional overrun counter */
} source_t;

#endif
