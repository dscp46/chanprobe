/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_SIGNAL_H
#define CHANPROBE_SIGNAL_H

#include <stddef.h>
#include "common.h"
#include "probe.h"

/* ------------------------------------------------------ generation --- */

typedef struct {
    float *x;
    size_t n, cap;
} sigbuf_t;

int  sig_tone(sigbuf_t *b, int fs, double f, double ms, double amp);
int  sig_probe(sigbuf_t *b, int fs, double ms, double gain);  /* L, faded */
int  sig_glg(sigbuf_t *b, int fs);   /* G, L1, G, L2, G */
void sig_free(sigbuf_t *b);

/* ------------------------------------------------------- detection --- */

/* Fraction of the block power found at frequency f (0..1). */
double tone_ratio(const float *x, int n, double f, int fs);

/* Wait until tone f is present for >= min_ms. timeout_ms <= 0: forever.
 * Returns 0 when found, 1 on timeout, <0 on error / stop. */
int wait_tone(source_t *s, double f, int min_ms, int timeout_ms);
/* Wait until tone f has gone (absent for >= 20 ms). */
int wait_tone_end(source_t *s, double f, int timeout_ms);

/* Receive G + L + G and analyse L. Returns 0 on success, 1 on timeout,
 * 2 if L could not be analysed, <0 on error / stop. */
int rx_probe(source_t *s, int timeout_ms, probe_result_t *res);

#endif
