/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_SESSION_H
#define CHANPROBE_SESSION_H

#include "common.h"
#include "probe.h"

/* Half duplex link as seen by the protocol. */
typedef struct {
    int fs;
    int tw_ms, timeout_ms;
    /* Keyed transmission: PTT on, txdelay, x, txtail, PTT off. */
    int  (*transmit)(void *ctx, const float *x, size_t n);
    /* Flush and start receiving through src. */
    int  (*rx_start)(void *ctx);
    void (*wait_ms)(void *ctx, int ms);
    source_t src;
    void *ctx;
} link_t;

typedef struct {
    probe_result_t tx;   /* measured by the remote side on our signal */
    probe_result_t rx;   /* measured locally on the remote signal */
    int have_tx, have_rx;
} session_result_t;

/* Return 0 when the sequence completed (results may still be partial),
 * 1 on failure, -1 on error / interruption. */
int session_initiator(link_t *l, session_result_t *res);
int session_responder(link_t *l, session_result_t *res);

#endif
