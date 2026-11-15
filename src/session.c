/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Probing protocol:
 *   initiator: A (300 ms)
 *   responder: [t_w] G L G           initiator measures
 *   initiator: [t_w] G L G           responder measures
 *   responder: [t_w] results (codec2 DATAC1 burst)
 *   initiator: [t_w] results (codec2 DATAC1 burst)
 */
#include <stdlib.h>
#include <string.h>

#include "modem.h"
#include "report.h"
#include "session.h"
#include "signal.h"

static int tx_tone(link_t *l, double f, int ms)
{
    sigbuf_t b = {0};
    int rc = sig_tone(&b, l->fs, f, ms, db2lin(TONE_DBFS));
    if (!rc)
        rc = l->transmit(l->ctx, b.x, b.n);
    sig_free(&b);
    return rc;
}

static int tx_glg(link_t *l)
{
    sigbuf_t b = {0};
    int rc = sig_glg(&b, l->fs);
    if (!rc)
        rc = l->transmit(l->ctx, b.x, b.n);
    sig_free(&b);
    return rc;
}

static int tx_results(link_t *l, const probe_result_t *r)
{
    uint8_t msg[REPORT_LEN];
    float *y = NULL;
    size_t len = report_pack(r, msg);
    size_t n = modem_build(l->fs, msg, len, &y);
    if (!n)
        return -1;
    dbg("tx: results burst %.2f s\n", (double)n / l->fs);
    int rc = l->transmit(l->ctx, y, n);
    free(y);
    return rc;
}

/* 0: got results, 1: none, -1: error. */
static int rx_results(link_t *l, probe_result_t *tx)
{
    uint8_t msg[1024];
    size_t len = 0;
    int heard = 0;
    if (l->rx_start(l->ctx))
        return -1;
    int r = rx_modem(&l->src, l->timeout_ms, msg, sizeof(msg), &len, &heard);
    if (r < 0)
        return -1;
    if (r == 1) {
        logmsg("no results received from the remote side (timeout)\n");
        return 1;
    }
    if (r == 2 || report_unpack(msg, len, tx)) {
        logmsg("results burst received but could not be decoded (modem held sync for "
               "%d ms of a %d ms burst)\n", heard, modem_burst_ms());
        return 1;
    }
    if (!tx->valid)
        logmsg("the remote side could not analyse our probe signal\n");
    return 0;
}

/* 0: probe received (rx->valid tells whether it could be analysed),
 * 1: nothing received, -1: error. */
static int rx_probe_phase(link_t *l, probe_result_t *rx, int timeout_ms)
{
    if (l->rx_start(l->ctx))
        return -1;
    int r = rx_probe(&l->src, timeout_ms, rx);
    if (r < 0)
        return -1;
    if (r == 1) {
        logmsg("no probe signal received (timeout)\n");
        return 1;
    }
    if (r == 2)
        logmsg("probe signal received but could not be analysed\n");
    else
        logmsg("probe signal analysed: level %.1f dBFS RMS (sent %.1f), clock offset %+.1f ppm\n",
               rx->rms_dbfs, L_RMS_DBFS, rx->ppm);
    logmsg("guard tones heard: leading %d ms, middle %d ms, trailing %d ms (sent %d ms each)\n",
           rx->lead_g_ms, rx->mid_g_ms, rx->trail_g_ms, DUR_G_MS);
    if (rx->trail_g_ms < DUR_G_MS - 40)
        logmsg("hint: the end of the remote transmission is cut by about %d ms:\n"
               "      raise --txtail on the remote side\n", DUR_G_MS - rx->trail_g_ms);
    if (rx->valid && (rx->jumps || rx->min_level_db < -3))
        logmsg("warning: the received audio is not continuous: lowest period level %+.1f dB,\n"
               "         %d timing jump(s), largest %.1f us (dropouts or sample slips in the\n"
               "         remote or local audio path)\n", rx->min_level_db, rx->jumps,
               rx->max_jump_us);
    if (rx->lead_g_ms < DUR_G_MS - 40)
        logmsg("hint: the start of the remote transmission is cut by about %d ms:\n"
               "      raise --txdelay on the remote side\n", DUR_G_MS - rx->lead_g_ms);
    return 0;
}

int session_initiator(link_t *l, session_result_t *res)
{
    memset(res, 0, sizeof(*res));
    /* Re-send tone A when the responder has not started its sequence
     * (leading guard tone) within A_WAIT_MS; give up after A_TRIES. */
    enum { A_TRIES = 5, A_WAIT_MS = 3000 };
    int r = 1;
    for (int t = 1; t <= A_TRIES && r == 1; t++) {
        logmsg("initiator: sending tone A (attempt %d of %d)\n", t, A_TRIES);
        if (tx_tone(l, TONE_A_HZ, DUR_A_MS))
            return -1;
        logmsg("initiator: waiting for the responder's probe\n");
        r = rx_probe_phase(l, &res->rx, A_WAIT_MS);
        if (r < 0)
            return -1;
    }
    if (r) {
        logmsg("initiator: no answer to tone A after %d attempts, giving up\n", A_TRIES);
        return 1;
    }
    res->have_rx = res->rx.valid;
    l->wait_ms(l->ctx, l->tw_ms);
    logmsg("initiator: sending probe\n");
    if (tx_glg(l))
        return -1;
    logmsg("initiator: waiting for the responder's results\n");
    r = rx_results(l, &res->tx);
    if (r < 0)
        return -1;
    res->have_tx = r == 0;
    l->wait_ms(l->ctx, l->tw_ms);
    logmsg("initiator: sending results\n");
    if (tx_results(l, &res->rx))
        return -1;
    return 0;
}

int session_responder(link_t *l, session_result_t *res)
{
    memset(res, 0, sizeof(*res));
    logmsg("responder: waiting for tone A\n");
    if (l->rx_start(l->ctx))
        return -1;
    if (wait_tone(&l->src, TONE_A_HZ, 200, 0))
        return -1;
    if (wait_tone_end(&l->src, TONE_A_HZ, 1000) < 0)
        return -1;
    l->wait_ms(l->ctx, l->tw_ms);
    logmsg("responder: sending probe\n");
    if (tx_glg(l))
        return -1;
    logmsg("responder: waiting for the initiator's probe\n");
    int r = rx_probe_phase(l, &res->rx, l->timeout_ms);
    if (r)
        return r;
    res->have_rx = res->rx.valid;
    l->wait_ms(l->ctx, l->tw_ms);
    logmsg("responder: sending results\n");
    if (tx_results(l, &res->rx))
        return -1;
    logmsg("responder: waiting for the initiator's results\n");
    r = rx_results(l, &res->tx);
    if (r < 0)
        return -1;
    res->have_tx = r == 0;
    return 0;
}
