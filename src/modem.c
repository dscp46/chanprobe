/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Results data link: codec2 FreeDV DATAC1 raw data burst.
 * Works with codec2 >= 1.0 (Debian bookworm: 1.0.5).
 */
#include <codec2/freedv_api.h>
#include <math.h>
#include <samplerate.h>
#include <stdlib.h>
#include <string.h>

#include "modem.h"

#define MODEM_FS     8000
#define MODEM_MODE   FREEDV_MODE_DATAC1
#define TX_RMS_DBFS  (-10.0)   /* same average level as A, G and the tune tone */

static struct freedv *open_modem(void)
{
    struct freedv *f = freedv_open(MODEM_MODE);
    if (!f)
        logmsg("codec2: cannot open the DATAC1 modem\n");
    return f;
}

static int frame_bytes(struct freedv *f) { return freedv_get_bits_per_modem_frame(f) / 8; }

size_t modem_payload_max(void)
{
    static size_t n;
    if (!n) {
        struct freedv *f = open_modem();
        if (!f)
            return 0;
        n = (size_t)frame_bytes(f) - 2;   /* last two bytes: CRC-16 */
        freedv_close(f);
    }
    return n;
}

int modem_burst_ms(void)
{
    static int ms;
    if (!ms) {
        struct freedv *f = open_modem();
        if (!f)
            return 5000;
        long n = freedv_get_n_tx_preamble_modem_samples(f) + freedv_get_n_tx_modem_samples(f) +
                 freedv_get_n_tx_postamble_modem_samples(f);
        ms = (int)(n * 1000 / MODEM_FS);
        freedv_close(f);
    }
    return ms;
}

static int crc_ok(const uint8_t *b, int nb)
{
    unsigned short c = freedv_gen_crc16((unsigned char *)b, nb - 2);
    return b[nb - 2] == (c >> 8) && b[nb - 1] == (c & 0xFF);
}

size_t modem_build(int fs, const uint8_t *data, size_t len, float **out)
{
    struct freedv *f = open_modem();
    if (!f)
        return 0;
    /* No codec2 clipper / tx band-pass: codec2 1.0.x has no tx filter for
     * DATAC1 (enabling either leaves it uninitialised and crashes). The
     * peak level is limited below instead. */

    int nb = frame_bytes(f);
    size_t n8 = 0, ny = 0;
    uint8_t *fr = calloc(nb, 1);
    int npre = freedv_get_n_tx_preamble_modem_samples(f);
    int nmod = freedv_get_n_tx_modem_samples(f);
    int npost = freedv_get_n_tx_postamble_modem_samples(f);
    short *s = calloc((size_t)npre + nmod + npost, sizeof(short));
    float *x8 = NULL, *y = NULL;
    if (!fr || !s || len > (size_t)nb - 2)
        goto out;

    memcpy(fr, data, len);
    unsigned short c = freedv_gen_crc16(fr, nb - 2);
    fr[nb - 2] = c >> 8;
    fr[nb - 1] = c & 0xFF;

    n8 = (size_t)freedv_rawdatapreambletx(f, s);
    freedv_rawdatatx(f, s + n8, fr);
    n8 += nmod;
    n8 += (size_t)freedv_rawdatapostambletx(f, s + n8);

    x8 = malloc(sizeof(float) * n8);
    double ratio = (double)fs / MODEM_FS;
    size_t cap = (size_t)ceil(n8 * ratio) + 256;
    y = malloc(sizeof(float) * cap);
    if (!x8 || !y)
        goto out;
    for (size_t i = 0; i < n8; i++)
        x8[i] = s[i] / 32768.0f;
    SRC_DATA d = {.data_in = x8, .data_out = y, .input_frames = (long)n8,
                  .output_frames = (long)cap, .end_of_input = 1, .src_ratio = ratio};
    int err = src_simple(&d, SRC_SINC_MEDIUM_QUALITY, 1);
    if (err) {
        logmsg("samplerate: %s\n", src_strerror(err));
        goto out;
    }
    ny = (size_t)d.output_frames_gen;

    /* Level: -10 dBFS RMS (full-scale sine = 0 dBFS), peaks capped. */
    double ms = 0, pk = 0;
    for (size_t i = 0; i < ny; i++) {
        ms += (double)y[i] * y[i];
        pk = fmax(pk, fabs(y[i]));
    }
    double rms = sqrt(2 * ms / ny);
    double g = db2lin(TX_RMS_DBFS) / rms;
    if (pk * g > db2lin(MODEM_PEAK_DBFS))
        g = db2lin(MODEM_PEAK_DBFS) / pk;
    for (size_t i = 0; i < ny; i++)
        y[i] *= (float)g;
    size_t nf = (size_t)(FADE_MS * fs / 1000);
    for (size_t i = 0; i < nf && 2 * i < ny; i++) {
        float w = (float)(0.5 - 0.5 * cos(M_PI * i / nf));
        y[i] *= w;
        y[ny - 1 - i] *= w;
    }
    dbg("modem: burst %zu bytes, %.2f s, %.1f dBFS RMS, peak %.1f dBFS\n", len,
        (double)ny / fs, 20 * log10(rms * g), 20 * log10(pk * g));
    *out = y;
    y = NULL;
out:
    if (!ny || y) {
        free(y);
        ny = 0;
    }
    free(x8);
    free(s);
    free(fr);
    freedv_close(f);
    return ny;
}

int rx_modem(source_t *s, int timeout_ms, uint8_t *out, size_t maxlen, size_t *outlen,
             int *heard_ms)
{
    const int fs = s->fs, B = fs / 100;
    const int burst = modem_burst_ms();
    struct freedv *f = open_modem();
    if (!f)
        return -1;
    freedv_set_frames_per_burst(f, 1);

    int nb = frame_bytes(f), nmax = freedv_get_n_max_modem_samples(f), err = 0;
    size_t fcap = (size_t)nmax + 2 * B;
    uint8_t *bytes = malloc(nb);
    short *dem = malloc(sizeof(short) * nmax);
    float *in = malloc(sizeof(float) * B), *fifo = malloc(sizeof(float) * fcap);
    SRC_STATE *src = src_new(SRC_SINC_FASTEST, 1, &err);
    int ret = -1, done = 0, any_sync = 0;
    long ms = 0, deadline = timeout_ms, sync_ms = -1, done_ms = 0, held = 0;
    size_t nf = 0;

    if (heard_ms)
        *heard_ms = 0;
    if (!bytes || !dem || !in || !fifo || !src) {
        if (!src)
            logmsg("samplerate: %s\n", src_strerror(err));
        goto out;
    }
    int nin = freedv_nin(f);
    for (;;) {
        if (g_stop || s->read(s->ctx, in, B) != B)
            break;
        ms += 10;
        SRC_DATA d = {.data_in = in, .data_out = fifo + nf, .input_frames = B,
                      .output_frames = (long)(fcap - nf), .end_of_input = 0,
                      .src_ratio = (double)MODEM_FS / fs};
        if ((err = src_process(src, &d))) {
            logmsg("samplerate: %s\n", src_strerror(err));
            break;
        }
        nf += (size_t)d.output_frames_gen;

        while (nf >= (size_t)nin) {
            for (int i = 0; i < nin; i++) {
                float v = fifo[i] * 32767.0f;
                dem[i] = (short)(v > 32767 ? 32767 : v < -32767 ? -32767 : v);
            }
            int got = freedv_rawdatarx(f, bytes, dem);
            memmove(fifo, fifo + nin, sizeof(float) * (nf - nin));
            nf -= (size_t)nin;
            nin = freedv_nin(f);

            int st = freedv_get_rx_status(f);
            if (st & FREEDV_RX_SYNC) {
                if (sync_ms < 0) {
                    sync_ms = ms;
                    any_sync = 1;
                    dbg("modem: sync\n");
                    /* Give a burst that starts now the time to complete. */
                    if (ms + burst + 1500 > deadline)
                        deadline = ms + burst + 1500;
                }
            } else if (sync_ms >= 0) {
                held = ms - sync_ms > held ? ms - sync_ms : held;
                sync_ms = -1;
            }
            if (got == nb && !done && crc_ok(bytes, nb)) {
                done = 1;
                done_ms = ms;
                if (sync_ms >= 0)
                    held = ms - sync_ms;
                size_t n = (size_t)nb - 2 < maxlen ? (size_t)nb - 2 : maxlen;
                memcpy(out, bytes, n);
                *outlen = n;
                dbg("modem: frame received\n");
            }
        }
        /* Let the postamble pass, so that t_w counts from the end of the burst. */
        if (done && ms - done_ms >= 120) {
            ret = 0;
            break;
        }
        if (!done && ms >= deadline) {
            if (sync_ms >= 0)
                held = ms - sync_ms > held ? ms - sync_ms : held;
            ret = any_sync ? 2 : 1;
            break;
        }
    }
    if (heard_ms)
        *heard_ms = (int)held;
out:
    if (src)
        src_delete(src);
    free(bytes);
    free(dem);
    free(in);
    free(fifo);
    freedv_close(f);
    return ret;
}
