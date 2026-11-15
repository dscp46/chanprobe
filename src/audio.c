/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * ALSA backend: S16_LE, interleaved, fixed rate (44100 or 48000 Hz).
 */
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "audio.h"

static int setup(snd_pcm_t *pcm, int fs, int ch, unsigned buffer_us, const char *what)
{
    snd_pcm_hw_params_t *hw;
    snd_pcm_sw_params_t *sw;
    int err;
    unsigned period_us = 20000;
    snd_pcm_uframes_t bufsize, persize;

    snd_pcm_hw_params_alloca(&hw);
    if ((err = snd_pcm_hw_params_any(pcm, hw)) < 0 ||
        (err = snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0 ||
        (err = snd_pcm_hw_params_set_format(pcm, hw, SND_PCM_FORMAT_S16_LE)) < 0 ||
        (err = snd_pcm_hw_params_set_channels(pcm, hw, ch)) < 0) {
        logmsg("%s: cannot set S16_LE / %d channel(s): %s\n", what, ch, snd_strerror(err));
        return -1;
    }
    if ((err = snd_pcm_hw_params_set_rate(pcm, hw, fs, 0)) < 0) {
        logmsg("%s: sample rate %d Hz not supported: %s\n", what, fs, snd_strerror(err));
        return -1;
    }
    snd_pcm_hw_params_set_buffer_time_near(pcm, hw, &buffer_us, NULL);
    snd_pcm_hw_params_set_period_time_near(pcm, hw, &period_us, NULL);
    if ((err = snd_pcm_hw_params(pcm, hw)) < 0) {
        logmsg("%s: cannot apply hw params: %s\n", what, snd_strerror(err));
        return -1;
    }
    snd_pcm_hw_params_get_buffer_size(hw, &bufsize);
    snd_pcm_hw_params_get_period_size(hw, &persize, NULL);
    dbg("%s: %d Hz, %d ch, buffer %lu, period %lu frames\n", what, fs, ch,
        (unsigned long)bufsize, (unsigned long)persize);

    snd_pcm_sw_params_alloca(&sw);
    snd_pcm_sw_params_current(pcm, sw);
    /* Playback starts once a period is queued; capture is started explicitly. */
    snd_pcm_sw_params_set_start_threshold(pcm, sw, persize);
    snd_pcm_sw_params_set_avail_min(pcm, sw, persize);
    if ((err = snd_pcm_sw_params(pcm, sw)) < 0) {
        logmsg("%s: cannot apply sw params: %s\n", what, snd_strerror(err));
        return -1;
    }
    return 0;
}

int audio_open(audio_t *a, const char *dev, int fs, int channels, int sel)
{
    int err;
    memset(a, 0, sizeof(*a));
    a->fs = fs;
    a->ch = channels;
    a->sel = sel;
    if (sel < 0 || sel >= channels) {
        logmsg("audio: channel %d out of range (0..%d)\n", sel, channels - 1);
        return -1;
    }
    if ((err = snd_pcm_open(&a->pb, dev, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        logmsg("audio: cannot open playback device '%s': %s\n", dev, snd_strerror(err));
        return -1;
    }
    if ((err = snd_pcm_open(&a->cap, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        logmsg("audio: cannot open capture device '%s': %s\n", dev, snd_strerror(err));
        audio_close(a);
        return -1;
    }
    if (setup(a->pb, fs, channels, 200000, "playback") ||
        setup(a->cap, fs, channels, 500000, "capture")) {
        audio_close(a);
        return -1;
    }
    a->ibuf_frames = (size_t)fs / 10;
    a->ibuf = malloc(sizeof(int16_t) * a->ibuf_frames * channels);
    if (!a->ibuf) {
        audio_close(a);
        return -1;
    }
    return 0;
}

void audio_close(audio_t *a)
{
    if (a->pb) {
        snd_pcm_drop(a->pb);
        snd_pcm_close(a->pb);
    }
    if (a->cap) {
        snd_pcm_drop(a->cap);
        snd_pcm_close(a->cap);
    }
    free(a->ibuf);
    memset(a, 0, sizeof(*a));
}

int audio_rx_start(audio_t *a)
{
    int err;
    snd_pcm_drop(a->cap);
    if ((err = snd_pcm_prepare(a->cap)) < 0 || (err = snd_pcm_start(a->cap)) < 0) {
        logmsg("audio: cannot start capture: %s\n", snd_strerror(err));
        return -1;
    }
    a->rx_running = 1;
    return 0;
}

void audio_rx_stop(audio_t *a)
{
    snd_pcm_drop(a->cap);
    a->rx_running = 0;
}

int audio_read(void *ctx, float *buf, int n)
{
    audio_t *a = ctx;
    int got = 0;
    while (got < n) {
        if (g_stop)
            return -1;
        size_t want = (size_t)(n - got);
        if (want > a->ibuf_frames)
            want = a->ibuf_frames;
        snd_pcm_sframes_t r = snd_pcm_readi(a->cap, a->ibuf, want);
        if (r == -EINTR)
            continue;
        if (r == -EPIPE || r == -ESTRPIPE) {
            a->xruns++;
            dbg("audio: capture overrun\n");
            if (snd_pcm_recover(a->cap, (int)r, 1) < 0 || snd_pcm_start(a->cap) < 0)
                return -1;
            continue;
        }
        if (r < 0) {
            logmsg("audio: read error: %s\n", snd_strerror((int)r));
            return -1;
        }
        for (snd_pcm_sframes_t i = 0; i < r; i++)
            buf[got + i] = a->ibuf[i * a->ch + a->sel] / 32768.0f;
        got += (int)r;
    }
    return got;
}

void audio_source(audio_t *a, source_t *s)
{
    s->read = audio_read;
    s->ctx = a;
    s->fs = a->fs;
    s->xruns = &a->xruns;
}

int audio_tx_start(audio_t *a)
{
    int err;
    snd_pcm_drop(a->pb);
    if ((err = snd_pcm_prepare(a->pb)) < 0) {
        logmsg("audio: cannot prepare playback: %s\n", snd_strerror(err));
        return -1;
    }
    return 0;
}

static int write_frames(audio_t *a, const float *x, size_t n)
{
    size_t done = 0;
    while (done < n) {
        if (g_stop)
            return -1;
        size_t k = n - done;
        if (k > a->ibuf_frames)
            k = a->ibuf_frames;
        for (size_t i = 0; i < k; i++) {
            float v = x ? x[done + i] : 0.0f;
            long s = lrintf(v * 32767.0f);
            if (s > 32767) s = 32767;
            if (s < -32768) s = -32768;
            for (int c = 0; c < a->ch; c++)
                a->ibuf[i * a->ch + c] = (int16_t)s;
        }
        size_t off = 0;
        while (off < k) {
            snd_pcm_sframes_t r = snd_pcm_writei(a->pb, a->ibuf + off * a->ch, k - off);
            if (r == -EINTR) {
                if (g_stop)
                    return -1;
                continue;
            }
            if (r == -EPIPE || r == -ESTRPIPE) {
                logmsg("warning: playback underrun\n");
                if (snd_pcm_recover(a->pb, (int)r, 1) < 0)
                    return -1;
                continue;
            }
            if (r < 0) {
                logmsg("audio: write error: %s\n", snd_strerror((int)r));
                return -1;
            }
            off += (size_t)r;
        }
        done += k;
    }
    return 0;
}

int audio_write(audio_t *a, const float *x, size_t n) { return write_frames(a, x, n); }
int audio_write_silence(audio_t *a, size_t n) { return write_frames(a, NULL, n); }

int audio_drain(audio_t *a)
{
    int err = snd_pcm_drain(a->pb);
    if (err < 0 && err != -EINTR) {
        logmsg("audio: drain failed: %s\n", snd_strerror(err));
        return -1;
    }
    return g_stop ? -1 : 0;
}

long audio_delay(audio_t *a)
{
    snd_pcm_sframes_t d;
    if (snd_pcm_delay(a->pb, &d) < 0)
        return -1;
    return (long)d;
}

void audio_tx_abort(audio_t *a)
{
    snd_pcm_drop(a->pb);
}
