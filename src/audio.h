/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_AUDIO_H
#define CHANPROBE_AUDIO_H

#include <alsa/asoundlib.h>
#include <stddef.h>
#include "common.h"

typedef struct {
    snd_pcm_t *pb, *cap;
    int fs, ch, sel;
    int16_t *ibuf;
    size_t ibuf_frames;
    unsigned long xruns;
    int rx_running;
} audio_t;

int  audio_open(audio_t *a, const char *dev, int fs, int channels, int sel);
void audio_close(audio_t *a);

/* Capture */
int  audio_rx_start(audio_t *a);   /* flush and (re)start capture */
void audio_rx_stop(audio_t *a);
int  audio_read(void *ctx, float *buf, int n);   /* source_t compatible */
void audio_source(audio_t *a, source_t *s);

/* Playback (mono float in, written to all channels) */
int  audio_tx_start(audio_t *a);
int  audio_write(audio_t *a, const float *x, size_t n);
int  audio_write_silence(audio_t *a, size_t n);
int  audio_drain(audio_t *a);      /* blocks until everything was played */
long audio_delay(audio_t *a);      /* frames queued before the DAC, or -1 */
void audio_tx_abort(audio_t *a);

#endif
