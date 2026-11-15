/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_MODEM_H
#define CHANPROBE_MODEM_H

#include <stddef.h>
#include <stdint.h>
#include "common.h"

/* Results data link: codec2 FreeDV DATAC1 burst (OFDM, 27 carriers around
 * 1.5 kHz, about 1.7 kHz wide, LDPC FEC, CRC-16), 8 kHz modem rate,
 * resampled to/from the interface rate with libsamplerate. */

#define MODEM_PEAK_DBFS (-3.0)

/* Largest payload one burst carries. */
size_t modem_payload_max(void);

/* Build a burst carrying len <= modem_payload_max() bytes, at rate fs.
 * Returns the number of samples (*out is malloc'd), 0 on error. */
size_t modem_build(int fs, const uint8_t *data, size_t len, float **out);

/* Nominal on-air duration of a burst, ms. */
int modem_burst_ms(void);

/* Receive one burst. Returns 0 when a frame was decoded (out / outlen;
 * outlen = modem_payload_max(), zero padded), 1 when nothing was heard
 * within timeout_ms, 2 when the modem synchronised but no valid frame was
 * decoded, <0 on error / stop. *heard_ms: time the demodulator held sync. */
int rx_modem(source_t *s, int timeout_ms, uint8_t *out, size_t maxlen, size_t *outlen,
             int *heard_ms);

#endif
