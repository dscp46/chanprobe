/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_REPORT_H
#define CHANPROBE_REPORT_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "probe.h"

#define REPORT_VERSION 5   /* 5: 3600 Hz is a mute carrier */
#define REPORT_HDR 18
#define REPORT_REC 7
#define REPORT_LEN (REPORT_HDR + NBINS * REPORT_REC)   /* 508 */

/* Serialise a result into the on-air message. Returns length. */
size_t report_pack(const probe_result_t *r, uint8_t *buf);
/* Parse an on-air message. Returns 0 on success. */
int report_unpack(const uint8_t *buf, size_t len, probe_result_t *r);

/* tx: measured by the remote side on our signal; rx: measured locally.
 * Either may be NULL (not available). */
void report_print(FILE *f, int local_fs, const probe_result_t *tx,
                  const probe_result_t *rx);
int  report_csv(const char *path, const probe_result_t *tx, const probe_result_t *rx);

#endif
