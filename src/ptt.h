/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 */
#ifndef CHANPROBE_PTT_H
#define CHANPROBE_PTT_H

typedef struct {
    int model;               /* 0: PTT disabled (VOX / bench) */
    const char *rig_port;    /* -r */
    int speed;               /* -s, 0 = backend default */
    const char *ptt_type;    /* -P: RIG, DTR, RTS, CM108, GPIO, ... */
    const char *ptt_port;    /* -p */
    const char *conf[32];    /* -C key=value */
    int nconf;
} ptt_cfg_t;

int  ptt_open(const ptt_cfg_t *cfg);
int  ptt_set(int on);
int  ptt_is_on(void);
void ptt_close(void);     /* unkeys first if needed */

#endif
