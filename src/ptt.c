/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Push-to-talk through Hamlib (>= 4.5).
 */
#include <hamlib/rig.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "ptt.h"

static RIG *rig;
static int keyed;

/* Hamlib 4.5's rigerror() returns the recent debug trace followed by the
 * error text: keep only the last non-empty line. */
static const char *rerr(int code)
{
    static char buf[128];
    const char *s = rigerror(code);
    if (!s)
        return "unknown error";
    const char *end = s + strlen(s);
    while (end > s && (end[-1] == '\n' || end[-1] == '\r'))
        end--;
    const char *beg = end;
    while (beg > s && beg[-1] != '\n')
        beg--;
    snprintf(buf, sizeof(buf), "%.*s", (int)(end - beg), beg);
    return buf;
}

static int conf(const char *key, const char *val)
{
    int r = rig_set_conf(rig, rig_token_lookup(rig, key), val);
    if (r != RIG_OK)
        logmsg("hamlib: cannot set %s=%s: %s\n", key, val, rerr(r));
    return r == RIG_OK ? 0 : -1;
}

int ptt_open(const ptt_cfg_t *c)
{
    if (!c->model)
        return 0;
    rig_set_debug(g_verbose > 1 ? RIG_DEBUG_VERBOSE : RIG_DEBUG_NONE);
    rig = rig_init((rig_model_t)c->model);
    if (!rig) {
        logmsg("hamlib: unknown rig model %d\n", c->model);
        return -1;
    }
    if (c->rig_port && conf("rig_pathname", c->rig_port))
        goto fail;
    if (c->speed > 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", c->speed);
        if (conf("serial_speed", buf))
            goto fail;
    }
    /* The dummy rig has no PTT method by default. */
    const char *ptype = c->ptt_type ? c->ptt_type : (c->model == 1 ? "RIG" : NULL);
    if (ptype && conf("ptt_type", ptype))
        goto fail;
    if (c->ptt_port && conf("ptt_pathname", c->ptt_port))
        goto fail;
    for (int i = 0; i < c->nconf; i++) {
        char *kv = strdup(c->conf[i]), *eq = strchr(kv, '=');
        if (!eq) {
            logmsg("hamlib: bad -C argument '%s' (expected key=value)\n", c->conf[i]);
            free(kv);
            goto fail;
        }
        *eq = 0;
        int r = conf(kv, eq + 1);
        free(kv);
        if (r)
            goto fail;
    }
    int r = rig_open(rig);
    if (r != RIG_OK) {
        logmsg("hamlib: cannot open rig: %s\n", rerr(r));
        goto fail;
    }
    dbg("hamlib: rig model %d opened\n", c->model);
    return 0;
fail:
    rig_cleanup(rig);
    rig = NULL;
    return -1;
}

int ptt_set(int on)
{
    if (!rig)
        return 0;
    int r = rig_set_ptt(rig, RIG_VFO_CURR, on ? RIG_PTT_ON : RIG_PTT_OFF);
    if (r != RIG_OK) {
        logmsg("hamlib: cannot set PTT %s: %s\n", on ? "on" : "off", rerr(r));
        return -1;
    }
    keyed = on;
    dbg("ptt: %s\n", on ? "ON" : "off");
    return 0;
}

int ptt_is_on(void) { return keyed; }

void ptt_close(void)
{
    if (!rig)
        return;
    if (keyed)
        ptt_set(0);
    rig_close(rig);
    rig_cleanup(rig);
    rig = NULL;
}
