/* chanprobe - bidirectional audio channel prober for half duplex radio links
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <errno.h>
#include <getopt.h>
#include <locale.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "audio.h"
#include "common.h"
#include "probe.h"
#include "ptt.h"
#include "report.h"
#include "session.h"
#include "signal.h"

volatile sig_atomic_t g_stop;
int g_verbose;

void logmsg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

void dbg(const char *fmt, ...)
{
    if (!g_verbose)
        return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static void msleep(int ms)
{
    struct timespec ts = {ms / 1000, (long)(ms % 1000) * 1000000L};
    while (nanosleep(&ts, &ts) && errno == EINTR && !g_stop)
        ;
}

typedef struct {
    audio_t a;
    source_t src;
    int txdelay, txtail, tw, timeout_ms, loop;
    double duration;
    const char *csv;
} ctx_t;

/* Key, play txdelay of silence + x, drain, wait txtail, unkey. */
static int transmit(ctx_t *c, const float *x, size_t n)
{
    int rc;
    audio_rx_stop(&c->a);
    if (audio_tx_start(&c->a))
        return -1;
    double t0 = now_ms();
    if (ptt_set(1))
        return -1;
    double t1 = now_ms();
    rc = audio_write_silence(&c->a, (size_t)c->txdelay * c->a.fs / 1000);
    if (!rc)
        rc = audio_write(&c->a, x, n);
    double t2 = now_ms(), t3 = t2;
    if (!rc) {
        /* Frames still queued when the last one was written. Some plugins
         * (PulseAudio / PipeWire) return from drain before they are played. */
        long q = audio_delay(&c->a);
        rc = audio_drain(&c->a);
        t3 = now_ms();
        double qms = q > 0 ? q * 1000.0 / c->a.fs : 0;
        if (!rc && t3 - t2 < qms - 5) {
            dbg("tx: drain returned %.0f ms early, waiting\n", qms - (t3 - t2));
            msleep((int)(qms - (t3 - t2)));
            t3 = now_ms();
        }
    }
    if (!rc)
        msleep(c->txtail);
    else
        audio_tx_abort(&c->a);
    double t4 = now_ms();
    ptt_set(0);
    double t5 = now_ms();
    dbg("tx: PTT on took %.0f ms; audio %.0f ms (txdelay %d + %.0f signal); "
        "played out %.0f ms after the last write; txtail %.0f ms; PTT off took %.0f ms\n",
        t1 - t0, (n + (double)c->txdelay * c->a.fs / 1000) * 1000 / c->a.fs, c->txdelay,
        n * 1000.0 / c->a.fs, t3 - t2, t4 - t3, t5 - t4);
    return rc;
}

static int link_transmit(void *ctx, const float *x, size_t n) { return transmit(ctx, x, n); }
static int link_rx_start(void *ctx) { return audio_rx_start(&((ctx_t *)ctx)->a); }
static void link_wait(void *ctx, int ms) { (void)ctx; msleep(ms); }

static void show(ctx_t *c, const session_result_t *r)
{
    const probe_result_t *tx = r->have_tx ? &r->tx : NULL;
    const probe_result_t *rx = r->have_rx ? &r->rx : NULL;
    report_print(stdout, c->a.fs, tx, rx);
    fflush(stdout);
    if (c->csv) {
        if (report_csv(c->csv, tx, rx))
            logmsg("cannot write %s: %s\n", c->csv, strerror(errno));
        else
            logmsg("results written to %s\n", c->csv);
    }
}

static int run_session(ctx_t *c, int initiator)
{
    link_t l = {
        .fs = c->a.fs, .tw_ms = c->tw, .timeout_ms = c->timeout_ms,
        .transmit = link_transmit, .rx_start = link_rx_start, .wait_ms = link_wait,
        .src = c->src, .ctx = c,
    };
    session_result_t r;
    if (initiator) {
        int rc = session_initiator(&l, &r);
        if (rc == 0)
            show(c, &r);
        return rc ? 1 : 0;
    }
    do {
        int rc = session_responder(&l, &r);
        if (rc < 0)
            return g_stop ? 0 : 1;
        if (rc == 0)
            show(c, &r);
    } while (c->loop && !g_stop);
    return 0;
}

static int run_tune(ctx_t *c)
{
    const int fs = c->a.fs, chunk = fs / 10;
    float *b = malloc(sizeof(float) * chunk);
    double amp = db2lin(TONE_DBFS), ph = 0, inc = 2 * M_PI * TUNE_HZ / fs;
    long total = c->duration > 0 ? lround(c->duration * fs) : -1, sent = 0;
    int rc = 0;

    if (!b)
        return 1;
    logmsg("tune: transmitting %.0f Hz at %.0f dBFS%s, Ctrl-C to stop\n", TUNE_HZ,
           TONE_DBFS, total > 0 ? "" : " until interrupted");
    audio_rx_stop(&c->a);
    if (audio_tx_start(&c->a) || ptt_set(1)) {
        free(b);
        return 1;
    }
    rc = audio_write_silence(&c->a, (size_t)c->txdelay * fs / 1000);
    while (!rc && !g_stop && (total < 0 || sent < total)) {
        for (int i = 0; i < chunk; i++) {
            b[i] = (float)(amp * sin(ph));
            ph += inc;
            if (ph > 2 * M_PI)
                ph -= 2 * M_PI;
        }
        rc = audio_write(&c->a, b, chunk);
        sent += chunk;
    }
    if (!rc && !g_stop) {
        audio_drain(&c->a);
        msleep(c->txtail);
    } else {
        audio_tx_abort(&c->a);
    }
    ptt_set(0);
    free(b);
    return (rc && !g_stop) ? 1 : 0;
}

static void usage(FILE *f)
{
    fprintf(f,
"Usage: chanprobe MODE [options]\n"
"Modes:\n"
"  initiator          start a probing session\n"
"  responder          wait for an initiator and answer it\n"
"  tune               transmit a 1 kHz tone at -10 dBFS (level / ALC setup)\n"
"Audio:\n"
"  -D, --device NAME      ALSA device (default: default)\n"
"      --rate HZ          44100 or 48000 (default: 48000)\n"
"      --channels N       channels to open (default: 1)\n"
"      --channel IDX      capture channel used for analysis (default: 0)\n"
"Push to talk (Hamlib):\n"
"  -m, --model ID         Hamlib rig model (rigctl -l); 1 = dummy\n"
"  -r, --rig-file PATH    rig CAT port, e.g. /dev/ttyUSB0\n"
"  -s, --serial-speed BD  CAT serial speed\n"
"  -P, --ptt-type TYPE    RIG, DTR, RTS, CM108, GPIO... (default: RIG)\n"
"  -p, --ptt-file PATH    PTT port when different from the CAT port\n"
"  -C, --set-conf K=V     extra Hamlib configuration (repeatable)\n"
"  Without -m and -P, PTT is not keyed (VOX / bench use).\n"
"Timing (ms):\n"
"      --txdelay MS       key-up to first audio sample (default: 100)\n"
"      --txtail MS        last audio sample to key-down (default: 30)\n"
"      --tw MS            turnaround wait t_w (default: 100)\n"
"  -t, --timeout S        receive timeout in seconds (default: 10)\n"
"Other:\n"
"      --loop             responder: serve sessions until interrupted\n"
"      --csv FILE         also write the results as CSV\n"
"      --duration S       tune: stop after S seconds (default: until Ctrl-C)\n"
"  -v, --verbose          more messages (twice: Hamlib debug)\n"
"  -V, --version          print version\n"
"  -h, --help             this help\n");
}

enum { O_RATE = 256, O_CHANNELS, O_CHANNEL, O_TXDELAY, O_TXTAIL, O_TW, O_LOOP, O_CSV, O_DUR };

int main(int argc, char **argv)
{
    static const struct option lo[] = {
        {"device", 1, 0, 'D'},       {"rate", 1, 0, O_RATE},
        {"channels", 1, 0, O_CHANNELS}, {"channel", 1, 0, O_CHANNEL},
        {"model", 1, 0, 'm'},        {"rig-file", 1, 0, 'r'},
        {"serial-speed", 1, 0, 's'}, {"ptt-type", 1, 0, 'P'},
        {"ptt-file", 1, 0, 'p'},     {"set-conf", 1, 0, 'C'},
        {"txdelay", 1, 0, O_TXDELAY}, {"txtail", 1, 0, O_TXTAIL},
        {"tw", 1, 0, O_TW},          {"timeout", 1, 0, 't'},
        {"loop", 0, 0, O_LOOP},      {"csv", 1, 0, O_CSV},
        {"duration", 1, 0, O_DUR},   {"verbose", 0, 0, 'v'},
        {"version", 0, 0, 'V'},      {"help", 0, 0, 'h'},
        {0, 0, 0, 0}};
    ctx_t c = {.txdelay = 100, .txtail = 30, .tw = 100, .timeout_ms = 10000};
    ptt_cfg_t pc = {0};
    const char *dev = "default";
    int fs = 48000, channels = 1, sel = 0, o;

    setlocale(LC_CTYPE, "");   /* UTF-8 block characters in the SNR histogram */
    while ((o = getopt_long(argc, argv, "D:m:r:s:P:p:C:t:vVh", lo, NULL)) != -1) {
        switch (o) {
        case 'D': dev = optarg; break;
        case O_RATE: fs = atoi(optarg); break;
        case O_CHANNELS: channels = atoi(optarg); break;
        case O_CHANNEL: sel = atoi(optarg); break;
        case 'm': pc.model = atoi(optarg); break;
        case 'r': pc.rig_port = optarg; break;
        case 's': pc.speed = atoi(optarg); break;
        case 'P': pc.ptt_type = optarg; break;
        case 'p': pc.ptt_port = optarg; break;
        case 'C':
            if (pc.nconf < (int)(sizeof(pc.conf) / sizeof(pc.conf[0])))
                pc.conf[pc.nconf++] = optarg;
            break;
        case O_TXDELAY: c.txdelay = atoi(optarg); break;
        case O_TXTAIL: c.txtail = atoi(optarg); break;
        case O_TW: c.tw = atoi(optarg); break;
        case 't': c.timeout_ms = (int)(atof(optarg) * 1000); break;
        case O_LOOP: c.loop = 1; break;
        case O_CSV: c.csv = optarg; break;
        case O_DUR: c.duration = atof(optarg); break;
        case 'v': g_verbose++; break;
        case 'V':
            printf("chanprobe " CP_VERSION "\n"
                   "Copyright (C) 2026 C\u00e9lestine GRAMAIZE <celestine+dev@gramaize.eu>\n"
                   "License GPLv3+: GNU GPL version 3 or later <https://gnu.org/licenses/gpl.html>\n"
                   "This is free software: you are free to change and redistribute it.\n"
                   "There is NO WARRANTY, to the extent permitted by law.\n");
            return 0;
        case 'h': usage(stdout); return 0;
        default: usage(stderr); return 2;
        }
    }
    if (optind != argc - 1) {
        usage(stderr);
        return 2;
    }
    const char *mode = argv[optind];
    if (strcmp(mode, "initiator") && strcmp(mode, "responder") && strcmp(mode, "tune")) {
        logmsg("unknown mode '%s'\n", mode);
        return 2;
    }
    if (fs != 44100 && fs != 48000) {
        logmsg("--rate must be 44100 or 48000\n");
        return 2;
    }
    if (channels < 1 || c.txdelay < 0 || c.txtail < 0 || c.tw < 0 || c.timeout_ms <= 0) {
        logmsg("invalid argument\n");
        return 2;
    }
    if (!pc.model && pc.ptt_type)
        pc.model = 1;   /* dummy rig: PTT through DTR / RTS / CM108 ... only */

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;   /* no SA_RESTART: interrupt blocking I/O */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    if (ptt_open(&pc))
        return 1;
    if (!pc.model)
        logmsg("note: no PTT control configured (VOX / bench mode)\n");
    if (audio_open(&c.a, dev, fs, channels, sel)) {
        ptt_close();
        return 1;
    }
    audio_source(&c.a, &c.src);

    int rc;
    if (!strcmp(mode, "initiator"))
        rc = run_session(&c, 1);
    else if (!strcmp(mode, "responder"))
        rc = run_session(&c, 0);
    else
        rc = run_tune(&c);

    audio_close(&c.a);
    ptt_close();     /* always unkeys */
    if (g_stop && rc)
        logmsg("interrupted\n");
    return rc;
}
