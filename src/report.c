/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>
 *
 * Results message (version 4, little endian, 508 bytes = one DATAC1 frame):
 *   0  'C' 'P'  version  flags (bit0 valid, bit1 L2 analysed)
 *   4  uint16   sample rate of the measuring side
 *   6  int16    clock offset, ppm * 100
 *   8  int16    received RMS level of L1, dBFS * 100
 *  10  int16    received RMS level of L2, dBFS * 100
 *  12  uint8    leading guard tone heard, 10 ms
 *  13  uint8    trailing guard tone heard, 10 ms
 *  14  int8     lowest period level re average, dB * 2
 *  15  uint8    number of timing jumps
 *  16  uint16   largest timing step, us * 10
 *  18  70 x 7 bytes (150..10500 Hz):
 *      tone: int16 amp*100, int16 phase*100, int8 snr*2, int8 comp*20,
 *            int8 AM/PM*2
 *      mute: int16 noise(L1)*100, int16 dist(L1)*100, int8 dist(L2)*2,
 *            int8 noise(L2)*2, uint8 flags (bit0 dist L1 ok, bit1 dist L2 ok)
 */
#include <math.h>
#include <string.h>
#include <langinfo.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "common.h"
#include "report.h"

static void put16(uint8_t *p, int v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static int get16(const uint8_t *p) { return (int16_t)(p[0] | p[1] << 8); }

static int qs(double v, double scale, int lim)
{
    double q = round(v * scale);
    return (int)(q > lim ? lim : q < -lim ? -lim : q);
}
#define Q16(v, s) qs(v, s, 32767)
#define Q8(v, s)  ((uint8_t)(int8_t)qs(v, s, 127))
#define G8(p)     ((int8_t)(p))

size_t report_pack(const probe_result_t *r, uint8_t *buf)
{
    memset(buf, 0, REPORT_LEN);
    buf[0] = 'C';
    buf[1] = 'P';
    buf[2] = REPORT_VERSION;
    buf[3] = (r->valid ? 1 : 0) | (r->has_l2 ? 2 : 0);
    put16(buf + 4, r->fs);
    put16(buf + 6, Q16(r->ppm, 100));
    put16(buf + 8, Q16(r->rms_dbfs, 100));
    put16(buf + 10, Q16(r->rms2_dbfs, 100));
    buf[12] = (uint8_t)(r->lead_g_ms / 10 > 255 ? 255 : r->lead_g_ms / 10);
    buf[13] = (uint8_t)(r->trail_g_ms / 10 > 255 ? 255 : r->trail_g_ms / 10);
    buf[14] = Q8(r->min_level_db, 2);
    buf[15] = (uint8_t)(r->jumps > 255 ? 255 : r->jumps);
    put16(buf + 16, (int)fmin(65535, round(r->max_jump_us * 10)));
    for (int h = 1; h <= NBINS; h++) {
        uint8_t *p = buf + REPORT_HDR + (h - 1) * REPORT_REC;
        if (probe_is_tone(h)) {
            put16(p, Q16(r->amp_db[h], 100));
            put16(p + 2, Q16(r->phase_deg[h], 100));
            p[4] = Q8(r->snr_db[h], 2);
            p[5] = Q8(r->comp_db[h], 20);
            p[6] = Q8(r->ampm_deg[h], 2);
        } else {
            put16(p, Q16(r->noise_dbc[h], 100));
            put16(p + 2, Q16(r->dist_dbc[h], 100));
            p[4] = Q8(r->dist2_dbc[h], 2);
            p[5] = Q8(r->noise2_dbc[h], 2);
            p[6] = (uint8_t)((r->dist_ok[h] ? 1 : 0) | (r->dist2_ok[h] ? 2 : 0));
        }
    }
    return REPORT_LEN;
}

int report_unpack(const uint8_t *buf, size_t len, probe_result_t *r)
{
    memset(r, 0, sizeof(*r));
    if (len < REPORT_LEN || buf[0] != 'C' || buf[1] != 'P' || buf[2] != REPORT_VERSION) {
        if (len >= 3 && buf[0] == 'C' && buf[1] == 'P')
            logmsg("results message version %d, expected %d: update both sides\n",
                   buf[2], REPORT_VERSION);
        return -1;
    }
    r->valid = buf[3] & 1;
    r->has_l2 = (buf[3] >> 1) & 1;
    r->fs = (uint16_t)get16(buf + 4);
    r->ppm = get16(buf + 6) / 100.0;
    r->rms_dbfs = get16(buf + 8) / 100.0;
    r->rms2_dbfs = get16(buf + 10) / 100.0;
    r->lead_g_ms = buf[12] * 10;
    r->trail_g_ms = buf[13] * 10;
    r->min_level_db = G8(buf[14]) / 2.0;
    r->jumps = buf[15];
    r->max_jump_us = (uint16_t)get16(buf + 16) / 10.0;
    for (int h = 1; h <= NBINS; h++) {
        const uint8_t *p = buf + REPORT_HDR + (h - 1) * REPORT_REC;
        if (probe_is_tone(h)) {
            r->amp_db[h] = get16(p) / 100.0;
            r->phase_deg[h] = get16(p + 2) / 100.0;
            r->snr_db[h] = G8(p[4]) / 2.0;
            r->comp_db[h] = G8(p[5]) / 20.0;
            r->ampm_deg[h] = G8(p[6]) / 2.0;
        } else {
            r->noise_dbc[h] = r->amp_db[h] = get16(p) / 100.0;
            r->dist_dbc[h] = get16(p + 2) / 100.0;
            r->dist2_dbc[h] = G8(p[4]) / 2.0;
            r->noise2_dbc[h] = G8(p[5]) / 2.0;
            r->dist_ok[h] = p[6] & 1;
            r->dist2_ok[h] = (p[6] >> 1) & 1;
        }
    }
    return 0;
}

/* Result cells, at most CELL_W characters wide. */
#define CELL_W 26
typedef void (*cell_fn)(char *buf, size_t n, const probe_result_t *r, int h);

static void cell(char *buf, size_t n, const probe_result_t *r, int h)
{
    if (!r || !r->valid)
        snprintf(buf, n, " %8s %8s %7s", "-", "-", "-");
    else if (probe_is_tone(h))
        snprintf(buf, n, " %+8.2f %+8.1f %7.1f", r->amp_db[h], r->phase_deg[h], r->snr_db[h]);
    else
        snprintf(buf, n, "  silent: %+7.1f dBc", r->noise_dbc[h]);
}

/* Distortion level, or "<" upper bound when it is lost in the noise. */
static void dist_txt(char *b, size_t n, double v, int ok)
{
    if (ok)
        snprintf(b, n, "%+6.1f", v);
    else
        snprintf(b, n, "<%+5.0f", v);
}

static void cell_lvl(char *buf, size_t n, const probe_result_t *r, int h)
{
    if (!r || !r->valid || !r->has_l2) {
        snprintf(buf, n, " %8s %9s", "-", "-");
    } else if (probe_is_tone(h)) {
        snprintf(buf, n, " %+8.2f %+9.1f", r->comp_db[h], r->ampm_deg[h]);
    } else {
        char d1[16], d2[16], o[8] = "";
        dist_txt(d1, sizeof(d1), r->dist_dbc[h], r->dist_ok[h]);
        dist_txt(d2, sizeof(d2), r->dist2_dbc[h], r->dist2_ok[h]);
        int k = probe_dist_order(r, h);
        if (k)
            snprintf(o, sizeof(o), "n=%d", k);
        snprintf(buf, n, "  dist %s/%s %s", d1, d2, o);
    }
}

/* Width of the terminal the report goes to, or 0 when it is not one. */
static int term_cols(FILE *f)
{
    struct winsize ws;
    if (!isatty(fileno(f)) || ioctl(fileno(f), TIOCGWINSZ, &ws) || !ws.ws_col)
        return 0;
    return ws.ws_col;
}

/* ---- SNR histogram: one column per 150 Hz bin, 10 rows ------------- */

#define H_ROWS  10
#define H_LBL   7                        /* "%5.0f |" */
#define H_W     (H_LBL + NBINS)          /* display width of one histogram */
#define H_LINES (1 + H_ROWS + 2)         /* title, bars, axis, frequencies */

static int utf8_term(void)
{
    const char *cs = nl_langinfo(CODESET);
    return cs && !strcmp(cs, "UTF-8");
}

/* Fill lines[] (each H_W display columns wide, padded) for one result. */
static void hist_lines(const probe_result_t *r, const char *title, double ymax, int utf8,
                       char lines[H_LINES][512])
{
    static const char *blk[] = {" ", "\u2581", "\u2582", "\u2583", "\u2584",
                                "\u2585", "\u2586", "\u2587", "\u2588"};
    double step = ymax / H_ROWS;
    snprintf(lines[0], 512, "%-*.*s", H_W, H_W, title);
    for (int i = 0; i < H_ROWS; i++) {
        double base = ymax - (i + 1) * step;
        char *p = lines[1 + i];
        p += sprintf(p, "%5.0f |", base + step);
        for (int h = 1; h <= NBINS; h++) {
            if (!probe_is_tone(h)) {             /* mute carrier: '*' at the bottom */
                *p++ = i == H_ROWS - 1 ? '*' : ' ';
                continue;
            }
            double v = (r->snr_db[h] - base) / step;
            v = v < 0 ? 0 : v > 1 ? 1 : v;
            if (utf8)
                p += sprintf(p, "%s", blk[(int)lround(v * 8)]);
            else
                *p++ = v >= 0.5 ? '#' : ' ';
        }
        *p = 0;
    }
    char *ax = lines[1 + H_ROWS];
    ax += sprintf(ax, "%5d +", 0);
    for (int h = 1; h <= NBINS; h++)
        *ax++ = h % 10 ? '-' : '+';
    *ax = 0;
    /* Frequency labels (kHz) centred under every 10th bin. */
    char lab[NBINS + 8];
    memset(lab, ' ', sizeof(lab));
    static const char *k[] = {"1.5", "3", "4.5", "6", "7.5", "9", "10.5"};
    for (int i = 0; i < 7; i++) {
        int col = (i + 1) * 10 - 1, n = (int)strlen(k[i]), at = col - n / 2;
        if (at + n > NBINS)
            at = NBINS - n;
        memcpy(lab + at, k[i], n);
    }
    lab[NBINS] = 0;
    snprintf(lines[2 + H_ROWS], 512, "%*s%s", H_LBL, "kHz   ", lab);
}

static void snr_histogram(FILE *f, const probe_result_t *tx, const probe_result_t *rx)
{
    const probe_result_t *r[2] = {tx && tx->valid ? tx : NULL, rx && rx->valid ? rx : NULL};
    static const char *title[2] = {"Tx SNR per carrier, dB (measured by the remote side)",
                                   "Rx SNR per carrier, dB (measured locally)"};
    if (!r[0] && !r[1])
        return;
    /* Shared scale: top at the best SNR rounded up to 10 dB, 20..80 dB. */
    double mx = 0;
    for (int i = 0; i < 2; i++)
        for (int h = 1; r[i] && h <= NBINS; h++)
            if (probe_is_tone(h) && r[i]->snr_db[h] > mx)
                mx = r[i]->snr_db[h];
    double ymax = ceil(mx / 10) * 10;
    ymax = ymax < 20 ? 20 : ymax > 80 ? 80 : ymax;

    int utf8 = utf8_term();
    static char L[2][H_LINES][512];
    int n = 0, idx[2];
    for (int i = 0; i < 2; i++)
        if (r[i]) {
            hist_lines(r[i], title[i], ymax, utf8, L[n]);
            idx[n++] = i;
        }
    (void)idx;
    int cols = term_cols(f);
    if (n == 2 && cols >= 2 * H_W + 3) {
        for (int j = 0; j < H_LINES; j++)
            fprintf(f, "%s   %s\n", L[0][j], L[1][j]);  /* every line is H_W wide */
    } else {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < H_LINES; j++)
                fprintf(f, "%s\n", L[i][j]);
            if (i + 1 < n)
                fprintf(f, "\n");
        }
    }
    fprintf(f, "One column per 150 Hz carrier. '*': mute carrier (not transmitted).\n"
               "Empty column: SNR <= 0 dB.\n\n");
}

/* As many tables side by side as the terminal allows, filled column by
 * column: lowest frequencies in the leftmost table, highest rightmost. */
static void print_table(FILE *f, const char *hdr, cell_fn fn, const probe_result_t *tx,
                        const probe_result_t *rx)
{
    /* Rows: "  %7d |      <cell> |      <cell>" */
    static const char *sep =
        "  --------+---------------------------------+--------------------------------";
    const int tw = 17 + CELL_W + 8 + CELL_W, gap = 3;
    int cols = term_cols(f);
    int ntab = cols > 0 ? (cols + gap) / (tw + gap) : 1;
    if (ntab < 1)
        ntab = 1;
    if (ntab > NBINS)
        ntab = NBINS;
    int rows = (NBINS + ntab - 1) / ntab;
    ntab = (NBINS + rows - 1) / rows;          /* no empty table at the right */

    for (int t = 0; t < ntab; t++)
        fprintf(f, "%-*s%s", t + 1 < ntab ? tw : 0, hdr, t + 1 < ntab ? "   " : "\n");
    for (int t = 0; t < ntab; t++)
        fprintf(f, "%-*s%s", t + 1 < ntab ? tw : 0, sep, t + 1 < ntab ? "   " : "\n");
    for (int i = 0; i < rows; i++) {
        for (int t = 0; t < ntab; t++) {
            int h = t * rows + i + 1;
            int last = t + 1 == ntab || (t + 1) * rows + i + 1 > NBINS;
            char row[192] = "", a[64], b[64];
            if (h <= NBINS) {
                fn(a, sizeof(a), tx, h);
                fn(b, sizeof(b), rx, h);
                snprintf(row, sizeof(row), "  %7d |      %-*s |      %s", h * BASE_HZ, CELL_W, a, b);
            }
            if (last) {
                fprintf(f, "%s\n", row);
                break;
            }
            fprintf(f, "%-*s   ", tw, row);
        }
    }
    fprintf(f, "\n");
}

void report_print(FILE *f, int local_fs, const probe_result_t *tx,
                  const probe_result_t *rx)
{
    fprintf(f, "\n");
    fprintf(f, "Tx results: measured by the remote side on our signal");
    if (tx && tx->valid)
        fprintf(f, " (remote fs %d Hz, clock offset our TX vs their RX %+.1f ppm)\n",
                tx->fs, tx->ppm);
    else
        fprintf(f, ": NOT AVAILABLE\n");
    fprintf(f, "Rx results: measured locally on the remote signal");
    if (rx && rx->valid)
        fprintf(f, " (local fs %d Hz, clock offset their TX vs our RX %+.1f ppm)\n",
                local_fs, rx->ppm);
    else
        fprintf(f, ": NOT AVAILABLE\n");
    for (int i = 0; i < 2; i++) {
        const probe_result_t *r = i ? rx : tx;
        if (r && r->valid)
        {
            fprintf(f, "%s: probe heard at %.1f dBFS RMS (sent %.1f), guard tones %d / %d ms "
                       "(sent %d)\n", i ? "Rx" : "Tx", r->rms_dbfs, L_RMS_DBFS, r->lead_g_ms,
                    r->trail_g_ms, DUR_G_MS);
            if (r->has_l2)
                fprintf(f, "%s: level step L1 -> L2 %.2f dB (sent %.2f)%s\n", i ? "Rx" : "Tx",
                        r->rms_dbfs - r->rms2_dbfs, L2_STEP_DB,
                        r->rms_dbfs - r->rms2_dbfs < L2_STEP_DB - 0.3 ? "  <-- compression" : "");
        }
    }
    for (int i = 0; i < 2; i++) {
        const probe_result_t *r = i ? rx : tx;
        if (r && r->valid)
            fprintf(f, "%s: continuity: lowest period level %+.1f dB, %d timing jump%s "
                       "(largest step %.1f us)%s\n", i ? "Rx" : "Tx", r->min_level_db, r->jumps,
                    r->jumps == 1 ? "" : "s", r->max_jump_us,
                    (r->jumps || r->min_level_db < -3) ? "  <-- audio glitch" : "");
    }
    fprintf(f, "\n");
    snr_histogram(f, tx, rx);
    fprintf(f, "amp: dB relative to the 1050 Hz carrier. phase: anomaly of each carrier in\n"
               "degrees (received minus sent phase, minus the best straight-line fit, i.e.\n"
               "bulk delay and constant offset). SNR: per tone, in its 150 Hz bin.\n"
               "silent: mute carrier (not transmitted); the level shown is the noise found\n"
               "in its bin, in dB re the 1050 Hz carrier (dBc). All of this from L1.\n\n");
    char ht[40], hr[40], hdr[128];
    snprintf(ht, sizeof(ht), " %8s %8s %7s", "Tx: amp", "phase", "SNR");   /* as cell() */
    snprintf(hr, sizeof(hr), " %8s %8s %7s", "Rx: amp", "phase", "SNR");
    snprintf(hdr, sizeof(hdr), "  Freq Hz |      %-*s |      %s", CELL_W, ht, hr);
    print_table(f, hdr, cell, tx, rx);

    if ((tx && tx->valid && tx->has_l2) || (rx && rx->valid && rx->has_l2)) {
        fprintf(f, "Level dependence: L1 versus L2, sent %.2f dB lower.\n"
                   "comp: change of the carrier's gain beyond that step, dB (negative: the\n"
                   "channel compresses at the higher level). AM/PM: phase change, degrees.\n"
                   "dist: distortion products found in the mute carrier's bin at L1 / L2,\n"
                   "dBc ('<': below the noise, upper bound). n: order of the distortion, from\n"
                   "how it scales (relative to the carriers it rises (n-1) x %.2f dB with L1).\n\n",
                L2_STEP_DB, L2_STEP_DB);
        snprintf(ht, sizeof(ht), " %8s %9s", "Tx: comp", "AM/PM");   /* as cell_lvl() */
        snprintf(hr, sizeof(hr), " %8s %9s", "Rx: comp", "AM/PM");
        snprintf(hdr, sizeof(hdr), "  Freq Hz |      %-*s |      %s", CELL_W, ht, hr);
        print_table(f, hdr, cell_lvl, tx, rx);
    }
}

static void csvcell(FILE *f, const probe_result_t *r, int h)
{
    if (!r || !r->valid) {
        fprintf(f, ",,,,,,,,");
    } else if (probe_is_tone(h)) {
        fprintf(f, ",%.2f,%.2f,%.1f", r->amp_db[h], r->phase_deg[h], r->snr_db[h]);
        if (r->has_l2)
            fprintf(f, ",%.2f,%.1f,,,", r->comp_db[h], r->ampm_deg[h]);
        else
            fprintf(f, ",,,,,");
    } else {
        fprintf(f, ",,,,,,%.2f", r->noise_dbc[h]);
        fprintf(f, r->dist_ok[h] ? ",%.2f" : ",<%.2f", r->dist_dbc[h]);
        if (r->has_l2)
            fprintf(f, r->dist2_ok[h] ? ",%.1f" : ",<%.1f", r->dist2_dbc[h]);
        else
            fprintf(f, ",");
    }
}

int report_csv(const char *path, const probe_result_t *tx, const probe_result_t *rx)
{
    FILE *f = fopen(path, "w");
    if (!f)
        return -1;
    fprintf(f, "freq_hz,kind,"
               "tx_amp_db,tx_phase_deg,tx_snr_db,tx_comp_db,tx_ampm_deg,tx_noise_dbc,"
               "tx_dist_l1_dbc,tx_dist_l2_dbc,"
               "rx_amp_db,rx_phase_deg,rx_snr_db,rx_comp_db,rx_ampm_deg,rx_noise_dbc,"
               "rx_dist_l1_dbc,rx_dist_l2_dbc\n");
    for (int h = 1; h <= NBINS; h++) {
        fprintf(f, "%d,%s", h * BASE_HZ, probe_is_tone(h) ? "tone" : "mute");
        csvcell(f, tx, h);
        csvcell(f, rx, h);
        fprintf(f, "\n");
    }
    if (tx && tx->valid)
        fprintf(f, "# tx_clock_offset_ppm,%.3f\n# tx_measuring_fs,%d\n# tx_level_dbfs,%.2f\n"
                   "# tx_level_l2_dbfs,%.2f\n# tx_guard_ms,%d,%d\n# tx_continuity,%.2f,%d,%.1f\n",
                tx->ppm, tx->fs, tx->rms_dbfs, tx->rms2_dbfs, tx->lead_g_ms, tx->trail_g_ms, tx->min_level_db, tx->jumps,
                tx->max_jump_us);
    if (rx && rx->valid)
        fprintf(f, "# rx_clock_offset_ppm,%.3f\n# rx_measuring_fs,%d\n# rx_level_dbfs,%.2f\n"
                   "# rx_level_l2_dbfs,%.2f\n# rx_guard_ms,%d,%d\n# rx_continuity,%.2f,%d,%.1f\n",
                rx->ppm, rx->fs, rx->rms_dbfs, rx->rms2_dbfs, rx->lead_g_ms, rx->trail_g_ms, rx->min_level_db, rx->jumps,
                rx->max_jump_us);
    return fclose(f);
}
