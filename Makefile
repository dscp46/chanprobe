# chanprobe - SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>

PREFIX     ?= /usr
BINDIR     ?= $(PREFIX)/bin
MANDIR     ?= $(PREFIX)/share/man
PKG_CONFIG ?= pkg-config

CFLAGS  ?= -O2 -g
WARN     = -Wall -Wextra -Wno-unused-parameter
STD      = -std=c11 -D_DEFAULT_SOURCE

ALSA_CFLAGS   := $(shell $(PKG_CONFIG) --cflags alsa)
ALSA_LIBS     := $(shell $(PKG_CONFIG) --libs alsa)
HAMLIB_CFLAGS := $(shell $(PKG_CONFIG) --cflags hamlib)
HAMLIB_LIBS   := $(shell $(PKG_CONFIG) --libs hamlib)
MODEM_CFLAGS  := $(shell $(PKG_CONFIG) --cflags codec2 samplerate)
MODEM_LIBS    := $(shell $(PKG_CONFIG) --libs codec2 samplerate)

DSP_SRC = src/probe.c src/modem.c src/signal.c src/report.c src/session.c
APP_SRC = src/main.c src/audio.c src/ptt.c $(DSP_SRC)

all: chanprobe

chanprobe: $(APP_SRC) src/*.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(STD) $(WARN) $(ALSA_CFLAGS) $(HAMLIB_CFLAGS) $(MODEM_CFLAGS) \
		-o $@ $(APP_SRC) $(LDFLAGS) $(ALSA_LIBS) $(HAMLIB_LIBS) $(MODEM_LIBS) -lm

tests/selftest: tests/selftest.c $(DSP_SRC) src/*.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(STD) $(WARN) $(MODEM_CFLAGS) -pthread -o $@ tests/selftest.c \
		$(DSP_SRC) $(LDFLAGS) $(MODEM_LIBS) -lm

check: tests/selftest
	./tests/selftest

install: chanprobe
	install -D -m 0755 chanprobe $(DESTDIR)$(BINDIR)/chanprobe
	install -D -m 0644 man/chanprobe.1 $(DESTDIR)$(MANDIR)/man1/chanprobe.1

clean:
	rm -f chanprobe tests/selftest

.PHONY: all check install clean
