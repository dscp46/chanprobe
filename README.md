# chanprobe

Bidirectional prober for half duplex audio channels (150–10500 Hz), such as a
radio link keyed through Hamlib. Two stations exchange an extended ITU-T V.34
line probing signal and send each other their measurements, so both ends see
the channel in both directions.

## Probe signal

60 cosines on a 150 Hz grid, repeating every 6.667 ms (294 samples at 44.1 kHz,
320 at 48 kHz, so each tone falls exactly on a DFT bin at either rate).

* 150–3750 Hz: V.34 Table 17 phases; 900, 1200, 1800 and 2400 Hz are silent,
  and so is 3600 Hz (unlike V.34).
* 3900–10500 Hz: 0/180° pattern optimised jointly with the V.34 part;
  4800, 6000, 7200, 8400 and 9600 Hz are silent.
* Crest factor ≈ 5.7 dB; peak −3 dBFS (L1), RMS −5.7 dBFS.

Tone A (900 Hz) and guard tone G (1200 Hz) sit on silent probe bins.

## Session

```
initiator  A 300 ms
responder        [t_w] G 200 ms · L1 1 s · G 200 ms · L2 1 s · G 200 ms
                                                              → initiator measures
initiator                [t_w] G · L1 · G · L2 · G             → responder measures
responder                        [t_w] results (DATAC1)
initiator                                [t_w] results (DATAC1)
```

Each transmission is wrapped in txdelay (default 100 ms) and txtail (30 ms).
A full session takes about 15.5 s of air time.

## Measurement

L is sent twice, as in V.34: L1 (peak −3 dBFS) then L2, exactly 6.02 dB lower,
each preceded and the last followed by a 200 ms guard tone. For each level the receiver splits L into periods, measures every
150 Hz bin in every period, estimates the sample clock offset between the two
sound interfaces from the phase drift, resamples to cancel it, and averages 128
periods (853 ms) taken from the middle of that level. Reported per tone (L1):

* amplitude in dB re 1050 Hz,
* phase anomaly per carrier: received minus sent phase, minus the best
  straight-line fit across the band (bulk delay and constant offset),
* SNR (per period, in the tone's 150 Hz bin),

plus the noise in the mute carriers' bins (dBc), the clock offset
in ppm, the received level, how long each guard tone was heard (a short
trailing tone means the end of the transmission is cut off), and the
continuity of the audio: the lowest level of any period (dropouts) and sudden
timing steps between periods (slipped or repeated samples).

### Level dependence (L1 versus L2)

* compression per carrier: gain change beyond the 6.02 dB step, and the
  overall level step,
* AM/PM per carrier: phase change between the two levels,
* distortion in each mute carrier's bin, at both levels: the part of the bin
  that repeats with the probe (distortion products), separated from the
  random part (noise),
* the order of that distortion, from how it scales: relative to the carriers
  it rises by (n−1) × 6.02 dB from L2 to L1 (2nd order: 6 dB, 3rd: 12 dB).

## Results data link

Results travel as one codec2 FreeDV DATAC1 burst: OFDM, 27 carriers around
1.5 kHz (about 1.7 kHz wide), pilot-aided equalisation, LDPC FEC and CRC-16,
about 4.4 s on air for the 444-byte message. It runs at 8 kHz and is
resampled with libsamplerate. Sent at -10 dBFS RMS (peaks about -5.5 dBFS, never above -3 dBFS).
On a 300-3000 Hz channel it decodes down to about 3 dB SNR and through an
80 ms dropout; the Bell 202 link of earlier versions needed about 9 dB and
lost the frame to any 10 ms dropout.

## Building

```
sudo apt install build-essential debhelper libasound2-dev libhamlib-dev \
                 libcodec2-dev libsamplerate0-dev pkgconf
dpkg-buildpackage -us -uc -b      # produces ../chanprobe_0.1.0_<arch>.deb
```

or simply `make && make check`. `make check` runs an offline self test:
signal properties, probe analysis through resampled / filtered / noisy channels,
glitch detection, the modem (noise, squelch blips, clipped tails), and complete initiator/responder sessions between a 44.1 kHz and a
48 kHz station with clock offsets.

Requires Hamlib ≥ 4.5 (Debian 12 "bookworm" ships 4.5.4).

## Quick start

```
chanprobe tune -m 1035 -r /dev/ttyUSB0 -s 38400 -D plughw:CODEC   # set levels
chanprobe responder -P RTS -p /dev/ttyUSB1 -D hw:1,0 --rate 44100
chanprobe initiator -m 1035 -r /dev/ttyUSB0 -s 38400 -D hw:1,0 --csv run.csv
```

See `man chanprobe` for all options.

## License

Copyright © 2026 Célestine GRAMAIZE <celestine+dev@gramaize.eu>

chanprobe is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version. See [COPYING](COPYING).
