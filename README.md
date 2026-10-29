# SX1262 bench: LoRa / GFSK at 868 MHz on a minimal modem, Meshtastic and Jentastic

Bench tools, raw results and reports from testing LoRa and GFSK on two **ProMicro nRF52840 + Ebyte E22P** (SX1262,
868 MHz) boards: first with a minimal modem firmware that measures the radio itself, then inside Meshtastic and
Jentastic. Companion to [sx1280-bench](https://github.com/sqtactical/sx1280-bench) (same tools, SX1280 at 2.4 GHz).

The firmware changes live in two forks:

- Meshtastic `develop` + FLRC/GFSK presets, GFSK on SX126x for EU_868:
  [sqtactical/firmware `sx1262-fsk`](https://github.com/sqtactical/firmware/tree/sx1262-fsk)
  ([docs](https://github.com/sqtactical/firmware/blob/sx1262-fsk/docs/sx1262-gfsk-868.md))
- Jentastic `testing` + SETFSK on SX1262:
  [sqtactical/jentastic `sx12xx-fsk-flrc`](https://github.com/sqtactical/jentastic/tree/sx12xx-fsk-flrc) (SX1280 and SX1262 together)
  ([docs](https://github.com/sqtactical/jentastic/blob/sx12xx-fsk-flrc/docs/sx1262-fsk.md))

## Headline numbers

Two boards in one room, SX1262 at −9 dBm (RSSI −6 to −8 dBm), 869.525 MHz.

| What | GFSK | LoRa |
|---|---|---|
| Minimal modem, file PC to PC, block ACK (fastest mode) | 260 kb/s (300k) | 56.5 kb/s (SF5 / 500 kHz) |
| Minimal modem, fastest mode that fits EU_868's 250 kHz | 135 kb/s (150k) | 28.4 kb/s (SF5 / 250 kHz) |
| Minimal modem, mesh-style relay (ACK per hop) | 111 kb/s (300k), 37-43 % of nominal | 25.4 kb/s (SF5 / 500 kHz) |
| Meshtastic `sx1262-fsk`, median DM→ACK round trip | 83-161 ms | 339 ms (SHORT_FAST), 987 ms (MEDIUM_FAST) |
| Meshtastic, most delivered without loss (raw serial frames) | ~37 kb/s | ~6.5 kb/s (SHORT_FAST) |
| Jentastic `sx12xx-fsk-flrc`, median DM→ACK round trip | 103-225 ms (SETFSK 25-300) | 1067 ms (MEDIUM_FAST) |

Every minimal-modem test (34 modes, 86 runs) delivered its file with the SHA-256 intact and no retransmissions.

## Layout

| Path | Contents |
|---|---|
| `firmware/sx1262_modem/` | Minimal Arduino modem (LoRa/GFSK, block ARQ, relay, burst), same protocol as the SX1280 one |
| `tools/` | Test scripts (below) |
| `results/modem/` | Raw JSON from the minimal-modem tests: `results_{lora,gfsk}.json` (block ACK), `_win1` (ACK per packet), `_relay` |
| `results/meshtastic/` | `mesh_e22p_results.json` (preset sweep), `raw_rate_results.json` (raw serial frames past the library) |
| `results/jentastic/` | `jt_e22p_results.json` (SETFSK sweeps on `26a0451`), `jt_e22p_rebased.json` (on `0ea3007` + this branch), `jt_e22p_stock0ea3007.json` |
| `report/` | `Informe_SX1262_Modem_LoRa_GFSK.pdf` (minimal modem), `Informe_SX1262_Meshtastic_JenTastic_LoRa_FSK.pdf` (firmwares) |

## Tools

All Python, using `pyserial` and, for the firmware tests, the `meshtastic` library (the scripts patch out its 1 s
sleep after every serial write on Windows).

| Script | Purpose |
|---|---|
| `build_modem.ps1` | Build `sx1262_modem` with arduino-cli (Adafruit nRF52 core 1.7.0, board `pca10056`) and convert it to UF2 |
| `flash.py` | Flash every connected board without touching reset: `DFU` command for the modem, `--meshtastic` for `--enter-dfu` |
| `sx1262_host.py` | Minimal modem: per mode a raw burst and a file transfer (SHA-256 checked); `--win 1` ACKs every packet, `--relay` is the mesh-style test |
| `mesh_speed.py` | Meshtastic: switch both nodes to a preset, then ACK test (DMs with want_ack) and stream test (broadcasts) |
| `raw_rate.py` | Meshtastic/Jentastic: write raw ToRadio frames to the USB at a fixed interval, past the library's ~30 kb/s ceiling |
| `jt_speed.py` | Jentastic: switch modes with `/nexus-local SETFSK …` and run the same ACK and stream tests |

Example, minimal modem (node A on COM41, B on COM40):

```bash
python tools/sx1262_host.py --tx COM41 --rx COM40 --modem gfsk
python tools/sx1262_host.py --tx COM41 --rx COM40 --modem lora --relay --range 8:
```

Paths inside some scripts point at the original bench setup (`%LOCALAPPDATA%\flrc-tools`); adjust before use.
Keep the power low on a bench (`--pwr -9`, `lora.tx_power -9`): with a ~30 dBm PA module the receivers saturate.

## Findings worth knowing

- **nRF52840 erratum 198**: with the SX1262 on SPIM3, every other packet came back with a false buffer status
  (2 bytes past the previous packet): the SPI transfer was corrupted. SPIM2 (8 MHz) fixed it.
- **SX1262 RX buffer**: in continuous RX each packet is written right after the previous one, wrapping around the
  256-byte buffer. The modem uses single RX and sets the buffer base before every SetRx.
- **Meshtastic is the bottleneck**: the radio moves 135-260 kb/s in GFSK; the firmware tops out around 40 kb/s.

Node configuration backups are deliberately not included: they contain the nodes' private keys.
