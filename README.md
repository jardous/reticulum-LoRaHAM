# LoRaHAMInterface – Reticulum Interface for the LoRaHAM Pi

A fully self-contained Reticulum custom-interface module for the
[LoRaHAM Pi HAT](https://www.loraham.de/produkt/loraham-pi/)
(SX1276 / SX1278 on 433 MHz).

All SX127x register access, SPI and GPIO handling is inlined directly —
**no pySX127x or any other LoRa library is required**.

---

## Files

| File | Purpose |
|---|---|
| `LoRaHAMInterface.py` | Reticulum interface module |
| `loraham_test.py` | Hardware test suite (run standalone, no RNS needed) |

---

## Requirements

| Dependency | Install |
|---|---|
| Reticulum | `pip install rns` |
| RPi.GPIO | `pip install RPi.GPIO` |
| spidev | `pip install spidev` |

No other LoRa library is needed.

---

## Hardware wiring (BCM pin numbering)

| SX127x pin | Raspberry Pi | BCM |
|---|---|---|
| NSS (CS) | SPI0 CE0 | 8 |
| MOSI | SPI0 MOSI | 10 |
| MISO | SPI0 MISO | 9 |
| SCK | SPI0 SCLK | 11 |
| DIO0 | GPIO | **25** |
| RESET | GPIO | **5** |

If your HAT uses different pins for DIO0 or RESET, you can specify them
in your Reticulum `config` file:

```ini
[[LoRaHAM 433]]
  # ... other options
  pin_dio0  = 25
  pin_reset = 5
```

Make sure SPI is enabled on the Pi: `sudo raspi-config` → Interfaces → SPI → Enable.

---

## Installation

1. Copy `LoRaHAMInterface.py` to `~/.reticulum/interfaces/`
2. Add a configuration block (see below) to `~/.reticulum/config`
3. Start Reticulum: `rnsd -v`

---

## Configuration block

```ini
[[LoRaHAM 433]]
  type              = LoRaHAMInterface
  interface_enabled = yes

  # Centre frequency in Hz (433.775 MHz = common LoRaHAM default)
  frequency         = 433775000

  # Bandwidth in Hz: 7800 / 10400 / 15600 / 20800 / 31250 /
  #                  41700 / 62500 / 125000 / 250000 / 500000
  bandwidth         = 125000

  # Spreading factor 7–12  (7 = fastest, shortest range)
  spreading_factor  = 7

  # Coding rate denominator: 5 = 4/5, 6 = 4/6, 7 = 4/7, 8 = 4/8
  coding_rate       = 5

  # TX power in dBm (2–20; LoRaHAM Pi has an external PA –
  # check your regional licence limits)
  tx_power          = 17

  # Sync word (0x12 = private LoRa network; 0x34 = LoRaWAN public)
  sync_word         = 0x12

  # Preamble length in symbols (default 8)
  preamble_length   = 8

  # Reticulum interface mode: full / point_to_point / access_point /
  #                           roaming / boundary / gateway
  mode              = full

  # To use LoRa implicit header mode (fixed-size packets), add:
  # implicit_header = yes
```

---

## Multiple Interfaces

You can run multiple LoRa modules on the same Raspberry Pi to bridge different networks (e.g., a 433 MHz "long range" and an 868 MHz "fast local" network).

Each interface must have a unique `[[Interface Name]]` and specify its own hardware pins and SPI chip select.

```ini
# --- Interface 1: 433 MHz (Standard LoRaHAM Pi HAT defaults) ---
[[LoRaHAM 433]]
  type              = LoRaHAMInterface
  interface_enabled = yes
  frequency         = 433775000
  bandwidth         = 125000
  spreading_factor  = 9
  coding_rate       = 5
  tx_power          = 17
  
  # Hardware pins for Module 1 (LoRaHAM Pi Board defaults)
  pin_dio0          = 25
  pin_reset         = 5
  spi_bus           = 0
  spi_cs            = 0

# --- Interface 2: 868 MHz (Secondary LoRaHAM Pi module) ---
[[LoRaHAM 868]]
  type              = LoRaHAMInterface
  interface_enabled = yes
  frequency         = 868125000
  bandwidth         = 125000
  spreading_factor  = 7
  coding_rate       = 5
  tx_power          = 14
  
  # Hardware pins for Module 2 (LoRaHAM Pi Board defaults)
  pin_dio0          = 16
  pin_reset         = 6
  spi_bus           = 0
  spi_cs            = 1
```

---

## Packet framing & Fragmentation

The SX127x hardware FIFO is limited to 255 bytes. Reticulum's transport MTU is 500 bytes. This interface handles fragmentation and reassembly internally to support the full 500-byte MTU.

Each LoRa frame is prefixed with a **1-byte control flag**:

| Flag | Meaning |
|---|---|
| `0x00` | **Single Frame:** Complete RNS packet (up to 254 bytes) |
| `0x01` | **Fragment 1:** First half of a fragmented packet |
| `0x02` | **Fragment 2:** Final half of a fragmented packet |

Fragmentation is transparent to Reticulum; the interface reports an `HW_MTU` of 500 bytes.

---

## On-air bit-rate reference

| BW (kHz) | SF | CR | Rb (bit/s) | Typical use |
|---|---|---|---|---|
| 125 | 7 | 4/5 | ~5469 | Short range, fast |
| 125 | 9 | 4/5 | ~1367 | Balanced |
| 62.5 | 10 | 4/5 | ~366 | Long range |
| 62.5 | 12 | 4/5 | ~91 | Maximum range |

> All nodes on the same Reticulum network segment must use identical
> frequency, bandwidth, spreading factor, coding rate, and sync word.

---

## Hardware test suite

Run before using with Reticulum to verify the hardware is working:

```bash
# Full register + hardware test (recommended first run)
sudo python3 loraham_test.py

# Just listen for packets from another node
sudo python3 loraham_test.py --rx-only --timeout 30

# Transmit N test packets and verify TxDone fires
sudo python3 loraham_test.py --tx-only --count 5

# TX→RX loopback (requires a second node echoing packets)
sudo python3 loraham_test.py --loopback --count 3 --timeout 15
```

The test suite checks SPI connectivity, chip version, all modem config
registers, frequency accuracy, PA config, DIO0 interrupt, IRQ flags,
RSSI, and live TX/RX. A result of 30/30 with RX skipped (no transmitter
in range) means the hardware is fully functional.

---

## Troubleshooting

**`RuntimeError: Failed to add edge detection`**
The DIO0 pin number may be wrong for your HAT. Run the test suite — test
11b fires a TX packet and polls nearby BCM pins to identify which one
actually pulses. Update `PIN_DIO0` accordingly.

**`RuntimeError: Unexpected SX127x version byte 0xFF`**
SPI is not reaching the chip. Check: SPI enabled in raspi-config, HAT
seated correctly, CE0 wiring.

**`RuntimeError: Unexpected SX127x version byte 0x00`**
SPI bus is returning zeros — usually a MISO wiring fault or wrong SPI bus/CS.

**Interface loads but no packets received**
Verify both nodes use identical frequency, BW, SF, CR, and sync word.
Check with `loraham_test.py --rx-only` while the other node runs
`--tx-only`.

---

## Licence

MIT – © 2026
