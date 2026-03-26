#!/usr/bin/env python3
"""
rx_monitor.py – bare-metal SX127x RX monitor for LoRaHAM Pi
Listens continuously and prints every signal event: RSSI, ValidHeader,
CRC errors, and full received packets.

Usage:
    python rx_monitor.py [--freq 867200000] [--sf 8] [--bw 125000]
"""
import argparse
import sys
import time
import signal

try:
    import RPi.GPIO as GPIO
    import spidev
except ImportError:
    print("ERROR: RPi.GPIO and spidev required")
    sys.exit(1)

# SX127x registers
REG_OP_MODE          = 0x01
REG_FR_MSB           = 0x06
REG_FR_MID           = 0x07
REG_FR_LSB           = 0x08
REG_PA_CONFIG        = 0x09
REG_LNA              = 0x0C
REG_FIFO_ADDR_PTR    = 0x0D
REG_FIFO_RX_BASE     = 0x0F
REG_FIFO_RX_CURRENT  = 0x10
REG_IRQ_FLAGS        = 0x12
REG_RX_NB_BYTES      = 0x13
REG_PKT_SNR          = 0x19
REG_PKT_RSSI         = 0x1A
REG_RSSI_WIDEBAND    = 0x2C
REG_MODEM_CONFIG1    = 0x1D
REG_MODEM_CONFIG2    = 0x1E
REG_MODEM_CONFIG3    = 0x26
REG_SYNC_WORD           = 0x39
REG_DETECTION_OPTIMIZE  = 0x31
REG_DETECTION_THRESHOLD = 0x37
REG_VERSION          = 0x42
REG_FIFO             = 0x00

MODE_LONG_RANGE = 0x80
MODE_SLEEP      = 0x00
MODE_STDBY      = 0x01
MODE_RX_CONT    = 0x05

IRQ_RX_DONE       = 0x40
IRQ_CRC_ERR       = 0x20
IRQ_VALID_HEADER  = 0x10
IRQ_TX_DONE       = 0x08

FXOSC = 32_000_000.0

PIN_CS    = 26
PIN_RESET = 6
PIN_DIO0  = 16
SPI_BUS   = 0
SPI_DEV   = 1


def spi_read(spi, reg):
    GPIO.output(PIN_CS, GPIO.LOW)
    r = spi.xfer2([reg & 0x7F, 0x00])[1]
    GPIO.output(PIN_CS, GPIO.HIGH)
    return r

def spi_write(spi, reg, val):
    GPIO.output(PIN_CS, GPIO.LOW)
    spi.xfer2([reg | 0x80, val & 0xFF])
    GPIO.output(PIN_CS, GPIO.HIGH)

def spi_read_burst(spi, reg, n):
    GPIO.output(PIN_CS, GPIO.LOW)
    r = spi.xfer2([reg & 0x7F] + [0x00] * n)[1:]
    GPIO.output(PIN_CS, GPIO.HIGH)
    return r


def setup(spi, freq, sf, bw_hz, sync_word):
    # Reset
    GPIO.output(PIN_RESET, GPIO.LOW);  time.sleep(0.01)
    GPIO.output(PIN_RESET, GPIO.HIGH); time.sleep(0.01)

    ver = spi_read(spi, REG_VERSION)
    print(f"  Chip version : 0x{ver:02X}", "(OK)" if ver in (0x11, 0x12) else "(UNEXPECTED)")

    # Sleep → set LoRa mode
    spi_write(spi, REG_OP_MODE, MODE_SLEEP)
    time.sleep(0.01)
    spi_write(spi, REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP)
    time.sleep(0.01)
    spi_write(spi, REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY)
    time.sleep(0.01)

    # Frequency
    frf = int((freq / FXOSC) * (1 << 19))
    spi_write(spi, REG_FR_MSB, (frf >> 16) & 0xFF)
    spi_write(spi, REG_FR_MID, (frf >>  8) & 0xFF)
    spi_write(spi, REG_FR_LSB, (frf      ) & 0xFF)

    # Bandwidth
    bw_table = {7800:0, 10400:1, 15600:2, 20800:3, 31250:4,
                41700:5, 62500:6, 125000:7, 250000:8, 500000:9}
    bw_reg = min(bw_table, key=lambda k: abs(k - bw_hz))
    bw_bits = bw_table[bw_reg] << 4
    mc1 = spi_read(spi, REG_MODEM_CONFIG1)
    spi_write(spi, REG_MODEM_CONFIG1, (mc1 & 0x0F) | bw_bits)

    # SF + CRC
    spi_write(spi, REG_MODEM_CONFIG2, (sf << 4) | 0x04)  # CRC on

    # Detection optimisation (required for SF>6)
    if sf == 6:
        spi_write(spi, REG_DETECTION_OPTIMIZE,  0xC5)
        spi_write(spi, REG_DETECTION_THRESHOLD, 0x0C)
    else:
        spi_write(spi, REG_DETECTION_OPTIMIZE,  0xC3)
        spi_write(spi, REG_DETECTION_THRESHOLD, 0x0A)

    # AGC on
    mc3 = spi_read(spi, REG_MODEM_CONFIG3)
    spi_write(spi, REG_MODEM_CONFIG3, mc3 | 0x04)

    # LNA: G1 (max) + HF boost (+3 dB for >525 MHz)
    spi_write(spi, REG_LNA, 0x23)

    # Sync word
    spi_write(spi, REG_SYNC_WORD, sync_word)

    # PA off during RX (use RFO, minimal drive)
    spi_write(spi, REG_PA_CONFIG, 0x00)

    # Enter RXCONT
    spi_write(spi, REG_FIFO_RX_BASE, 0x00)
    spi_write(spi, REG_FIFO_ADDR_PTR, 0x00)
    spi_write(spi, REG_IRQ_FLAGS, 0xFF)
    spi_write(spi, REG_OP_MODE, MODE_LONG_RANGE | MODE_RX_CONT)

    mode = spi_read(spi, REG_OP_MODE)
    lna  = spi_read(spi, REG_LNA)
    print(f"  OP_MODE      : 0x{mode:02X}", "(RXCONT)" if (mode & 0x07) == 0x05 else "(NOT RXCONT!)")
    print(f"  LNA reg      : 0x{lna:02X}")
    print(f"  Sync word    : 0x{sync_word:02X}")
    print(f"  Frequency    : {freq/1e6:.4f} MHz  SF{sf}  BW{bw_reg//1000}kHz")


def run(spi):
    print("\n  Listening… (Ctrl-C to stop)\n")
    last_rssi_print = 0
    pkt_count = 0
    header_count = 0

    while True:
        flags = spi_read(spi, REG_IRQ_FLAGS)

        if flags & IRQ_VALID_HEADER:
            header_count += 1
            rssi_raw = spi_read(spi, REG_PKT_RSSI)
            rssi = -157 + rssi_raw
            print(f"[{time.strftime('%H:%M:%S')}] ValidHeader #{header_count}  RSSI≈{rssi} dBm")
            spi_write(spi, REG_IRQ_FLAGS, IRQ_VALID_HEADER)

        if flags & IRQ_RX_DONE:
            if flags & IRQ_CRC_ERR:
                rssi_raw = spi_read(spi, REG_PKT_RSSI)
                snr_raw  = spi_read(spi, REG_PKT_SNR)
                rssi = -157 + rssi_raw
                snr  = snr_raw / 4 if snr_raw < 128 else (snr_raw - 256) / 4
                print(f"[{time.strftime('%H:%M:%S')}] CRC ERROR  RSSI={rssi} dBm  SNR={snr:.1f} dB  IRQ=0x{flags:02X}")
                spi_write(spi, REG_IRQ_FLAGS, 0xFF)
            else:
                ptr = spi_read(spi, REG_FIFO_RX_CURRENT)
                spi_write(spi, REG_FIFO_ADDR_PTR, ptr)
                n   = spi_read(spi, REG_RX_NB_BYTES)
                raw = bytes(spi_read_burst(spi, REG_FIFO, n))
                rssi_raw = spi_read(spi, REG_PKT_RSSI)
                snr_raw  = spi_read(spi, REG_PKT_SNR)
                rssi = -157 + rssi_raw
                snr  = snr_raw / 4 if snr_raw < 128 else (snr_raw - 256) / 4
                pkt_count += 1
                print(f"[{time.strftime('%H:%M:%S')}] PKT #{pkt_count}  {n} B  "
                      f"RSSI={rssi} dBm  SNR={snr:.1f} dB  IRQ=0x{flags:02X}")
                print(f"  hex : {raw.hex()}")
                try:
                    print(f"  text: {raw.decode('utf-8', errors='replace')}")
                except Exception:
                    pass
                spi_write(spi, REG_IRQ_FLAGS, 0xFF)

            # Re-arm RX
            spi_write(spi, REG_FIFO_RX_BASE, 0x00)
            spi_write(spi, REG_FIFO_ADDR_PTR, 0x00)
            spi_write(spi, REG_OP_MODE, MODE_LONG_RANGE | MODE_RX_CONT)

        # Print wideband RSSI every 5 s to confirm radio is active
        now = time.time()
        if now - last_rssi_print >= 5.0:
            wb = spi_read(spi, REG_RSSI_WIDEBAND)
            pkt_rssi_raw = spi_read(spi, REG_PKT_RSSI)
            mode = spi_read(spi, REG_OP_MODE)
            print(f"  [{time.strftime('%H:%M:%S')}] ambient wideband RSSI raw=0x{wb:02X}  "
                  f"pkt_rssi raw=0x{pkt_rssi_raw:02X} ({-157+pkt_rssi_raw} dBm)  "
                  f"mode=0x{mode:02X}  pkts={pkt_count}")
            last_rssi_print = now

        time.sleep(0.002)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--freq",      type=int,   default=867_200_000)
    p.add_argument("--sf",        type=int,   default=8)
    p.add_argument("--bw",        type=int,   default=125_000)
    p.add_argument("--sync-word", type=lambda x: int(x, 16), default=0x12)
    args = p.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_RESET, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(PIN_DIO0,  GPIO.IN,  pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(PIN_CS,    GPIO.OUT, initial=GPIO.HIGH)

    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEV)
    spi.max_speed_hz = 5_000_000
    spi.mode = 0

    def cleanup(sig=None, frame=None):
        print("\n  Stopping.")
        try: spi.close()
        except Exception: pass
        GPIO.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)

    print("SX127x RX Monitor")
    setup(spi, args.freq, args.sf, args.bw, args.sync_word)
    run(spi)


if __name__ == "__main__":
    main()
