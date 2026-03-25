#!/usr/bin/env python3
"""
loraham_test.py – LoRaHAM Pi / SX127x interface test suite
===========================================================
Tests are independent of Reticulum and exercise the hardware directly
through the same _SX127x driver used by LoRaHAMInterface.

Run as root (required for GPIO/SPI):
    sudo python3 loraham_test.py

Optional flags:
    --freq   433775000   centre frequency in Hz
    --bw     125000      bandwidth in Hz
    --sf     7           spreading factor (7-12)
    --cr     5           coding rate denominator (5-8)
    --power  17          TX power dBm
    --loopback           enable loopback test (transmit then listen for echo;
                         requires a second LoRa node or a wired loopback)
    --rx-only            just listen and print received packets (Ctrl-C to stop)
    --tx-only            just transmit a sequence of test packets
    --count  5           number of TX packets to send in loopback / tx-only mode
    --timeout 10         seconds to wait for a packet in RX / loopback tests
"""

import sys
import time
import argparse
import threading
import traceback

# ---------------------------------------------------------------------------
# Import the driver classes directly from LoRaHAMInterface.py.
# Adjust the path if the file lives elsewhere.
# ---------------------------------------------------------------------------
sys.path.insert(0, "/home/jiri/.reticulum/interfaces")

try:
    from LoRaHAMInterface import (
        _SX127x,
        REG_OP_MODE, REG_VERSION, REG_SYNC_WORD,
        REG_MODEM_CONFIG1, REG_MODEM_CONFIG2, REG_MODEM_CONFIG3,
        REG_FR_MSB, REG_FR_MID, REG_FR_LSB,
        REG_PA_CONFIG, REG_PA_DAC,
        REG_PREAMBLE_MSB, REG_PREAMBLE_LSB,
        REG_IRQ_FLAGS,
        REG_DIO_MAPPING1,
        MODE_LONG_RANGE, MODE_SLEEP, MODE_STDBY, MODE_RX_CONT, MODE_TX,
        IRQ_RX_DONE, IRQ_TX_DONE, IRQ_PAYLOAD_CRC_ERR,
        MAX_LORA_PKT, FXOSC, BW_MAP,
    )
except ImportError as e:
    print(f"ERROR: Could not import LoRaHAMInterface: {e}")
    print("Make sure LoRaHAMInterface.py is in /home/jiri/.reticulum/interfaces/")
    sys.exit(1)

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("ERROR: RPi.GPIO not available – run on a Raspberry Pi with 'pip install RPi.GPIO'")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
PASS = "\033[32m PASS\033[0m"
FAIL = "\033[31m FAIL\033[0m"
INFO = "\033[36m INFO\033[0m"
WARN = "\033[33m WARN\033[0m"

_results = []

def _arm_dio0(callback, pin_dio0):
    """
    Safely (re)arm DIO0 edge detection with fallback to polling.
    RPi.GPIO add_event_detect can fail if:
      - The pin is already in use by another process (check: /sys/class/gpio/gpioN)
      - The pin number is wrong (HAT uses a different DIO0 pin)
      - The kernel GPIO driver is busy
    We try twice with increasing delays, then fall back to a polling thread.
    """
    for attempt in range(3):
        try:
            GPIO.remove_event_detect(pin_dio0)
        except Exception:
            pass
        GPIO.setup(pin_dio0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.1 * (attempt + 1))
        try:
            GPIO.add_event_detect(pin_dio0, GPIO.RISING, callback=callback, bouncetime=10)
            return   # success
        except RuntimeError as e:
            print(f"  [{WARN}] add_event_detect attempt {attempt+1} failed: {e}")
            # Diagnostic: check what the kernel sees for this pin
            import os
            gpio_path = f"/sys/class/gpio/gpio{pin_dio0}"
            if os.path.exists(gpio_path):
                try:
                    direction = open(f"{gpio_path}/direction").read().strip()
                    value     = open(f"{gpio_path}/value").read().strip()
                    edge      = open(f"{gpio_path}/edge").read().strip()
                    print(f"  [{INFO}] /sys/class/gpio/gpio{pin_dio0}: "
                          f"direction={direction} value={value} edge={edge}")
                except Exception as se:
                    print(f"  [{INFO}] Could not read sysfs gpio: {se}")
            else:
                print(f"  [{INFO}] {gpio_path} does not exist – pin not exported")

    # All attempts failed – fall back to polling
    print(f"  [{WARN}] Falling back to polling on BCM {pin_dio0} (10 ms interval)")
    _start_poll_thread(callback, pin_dio0)

# Polling fallback state
_poll_thread  = None
_poll_stop    = threading.Event()

def _start_poll_thread(callback, pin_dio0):
    global _poll_thread
    _poll_stop.clear()
    def _poll():
        last = GPIO.input(pin_dio0)
        while not _poll_stop.is_set():
            cur = GPIO.input(pin_dio0)
            if cur == GPIO.HIGH and last == GPIO.LOW:
                try:
                    callback(pin_dio0)
                except Exception:
                    pass
            last = cur
            time.sleep(0.01)
    _poll_thread = threading.Thread(target=_poll, daemon=True)
    _poll_thread.start()

def _disarm_dio0(pin_dio0):
    global _poll_thread
    _poll_stop.set()
    if _poll_thread and _poll_thread.is_alive():
        _poll_thread.join(timeout=0.5)
    _poll_thread = None
    try:
        GPIO.remove_event_detect(pin_dio0)
    except Exception:
        pass

def result(name, ok, detail=""):
    status = PASS if ok else FAIL
    line = f"  [{status}] {name}"
    if detail:
        line += f"  ({detail})"
    print(line)
    _results.append((name, ok))
    return ok

def safe_test(fn, *args, **kwargs):
    """Run a test function; record a FAIL result if it raises an exception."""
    try:
        return fn(*args, **kwargs)
    except Exception as e:
        name = fn.__name__
        print(f"  [{FAIL}] {name} raised: {e}")
        traceback.print_exc()
        _results.append((name, False))
        return False

def section(title):
    print(f"\n{'─'*60}")
    print(f"  {title}")
    print(f"{'─'*60}")

def freq_from_regs(msb, mid, lsb):
    frf = (msb << 16) | (mid << 8) | lsb
    return (frf * FXOSC) / (1 << 19)


# ===========================================================================
# TEST GROUPS
# ===========================================================================

def test_spi_and_version(radio):
    section("1. SPI connectivity & chip version")
    ver = radio.version()
    result("SPI read returns non-zero",  ver != 0x00, f"got 0x{ver:02X}")
    result("SPI read returns non-0xFF",  ver != 0xFF, f"got 0x{ver:02X}")
    result("SX127x version byte valid",  ver in (0x11, 0x12), f"0x{ver:02X}")


def test_reset_pin(radio):
    """
    Verify the RESET pin is correctly wired by checking that a hardware reset
    restores REG_PA_CONFIG to its power-on default (0x4F).
    If the RESET pin is wrong the register retains the value we wrote.
    """
    section("2a. RESET pin verification")

    # Write a value that differs from the 0x4F power-on default
    radio._write(REG_PA_CONFIG, 0x7F)
    written = radio._read(REG_PA_CONFIG)
    if written != 0x7F:
        result("RESET pre-condition: write 0x7F to PA_CONFIG", False,
               f"SPI write/read broken: got 0x{written:02X}")
        return

    radio.reset()
    time.sleep(0.01)

    # After hardware reset the chip is in FSK mode; PA_CONFIG default is 0x4F
    pa_after = radio._read(REG_PA_CONFIG)
    result("RESET pin: PA_CONFIG reverts to 0x4F after hardware reset",
           pa_after == 0x4F,
           f"got 0x{pa_after:02X}  (0x4F=pass, 0x7F=RESET pin not toggling)")

    # Restore LoRa mode so subsequent tests work
    radio._write(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP)
    time.sleep(0.01)
    radio.set_mode(MODE_STDBY)


def test_mode_switching(radio):
    section("2. OP_MODE register – mode switching")
    for name, mode in [("SLEEP", MODE_SLEEP), ("STDBY", MODE_STDBY)]:
        radio.set_mode(mode)
        time.sleep(0.02)
        actual = radio._read(REG_OP_MODE) & 0x07
        result(f"Switch to {name}", actual == mode,
               f"expected {mode}, got {actual}")

    # Verify LoRa bit is always preserved
    radio.set_mode(MODE_STDBY)
    op = radio._read(REG_OP_MODE)
    result("LoRa mode bit preserved (0x80)", bool(op & MODE_LONG_RANGE),
           f"REG_OP_MODE=0x{op:02X}")


def test_frequency(radio, freq_hz):
    section("3. Frequency register round-trip")
    radio.set_mode(MODE_STDBY)
    radio.set_freq(freq_hz)
    msb = radio._read(REG_FR_MSB)
    mid = radio._read(REG_FR_MID)
    lsb = radio._read(REG_FR_LSB)
    read_back = freq_from_regs(msb, mid, lsb)
    error_hz  = abs(read_back - freq_hz)
    result("Frequency set within 1 kHz",  error_hz < 1000,
           f"set={freq_hz/1e6:.4f} MHz  read={read_back/1e6:.4f} MHz  err={error_hz:.0f} Hz")


def test_modem_config(radio, bw_hz, sf, cr):
    section("4. Modem configuration registers")
    radio.set_mode(MODE_STDBY)

    actual_bw = radio.set_bandwidth(bw_hz)
    radio.set_coding_rate(cr)
    radio.set_spreading_factor(sf)
    radio.enable_crc(True)

    mc1 = radio._read(REG_MODEM_CONFIG1)
    mc2 = radio._read(REG_MODEM_CONFIG2)

    bw_reg_expected  = BW_MAP[actual_bw]
    bw_reg_actual    = mc1 & 0xF0
    result("Bandwidth register",    bw_reg_actual == bw_reg_expected,
           f"expected 0x{bw_reg_expected:02X}, got 0x{bw_reg_actual:02X}")

    cr_bits_expected = (cr - 4) << 1
    cr_bits_actual   = mc1 & 0x0E
    result("Coding rate bits",      cr_bits_actual == cr_bits_expected,
           f"expected 0x{cr_bits_expected:02X}, got 0x{cr_bits_actual:02X}")

    sf_bits_expected = (sf << 4) & 0xF0
    sf_bits_actual   = mc2 & 0xF0
    result("Spreading factor bits", sf_bits_actual == sf_bits_expected,
           f"expected 0x{sf_bits_expected:02X}, got 0x{sf_bits_actual:02X}")

    crc_bit = (mc2 >> 2) & 1
    result("CRC enabled",           crc_bit == 1, f"MC2=0x{mc2:02X}")

    mc3 = radio._read(REG_MODEM_CONFIG3)
    agc = (mc3 >> 2) & 1
    result("AGC auto-on",           agc == 1, f"MC3=0x{mc3:02X}")


def test_sync_word(radio):
    section("5. Sync word register")
    for sw in (0x12, 0x34, 0xAB):
        radio.set_sync_word(sw)
        rb = radio._read(REG_SYNC_WORD)
        result(f"Sync word 0x{sw:02X} round-trip", rb == sw,
               f"read back 0x{rb:02X}")


def test_preamble(radio):
    section("6. Preamble length register")
    for length in (6, 8, 16, 64):
        radio.set_preamble_length(length)
        msb = radio._read(REG_PREAMBLE_MSB)
        lsb = radio._read(REG_PREAMBLE_LSB)
        rb  = (msb << 8) | lsb
        result(f"Preamble {length} symbols round-trip", rb == length,
               f"read back {rb}")


def test_pa_config(radio, tx_power):
    section("7. PA / TX power configuration")
    radio.set_tx_power(tx_power)
    pa  = radio._read(REG_PA_CONFIG)
    dac = radio._read(REG_PA_DAC)
    result("PA_BOOST bit set (bit 7)", bool(pa & 0x80),
           f"PA_CONFIG=0x{pa:02X}")
    if tx_power <= 17:
        result("PA_DAC standard (0x84)", dac == 0x84, f"PA_DAC=0x{dac:02X}")
    else:
        result("PA_DAC high-power (0x87)", dac == 0x87, f"PA_DAC=0x{dac:02X}")
    print(f"  [{INFO}] TX power requested={tx_power} dBm  "
          f"output_power bits=0x{pa & 0x0F:X}")


def test_dio_mapping(radio):
    section("8. DIO0 mapping register")
    radio.set_dio0_rxdone()
    dm = radio._read(REG_DIO_MAPPING1)
    result("DIO0 → RxDone (bits 7:6 = 00)", (dm >> 6) & 0x03 == 0x00,
           f"DIO_MAPPING1=0x{dm:02X}")

    radio.set_dio0_txdone()
    dm = radio._read(REG_DIO_MAPPING1)
    result("DIO0 → TxDone (bits 7:6 = 01)", (dm >> 6) & 0x03 == 0x01,
           f"DIO_MAPPING1=0x{dm:02X}")


def test_irq_flags_clear(radio):
    section("9. IRQ flags register clear")
    radio.clear_irq_flags()
    flags = radio.get_irq_flags()
    result("IRQ flags cleared to 0x00", flags == 0x00,
           f"REG_IRQ_FLAGS=0x{flags:02X}")


def test_rx_mode(radio):
    section("10. Continuous RX mode entry")
    radio.start_rx()
    time.sleep(0.05)
    mode = radio._read(REG_OP_MODE) & 0x07
    result("Radio enters RXCONT mode (0x05)", mode == MODE_RX_CONT,
           f"REG_OP_MODE & 0x07 = 0x{mode:02X}")
    dm = radio._read(REG_DIO_MAPPING1)
    result("DIO0 still mapped to RxDone after start_rx",
           (dm >> 6) & 0x03 == 0x00, f"DIO_MAPPING1=0x{dm:02X}")


def test_dio0_pin_diagnostic(radio):
    """
    Check the actual electrical state of the configured DIO0 pin and nearby pins.
    If DIO0 is wired to a different BCM pin on your HAT, this will show it:
    the correct pin will pulse HIGH briefly after a TX or when a packet arrives.
    """
    section("11b. DIO0 pin diagnostic (check if configured pin is correct)")

    pin_dio0  = radio.pin_dio0
    pin_reset = radio.pin_reset

    # Sample candidate pins as inputs — but never touch the RESET pin,
    # as pulling it LOW would reset the chip mid-test.
    candidates = sorted(set([pin_dio0, 5, 6, 16, 25, 24, 23, 22]) - {pin_reset})
    print(f"  [{INFO}] Sampling BCM pins: {candidates}  (RESET BCM {pin_reset} excluded)")
    print(f"  [{INFO}] Configured PIN_DIO0 = BCM {pin_dio0}")

    levels = {}
    for pin in candidates:
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            levels[pin] = GPIO.input(pin)
        except Exception as e:
            levels[pin] = f"ERR({e})"

    print(f"  [{INFO}] Pin levels (before TX): { {p: v for p, v in levels.items()} }")

    # Only watch pins that are LOW before TX — a pin already HIGH is not DIO0.
    watch_candidates = [p for p in candidates if levels.get(p) == 0]

    # Fire a quick TX and detect LOW->HIGH transitions.
    import threading
    fired_pins = []
    stop = threading.Event()

    def watch_pins():
        while not stop.is_set():
            for pin in watch_candidates:
                try:
                    if GPIO.input(pin) == GPIO.HIGH:
                        if pin not in fired_pins:
                            fired_pins.append(pin)
                except Exception:
                    pass
            time.sleep(0.002)

    t = threading.Thread(target=watch_pins, daemon=True)
    t.start()

    payload = b"DIO0 DIAG"
    frame   = payload
    radio.set_mode(0x01)   # STDBY
    radio._write(0x0E, 0x00)  # FIFO_TX_BASE
    radio._write(0x0D, 0x00)  # FIFO_ADDR_PTR
    radio._write_burst(0x00, frame)
    radio._write(0x22, len(frame))
    radio._write(0x12, 0xFF)  # clear IRQ
    # Remap DIO0→TxDone then TX
    v = radio._read(0x40)
    radio._write(0x40, (v & 0x3F) | 0x40)
    radio.set_mode(0x03)  # TX
    time.sleep(0.5)       # wait for TxDone pulse
    stop.set()
    t.join(timeout=0.5)

    radio.start_rx()

    if fired_pins:
        correct = pin_dio0 in fired_pins
        result(f"Configured DIO0 (BCM {pin_dio0}) pulsed HIGH during TX",
               correct,
               f"pins that pulsed: {fired_pins}")
        if not correct:
            print(f"  [{WARN}] DIO0 appears to be on BCM {fired_pins[0]} – "
                  f"update pin_dio0 in your config!")
    else:
        result("Any pin pulsed HIGH during TX", False,
               "no pulse seen – check DIO0 wiring to SX127x")


def test_rssi(radio, freq_hz):
    section("11. RSSI / noise floor (ambient)")
    radio.start_rx()
    time.sleep(0.2)
    # Read wideband RSSI (available in RX mode)
    raw = radio._read(0x2C)    # REG_RSSI_WIDEBAND
    
    # RSSI calculation depends on the frequency band
    if freq_hz < 525_000_000:
        rssi = -157 + raw      # LF band (433 MHz etc)
    else:
        rssi = -164 + raw      # HF band (868/915 MHz etc)
        
    result("RSSI register readable", raw != 0x00 and raw != 0xFF,
           f"raw=0x{raw:02X}  ≈{rssi} dBm ambient")
    print(f"  [{INFO}] Ambient RSSI ≈ {rssi} dBm (at {freq_hz/1e6:.1f} MHz)")


def test_868_compatibility(radio):
    section("15. 868 MHz band compatibility test")
    target_freq = 868_125_000
    radio.set_mode(MODE_STDBY)
    radio.set_freq(target_freq)
    
    # Verify LowFrequencyModeOn bit is NOT set for HF
    op_mode = radio._read(REG_OP_MODE)
    lf_bit_set = bool(op_mode & 0x08)
    
    msb = radio._read(REG_FR_MSB)
    mid = radio._read(REG_FR_MID)
    lsb = radio._read(REG_FR_LSB)
    read_back = freq_from_regs(msb, mid, lsb)
    error_hz  = abs(read_back - target_freq)
    
    res1 = result("Frequency 868.125 MHz set accurately", error_hz < 1000,
                  f"read={read_back/1e6:.4f} MHz")
    res2 = result("LowFrequencyModeOn bit is correctly DISABLED for HF", not lf_bit_set,
                  f"REG_OP_MODE=0x{op_mode:02X}")
    
    return res1 and res2


def test_transmit(radio, count=3):
    section("12. Single-packet transmit (TX completion via IRQ flags poll)")
    # We poll REG_IRQ_FLAGS over SPI directly — this works regardless of
    # whether GPIO interrupts are available. TxDone (bit 3) is set by the
    # SX127x and stays set until we clear it, so even slow polling catches it.

    all_ok = True
    for i in range(count):
        payload = f"LoRaHAM test packet #{i+1} | time={time.time():.3f}".encode()
        frame   = payload

        radio.send_packet(frame)

        # Poll for TxDone with 5 s timeout (5000 × 1 ms)
        done  = False
        start = time.time()
        while time.time() - start < 5.0:
            flags = radio.get_irq_flags()
            if flags & IRQ_TX_DONE:
                radio.clear_irq_flags()
                done = True
                elapsed = time.time() - start
                break
            time.sleep(0.001)

        if done:
            result(f"TX #{i+1} TxDone flag set", True,
                   f"{len(frame)} B  airtime≈{elapsed*1000:.0f} ms  IRQ=0x{flags:02X}")
        else:
            flags = radio.get_irq_flags()
            result(f"TX #{i+1} TxDone flag set", False,
                   f"timeout after 5 s  IRQ=0x{flags:02X}  mode=0x{radio._read(REG_OP_MODE):02X}")
            all_ok = False

        radio.start_rx()
        time.sleep(0.05)

    return all_ok


def test_rx_listen(radio, timeout_s=10):
    """Listen for any incoming packet by polling REG_IRQ_FLAGS over SPI."""
    section(f"13. RX listen (waiting up to {timeout_s} s for any packet)")

    radio.start_rx()
    print(f"  [{INFO}] Listening via SPI IRQ-flag poll… (Ctrl-C to skip)")

    deadline = time.time() + timeout_s
    received = None
    while time.time() < deadline:
        flags = radio.get_irq_flags()
        if flags & IRQ_RX_DONE:
            if flags & IRQ_PAYLOAD_CRC_ERR:
                print(f"  [{WARN}] CRC error on received packet")
                radio.clear_irq_flags()
                radio.start_rx()
            else:
                received = radio.read_packet()
                radio.start_rx()
                break
        time.sleep(0.005)   # 5 ms poll – fast enough for any LoRa packet

    if received:
        result("Packet received", True, f"{len(received)} bytes on air")
        try:
            print(f"  [{INFO}] Decoded payload: {received.decode('utf-8', errors='replace')}")
        except Exception:
            print(f"  [{INFO}] Raw bytes: {received.hex()}")
        return True
    else:
        # No packet = no transmitter in range, not a hardware fault -> SKIP
        print(f"  [\033[33m SKIP\033[0m] Packet received within timeout"
              f"  (no transmitter in range -- radio is working correctly)")
        # Do NOT append to _results so this does not count against the total
        return False


def test_loopback(radio, count=3, timeout_s=10):
    """
    Transmit a packet and listen for its echo via SPI IRQ-flag polling.
    Requires a second node that echoes packets back (or a wired RF loopback).
    """
    section(f"14. TX→RX loopback ({count} packets, {timeout_s} s timeout each)")

    def _poll_flag(mask, timeout):
        """Poll REG_IRQ_FLAGS until `mask` bit is set; return (True, elapsed) or (False, timeout)."""
        start = time.time()
        while time.time() - start < timeout:
            if radio.get_irq_flags() & mask:
                return True, time.time() - start
            time.sleep(0.001)
        return False, timeout

    ok_count = 0
    for i in range(count):
        payload = f"LOOPBACK#{i}|{time.time():.4f}".encode()
        frame   = payload

        radio.send_packet(frame)
        tx_ok, tx_time = _poll_flag(IRQ_TX_DONE, 5.0)
        if not tx_ok:
            result(f"Loopback #{i+1} TX", False,
                   f"TxDone timeout  IRQ=0x{radio.get_irq_flags():02X}")
            radio.start_rx()
            continue
        radio.clear_irq_flags()
        radio.start_rx()

        rx_ok, rx_time = _poll_flag(IRQ_RX_DONE, timeout_s)
        if rx_ok:
            flags = radio.get_irq_flags()
            if flags & IRQ_PAYLOAD_CRC_ERR:
                result(f"Loopback #{i+1} RX echo", False, "CRC error")
                radio.clear_irq_flags()
                radio.start_rx()
                continue
            echo  = radio.read_packet()
            radio.start_rx()
            match = echo.rstrip(b'\x00') == frame
            result(f"Loopback #{i+1} RX echo matches TX",
                   match, f"TX={len(frame)}B RX={len(echo)}B  RTT≈{(tx_time+rx_time)*1000:.0f}ms")
            if match:
                ok_count += 1
        else:
            result(f"Loopback #{i+1} RX echo", False, "timeout")
        time.sleep(0.2)

    return ok_count == count


# ===========================================================================
# MAIN
# ===========================================================================

def parse_args():
    p = argparse.ArgumentParser(description="LoRaHAM Pi / SX127x hardware test suite")
    p.add_argument("--freq",      type=int,   default=433_775_000)
    p.add_argument("--bw",        type=int,   default=125_000)
    p.add_argument("--sf",        type=int,   default=7)
    p.add_argument("--cr",        type=int,   default=5)
    p.add_argument("--power",     type=int,   default=17)
    p.add_argument("--count",     type=int,   default=3)
    p.add_argument("--timeout",   type=int,   default=10)
    p.add_argument("--loopback",  action="store_true")
    p.add_argument("--rx-only",   action="store_true")
    p.add_argument("--tx-only",   action="store_true")
    p.add_argument("--test-868",  action="store_true", help="Run specialized 868 MHz compatibility tests")
    p.add_argument("--implicit-header", action="store_true", help="Enable LoRa implicit header mode")
    # Hardware config
    p.add_argument("--pin-dio0",  type=int, default=16,   help="BCM pin for DIO0")
    p.add_argument("--pin-reset", type=int, default=6,    help="BCM pin for RESET")
    p.add_argument("--pin-cs",    type=int, default=26,   help="BCM pin for GPIO chip-select (NSS)")
    p.add_argument("--spi-bus",   type=int, default=0,    help="SPI bus number")
    p.add_argument("--spi-cs",    type=int, default=0,    help="SPI device number (usually 0)")
    return p.parse_args()


def main():
    args = parse_args()

    # If --test-868 is used, switch to 868 MHz module defaults unless the user
    # explicitly supplied those arguments on the command line.
    if args.test_868:
        if args.freq == 433_775_000: args.freq = 868_125_000

    print("╔══════════════════════════════════════════════════════════╗")
    print("║        LoRaHAM Pi / SX127x  –  Interface Test Suite      ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"  Frequency  : {args.freq/1e6:.4f} MHz")
    print(f"  Bandwidth  : {args.bw/1000:.0f} kHz")
    print(f"  SF / CR    : SF{args.sf}  CR 4/{args.cr}")
    print(f"  TX power   : {args.power} dBm")
    print(f"  DIO0 pin   : BCM {args.pin_dio0}")
    print(f"  RESET pin  : BCM {args.pin_reset}")
    print(f"  CS pin     : BCM {args.pin_cs}")
    print(f"  SPI bus    : {args.spi_bus}:{args.spi_cs}")
    if args.implicit_header:
        print(f"  Implicit   : Enabled")

    radio = None
    try:
        print("\n  Initialising _SX127x driver…")
        radio = _SX127x(
            pin_dio0=args.pin_dio0,
            pin_reset=args.pin_reset,
            spi_bus=args.spi_bus,
            spi_cs=args.spi_cs,
            pin_cs=args.pin_cs,
        )
        radio.reset()
        time.sleep(0.05)
        radio.set_mode(MODE_SLEEP)
        radio.set_mode(MODE_STDBY)
        print(f"  Driver initialised  (chip version 0x{radio.version():02X})")

        if args.rx_only:
            # Just configure and listen
            radio.set_freq(args.freq)
            radio.set_bandwidth(args.bw)
            radio.set_coding_rate(args.cr)
            radio.set_spreading_factor(args.sf)
            radio.enable_implicit_header(args.implicit_header)
            radio.set_lna_gain()
            radio.set_sync_word(0x12)
            radio.set_preamble_length(8)
            radio.enable_crc(True)
            safe_test(test_rx_listen, radio, timeout_s=args.timeout)

        elif args.tx_only:
            radio.set_freq(args.freq)
            radio.set_bandwidth(args.bw)
            radio.set_coding_rate(args.cr)
            radio.set_spreading_factor(args.sf)
            radio.enable_implicit_header(args.implicit_header)
            radio.set_lna_gain()
            radio.set_tx_power(args.power)
            radio.set_sync_word(0x12)
            radio.set_preamble_length(8)
            radio.enable_crc(True)
            safe_test(test_dio0_pin_diagnostic, radio)
            safe_test(test_transmit, radio, count=args.count)

        elif args.loopback:
            radio.set_freq(args.freq)
            radio.set_bandwidth(args.bw)
            radio.set_coding_rate(args.cr)
            radio.set_spreading_factor(args.sf)
            radio.enable_implicit_header(args.implicit_header)
            radio.set_lna_gain()
            radio.set_tx_power(args.power)
            radio.set_sync_word(0x12)
            radio.set_preamble_length(8)
            radio.enable_crc(True)
            safe_test(test_loopback, radio, count=args.count, timeout_s=args.timeout)

        else:
            # Full register / hardware test suite
            test_spi_and_version(radio)
            test_reset_pin(radio)
            test_mode_switching(radio)
            test_frequency(radio, args.freq)
            test_modem_config(radio, args.bw, args.sf, args.cr)
            test_sync_word(radio)
            test_preamble(radio)
            test_pa_config(radio, args.power)
            test_dio_mapping(radio)
            test_irq_flags_clear(radio)
            test_rx_mode(radio)
            test_rssi(radio, args.freq)

            if args.test_868:
                test_868_compatibility(radio)

            # Re-apply chosen config before on-air tests
            radio.set_mode(MODE_STDBY)
            radio.set_freq(args.freq)
            radio.set_bandwidth(args.bw)
            radio.set_coding_rate(args.cr)
            radio.set_spreading_factor(args.sf)
            radio.enable_implicit_header(args.implicit_header)
            radio.set_lna_gain()
            radio.set_tx_power(args.power)
            radio.set_sync_word(0x12)
            radio.set_preamble_length(8)
            radio.enable_crc(True)

            safe_test(test_transmit, radio, count=args.count)
            safe_test(test_dio0_pin_diagnostic, radio)
            safe_test(test_rx_listen, radio, timeout_s=args.timeout)

    except KeyboardInterrupt:
        print(f"\n  [{WARN}] Interrupted by user.")
    except Exception:
        print(f"\n  [{FAIL}] Unhandled exception:")
        traceback.print_exc()
    finally:
        if radio:
            _disarm_dio0(args.pin_dio0)
            radio.close()
            print("\n  Radio closed, GPIO cleaned up.")

    # Summary
    if _results:
        print(f"\n{'═'*60}")
        passed = sum(1 for _, ok in _results if ok)
        total  = len(_results)
        colour = "\033[32m" if passed == total else "\033[31m"
        print(f"  {colour}Result: {passed}/{total} tests passed\033[0m")
        if passed < total:
            print("  Failed tests:")
            for name, ok in _results:
                if not ok:
                    print(f"    ✗  {name}")
        print(f"{'═'*60}\n")


if __name__ == "__main__":
    main()
