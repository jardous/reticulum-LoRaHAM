##############################################################################
# LoRaHAM Pi – Reticulum Interface  (fully self-contained)
#
# All SX1276/SX1278 register access, SPI and GPIO handling is inlined here.
# No pySX127x package is required.  Only RPi.GPIO and spidev are needed.
#
# Place this file in ~/.reticulum/interfaces/
# Add to ~/.reticulum/config:
#
#   [[LoRaHAM 868]]
#     type              = LoRaHAMInterface
#     interface_enabled = yes
#     frequency         = 868100000
#     bandwidth         = 125000
#     spreading_factor  = 7
#     coding_rate       = 5
#     tx_power          = 17
#     # Optional:
#     # pin_cs          = 26
#     # pin_dio0        = 16
#     # pin_reset       = 6
#     # sync_word       = 0x12
#     # preamble_length = 8
#     # mode            = full
#
# Hardware requirements (LoRaHAM Pi HAT, BCM pin numbering):
#   SPI0 MOSI (BCM 10) → SX127x MOSI
#   SPI0 MISO (BCM 9)  → SX127x MISO
#   SPI0 SCLK (BCM 11) → SX127x SCK
#   BCM 26             → SX127x NSS  (GPIO chip-select, not hardware CE)
#   BCM 16             → SX127x DIO0  (RxDone / TxDone)
#   BCM 6              → SX127x RESET
#
# The CS line (NSS) is driven manually as a GPIO because the HAT does not
# use the hardware SPI CE pins.  spidev is opened with no_cs=True.
# Adjust pin_cs / pin_dio0 / pin_reset in config if your wiring differs.
#
# Licence: MIT  –  © 2026
##############################################################################

import time
import threading
import traceback
import struct

try:
    import RNS
    from RNS.Interfaces.Interface import Interface
    _RNS_AVAILABLE = True
except ImportError:
    _RNS_AVAILABLE = True # Placeholder for class definition
    class Interface:
        MODE_FULL = 0
        MODE_POINT_TO_POINT = 1
        MODE_ACCESS_POINT = 2
        MODE_ROAMING = 3
        MODE_BOUNDARY = 4
        MODE_GATEWAY = 5
        def __init__(self, *args, **kwargs): pass
    _RNS_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    import spidev
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False


##############################################################################
# SX127x register map (LoRa mode)
##############################################################################
REG_FIFO                = 0x00
REG_OP_MODE             = 0x01
REG_FR_MSB              = 0x06
REG_FR_MID              = 0x07
REG_FR_LSB              = 0x08
REG_PA_CONFIG           = 0x09
REG_OCP                 = 0x0B
REG_FIFO_ADDR_PTR       = 0x0D
REG_FIFO_TX_BASE_ADDR   = 0x0E
REG_FIFO_RX_BASE_ADDR   = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS_MASK      = 0x11
REG_IRQ_FLAGS           = 0x12
REG_RX_NB_BYTES         = 0x13
REG_PKT_SNR_VALUE       = 0x19
REG_PKT_RSSI_VALUE      = 0x1A
REG_MODEM_CONFIG1       = 0x1D
REG_MODEM_CONFIG2       = 0x1E
REG_PREAMBLE_MSB        = 0x20
REG_PREAMBLE_LSB        = 0x21
REG_PAYLOAD_LENGTH      = 0x22
REG_MODEM_CONFIG3       = 0x26
REG_FREQ_ERROR_MSB      = 0x28
REG_RSSI_WIDEBAND       = 0x2C
REG_DETECTION_OPTIMIZE  = 0x31
REG_DETECTION_THRESHOLD = 0x37
REG_SYNC_WORD           = 0x39
REG_DIO_MAPPING1        = 0x40
REG_VERSION             = 0x42
REG_PA_DAC              = 0x4D

# REG_OP_MODE bits
MODE_LONG_RANGE = 0x80   # LoRa mode flag
MODE_LF_BAND    = 0x08   # Low Frequency Mode (bit 3) for bands < 525 MHz
MODE_SLEEP      = 0x00
MODE_STDBY      = 0x01
MODE_TX         = 0x03
MODE_RX_CONT    = 0x05

# IRQ flag bits
IRQ_RX_DONE         = 0x40
IRQ_TX_DONE         = 0x08
IRQ_PAYLOAD_CRC_ERR = 0x20
IRQ_VALID_HEADER    = 0x10

# Bandwidth register values (index → Hz)
BW_MAP = {
    7800:   0x00,
    10400:  0x10,
    15600:  0x20,
    20800:  0x30,
    31250:  0x40,
    41700:  0x50,
    62500:  0x60,
    125000: 0x70,
    250000: 0x80,
    500000: 0x90,
}

# On-air framing
# Raw Reticulum bytes are sent directly — no prefix header — identical to
# RNodeInterface. This ensures interoperability with RNode hardware.
# HW_MTU is set to MAX_LORA_PKT so Reticulum never hands us a packet that
# exceeds the SX127x FIFO.
MAX_LORA_PKT  = 255          # SX127x hardware FIFO limit
TX_GUARD_TIME = 0.05
FXOSC         = 32_000_000.0   # SX127x crystal frequency Hz


##############################################################################
# Low-level SX127x driver
##############################################################################
class _SX127x:
    """
    Bare-metal SX1276/SX1278 driver over SPI0 + RPi.GPIO.
    All register names match the Semtech SX1276 datasheet.
    """

    def __init__(self, pin_dio0, pin_reset, spi_bus=0, spi_cs=0, spi_speed=5_000_000,
                 pin_cs=None):
        self.pin_dio0  = pin_dio0
        self.pin_reset = pin_reset
        self._pin_cs   = pin_cs
        self._lock = threading.RLock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin_reset, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.pin_dio0,  GPIO.IN,  pull_up_down=GPIO.PUD_DOWN)
        if self._pin_cs is not None:
            GPIO.setup(self._pin_cs, GPIO.OUT, initial=GPIO.HIGH)

        self._spi = spidev.SpiDev()
        self._spi.open(spi_bus, spi_cs)
        self._spi.max_speed_hz = spi_speed
        self._spi.mode = 0b00
        if self._pin_cs is not None:
            self._spi.no_cs = True
        self._implicit_header = False
        self._mode_base = MODE_LONG_RANGE

    def _cs_low(self):
        if self._pin_cs is not None:
            GPIO.output(self._pin_cs, GPIO.LOW)

    def _cs_high(self):
        if self._pin_cs is not None:
            GPIO.output(self._pin_cs, GPIO.HIGH)

    # ---- SPI primitives --------------------------------------------------

    def _read(self, reg):
        self._cs_low()
        result = self._spi.xfer2([reg & 0x7F, 0x00])[1]
        self._cs_high()
        return result

    def _write(self, reg, val):
        self._cs_low()
        self._spi.xfer2([reg | 0x80, val & 0xFF])
        self._cs_high()

    def _read_burst(self, reg, length):
        self._cs_low()
        result = self._spi.xfer2([reg & 0x7F] + [0x00] * length)[1:]
        self._cs_high()
        return result

    def _write_burst(self, reg, data):
        self._cs_low()
        self._spi.xfer2([reg | 0x80] + list(data))
        self._cs_high()

    # ---- Hardware reset --------------------------------------------------

    def reset(self):
        with self._lock:
            GPIO.output(self.pin_reset, GPIO.LOW)
            time.sleep(0.01)
            GPIO.output(self.pin_reset, GPIO.HIGH)
            time.sleep(0.01)

    # ---- Mode control ----------------------------------------------------

    def set_mode(self, mode):
        """Set operating mode, preserving the LoRa and LF bits."""
        with self._lock:
            self._write(REG_OP_MODE, self._mode_base | mode)
            # Wait until mode is accepted (up to 10 ms)
            deadline = time.time() + 0.01
            while time.time() < deadline:
                if (self._read(REG_OP_MODE) & 0x07) == mode:
                    return
                time.sleep(0.001)

    def get_mode(self):
        with self._lock:
            return self._read(REG_OP_MODE) & 0x07

    # ---- Frequency -------------------------------------------------------

    def set_freq(self, freq_hz):
        with self._lock:
            frf = int((freq_hz / FXOSC) * (1 << 19))
            self._write(REG_FR_MSB, (frf >> 16) & 0xFF)
            self._write(REG_FR_MID, (frf >>  8) & 0xFF)
            self._write(REG_FR_LSB, (frf      ) & 0xFF)
            
            # SX1276 requires LowFrequencyModeOn for bands < 525 MHz
            if freq_hz < 525_000_000:
                self._mode_base |= MODE_LF_BAND
            else:
                self._mode_base &= ~MODE_LF_BAND

    # ---- Modem config ----------------------------------------------------

    def set_bandwidth(self, bw_hz):
        with self._lock:
            bw_key = min(BW_MAP.keys(), key=lambda k: abs(k - bw_hz))
            bw_reg = BW_MAP[bw_key]
            mc1 = self._read(REG_MODEM_CONFIG1)
            self._write(REG_MODEM_CONFIG1, (mc1 & 0x0F) | bw_reg)
            return bw_key   # return actual BW used

    def set_coding_rate(self, denominator):
        """denominator: 5=4/5, 6=4/6, 7=4/7, 8=4/8"""
        with self._lock:
            cr = max(5, min(8, denominator)) - 4   # 1..4
            mc1 = self._read(REG_MODEM_CONFIG1)
            self._write(REG_MODEM_CONFIG1, (mc1 & 0xF1) | (cr << 1))

    def set_spreading_factor(self, sf):
        with self._lock:
            sf = max(6, min(12, sf))
            if sf == 6:
                self._write(REG_DETECTION_OPTIMIZE,  0xC5)
                self._write(REG_DETECTION_THRESHOLD, 0x0C)
            else:
                self._write(REG_DETECTION_OPTIMIZE,  0xC3)
                self._write(REG_DETECTION_THRESHOLD, 0x0A)
            mc2 = self._read(REG_MODEM_CONFIG2)
            self._write(REG_MODEM_CONFIG2, (mc2 & 0x0F) | ((sf << 4) & 0xF0))

    def set_lna_gain(self):
        """Maximum LNA gain + AGC on."""
        with self._lock:
            mc3 = self._read(REG_MODEM_CONFIG3)
            self._write(REG_MODEM_CONFIG3, mc3 | 0x04)   # AgcAutoOn

    def set_tx_power(self, dbm):
        """
        Use PA_BOOST (up to 20 dBm).  LoRaHAM Pi has an external PA so the
        SX127x output_power register sets the drive level into that PA.
        """
        with self._lock:
            dbm = max(2, min(20, dbm))
            if dbm <= 17:
                self._write(REG_PA_DAC, 0x84)   # default PA_DAC
                self._write(REG_PA_CONFIG, 0x80 | (dbm - 2))
            else:
                self._write(REG_PA_DAC, 0x87)   # high-power mode (+20 dBm)
                self._write(REG_PA_CONFIG, 0x80 | (dbm - 5))

    def set_sync_word(self, sw):
        with self._lock:
            self._write(REG_SYNC_WORD, sw & 0xFF)

    def set_preamble_length(self, length):
        with self._lock:
            self._write(REG_PREAMBLE_MSB, (length >> 8) & 0xFF)
            self._write(REG_PREAMBLE_LSB, (length     ) & 0xFF)

    def enable_crc(self, enabled=True):
        with self._lock:
            mc2 = self._read(REG_MODEM_CONFIG2)
            if enabled:
                self._write(REG_MODEM_CONFIG2, mc2 | 0x04)
            else:
                self._write(REG_MODEM_CONFIG2, mc2 & ~0x04)

    def enable_implicit_header(self, enabled=True):
        """Set implicit header mode; payload length is fixed."""
        with self._lock:
            self._implicit_header = enabled
            mc1 = self._read(REG_MODEM_CONFIG1)
            if enabled:
                self._write(REG_MODEM_CONFIG1, mc1 | 0x01)
            else:
                self._write(REG_MODEM_CONFIG1, mc1 & ~0x01)

    # ---- DIO mapping -----------------------------------------------------

    def set_dio0_rxdone(self):
        """DIO0 → RxDone (00 in bits 7:6 of DIO_MAPPING1)."""
        with self._lock:
            v = self._read(REG_DIO_MAPPING1)
            self._write(REG_DIO_MAPPING1, v & 0x3F)

    def set_dio0_txdone(self):
        """DIO0 → TxDone (01 in bits 7:6 of DIO_MAPPING1)."""
        with self._lock:
            v = self._read(REG_DIO_MAPPING1)
            self._write(REG_DIO_MAPPING1, (v & 0x3F) | 0x40)

    # ---- FIFO / packet ---------------------------------------------------

    def start_rx(self, clear_irqs=True):
        with self._lock:
            if self._implicit_header:
                self._write(REG_PAYLOAD_LENGTH, MAX_LORA_PKT)
            self._write(REG_FIFO_RX_BASE_ADDR, 0x00)
            self._write(REG_FIFO_ADDR_PTR,     0x00)
            if clear_irqs:
                self._write(REG_IRQ_FLAGS, 0xFF)   # clear all IRQs
            self.set_dio0_rxdone()
            self._write(REG_OP_MODE, self._mode_base | MODE_RX_CONT)

    def read_packet(self):
        """Read the last received packet bytes from the FIFO."""
        with self._lock:
            # Move FIFO pointer to start of received packet
            ptr = self._read(REG_FIFO_RX_CURRENT_ADDR)
            self._write(REG_FIFO_ADDR_PTR, ptr)

            if self._implicit_header:
                n = MAX_LORA_PKT
            else:
                n = self._read(REG_RX_NB_BYTES)

            data = bytes(self._read_burst(REG_FIFO, n))
            self._write(REG_IRQ_FLAGS, IRQ_RX_DONE | IRQ_VALID_HEADER | IRQ_PAYLOAD_CRC_ERR)
            return data

    def send_packet(self, data: bytes):
        """Transmit a packet."""
        with self._lock:
            self._write(REG_OP_MODE, self._mode_base | MODE_STDBY)
            self._write(REG_FIFO_TX_BASE_ADDR, 0x00)
            self._write(REG_FIFO_ADDR_PTR,     0x00)

            if self._implicit_header:
                payload_len = MAX_LORA_PKT
                data = data.ljust(MAX_LORA_PKT, b'\x00')
            else:
                payload_len = len(data)

            self._write_burst(REG_FIFO, data)
            self._write(REG_PAYLOAD_LENGTH, payload_len)
            self._write(REG_IRQ_FLAGS, 0xFF)
            v = self._read(REG_DIO_MAPPING1)
            self._write(REG_DIO_MAPPING1, (v & 0x3F) | 0x40)
            self._write(REG_OP_MODE, self._mode_base | MODE_TX)

    def get_irq_flags(self):
        with self._lock:
            return self._read(REG_IRQ_FLAGS)

    def clear_irq_flags(self, mask=0xFF):
        with self._lock:
            self._write(REG_IRQ_FLAGS, mask)

    def version(self):
        with self._lock:
            return self._read(REG_VERSION)

    def close(self):
        with self._lock:
            try:
                self._write(REG_OP_MODE, self._mode_base | MODE_SLEEP)
            except Exception:
                pass
            try:
                self._spi.close()
            except Exception:
                pass
            try:
                GPIO.cleanup([self.pin_dio0, self.pin_reset])
            except Exception:
                pass


##############################################################################
# Reticulum Interface
##############################################################################
class LoRaHAMInterface(Interface):

    # 8-byte IFAC (same as RNodeInterface – LoRa phy, small frames)
    DEFAULT_IFAC_SIZE = 8

    def __init__(self, owner, configuration):
        super().__init__()

        if not _HW_AVAILABLE:
            raise ImportError("RPi.GPIO and spidev are required: pip install RPi.GPIO spidev")

        self.name  = configuration.get("name", "LoRaHAMInterface")
        self.owner = owner

        # Radio parameters
        self.frequency        = int(configuration.get("frequency",        433_775_000))
        self.bandwidth        = int(configuration.get("bandwidth",        125_000))
        self.spreading_factor = int(configuration.get("spreading_factor", 7))
        self.coding_rate      = int(configuration.get("coding_rate",      5))
        self.tx_power         = int(configuration.get("tx_power",         17))
        
        raw_sw = configuration.get("sync_word", "0x12")
        if isinstance(raw_sw, str):
            self.sync_word = int(raw_sw, 16)
        else:
            self.sync_word = int(raw_sw)

        self.preamble_length  = int(configuration.get("preamble_length",  8))
        self.implicit_header  = configuration.get("implicit_header", False)

        # Hardware pin/bus config
        self.pin_dio0         = int(configuration.get("pin_dio0",  16))
        self.pin_reset        = int(configuration.get("pin_reset",  6))
        self.spi_bus          = int(configuration.get("spi_bus",    0))
        self.spi_cs           = int(configuration.get("spi_cs",     0))
        pin_cs_raw = configuration.get("pin_cs", 26)
        self.pin_cs           = int(pin_cs_raw) if pin_cs_raw not in (None, "none", "None", "") else None

        # Reticulum interface mode
        mode_str = configuration.get("mode", "full").lower().strip()
        mode_map = {
            "full":           Interface.MODE_FULL,
            "point_to_point": Interface.MODE_POINT_TO_POINT,
            "access_point":   Interface.MODE_ACCESS_POINT,
            "roaming":        Interface.MODE_ROAMING,
            "boundary":       Interface.MODE_BOUNDARY,
            "gateway":        Interface.MODE_GATEWAY,
        }
        self.mode = mode_map.get(mode_str, Interface.MODE_FULL)

        self._radio         = None
        self._tx_lock       = threading.Lock()
        self._tx_done_event = threading.Event()
        self._poll_stop     = threading.Event()
        self._poll_thread   = None

        self._start()

    # ------------------------------------------------------------------

    def _start(self):
        try:
            self._radio = _SX127x(
                pin_dio0=self.pin_dio0,
                pin_reset=self.pin_reset,
                spi_bus=self.spi_bus,
                spi_cs=self.spi_cs,
                pin_cs=self.pin_cs,
            )
            self._radio.reset()

            ver = self._radio.version()
            if ver not in (0x11, 0x12):
                raise RuntimeError(
                    f"Unexpected SX127x version byte 0x{ver:02X} – "
                    "check SPI wiring and that the LoRaHAM Pi HAT is seated correctly."
                )

            # Must be in SLEEP to switch to LoRa mode
            self._radio.set_mode(MODE_SLEEP)
            self._radio.set_mode(MODE_STDBY)

            self._radio.set_freq(self.frequency)
            actual_bw = self._radio.set_bandwidth(self.bandwidth)
            self._radio.set_coding_rate(self.coding_rate)
            self._radio.set_spreading_factor(self.spreading_factor)
            self._radio.set_lna_gain()
            self._radio.set_tx_power(self.tx_power)
            self._radio.set_sync_word(self.sync_word)
            self._radio.set_preamble_length(self.preamble_length)
            self._radio.enable_crc(True)
            self._radio.enable_implicit_header(self.implicit_header)

            self.bitrate = self._calc_bitrate(actual_bw)
            self.HW_MTU  = MAX_LORA_PKT   # raw frames, no fragmentation

            # Start the IRQ polling thread.
            # We poll REG_IRQ_FLAGS over SPI rather than relying on GPIO
            # edge detection, which can fail on some kernels/HATs.
            self._poll_stop = threading.Event()
            self._poll_thread = threading.Thread(
                target=self._irq_poll_loop, daemon=True)
            self._poll_thread.start()

            self._radio.start_rx()
            self.online = True

            if _RNS_AVAILABLE:
                RNS.log(
                    f"[{self}] LoRaHAM interface online – "
                    f"freq={self.frequency/1e6:.3f} MHz  "
                    f"bw={actual_bw/1000:.0f}kHz  "
                    f"sf={self.spreading_factor}  "
                    f"cr=4/{self.coding_rate}  "
                    f"txp={self.tx_power}dBm",
                    RNS.LOG_NOTICE,
                )
                RNS.log(
                    f"[{self}] Hardware: "
                    f"dio0={self.pin_dio0}  "
                    f"reset={self.pin_reset}  "
                    f"spi={self.spi_bus}:{self.spi_cs}  "
                    f"chip v0x{ver:02X}",
                    RNS.LOG_VERBOSE,
                )

        except Exception:
            if _RNS_AVAILABLE:
                RNS.log(
                    f"[{self}] Failed to initialise LoRaHAM hardware:\n" +
                    traceback.format_exc(),
                    RNS.LOG_ERROR,
                )
            self.online = False
            raise

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # IRQ polling loop  (runs in daemon thread, replaces GPIO interrupt)
    # ------------------------------------------------------------------

    def _irq_poll_loop(self):
        """
        Poll REG_IRQ_FLAGS every 2 ms.  Avoids reliance on RPi.GPIO edge
        detection which fails on some kernels and HAT configurations.
        """
        while not self._poll_stop.is_set():
            try:
                flags = self._radio.get_irq_flags()

                if flags & IRQ_TX_DONE:
                    self._radio.clear_irq_flags(IRQ_TX_DONE)
                    self._tx_done_event.set()
                    self._radio.start_rx(clear_irqs=False)

                if flags & IRQ_RX_DONE:
                    if flags & IRQ_PAYLOAD_CRC_ERR:
                        RNS.log(f'[{self}] RX CRC error (IRQ=0x{flags:02X})', RNS.LOG_WARNING)
                        self._radio.clear_irq_flags(IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERR)
                        self._radio.start_rx(clear_irqs=False)
                    else:
                        raw = self._radio.read_packet()
                        RNS.log(f'[{self}] RX {len(raw)} B  flag=0x{raw[0]:02X}  IRQ=0x{flags:02X}', RNS.LOG_DEBUG)
                        self._radio.start_rx(clear_irqs=False)
                        # Process received packet (fragment reassembly is handled here)
                        self._receive_raw(raw)

            except Exception:
                if not self._poll_stop.is_set():
                    if _RNS_AVAILABLE:
                        RNS.log(
                            f'[{self}] Exception in IRQ poll loop:\n' +
                            traceback.format_exc(),
                            RNS.LOG_ERROR,
                        )
            time.sleep(0.002)   # 2 ms – fast enough, low CPU (~0.1%)

    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Receive
    # ------------------------------------------------------------------

    def _receive_raw(self, raw: bytes):
        if len(raw) < 1:
            RNS.log(f"[{self}] RX empty frame, discarding", RNS.LOG_DEBUG)
            return
        self.process_incoming(raw)

    # ------------------------------------------------------------------
    # Transmit
    # ------------------------------------------------------------------

    def _tx_frame(self, frame: bytes):
        """Transmit a single on-air frame and block until TxDone (5 s watchdog)."""
        self._tx_done_event.clear()
        self._radio.send_packet(frame)
        if not self._tx_done_event.wait(timeout=5.0):
            RNS.log(f"[{self}] TX timeout – forcing back to RX", RNS.LOG_WARNING)
            self._radio.start_rx()
        time.sleep(TX_GUARD_TIME)

    def process_outgoing(self, data: bytes):
        if not self.online or self.detached:
            return

        pkt_len = len(data)

        with self._tx_lock:
            try:
                if pkt_len > MAX_LORA_PKT:
                    RNS.log(
                        f"[{self}] TX packet too large ({pkt_len} B > {MAX_LORA_PKT} B) – dropping.",
                        RNS.LOG_ERROR,
                    )
                    return

                self._tx_frame(data)
                RNS.log(f"[{self}] TX {pkt_len} B", RNS.LOG_DEBUG)
                self.txb += pkt_len

            except Exception:
                RNS.log(
                    f"[{self}] TX exception:\n" + traceback.format_exc(),
                    RNS.LOG_ERROR,
                )
                try:
                    self._radio.start_rx()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def detach(self):
        self.detached = True
        self.online   = False
        try:
            self._poll_stop.set()
            if self._poll_thread:
                self._poll_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._radio:
                self._radio.close()
        except Exception:
            pass

    # ------------------------------------------------------------------

    def _calc_bitrate(self, bw_hz: int) -> int:
        sf = self.spreading_factor
        cr = self.coding_rate
        return max(1, int(sf * (4.0 / cr) * bw_hz / (2 ** sf)))

    def process_incoming(self, data: bytes):
        self.rxb += len(data)
        self.owner.inbound(data, self)

    def __str__(self):
        return f"LoRaHAMInterface[{self.name}]"


##############################################################################
# Reticulum loader entry-point
##############################################################################

def get_interfaces(owner, configuration):
    return [LoRaHAMInterface(owner, configuration)]

interface_class = LoRaHAMInterface
