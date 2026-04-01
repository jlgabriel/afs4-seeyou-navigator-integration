#!/usr/bin/env python3
# Copyright (c) 2026 Juan Luis Gabriel
# Aerofly FS4 to SeeYou Navigator Bridge
# This software is released under the MIT License.

"""
Bridge between Aerofly FS4 and SeeYou Navigator via serial COM port.

This program:
1. Receives UDP data from Aerofly FS4 flight simulator (port 49002)
2. Optionally reads high-quality data from AeroflyReader DLL (shared memory)
3. Converts the data to NMEA format (GPGGA, GPRMC, LXWP0)
4. Sends NMEA sentences via serial port to Naviter dongle → SeeYou Navigator

The NMEA output matches what Condor 3 sends to the Naviter dongle,
ensuring full compatibility with SeeYou Navigator.
"""

import socket
import threading
import struct
import math
import mmap
import re
import time
import datetime
import argparse
import sys
import tkinter as tk
from tkinter import ttk
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

VERSION = "1.0.0"

# Default configuration
DEFAULT_UDP_PORT = 49002          # Aerofly FS4 default UDP port
DEFAULT_COM_PORT = "COM7"         # Default serial port (Naviter dongle)
DEFAULT_BAUDRATE = 4800           # NMEA standard baud rate (same as Condor 3)
DEFAULT_UPDATE_RATE = 1           # 1 Hz (matching Condor 3)
DEFAULT_MAGNETIC_VARIATION = 0.0  # Magnetic variation in degrees (East positive)
DEFAULT_DEBUG_LEVEL = 1           # Debug level: 0=minimal, 1=normal, 2=verbose

# ── Data classes ─────────────────────────────────────────────────────

@dataclass
class GPSData:
    """Store GPS data received from Aerofly FS4 (UDP or DLL)."""
    longitude: float = 0.0
    latitude: float = 0.0
    altitude: float = 0.0              # MSL altitude in meters
    track: float = 0.0
    ground_speed: float = 0.0          # m/s
    timestamp: float = 0.0
    vertical_speed: Optional[float] = None
    barometric_altitude: Optional[float] = None
    indicated_airspeed: Optional[float] = None   # m/s (DLL only)
    wind_x: Optional[float] = None               # m/s North component (after ECEF→ENU)
    wind_y: Optional[float] = None               # m/s (unused vertical)
    wind_z: Optional[float] = None               # m/s East component (after ECEF→ENU)

@dataclass
class AttitudeData:
    """Store attitude data received from Aerofly FS4."""
    true_heading: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    timestamp: float = 0.0

# ── Vario Calculator ─────────────────────────────────────────────────

class VarioCalculator:
    """
    Calculates vertical speed (vario) from altitude changes over time.
    Uses a moving average for smoothing and tracks a longer-term average.
    """
    SMOOTH_WINDOW = 5
    AVG_WINDOW_SEC = 30.0

    def __init__(self):
        self._prev_altitude: Optional[float] = None
        self._prev_timestamp: float = 0.0
        self._smooth_buffer: deque = deque(maxlen=self.SMOOTH_WINDOW)
        self._avg_buffer: deque = deque()
        self.vario: float = 0.0
        self.average_vario: float = 0.0

    def update(self, altitude: float, timestamp: float) -> None:
        if self._prev_altitude is not None and timestamp > self._prev_timestamp:
            dt = timestamp - self._prev_timestamp
            if 0.01 < dt < 2.0:
                raw_vario = (altitude - self._prev_altitude) / dt
                # Discard unrealistic spikes (> 50 m/s is impossible in a glider)
                if abs(raw_vario) > 50.0:
                    self._prev_altitude = altitude
                    self._prev_timestamp = timestamp
                    return
                self._smooth_buffer.append(raw_vario)
                self.vario = sum(self._smooth_buffer) / len(self._smooth_buffer)
                self._avg_buffer.append((timestamp, self.vario))
                cutoff = timestamp - self.AVG_WINDOW_SEC
                while self._avg_buffer and self._avg_buffer[0][0] < cutoff:
                    self._avg_buffer.popleft()
                if self._avg_buffer:
                    self.average_vario = sum(v for _, v in self._avg_buffer) / len(self._avg_buffer)
        self._prev_altitude = altitude
        self._prev_timestamp = timestamp

    @property
    def is_valid(self) -> bool:
        return len(self._smooth_buffer) >= 2

# ── Aerofly UDP Receiver ─────────────────────────────────────────────

class AeroflyReceiver:
    """Receives and parses UDP data from Aerofly FS4."""

    def __init__(self, port: int = DEFAULT_UDP_PORT, debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.port = port
        self.socket = None
        self.gps_data = GPSData()
        self.attitude_data = AttitudeData()
        self.running = False
        self.receive_thread = None
        self.last_receive_time = 0
        self.debug_level = debug_level

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.settimeout(0.5)
        self.socket.bind(('', self.port))
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        print(f"Started UDP receiver on port {self.port}")

    def _receive_loop(self):
        while self.running:
            try:
                data, _ = self.socket.recvfrom(1024)
                self.last_receive_time = time.time()
                message = data.decode('utf-8')
                if message.startswith('XGPS'):
                    gps_data = self._parse_gps_message(message)
                    if gps_data:
                        self.gps_data = gps_data
                elif message.startswith('XATT'):
                    attitude_data = self._parse_attitude_message(message)
                    if attitude_data:
                        self.attitude_data = attitude_data
            except socket.timeout:
                pass
            except Exception as e:
                print(f"Error receiving UDP data: {e}")

    def _parse_gps_message(self, message: str) -> Optional[GPSData]:
        pattern = r'XGPSAerofly FS 4,([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)'
        match = re.match(pattern, message)
        if match:
            lon, lat, alt, trk, spd = map(float, match.groups())
            return GPSData(
                longitude=lon, latitude=lat, altitude=alt,
                track=trk, ground_speed=spd, timestamp=time.time()
            )
        return None

    def _parse_attitude_message(self, message: str) -> Optional[AttitudeData]:
        pattern = r'XATTAerofly FS 4,([-\d.]+),([-\d.]+),([-\d.]+)'
        match = re.match(pattern, message)
        if match:
            heading, pitch, roll = map(float, match.groups())
            return AttitudeData(
                true_heading=heading, pitch=pitch, roll=roll,
                timestamp=time.time()
            )
        return None

    def is_connected(self, timeout=5.0):
        return (time.time() - self.last_receive_time) < timeout

    def stop(self):
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(1.0)
        if self.socket:
            self.socket.close()

# ── DLL Shared Memory Reader ─────────────────────────────────────────

class DLLReader:
    """
    Reads flight data from AeroflyReader DLL via Windows Shared Memory.
    Provides high-quality data at 50-60Hz including IAS, direct vario, and wind.
    """
    MAPPING_NAME = "AeroflyReaderData"
    POLL_INTERVAL = 0.02
    RECONNECT_INTERVAL = 3.0
    STALE_TIMEOUT = 2.0

    _OFF_TIMESTAMP = 0
    _OFF_DATA_VALID = 8
    _OFF_UPDATE_COUNTER = 12
    _OFF_LATITUDE = 16
    _OFF_LONGITUDE = 24
    _OFF_ALTITUDE = 32
    _OFF_HEIGHT = 40
    _OFF_PITCH = 48
    _OFF_BANK = 56
    _OFF_TRUE_HEADING = 64
    _OFF_MAGNETIC_HEADING = 72
    _OFF_IAS = 80
    _OFF_GROUND_SPEED = 88
    _OFF_VERTICAL_SPEED = 96
    _OFF_WIND_X = 192
    _OFF_WIND_Y = 200
    _OFF_WIND_Z = 208
    _MIN_SIZE = 216

    def __init__(self, debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.debug_level = debug_level
        self._mmap: Optional[mmap.mmap] = None
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_counter = 0
        self._last_valid_time = 0.0
        self.gps_data = GPSData()
        self.attitude_data = AttitudeData()

    def start(self):
        if sys.platform != 'win32':
            print("DLL shared memory is only available on Windows.")
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("DLL reader started, looking for AeroflyReader shared memory...")

    def _try_connect(self) -> bool:
        """Open existing shared memory (without creating a new one).
        Uses ctypes OpenFileMappingW to avoid mmap(-1,...) which creates
        an empty mapping if it doesn't exist yet, shadowing the DLL's real one.
        """
        try:
            import ctypes
            from ctypes import wintypes
            kernel32 = ctypes.windll.kernel32
            FILE_MAP_READ = 0x0004
            handle = kernel32.OpenFileMappingW(FILE_MAP_READ, False, self.MAPPING_NAME)
            if not handle:
                return False
            # Now use mmap with the validated handle — the mapping already exists
            kernel32.CloseHandle(handle)
            self._mmap = mmap.mmap(-1, 1024, self.MAPPING_NAME, access=mmap.ACCESS_READ)
            self._connected = True
            if self.debug_level >= 1:
                print(f"Connected to DLL shared memory: {self.MAPPING_NAME}")
            return True
        except Exception:
            self._cleanup()
            return False

    def _cleanup(self):
        if self._mmap:
            try:
                self._mmap.close()
            except Exception:
                pass
        self._mmap = None
        self._connected = False

    def _read_double(self, offset: int) -> float:
        self._mmap.seek(offset)
        return struct.unpack('d', self._mmap.read(8))[0]

    def _read_uint32(self, offset: int) -> int:
        self._mmap.seek(offset)
        return struct.unpack('I', self._mmap.read(4))[0]

    def _read_loop(self):
        last_reconnect = 0.0
        was_connected = False
        while self._running:
            if not self._connected:
                now = time.time()
                if now - last_reconnect >= self.RECONNECT_INTERVAL:
                    last_reconnect = now
                    if self._try_connect() and not was_connected:
                        was_connected = True
                else:
                    time.sleep(0.1)
                    continue
            try:
                data_valid = self._read_uint32(self._OFF_DATA_VALID)
                counter = self._read_uint32(self._OFF_UPDATE_COUNTER)
                if data_valid > 0 and counter != self._last_counter:
                    self._last_counter = counter
                    self._last_valid_time = time.time()
                    self._update_data()
                elif time.time() - self._last_valid_time > self.STALE_TIMEOUT and self._last_valid_time > 0:
                    if self.debug_level >= 1:
                        print("DLL data stale, reconnecting...")
                    self._cleanup()
                    was_connected = False
            except Exception as e:
                if self.debug_level >= 1:
                    print(f"DLL read error: {e}")
                self._cleanup()
                was_connected = False
            time.sleep(self.POLL_INTERVAL)

    def _update_data(self):
        now = time.time()
        lat_rad = self._read_double(self._OFF_LATITUDE)
        lon_rad = self._read_double(self._OFF_LONGITUDE)
        alt_m = self._read_double(self._OFF_ALTITUDE)
        ias_ms = self._read_double(self._OFF_IAS)
        gs_ms = self._read_double(self._OFF_GROUND_SPEED)
        vs_ms = self._read_double(self._OFF_VERTICAL_SPEED)
        true_hdg_rad = self._read_double(self._OFF_TRUE_HEADING)
        mag_hdg_rad = self._read_double(self._OFF_MAGNETIC_HEADING)
        pitch_rad = self._read_double(self._OFF_PITCH)
        bank_rad = self._read_double(self._OFF_BANK)
        wind_ecef_x = self._read_double(self._OFF_WIND_X)
        wind_ecef_y = self._read_double(self._OFF_WIND_Y)
        wind_ecef_z = self._read_double(self._OFF_WIND_Z)

        lat_deg = math.degrees(lat_rad)
        lon_deg = math.degrees(lon_rad)
        if lon_deg > 180.0:
            lon_deg -= 360.0

        true_hdg_deg = (90.0 - math.degrees(true_hdg_rad)) % 360
        pitch_deg = math.degrees(pitch_rad)
        bank_deg = math.degrees(bank_rad)

        # ECEF to local ENU wind conversion
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)
        wind_east = -sin_lon * wind_ecef_x + cos_lon * wind_ecef_y
        wind_north = (-sin_lat * cos_lon * wind_ecef_x
                      - sin_lat * sin_lon * wind_ecef_y
                      + cos_lat * wind_ecef_z)

        track = true_hdg_deg

        self.gps_data = GPSData(
            longitude=lon_deg, latitude=lat_deg, altitude=alt_m,
            track=track, ground_speed=gs_ms, timestamp=now,
            vertical_speed=vs_ms, barometric_altitude=alt_m,
            indicated_airspeed=ias_ms,
            wind_x=wind_north, wind_y=0.0, wind_z=wind_east,
        )
        self.attitude_data = AttitudeData(
            true_heading=true_hdg_deg, pitch=pitch_deg,
            roll=bank_deg, timestamp=now,
        )

    def is_connected(self, timeout=2.0) -> bool:
        if not self._connected:
            return False
        return (time.time() - self._last_valid_time) < timeout

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(1.0)
        self._cleanup()

# ── IAS to TAS ────────────────────────────────────────────────────────

def ias_to_tas(ias_ms: float, altitude_m: float) -> float:
    """Convert IAS to TAS using ISA standard atmosphere model."""
    T0 = 288.15
    L = 0.0065
    g = 9.80665
    R = 287.058
    h = max(0.0, min(altitude_m, 11000.0))
    T = T0 - L * h
    if T <= 0:
        return ias_ms
    exponent = g / (R * L) - 1.0
    density_ratio = (T / T0) ** exponent
    if density_ratio <= 0:
        return ias_ms
    return ias_ms / math.sqrt(density_ratio)

# ── NMEA Converter ────────────────────────────────────────────────────

class NMEAConverter:
    """Converts Aerofly data to NMEA sentences for SeeYou Navigator."""

    def __init__(self, magnetic_variation=DEFAULT_MAGNETIC_VARIATION, debug_level=DEFAULT_DEBUG_LEVEL):
        self.magnetic_variation = magnetic_variation
        self.debug_level = debug_level

    def create_seeyou_sentences(self, gps: GPSData, attitude: AttitudeData,
                                vario: Optional[VarioCalculator] = None,
                                dll_connected: bool = False) -> List[str]:
        """Create NMEA sentences matching Condor 3 output order: GPGGA, GPRMC, LXWP0."""
        sentences = []
        if gps and gps.latitude != 0 and gps.longitude != 0:
            try:
                utc_time = datetime.datetime.now(datetime.UTC)
            except AttributeError:
                utc_time = datetime.datetime.utcnow()

            time_str = utc_time.strftime("%H%M%S") + f".{utc_time.microsecond // 1000:03d}"

            sentences.append(self._create_gga_sentence(gps, time_str))
            sentences.append(self._create_rmc_sentence(gps, time_str))
            sentences.append(self._create_lxwp0_sentence(gps, attitude, vario, dll_connected))

        return sentences

    def _create_gga_sentence(self, gps: GPSData, time_str: str) -> str:
        """Create GPGGA sentence matching Condor 3 format."""
        lat_nmea, ns = self._convert_latitude_to_nmea(gps.latitude)
        lon_nmea, ew = self._convert_longitude_to_nmea(gps.longitude)
        alt_meters = gps.barometric_altitude if gps.barometric_altitude is not None else gps.altitude

        # Match Condor 3: 12 satellites, HDOP 10, trailing ,,,,,0000
        parts = [
            "GPGGA",
            time_str,
            lat_nmea, ns,
            lon_nmea, ew,
            "1",                    # Fix quality: GPS fix
            "12",                   # Satellites (Condor 3 uses 12)
            "10",                   # HDOP (Condor 3 uses 10)
            f"{alt_meters:.1f}",
            "M",
            "", "", "", "",         # Geoidal sep, units, DGPS age, DGPS ID
            "0000",                 # Condor 3 trailing field
        ]
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        return f"${sentence}*{checksum}"

    def _create_rmc_sentence(self, gps: GPSData, time_str: str) -> str:
        """Create GPRMC sentence matching Condor 3 format."""
        lat_nmea, ns = self._convert_latitude_to_nmea(gps.latitude)
        lon_nmea, ew = self._convert_longitude_to_nmea(gps.longitude)
        speed_knots = gps.ground_speed * 1.94384

        # Condor 3 sends empty date and mag var fields
        parts = [
            "GPRMC",
            time_str,
            "A",
            lat_nmea, ns,
            lon_nmea, ew,
            f"{speed_knots:.2f}",
            f"{gps.track:.2f}",
            "",                     # Date (Condor 3 leaves empty)
            "", "",                 # Mag var, direction (Condor 3 leaves empty)
            "",                     # Mode indicator
        ]
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        return f"${sentence}*{checksum}"

    def _create_lxwp0_sentence(self, gps: GPSData, attitude: AttitudeData,
                                vario: Optional[VarioCalculator] = None,
                                dll_connected: bool = False) -> str:
        """Create LXWP0 sentence (LX Navigation format, same as Condor 3).
        Format: $LXWP0,Y,<TAS_kph>,<baro_alt_m>,<vario_ms>,,,,,,<heading>,<wind_dir>,<wind_speed_kph>
        """
        alt_m = gps.barometric_altitude if gps.barometric_altitude is not None else gps.altitude

        # TAS: DLL IAS→TAS, or groundspeed as fallback
        if dll_connected and gps.indicated_airspeed is not None:
            tas_ms = ias_to_tas(gps.indicated_airspeed, alt_m)
        else:
            tas_ms = gps.ground_speed
        tas_kph = tas_ms * 3.6

        # Vario: DLL direct, or calculated fallback
        if dll_connected and gps.vertical_speed is not None:
            vario_ms = gps.vertical_speed
        elif vario and vario.is_valid:
            vario_ms = vario.vario
        else:
            vario_ms = 0.0

        # Heading
        heading = attitude.true_heading if attitude and attitude.timestamp > 0 else 0.0

        # Wind: DLL provides real wind, otherwise leave at 0
        wn = gps.wind_x if gps.wind_x is not None else 0.0
        we = gps.wind_z if gps.wind_z is not None else 0.0
        wind_speed_ms = math.sqrt(wn * wn + we * we)
        wind_speed_kph = wind_speed_ms * 3.6

        if wind_speed_ms > 0.1:
            wind_dir = math.degrees(math.atan2(we, wn)) % 360.0
        else:
            wind_dir = 0.0

        parts = [
            "LXWP0",
            "Y",
            f"{tas_kph:.1f}",
            f"{alt_m:.1f}",
            f"{vario_ms:.2f}",
            "", "", "", "", "",
            f"{heading:.0f}",
            f"{wind_dir:.0f}",
            f"{wind_speed_kph:.1f}",
        ]
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        return f"${sentence}*{checksum}"

    @staticmethod
    def _convert_latitude_to_nmea(latitude: float) -> Tuple[str, str]:
        lat_abs = abs(latitude)
        degrees = int(lat_abs)
        minutes = (lat_abs - degrees) * 60
        nmea_str = f"{degrees:02d}{minutes:07.4f}"
        direction = "S" if latitude < 0 else "N"
        return nmea_str, direction

    @staticmethod
    def _convert_longitude_to_nmea(longitude: float) -> Tuple[str, str]:
        lon_abs = abs(longitude)
        degrees = int(lon_abs)
        minutes = (lon_abs - degrees) * 60
        nmea_str = f"{degrees:03d}{minutes:07.4f}"
        direction = "W" if longitude < 0 else "E"
        return nmea_str, direction

    @staticmethod
    def _calculate_checksum(sentence: str) -> str:
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"{checksum:02X}"

# ── Serial Writer ─────────────────────────────────────────────────────

class SerialWriter:
    """Writes NMEA sentences to a COM port for the Naviter dongle."""

    RECONNECT_INTERVAL = 3.0

    def __init__(self, port: str = DEFAULT_COM_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.port = port
        self.baudrate = baudrate
        self.debug_level = debug_level
        self._serial: Optional[serial.Serial] = None
        self._connected = False
        self._lock = threading.Lock()
        self.sentences_sent = 0

    def start(self):
        """Open the serial port."""
        self._connect()

    def _connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                write_timeout=1,
            )
            self._connected = True
            print(f"Serial port {self.port} opened at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Could not open {self.port}: {e}")
            self._connected = False
            return False

    def is_connected(self) -> bool:
        return self._connected and self._serial is not None and self._serial.is_open

    def send(self, sentence: str):
        """Send a single NMEA sentence with CR+LF termination."""
        if not self.is_connected():
            return
        with self._lock:
            try:
                self._serial.write((sentence + "\r\n").encode('ascii'))
                self.sentences_sent += 1
            except serial.SerialException as e:
                if self.debug_level >= 1:
                    print(f"Serial write error: {e}")
                self._connected = False
                try:
                    self._serial.close()
                except Exception:
                    pass

    def reconnect(self):
        """Attempt to reconnect to the serial port."""
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._connected = False
        self._connect()

    def stop(self):
        """Close the serial port."""
        self._connected = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    @staticmethod
    def list_ports() -> List[str]:
        """List available COM ports."""
        if not HAS_PYSERIAL:
            return []
        ports = serial.tools.list_ports.comports()
        return [p.device for p in sorted(ports, key=lambda x: x.device)]

    @staticmethod
    def list_ports_detailed() -> List[Tuple[str, str]]:
        """List available COM ports with descriptions."""
        if not HAS_PYSERIAL:
            return []
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in sorted(ports, key=lambda x: x.device)]

# ── Orchestrator ──────────────────────────────────────────────────────

class AeroflyToSeeYou:
    """
    Main application class that coordinates the UDP receiver,
    DLL reader (optional), NMEA converter, and serial writer.
    """
    def __init__(self, udp_port=DEFAULT_UDP_PORT, com_port=DEFAULT_COM_PORT,
                 baudrate=DEFAULT_BAUDRATE, update_rate=DEFAULT_UPDATE_RATE,
                 magnetic_variation=DEFAULT_MAGNETIC_VARIATION,
                 debug_level=DEFAULT_DEBUG_LEVEL, use_dll=True):
        self.udp_port = udp_port
        self.com_port = com_port
        self.baudrate = baudrate
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate if update_rate > 0 else 1.0
        self.debug_level = debug_level
        self.magnetic_variation = magnetic_variation
        self.use_dll = use_dll

        self.receiver = AeroflyReceiver(udp_port, debug_level)
        self.dll_reader = DLLReader(debug_level) if use_dll else None
        self.converter = NMEAConverter(magnetic_variation, debug_level)
        self.serial_writer = SerialWriter(com_port, baudrate, debug_level)
        self.vario_calculator = VarioCalculator()

        self.running = False
        self.main_thread = None
        self.start_time = time.time()
        self.sentences_count = 0

    def start_services(self):
        """Start all background services."""
        print(f"Starting Aerofly FS4 to SeeYou Navigator bridge v{VERSION}...")
        print(f"UDP Port: {self.udp_port}, Serial: {self.com_port} @ {self.baudrate} baud")
        print(f"Update rate: {self.update_rate} Hz")
        print(f"DLL support: {'enabled' if self.use_dll else 'disabled'}")

        self.receiver.start()
        if self.dll_reader:
            self.dll_reader.start()
        self.serial_writer.start()

        self.running = True
        self.main_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.main_thread.start()

    def start(self):
        """Start services and block in terminal mode (CLI)."""
        self.start_services()
        print("\nSeeYou Navigator should detect the NMEA data on the Naviter dongle.")
        print("Press Ctrl+C to stop.\n")

        try:
            while self.running:
                time.sleep(0.5)
                if not self.receiver.is_connected():
                    print("Waiting for data from Aerofly FS4...", end="\r")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            print("\nStopping bridge...")
            self.stop()

    def get_status(self) -> dict:
        dll_connected = self.dll_reader and self.dll_reader.is_connected()

        if dll_connected:
            gps = self.dll_reader.gps_data
            att = self.dll_reader.attitude_data
        else:
            gps = self.receiver.gps_data
            att = self.receiver.attitude_data

        runtime = time.time() - self.start_time if self.running else 0
        rate = self.sentences_count / runtime if runtime > 0 else 0

        if dll_connected and gps.vertical_speed is not None:
            vario_val = gps.vertical_speed
            avg_vario_val = gps.vertical_speed
        elif self.vario_calculator.is_valid:
            vario_val = self.vario_calculator.vario
            avg_vario_val = self.vario_calculator.average_vario
        else:
            vario_val = None
            avg_vario_val = None

        ias_ms = gps.indicated_airspeed
        tas_ms = ias_to_tas(ias_ms, gps.altitude) if ias_ms is not None else None

        if gps.wind_x is not None and gps.wind_z is not None:
            wn, we = gps.wind_x, gps.wind_z
            wind_speed_ms = math.sqrt(wn**2 + we**2)
            wind_dir = math.degrees(math.atan2(we, wn)) % 360.0 if wind_speed_ms > 0.1 else 0.0
        else:
            wind_speed_ms = None
            wind_dir = None

        return {
            'running': self.running,
            'afs4_connected': self.receiver.is_connected() or dll_connected,
            'dll_connected': dll_connected,
            'data_source': 'DLL' if dll_connected else 'UDP',
            'serial_connected': self.serial_writer.is_connected(),
            'com_port': self.com_port,
            'latitude': gps.latitude,
            'longitude': gps.longitude,
            'altitude_m': gps.altitude,
            'altitude_ft': gps.altitude * 3.28084,
            'ground_speed_kmh': gps.ground_speed * 3.6,
            'ground_speed_kts': gps.ground_speed * 1.94384,
            'track': gps.track,
            'heading': att.true_heading,
            'vario': vario_val,
            'avg_vario': avg_vario_val,
            'ias_kts': ias_ms * 1.94384 if ias_ms is not None else None,
            'tas_kts': tas_ms * 1.94384 if tas_ms is not None else None,
            'wind_speed_kts': wind_speed_ms * 1.94384 if wind_speed_ms is not None else None,
            'wind_dir': wind_dir,
            'sentences_count': self.sentences_count,
            'sentences_rate': rate,
            'uptime': runtime,
        }

    def _process_loop(self):
        last_update_time = 0
        last_stats_time = 0
        last_reconnect_time = 0

        while self.running:
            current_time = time.time()

            # Auto-reconnect serial if disconnected
            if not self.serial_writer.is_connected():
                if current_time - last_reconnect_time >= SerialWriter.RECONNECT_INTERVAL:
                    last_reconnect_time = current_time
                    if self.debug_level >= 1:
                        print(f"Reconnecting to {self.com_port}...")
                    self.serial_writer.reconnect()

            if current_time - last_update_time >= self.update_interval:
                dll_connected = self.dll_reader and self.dll_reader.is_connected()
                if dll_connected:
                    gps_data = self.dll_reader.gps_data
                    attitude_data = self.dll_reader.attitude_data
                else:
                    gps_data = self.receiver.gps_data
                    attitude_data = self.receiver.attitude_data

                if gps_data.timestamp > 0:
                    self.vario_calculator.update(gps_data.altitude, gps_data.timestamp)

                nmea_sentences = self.converter.create_seeyou_sentences(
                    gps_data, attitude_data, self.vario_calculator,
                    dll_connected=dll_connected)

                for sentence in nmea_sentences:
                    self.serial_writer.send(sentence)
                    self.sentences_count += 1

                last_update_time = current_time

            # Periodic stats (every 10 seconds)
            if current_time - last_stats_time >= 10:
                dll_connected = self.dll_reader and self.dll_reader.is_connected()
                source = "[DLL]" if dll_connected else "[UDP]"
                serial_status = "OK" if self.serial_writer.is_connected() else "DISCONNECTED"
                runtime = current_time - self.start_time
                rate = self.sentences_count / runtime if runtime > 0 else 0

                vario_str = ""
                if dll_connected and self.dll_reader.gps_data.vertical_speed is not None:
                    vario_str = f", Vario: {self.dll_reader.gps_data.vertical_speed:+.1f} m/s"
                elif self.vario_calculator.is_valid:
                    vario_str = f", Vario: {self.vario_calculator.vario:+.1f} m/s"

                print(f"{source} {self.com_port}:{serial_status} | {self.sentences_count} sent ({rate:.1f}/sec){vario_str}")
                last_stats_time = current_time

            time.sleep(0.01)

    def stop(self):
        self.running = False
        self.receiver.stop()
        if self.dll_reader:
            self.dll_reader.stop()
        self.serial_writer.stop()
        print("Bridge stopped.")

# ── GUI ───────────────────────────────────────────────────────────────

class BridgeGUI:
    """Tkinter GUI for the AFS4 to SeeYou Navigator bridge."""

    UPDATE_MS = 200
    FONT_LABEL = ("Arial", 10)
    FONT_VALUE = ("Consolas", 11, "bold")
    FONT_STATUS = ("Arial", 10, "bold")
    FONT_FOOTER = ("Consolas", 9)
    COLOR_GREEN = "#4CAF50"
    COLOR_RED = "#F44336"
    COLOR_BG = "#f5f5f5"

    def __init__(self, bridge: AeroflyToSeeYou):
        self.bridge = bridge

        self.root = tk.Tk()
        self.root.title(f"AFS4 → SeeYou Navigator v{VERSION}")
        self.root.geometry("440x520")
        self.root.resizable(False, False)
        self.root.configure(bg=self.COLOR_BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()

    def _build_ui(self):
        bg = self.COLOR_BG

        # ── Serial port config ──
        config_frame = tk.LabelFrame(self.root, text=" Serial Port ", font=self.FONT_LABEL,
                                      bg=bg, padx=10, pady=6)
        config_frame.pack(fill="x", padx=10, pady=(8, 4))

        row0 = tk.Frame(config_frame, bg=bg)
        row0.pack(fill="x")

        tk.Label(row0, text="Port:", font=self.FONT_LABEL, bg=bg).pack(side="left")
        self.combo_port = ttk.Combobox(row0, width=8, state="readonly")
        self.combo_port.pack(side="left", padx=(4, 12))

        tk.Label(row0, text="Baud:", font=self.FONT_LABEL, bg=bg).pack(side="left")
        self.combo_baud = ttk.Combobox(row0, width=8, state="readonly",
                                        values=["4800", "9600", "19200"])
        self.combo_baud.pack(side="left", padx=(4, 12))

        btn_refresh = tk.Button(row0, text="Refresh", font=("Arial", 9),
                                command=self._refresh_ports)
        btn_refresh.pack(side="right")

        self._refresh_ports()
        self.combo_baud.set(str(self.bridge.baudrate))

        # ── Status indicators ──
        status_frame = tk.Frame(self.root, bg=bg)
        status_frame.pack(fill="x", padx=10, pady=(4, 2))

        self.lbl_afs4 = tk.Label(status_frame, text="● AFS4: ---", font=self.FONT_STATUS,
                                  fg=self.COLOR_RED, bg=bg, anchor="w")
        self.lbl_afs4.pack(side="left", expand=True, fill="x")

        self.lbl_serial = tk.Label(status_frame, text="● Serial: ---", font=self.FONT_STATUS,
                                    fg=self.COLOR_RED, bg=bg, anchor="e")
        self.lbl_serial.pack(side="right", expand=True, fill="x")

        dll_frame = tk.Frame(self.root, bg=bg)
        dll_frame.pack(fill="x", padx=10, pady=(0, 4))

        self.lbl_dll = tk.Label(dll_frame, text="● DLL: ---", font=self.FONT_STATUS,
                                fg=self.COLOR_RED, bg=bg, anchor="w")
        self.lbl_dll.pack(side="left", expand=True, fill="x")

        self.lbl_source = tk.Label(dll_frame, text="Source: ---", font=self.FONT_STATUS,
                                    fg="#666666", bg=bg, anchor="e")
        self.lbl_source.pack(side="right", expand=True, fill="x")

        # Separator
        tk.Frame(self.root, height=1, bg="#cccccc").pack(fill="x", padx=10, pady=2)

        # ── Flight data ──
        data_frame = tk.LabelFrame(self.root, text=" Flight Data ", font=self.FONT_LABEL,
                                    bg=bg, padx=10, pady=6)
        data_frame.pack(fill="both", expand=True, padx=10, pady=4)

        self.data_labels = {}
        fields = [
            ("altitude", "Altitude"),
            ("speed", "Speed"),
            ("ias_tas", "IAS / TAS"),
            ("heading", "Heading"),
            ("track", "Track"),
            ("vario", "Vario"),
            ("avg_vario", "Avg Vario"),
            ("wind", "Wind"),
        ]
        for i, (key, label) in enumerate(fields):
            tk.Label(data_frame, text=f"{label}:", font=self.FONT_LABEL, bg=bg,
                     anchor="w", width=10).grid(row=i, column=0, sticky="w", pady=1)
            val = tk.Label(data_frame, text="---", font=self.FONT_VALUE, bg=bg, anchor="w")
            val.grid(row=i, column=1, sticky="w", padx=(8, 0), pady=1)
            self.data_labels[key] = val

        # Separator
        tk.Frame(self.root, height=1, bg="#cccccc").pack(fill="x", padx=10, pady=2)

        # ── Footer ──
        footer_frame = tk.Frame(self.root, bg=bg)
        footer_frame.pack(fill="x", padx=10, pady=2)

        self.lbl_nmea = tk.Label(footer_frame, text="NMEA: 0 sent (0.0/sec)",
                                  font=self.FONT_FOOTER, bg=bg, anchor="w")
        self.lbl_nmea.pack(side="left")

        self.lbl_uptime = tk.Label(footer_frame, text="Uptime: 00:00:00",
                                    font=self.FONT_FOOTER, bg=bg, anchor="e")
        self.lbl_uptime.pack(side="right")

        # ── Buttons ──
        btn_frame = tk.Frame(self.root, bg=bg)
        btn_frame.pack(fill="x", padx=10, pady=(4, 10))

        self.btn_start = tk.Button(btn_frame, text="Start", font=self.FONT_LABEL,
                                    bg=self.COLOR_GREEN, fg="white", width=12,
                                    command=self._on_start)
        self.btn_start.pack(side="left", expand=True, padx=4)

        self.btn_stop = tk.Button(btn_frame, text="Stop", font=self.FONT_LABEL,
                                   bg=self.COLOR_RED, fg="white", width=12,
                                   command=self._on_stop, state="disabled")
        self.btn_stop.pack(side="right", expand=True, padx=4)

    def _refresh_ports(self):
        ports = SerialWriter.list_ports_detailed()
        port_names = [p[0] for p in ports]
        self.combo_port['values'] = port_names
        # Try to select the configured port
        if self.bridge.com_port in port_names:
            self.combo_port.set(self.bridge.com_port)
        elif port_names:
            self.combo_port.set(port_names[0])

        # Show tooltips in terminal
        if ports:
            print("Available COM ports:")
            for name, desc in ports:
                print(f"  {name}: {desc}")

    def _on_start(self):
        # Update bridge config from GUI selections
        selected_port = self.combo_port.get()
        selected_baud = int(self.combo_baud.get())
        if selected_port:
            self.bridge.com_port = selected_port
            self.bridge.serial_writer.port = selected_port
        self.bridge.serial_writer.baudrate = selected_baud

        self.bridge.start_services()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.combo_port.config(state="disabled")
        self.combo_baud.config(state="disabled")
        self._schedule_update()

    def _on_stop(self):
        self.bridge.stop()
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.combo_port.config(state="readonly")
        self.combo_baud.config(state="readonly")
        self._reset_display()

    def _on_close(self):
        if self.bridge.running:
            self.bridge.stop()
        self.root.destroy()

    def _schedule_update(self):
        if not self.bridge.running:
            return
        self._update_display()
        self.root.after(self.UPDATE_MS, self._schedule_update)

    def _update_display(self):
        s = self.bridge.get_status()

        # Connection status
        if s['afs4_connected']:
            self.lbl_afs4.config(text="● AFS4: Connected", fg=self.COLOR_GREEN)
        else:
            self.lbl_afs4.config(text="● AFS4: Waiting...", fg=self.COLOR_RED)

        if s['serial_connected']:
            self.lbl_serial.config(text=f"● Serial: {s['com_port']}", fg=self.COLOR_GREEN)
        else:
            self.lbl_serial.config(text=f"● Serial: Disconnected", fg=self.COLOR_RED)

        if s['dll_connected']:
            self.lbl_dll.config(text="● DLL: Connected", fg=self.COLOR_GREEN)
        elif self.bridge.use_dll:
            self.lbl_dll.config(text="● DLL: Not found", fg=self.COLOR_RED)
        else:
            self.lbl_dll.config(text="● DLL: Disabled", fg="#999999")

        self.lbl_source.config(text=f"Source: {s['data_source']}")

        # Flight data
        self.data_labels["altitude"].config(
            text=f"{s['altitude_m']:.0f} m  ({s['altitude_ft']:.0f} ft)")
        self.data_labels["speed"].config(
            text=f"{s['ground_speed_kmh']:.0f} km/h  ({s['ground_speed_kts']:.0f} kts)")

        if s.get('ias_kts') is not None and s.get('tas_kts') is not None:
            self.data_labels["ias_tas"].config(
                text=f"{s['ias_kts']:.0f} / {s['tas_kts']:.0f} kts")
        else:
            self.data_labels["ias_tas"].config(text="--- (no DLL)")

        self.data_labels["heading"].config(text=f"{s['heading']:.1f}°")
        self.data_labels["track"].config(text=f"{s['track']:.1f}°")

        if s['vario'] is not None:
            arrow = "▲" if s['vario'] > 0.1 else ("▼" if s['vario'] < -0.1 else "─")
            self.data_labels["vario"].config(text=f"{s['vario']:+.1f} m/s  {arrow}")
        else:
            self.data_labels["vario"].config(text="---")

        if s['avg_vario'] is not None:
            self.data_labels["avg_vario"].config(text=f"{s['avg_vario']:+.1f} m/s")
        else:
            self.data_labels["avg_vario"].config(text="---")

        if s.get('wind_speed_kts') is not None and s.get('wind_dir') is not None:
            self.data_labels["wind"].config(
                text=f"{s['wind_dir']:.0f}° / {s['wind_speed_kts']:.0f} kts")
        else:
            self.data_labels["wind"].config(text="--- (no DLL)")

        # Footer
        self.lbl_nmea.config(text=f"NMEA: {s['sentences_count']} sent ({s['sentences_rate']:.1f}/sec)")
        uptime = int(s['uptime'])
        h, m, sec = uptime // 3600, (uptime % 3600) // 60, uptime % 60
        self.lbl_uptime.config(text=f"Uptime: {h:02d}:{m:02d}:{sec:02d}")

    def _reset_display(self):
        self.lbl_afs4.config(text="● AFS4: ---", fg=self.COLOR_RED)
        self.lbl_serial.config(text="● Serial: ---", fg=self.COLOR_RED)
        self.lbl_dll.config(text="● DLL: ---", fg=self.COLOR_RED)
        self.lbl_source.config(text="Source: ---")
        for lbl in self.data_labels.values():
            lbl.config(text="---")
        self.lbl_nmea.config(text="NMEA: 0 sent (0.0/sec)")
        self.lbl_uptime.config(text="Uptime: 00:00:00")

    def run(self):
        self.root.mainloop()

# ── Main ──────────────────────────────────────────────────────────────

def main():
    if not HAS_PYSERIAL:
        print("ERROR: pyserial is required. Install with: pip install pyserial")
        sys.exit(1)

    parser = argparse.ArgumentParser(
        description='Aerofly FS4 to SeeYou Navigator bridge via serial COM port')

    parser.add_argument('--udp-port', type=int, default=DEFAULT_UDP_PORT,
                        help=f'UDP port to receive Aerofly data (default: {DEFAULT_UDP_PORT})')
    parser.add_argument('--com-port', type=str, default=DEFAULT_COM_PORT,
                        help=f'Serial COM port for Naviter dongle (default: {DEFAULT_COM_PORT})')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE,
                        choices=[4800, 9600, 19200],
                        help=f'Serial baud rate (default: {DEFAULT_BAUDRATE})')
    parser.add_argument('--update-rate', type=float, default=DEFAULT_UPDATE_RATE,
                        help=f'NMEA update rate in Hz (default: {DEFAULT_UPDATE_RATE})')
    parser.add_argument('--mag-var', type=float, default=DEFAULT_MAGNETIC_VARIATION,
                        help=f'Magnetic variation in degrees (default: {DEFAULT_MAGNETIC_VARIATION})')
    parser.add_argument('--debug', type=int, default=DEFAULT_DEBUG_LEVEL, choices=[0, 1, 2],
                        help=f'Debug level (default: {DEFAULT_DEBUG_LEVEL})')
    parser.add_argument('--no-gui', action='store_true',
                        help='Run in terminal mode without GUI')
    parser.add_argument('--no-dll', action='store_true',
                        help='Disable DLL shared memory reader (UDP only)')
    parser.add_argument('--list-ports', action='store_true',
                        help='List available COM ports and exit')

    args = parser.parse_args()

    if args.list_ports:
        ports = SerialWriter.list_ports_detailed()
        if ports:
            print("Available COM ports:")
            for name, desc in ports:
                print(f"  {name}: {desc}")
        else:
            print("No COM ports found.")
        sys.exit(0)

    bridge = AeroflyToSeeYou(
        udp_port=args.udp_port,
        com_port=args.com_port,
        baudrate=args.baudrate,
        update_rate=args.update_rate,
        magnetic_variation=args.mag_var,
        debug_level=args.debug,
        use_dll=not args.no_dll,
    )

    if args.no_gui:
        bridge.start()
    else:
        gui = BridgeGUI(bridge)
        gui.run()


if __name__ == "__main__":
    main()
