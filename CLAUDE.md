# afs4-seeyou-navigator-integration

Bridge that sends Aerofly FS4 flight data to SeeYou Navigator via serial COM port (Naviter dongle).

## Architecture

Single-file Python application (`afs4-seeyou-navigator-integration.py`) with these components:

| Class | Responsibility |
|-------|---------------|
| `AeroflyReceiver` | UDP listener on port 49002, parses ForeFlight protocol (XGPS/XATT) |
| `DLLReader` | Reads AeroflyReader DLL shared memory ("AeroflyReaderData") for IAS, vario, wind |
| `VarioCalculator` | Calculates vertical speed from altitude deltas (fallback when no DLL) |
| `NMEAConverter` | Generates NMEA sentences: GPGGA, GPRMC, LXWP0 |
| `SerialWriter` | Writes NMEA to COM port via pyserial (replaces TCPServer from XCSoar project) |
| `AeroflyToSeeYou` | Orchestrator: DLL (preferred) or UDP -> vario -> converter -> serial |
| `BridgeGUI` | Tkinter GUI with COM port selector, flight data, and connection status |
| `ias_to_tas()` | IAS to TAS conversion using full ISA standard atmosphere model |

## Data flow

```
AFS4 DLL -> Shared Memory -> DLLReader (50Hz, preferred)
AFS4 UDP 49002 -> AeroflyReceiver (fallback)
                          |
                    NMEAConverter -> SerialWriter -> COM port -> Naviter dongle -> SeeYou Navigator
```

## NMEA output

Three sentences at 1 Hz, matching Condor 3's exact format:

- **GPGGA** -- position, altitude MSL (12 sats, HDOP 10, trailing 0000 like Condor 3)
- **GPRMC** -- position, speed, track (empty date/mag var fields like Condor 3)
- **LXWP0** -- TAS (kph), baro alt (m), vario (m/s), heading, wind dir/speed (kph)

LXWP0 is always sent: with DLL data when available, with calculated fallback otherwise.

## DLL integration

Same as afs4-xcsoar-integration. The DLLReader connects to shared memory and provides:
- **IAS** -> converted to **TAS** using ISA atmosphere model
- **Vertical speed** (direct, 50Hz)
- **Wind** (3D ECEF -> local ENU, direction FROM)

**CRITICAL:** Shared memory must be opened with `OpenFileMappingW` check first.
Never use `mmap(-1, size, name)` directly — it creates an empty mapping that shadows the DLL's real one.

## AFS4 DLL data conventions (CRITICAL)

1. **Vector3D values are in ECEF coordinates**, not local North/East/Up
2. **Heading uses math convention** (CCW from East), convert: `nav = (90 - degrees(rad)) % 360`
3. **Longitude** from DLL is 0-2pi range, normalize to -180/+180
4. **Wind vector points FROM** (meteorological convention)
5. **Altitude** from DLL is meters MSL

## Serial communication

- Default: **4800 baud**, 8N1 (matching Condor 3 / NMEA standard)
- Naviter dongle: Silicon Labs CP210x USB to UART Bridge (typically COM7)
- Sentences terminated with `\r\n`
- Supports 4800, 9600, 19200 baud rates

## Relationship to afs4-xcsoar-integration

This project shares the same architecture and core classes. Key differences:
- **Output**: Serial COM port (SerialWriter) instead of TCP socket (TCPServer)
- **Sentences**: Only GPGGA + GPRMC + LXWP0 (no HCHDT, PGRMZ, PTAS1)
- **Update rate**: 1 Hz (vs 5 Hz for XCSoar)
- **Format**: Matches Condor 3 output exactly (verified by reverse-engineering)
- **Dependency**: Requires pyserial (XCSoar project is pure stdlib)

## Key constraints

- Requires `pyserial` package (`pip install pyserial`)
- DLL shared memory is Windows-only (mmap)
- `--no-dll` flag disables DLL reader (UDP only mode)

## Running

```bash
python afs4-seeyou-navigator-integration.py              # GUI mode with DLL (default)
python afs4-seeyou-navigator-integration.py --no-dll     # GUI mode, UDP only
python afs4-seeyou-navigator-integration.py --no-gui     # Terminal mode
python afs4-seeyou-navigator-integration.py --list-ports # Show available COM ports
```
