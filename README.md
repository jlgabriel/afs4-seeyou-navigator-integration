# AFS4 SeeYou Navigator Integration

Bridge that connects [Aerofly FS4](https://www.aerofly.com/) flight simulator to [SeeYou Navigator](https://naviter.com/seeyou-navigator/) via serial COM port, using the [Naviter USB dongle](https://naviter.com/) (Silicon Labs CP210x).

The application receives flight data from Aerofly FS4, converts it to NMEA sentences, and sends them through the serial port — exactly like Condor 3 does. SeeYou Navigator on your phone sees it as a real flight instrument.

## Features

- Real-time GPS position, altitude, speed, and heading to SeeYou Navigator
- Vario (vertical speed) data via `$LXWP0` sentence
- True Airspeed (TAS) and wind data when AeroflyReader DLL is available
- NMEA output matches Condor 3 format for maximum compatibility
- GUI with COM port selector, flight data display, and connection status
- Terminal mode available (`--no-gui`)

## Requirements

- **Python 3.7+**
- **pyserial**: `pip install pyserial`
- **Naviter USB dongle** (Silicon Labs CP210x USB to UART Bridge) connected to your PC
- **SeeYou Navigator** app on your phone, paired with the dongle
- **Aerofly FS4** running with UDP broadcast enabled (default)
- **AeroflyReader DLL** (optional) — provides IAS, direct vario, and wind data

## Data Flow

```
Aerofly FS4
    │
    ├─ UDP broadcast (port 49002, ForeFlight protocol)
    │
    └─ AeroflyReader DLL (shared memory, 50Hz) [optional]
            │
     AFS4 SeeYou Bridge
            │
            ├─ $GPGGA  (position + altitude)
            ├─ $GPRMC  (position + speed + heading)
            └─ $LXWP0  (TAS + baro alt + vario + wind)
                    │
              Serial COM port (4800 baud)
                    │
              Naviter USB dongle
                    │
              SeeYou Navigator (phone)
```

## NMEA Sentences

The bridge generates three NMEA sentences per cycle at 1 Hz, matching Condor 3's output:

| Sentence | Content | Source |
|----------|---------|--------|
| `$GPGGA` | Position, altitude MSL, fix quality | UDP or DLL |
| `$GPRMC` | Position, ground speed, track | UDP or DLL |
| `$LXWP0` | TAS, baro altitude, vario, wind | DLL preferred, UDP fallback |

## Usage

### GUI mode (default)

```bash
python afs4-seeyou-navigator-integration.py
```

Select your COM port and baud rate from the dropdowns, then click **Start**.

### Terminal mode

```bash
python afs4-seeyou-navigator-integration.py --no-gui --com-port COM7
```

### Options

```
--com-port PORT    Serial COM port (default: COM7)
--baudrate RATE    Baud rate: 4800, 9600, 19200 (default: 4800)
--udp-port PORT    UDP port for AFS4 data (default: 49002)
--update-rate HZ   NMEA update rate in Hz (default: 1)
--no-dll           Disable DLL reader (UDP only)
--no-gui           Run in terminal mode
--list-ports       List available COM ports and exit
--debug LEVEL      Debug level: 0, 1, 2 (default: 1)
```

## Data Sources

The bridge supports two data sources from Aerofly FS4:

| | UDP (default) | DLL (preferred) |
|---|---|---|
| Position | Yes | Yes |
| Altitude | GPS only | MSL + barometric |
| Speed | Ground speed | Ground speed + IAS → TAS |
| Heading | From XATT | True + magnetic |
| Vario | Calculated from altitude | Direct at 50Hz |
| Wind | Not available | Real wind (ECEF → ENU) |

When the DLL is available, it is used automatically. The bridge falls back to UDP when the DLL is not detected.

## Naviter Dongle Setup

1. Connect the Naviter USB dongle to your PC
2. Check which COM port it was assigned in Device Manager (typically `COM7`)
3. Pair SeeYou Navigator with the dongle via Bluetooth
4. In SeeYou Navigator: **Settings > Devices** — the dongle should appear

## Tools

### nmea_sniffer.py

Utility to capture and analyze NMEA sentences from any COM port. Useful for debugging or reverse-engineering other simulators' output.

```bash
python nmea_sniffer.py --port COM4 --log capture.txt
```

## License

MIT License — see [LICENSE](LICENSE) file.
