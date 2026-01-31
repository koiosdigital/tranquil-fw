# Tranquil Sand Table - System Architecture

## Overview

ESP32-S3 firmware for a kinetic sand table using polar coordinate motion control. The system draws patterns in sand using a ball bearing driven by two stepper motors (theta rotation, rho radial movement).

## Project Structure

```
tranquil-fw/
├── main/                      # Application entry point
│   ├── main.cpp              # app_main(), initialization sequence
│   ├── api/                  # HTTP REST API handlers
│   ├── storage/              # SD card, manifest database
│   └── usb_pd/               # USB Power Delivery
├── components/
│   ├── sand_table/           # Motion control system
│   ├── tmcstepper/           # TMC2209 stepper drivers
│   ├── stusb4500/            # USB-C PD controller
│   ├── kd_pixdriver/         # RGB/RGBW LED driver
│   ├── kd_common/            # WiFi, BLE, API framework
│   └── tranquil-app/         # Static web UI assets
└── partitions.csv            # Flash partition layout
```

## Motion Control System

### Architecture

```
HTTP API / SandTablePlayer
         ↓
   MotionController (state machine)
         ↓
   PathPlanner (interpolation)
         ↓
   RingBuffer<MotionSegment>
         ↓
   VelocityPlanner (lookahead)
         ↓
   StepperDriver (RMT peripheral)
         ↓
   TMC2209 → Stepper Motors
```

### Coordinate System

- **Polar coordinates**: theta (angle in radians), rho (0.0-1.0 normalized radius)
- **Theta axis**: Stage rotation via gear reduction (8.05:1 default)
- **Rho axis**: Radial carriage movement via pinion/rack

### Key Components

| Component | File | Purpose |
|-----------|------|---------|
| MotionController | `motion_controller.h` | Top-level API, state machine |
| PathPlanner | `path_planner.h` | Cartesian/polar interpolation |
| VelocityPlanner | `velocity_planner.h` | Lookahead acceleration planning |
| StepperDriver | `stepper_driver.h` | RMT-based pulse generation |
| HomingController | `homing_controller.h` | Hall sensor + StallGuard homing |

### Motion States

```
Idle → Homing → Running ↔ Paused
  ↓       ↓        ↓
       Error ← EStop
```

## Pattern/Playlist System

### Data Model

**Pattern**: A trajectory file with metadata
- UUID, name, creator, popularity
- Flags: reversible, start_point, encrypted
- Timestamps: created_at, last_played_at, downloaded_at

**Playlist**: Ordered collection of patterns
- UUID, name, description
- Featured pattern reference
- Many-to-many relationship with patterns

### Storage

| Data | Location | Format |
|------|----------|--------|
| Manifest | `/sd/manifest.db` | SQLite database |
| Pattern files | `/sd/patterns/{uuid}.thr` | Text (theta rho per line) |
| Device config | NVS | Key-value pairs |

### Pattern File Format (.thr)

```
# Comment lines start with #
0.0 0.5          # theta (radians), rho (0-1)
0.1 0.52
0.2 0.54
...
```

### ManifestDatabase

SQLite-backed storage for patterns and playlists. Thread-safe singleton using PSRAM for allocations.

```cpp
auto& db = ManifestDatabase::instance();
db.initialize();

// Pattern operations
auto patterns = db.getAllPatterns();
auto pattern = db.getPattern("uuid-here");
db.addPattern(pattern);

// Playlist operations
auto playlists = db.getAllPlaylists();
db.addPatternToPlaylist(playlistUuid, patternUuid);
```

### Database Schema

```sql
patterns (
    id, uuid, name, creator, date, popularity,
    reversible, start_point, encrypted, size_bytes,
    created_at, last_played_at, downloaded_at
)

playlists (
    id, uuid, name, description, featured_pattern_uuid,
    date, created_at, updated_at
)

playlist_patterns (
    playlist_id, pattern_id, position  -- junction table
)
```

## Configuration Management

### NVS Namespace: `tranquil_cfg`

**Motion Config**
- steps_per_rev, microsteps, gear_ratio
- theta_max_rpm, rho_max_rpm
- motor currents, StallGuard threshold
- acceleration limits

**LED Config**
- has_leds, led_count, is_rgbw

**Calibration**
- theta_steps_per_rotation, rho_max_steps

### ConfigManager

Singleton providing runtime configuration with NVS persistence:

```cpp
auto& cfg = ConfigManager::instance();
cfg.motion_config().theta_max_rpm;
cfg.set_motion_config(new_config);
cfg.save_calibration(calibration_data);
```

## HTTP API

### Patterns

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/patterns?page=0&per_page=20` | List patterns (paginated) |
| GET | `/api/patterns/{uuid}` | Get pattern details |
| POST | `/api/patterns` | Upload pattern |
| DELETE | `/api/patterns/{uuid}` | Delete pattern |

### Playlists

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/playlists?page=0&per_page=20` | List playlists |
| GET | `/api/playlists/{uuid}` | Get playlist details |
| POST | `/api/playlists` | Create playlist |
| POST | `/api/playlists/{uuid}` | Add/remove pattern |
| POST | `/api/playlists/{uuid}/order` | Reorder patterns |
| DELETE | `/api/playlists/{uuid}` | Delete playlist |

### Player Control

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/player/play` | Start playback |
| POST | `/api/player/pause` | Pause playback |
| POST | `/api/player/resume` | Resume playback |
| POST | `/api/player/stop` | Stop playback |

### Pagination Response Format

```json
{
  "pagination": {
    "page": 0,
    "per_page": 20,
    "total_pages": 5,
    "total_items": 100
  },
  "patterns": [...]
}
```

## Hardware Configuration

### Pin Assignments (from sdkconfig)

| Function | GPIO |
|----------|------|
| Theta Step | 39 |
| Theta Dir | 40 |
| Rho Step | 11 |
| Rho Dir | 10 |
| Motor Enable | 21 |
| TMC UART TX | 12 |
| TMC UART RX | 13 |
| Theta Hall | 17 |
| Rho StallGuard | 14 |
| LED Data | 18 |
| SD CLK | 6 |
| SD CMD | 7 |
| SD D0 | 5 |

### TMC2209 Configuration

- Shared UART bus, addresses 0 (rho) and 2 (theta)
- 115200 baud
- StallGuard for sensorless homing on rho axis
- Configurable run/hold current (default 400mA)

## Memory Architecture

### PSRAM (4MB)

- SQLite database operations
- Large buffers and vectors
- Motion segment ring buffer

### Internal DRAM

- FreeRTOS stacks (critical tasks)
- DMA buffers
- Small allocations (<16KB)

## FreeRTOS Tasks

| Task | Priority | Core | Purpose |
|------|----------|------|---------|
| stepper_task | 24 | 1 | Real-time step generation |
| sand_player | 5 | 0 | Pattern file reading |
| httpd | Medium | 0 | HTTP request handling |

## Initialization Sequence

1. Event loop creation
2. USB PD controller init
3. kd_common init (WiFi, BLE, HTTP server)
4. ConfigManager init (NVS)
5. ManifestDatabase init (SQLite on SD)
6. MotionController init + start
7. SandTablePlayer init
8. LED driver init (if enabled)
9. API endpoint registration
10. Homing sequence
