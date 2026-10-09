# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

A PlatformIO firmware project targeting the Arduino Uno (`atmelavr` platform, `arduino` framework). All application code currently lives in [src/main.cpp](src/main.cpp), which drives a relay on pin 9 to cycle a fan on/off (3s on, 3s off) and logs status over Serial at 9600 baud.

## Commands

Use the PlatformIO CLI (`pio`), not raw avr-gcc/make:

- Build: `pio run`
- Upload to board: `pio run --target upload`
- Serial monitor: `pio device monitor` (9600 baud, matching `Serial.begin(9600)` in main.cpp)
- Clean build artifacts: `pio run --target clean`
- Unit tests (PlatformIO Test Runner): `pio test` — no tests currently exist under [test/](test/)

The environment is defined in [platformio.ini](platformio.ini) as `[env:uno]`. If commands need a specific environment (once more are added), pass `-e uno`.

## Architecture

This is a single-file embedded firmware project, not a multi-module application:

- [src/main.cpp](src/main.cpp) — entry point with `setup()`/`loop()`. Currently a standalone relay/fan test.
- [include/](include/) — intended for project header files shared across sources; empty aside from the PlatformIO scaffold README.
- [lib/](lib/) — intended for private, self-contained project libraries (each in its own subdirectory with its own headers/sources); empty aside from the PlatformIO scaffold README.
- [test/](test/) — intended for PlatformIO unit tests; empty aside from the PlatformIO scaffold README.

Since the project is still minimal (a single hardware I/O test), there is no cross-file architecture yet — future functionality should be added either directly in `src/`, or split into `lib/` libraries as it grows, following PlatformIO's standard project layout.

## Hardware notes

- Target board: Arduino Uno.
- `RELAY_PIN` (pin 9) drives a relay module; `HIGH` energizes the relay (fan/load ON), `LOW` de-energizes it (OFF). Keep this polarity in mind when modifying relay logic — inverted-logic relay boards would reverse this.

### MCP platform.ini file
[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 9600
lib_deps =
    adafruit/DHT sensor library@^1.4.4
    adafruit/Adafruit Unified Sensor@^1.1.9
    marcoschwartz/LiquidCrystal_I2C@^1.1.4

