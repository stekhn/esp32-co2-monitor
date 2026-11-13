# ESP32-S3 CO2 Monitor

Battery-powered indoor air quality monitor built with the [Adafruit ESP32-S3 Reverse TFT Feather](https://www.adafruit.com/product/5691) and [SCD-41 CO2 sensor](https://www.adafruit.com/product/5190). Tracks CO2, temperature, and humidity with a color-coded display, 24-hour history graph, and deep sleep for low power consumption. No soldering required.

## Features

- Real-time CO2, temperature, and humidity monitoring
- Color-coded thresholds with trend arrows
- 24-hour history graph
- Three display modes: Day, Night, ECO (deep sleep)
- Hyperdrive mode: faster readings when CO2 changes rapidly
- Battery monitoring via [MAX17048](https://www.adafruit.com/product/5580) fuel gauge

## Controls

- **D0 (Boot):** Cycle value from CO2 → Temp → Humidity
- **D1:** Toggle view from Big Number ↔ Graph
- **D2:** Cycle mode from Day → Night → ECO

In ECO mode, pressing D1 or D2 wakes the device from deep sleep. D0 only works while the display is on.

## Display Modes

- **Day:** Always on at 78% brightness, sensor reads every 5 min
- **Night:** Always on at 31% brightness, sensor reads every 5 min
- **ECO:** Display on for 10s after button press, then deep sleep

### ECO Mode

- Display turns off after 10 seconds of inactivity
- ESP32 enters [deep sleep](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/sleep_modes.html) between sensor readings (~10-50 uA)
- Sensor reads every 5 minutes via [single-shot measurement](https://sensirion.com/media/documents/48C4B7FB/66E05452/CD_DS_SCD40_SCD41_Datasheet_D1.pdf)
- All state (history, readings, UI) persisted in RTC memory
- Wake sources: 5-minute timer and button press (D1/D2)

### Hyperdrive

Automatically activates when CO2 changes rapidly (opening a window, etc.):

- **Trigger**: CO2 changes by >=100 ppm between readings
- **Effect**: Read interval changes from 5 minutes to 1 minute
- **Exit**: After 3 consecutive stable readings
- **Indicator**: Lightning bolt icon in status bar
- Disabled in ECO mode

## Color Thresholds

All metrics use color-coded thresholds. Temperature and humidity use diverging scales where white indicates optimal values.

### CO2 (ppm)

| Range     | Color  | Meaning    |
| --------- | ------ | ---------- |
| < 800     | Green  | Excellent  |
| 800-1000  | Yellow | Acceptable |
| 1000-1200 | Orange | Poor       |
| > 1200    | Red    | Unhealthy  |

### Temperature (C)

| Range | Color  | Meaning |
| ----- | ------ | ------- |
| < 16  | Blue   | Cold    |
| 16-20 | Cyan   | Cool    |
| 20-24 | White  | Optimal |
| 24-28 | Yellow | Warm    |
| > 28  | Orange | Hot     |

### Humidity (%)

| Range | Color  | Meaning |
| ----- | ------ | ------- |
| < 30  | Orange | Dry     |
| 30-40 | Yellow | Low     |
| 40-60 | White  | Optimal |
| 60-70 | Cyan   | High    |
| > 70  | Blue   | Wet     |

## Views

- **Number view:** Large primary value with unit and trend arrow, secondary values at the bottom
- **Graph view:** 24-hour history with color-coded threshold lines

Both views show a status bar with the selected metric, display mode, and battery level.

## Installation

### PlatformIO (Recommended)

```bash
pio run                  # Build
pio run --target upload  # Flash
pio device monitor       # Serial output
```

### Arduino IDE

1. Add [ESP32 board support](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) (URL: <https://espressif.github.io/arduino-esp32/package_esp32_index.json>)
2. Install libraries: [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library), [Adafruit ST7735/ST7789](https://github.com/adafruit/Adafruit-ST7735-Library), [Sensirion I2C SCD4x](https://github.com/Sensirion/arduino-i2c-scd4x), [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO), [Adafruit MAX1704X](https://github.com/adafruit/Adafruit_MAX1704X)
3. Select board: Adafruit Feather ESP32-S3 Reverse TFT
4. Copy `src/main.cpp` into a new sketch and upload

## Configuration

Edit the `#define` values at the top of `src/main.cpp`:

```cpp
// Timing
#define SENSOR_READ_INTERVAL       300000  // Normal: 5 minutes
#define SENSOR_READ_INTERVAL_FAST  60000   // Hyperdrive: 1 minute
#define DISPLAY_TIMEOUT            10000   // ECO display timeout (10s)
#define DEEP_SLEEP_US              300000000ULL  // ECO sleep duration (5 min)

// Brightness (PWM 0-255)
#define BRIGHTNESS_DAY             200
#define BRIGHTNESS_NIGHT           80
#define BRIGHTNESS_ECO             30

// Hyperdrive
#define HYPERDRIVE_THRESHOLD       100     // CO2 ppm change to trigger
#define HYPERDRIVE_STABLE_COUNT    3       // Stable readings to exit

// Battery protection
#define BATTERY_SHUTDOWN_VOLTAGE   3.30    // Shut down below this cell voltage
#define BATTERY_RECOVER_VOLTAGE    3.60    // Resume above this cell voltage

// Sensor
#define SCD41_TEMPERATURE_OFFSET   4.5     // Self-heating compensation (C)
#define SCD41_ALTITUDE             525     // Altitude for pressure correction (m)
```

## Troubleshooting

- **"Sensor Error" on startup:** Press the reset button. Check STEMMA QT cable connection. Try a different cable.
- **Readings seem incorrect:** SCD-41 needs ~30s warmup. Keep sensor away from direct breath and heat sources.
- **Battery shows "USB":** Normal when running on USB power without a LiPo battery connected.
- **"Battery low", then the display stays off:** The device shuts down below 3.3 V to protect the LiPo. Connect USB; it restarts on its own once the battery is above 3.6 V (checked every 30 minutes, or immediately when pressing D1 or D2). The history graph is reset.
- **Hyperdrive not activating:** Disabled in ECO mode. Requires >=100 ppm change between readings.
- **Deep sleep not working:** USB Serial/JTAG can interfere with deep sleep on ESP32-S3. Disconnect serial monitor for accurate power measurements.

## Disclaimer

Parts of the documentation and code were written with the help of [Claude Code](https://docs.anthropic.com/en/docs/claude-code).

## License

MIT
