# ESP32-S3 CO2 Monitor

A firmware for monitoring indoor air quality using the Adafruit ESP32-S3 Reverse TFT Feather and SCD-41 CO2 sensor.

**Version:** 0.4

## Hardware Required

- [Adafruit ESP32-S3 Reverse TFT Feather](https://www.adafruit.com/product/5691)
- [Adafruit SCD-41 CO2 Sensor](https://www.adafruit.com/product/5190) (STEMMA QT / I2C)
- STEMMA QT cable (included with sensor)
- LiPo battery (optional, for portable use)

## Wiring

Connect the SCD-41 to the ESP32-S3 Reverse TFT Feather using the STEMMA QT connector - no soldering required.

## Features

- **Real-time monitoring** of CO2, Temperature, and Humidity
- **Color-coded thresholds** for quick visual assessment (configurable)
- **Trend indicators** showing if values are rising, falling, or stable
- **24-hour history graph** with threshold lines
- **Battery monitoring** with MAX17048 fuel gauge
- **Smart display refresh**: only updates when values change
- **Three display modes**: Bright, Dim, and Power Save
- **Hyperdrive mode**:  automatic faster readings when CO2 changes rapidly
- **Power saving**: WiFi/Bluetooth disabled, light sleep, optimized GPIO configuration

## Display Modes

| Mode       | Brightness | Behavior                                                               |
| ---------- | ---------- | ---------------------------------------------------------------------- |
| Bright     | 100%       | Always on                                                              |
| Dim        | ~25%       | Always on, reduced brightness                                          |
| Power Save | Off        | Display activates for 10s when CO2 exceeds threshold or button pressed |

## Hyperdrive Mode

When CO2 levels change rapidly (e.g., opening a window), hyperdrive mode automatically activates:

- **Trigger**: CO2 changes by ≥100 ppm between readings
- **Effect**: Sensor read interval changes from 5 minutes to 1 minute
- **Exit**: After 3 consecutive stable readings (<100 ppm change)
- **Indicator**: Status bar shows `[H]` when active
- **Note**: Disabled in Power Save mode to conserve battery

## Display Colors

All metrics use color-coded thresholds. Temperature and humidity use diverging scales where white indicates optimal values.

| Metric      | Color Scale                                                              |
| ----------- | ------------------------------------------------------------------------ |
| CO2         | Green → Yellow → Orange → Red (sequential, lower is better)              |
| Temperature | Blue → Cyan → **White** → Yellow → Orange (diverging, middle is optimal) |
| Humidity    | Orange → Yellow → **White** → Cyan → Blue (diverging, middle is optimal) |

## CO2 Thresholds

| Level    | CO2 (ppm) | Color  | Meaning                          |
| -------- | --------- | ------ | -------------------------------- |
| Low      | < 800     | Green  | Excellent air quality            |
| Moderate | 800-1000  | Yellow | Acceptable, consider ventilating |
| High     | 1000-1200 | Orange | Poor, should ventilate           |
| Critical | > 1200    | Red    | Unhealthy, ventilate immediately |

## Temperature Thresholds

| Level   | Temperature (°C) | Color  | Meaning  |
| ------- | ---------------- | ------ | -------- |
| Cold    | < 16             | Blue   | Too cold |
| Cool    | 16-20            | Cyan   | Cool     |
| Optimal | 20-24            | White  | Optimal  |
| Warm    | 24-28            | Yellow | Warm     |
| Hot     | > 28             | Orange | Too hot  |

## Humidity Thresholds

| Level   | Humidity (%) | Color  | Meaning |
| ------- | ------------ | ------ | ------- |
| Dry     | < 30         | Orange | Too dry |
| Low     | 30-40        | Yellow | Low     |
| Optimal | 40-60        | White  | Optimal |
| High    | 60-70        | Cyan   | High    |
| Wet     | > 70         | Blue   | Too wet |

## Button Functions

| Button    | Function                                      |
| --------- | --------------------------------------------- |
| D0 (Boot) | Cycle displayed value: CO2 → Temp → Humidity  |
| D1        | Toggle view: Number ↔ Graph                   |
| D2        | Cycle display mode: Bright → Dim → Power Save |

### Power Save Mode

In Power Save mode, the display is off by default to conserve battery. It automatically turns on for 10 seconds when:

- CO2 exceeds the "High" threshold (1200 ppm) after a sensor reading
- Any button is pressed

The device enters light sleep between sensor readings, waking on button press or timer.

## Display Layout

### Number View

```
+-----------------------------------------+
| [*] CO2      BRIGHT          [===] 85%  |  <- Status bar
|                                         |
|   1234 ppm                        ^     |  <- Big value + unit + trend
|                                         |
|   23.5 C                       45%      |  <- Secondary values
|   Temp                      Humidity    |
+-----------------------------------------+
```

### Graph View

```
+-----------------------------------------+
| [*] CO2      DIM [H]         [===] 85%  |  <- Status bar ([H] = hyperdrive)
|                                         |
|   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~    2500  |  <- Graph with threshold
|   ----------------------------          |     lines and scale
|   ============================          |
|   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~     400  |
+-----------------------------------------+
```

## Installation

### Using PlatformIO (Recommended)

1. Install [PlatformIO](https://platformio.org/install)

2. Clone or download this repository

3. Open the project folder in VS Code with PlatformIO extension, or use the CLI:

   ```bash
   cd esp32-co2-monitor
   pio run
   ```

4. Connect the ESP32-S3 via USB and flash:

   ```bash
   pio run --target upload
   ```

5. Monitor serial output (optional):
   ```bash
   pio device monitor
   ```

### Using Arduino IDE

1. Install the [Arduino IDE](https://www.arduino.cc/en/software)

2. Add ESP32 board support:

   - Go to File -> Preferences
   - Add to "Additional Board Manager URLs":
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```
   - Go to Tools -> Board -> Board Manager
   - Search for "esp32" and install "esp32 by Espressif Systems"

3. Install required libraries via Library Manager:

   - Adafruit GFX Library
   - Adafruit ST7735 and ST7789 Library
   - Sensirion I2C SCD4x
   - Adafruit BusIO
   - Adafruit MAX1704X

4. Select board: Tools -> Board -> ESP32 Arduino -> Adafruit Feather ESP32-S3 Reverse TFT

5. Copy `src/main.cpp` content and paste into a new Arduino sketch

6. Upload to board

## Configuration

Edit the `#define` values at the top of `src/main.cpp` to customize:

### Thresholds and Colors

```cpp
// CO2 Thresholds (in ppm) - Sequential scale
#define CO2_COLOR_LOW         COLOR_GREEN   // Excellent
#define CO2_COLOR_MODERATE    COLOR_YELLOW  // Acceptable
#define CO2_COLOR_HIGH        COLOR_ORANGE  // Poor
#define CO2_COLOR_CRITICAL    COLOR_RED     // Unhealthy
#define CO2_THRESHOLD_LOW     800
#define CO2_THRESHOLD_MODERATE 1000
#define CO2_THRESHOLD_HIGH    1200
#define CO2_THRESHOLD_CRITICAL 1400

// Temperature thresholds (in Celsius) - Diverging scale
#define TEMP_COLOR_COLD       COLOR_BLUE
#define TEMP_COLOR_COOL       COLOR_CYAN
#define TEMP_COLOR_OPTIMAL    COLOR_WHITE
#define TEMP_COLOR_WARM       COLOR_YELLOW
#define TEMP_COLOR_HOT        COLOR_ORANGE
#define TEMP_THRESHOLD_COLD   16.0
#define TEMP_THRESHOLD_COOL   20.0
#define TEMP_THRESHOLD_OPTIMAL 24.0
#define TEMP_THRESHOLD_WARM   28.0

// Humidity thresholds (in %) - Diverging scale
#define HUMID_COLOR_DRY       COLOR_ORANGE
#define HUMID_COLOR_LOW       COLOR_YELLOW
#define HUMID_COLOR_OPTIMAL   COLOR_WHITE
#define HUMID_COLOR_HIGH      COLOR_CYAN
#define HUMID_COLOR_WET       COLOR_BLUE
#define HUMID_THRESHOLD_DRY   30.0
#define HUMID_THRESHOLD_LOW   40.0
#define HUMID_THRESHOLD_OPTIMAL 60.0
#define HUMID_THRESHOLD_HIGH  70.0
```

### Timing and Behavior

```cpp
// Timing Configuration
#define SENSOR_READ_INTERVAL       300000  // Normal: 5 minutes
#define SENSOR_READ_INTERVAL_FAST  60000   // Hyperdrive: 1 minute
#define GRAPH_SAMPLE_INTERVAL      300000  // Graph point every 5 minutes
#define DISPLAY_TIMEOUT            10000   // Power Save display timeout (10s)

// Hyperdrive Configuration
#define HYPERDRIVE_THRESHOLD       100     // CO2 change (ppm) to trigger
#define HYPERDRIVE_STABLE_COUNT    3       // Stable readings to exit

// Display Brightness (PWM values 0-255)
#define BRIGHTNESS_HIGH            255     // Full brightness
#define BRIGHTNESS_DIM             64      // Dimmed (~25%)
```

## Power Saving Features

The firmware implements several power-saving measures:

- **WiFi disabled** - Not used, saves significant power
- **Bluetooth disabled** - Not used, saves power
- **Unused GPIO pins** - Configured as floating inputs to minimize leakage
- **NeoPixel disabled** - Onboard LED turned off
- **Light sleep** - In Power Save mode, CPU enters light sleep between sensor reads
- **PWM backlight** - Allows dimming instead of just on/off

## Troubleshooting

**"Sensor Error!" on startup**

- Check STEMMA QT cable connection
- Ensure SCD-41 is properly powered
- Try a different I2C cable

**Display not working**

- Ensure you're using the latest ESP32 board support package
- The TFT_I2C_POWER pin must be set HIGH (done automatically in code)

**Readings seem incorrect**

- SCD-41 requires ~30 seconds warmup after power-on
- Place sensor away from direct breath and heat sources
- Ensure adequate ventilation around the sensor

**Battery shows "USB" instead of percentage**

- The MAX17048 battery monitor requires a LiPo battery to be connected
- When running on USB power only, "USB" is displayed

**Hyperdrive not activating**

- Hyperdrive is disabled in Power Save mode
- Ensure CO2 change is ≥100 ppm between readings

**Light sleep not working**

- Light sleep only activates in Power Save mode when display is off
- Serial monitor may interfere with sleep; disconnect for true power measurement

## License

MIT License - feel free to modify and use for your projects.
