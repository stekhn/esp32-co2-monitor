# ESP32-S3 CO2 Monitor

A firmware for monitoring indoor air quality using the Adafruit ESP32-S3 Reverse TFT Feather and SCD-41 CO2 sensor.

## Hardware Required

- [Adafruit ESP32-S3 Reverse TFT Feather](https://www.adafruit.com/product/5691)
- [Adafruit SCD-41 CO2 Sensor](https://www.adafruit.com/product/5190) (STEMMA QT / I2C)
- STEMMA QT cable (included with sensor)

## Wiring

Connect the SCD-41 to the ESP32-S3 Reverse TFT Feather using the STEMMA QT connector - no soldering required.

## Features

- **Real-time monitoring** of CO2, Temperature, and Humidity
- **Color-coded thresholds** for quick visual assessment
- **24-hour history graph** (120 data points, sampled every 12 minutes)
- **Three display modes** controlled via buttons
- **Power saving warn mode** - display activates only when thresholds exceeded

## CO2 Thresholds

| Level  | CO2 (ppm) | Meaning                        |
|--------|-----------|--------------------------------|
| Green  | < 800     | Excellent air quality          |
| Yellow | 800-1000  | Acceptable, consider ventilating |
| Orange | 1000-1500 | Poor, should ventilate         |
| Red    | > 1500    | Unhealthy, ventilate immediately |

## Button Functions

| Button | Function                                    |
|--------|---------------------------------------------|
| D0 (Boot) | Cycle displayed value: CO2 -> Temp -> Humidity |
| D1     | Toggle view: Number <-> Graph               |
| D2     | Toggle mode: Always On <-> Warn Mode        |

### Warn Mode

In warn mode, the display turns off when CO2 levels are acceptable (<800 ppm). It automatically turns on when:
- CO2 exceeds the yellow threshold (800 ppm)
- Any button is pressed (temporary wake for 10 seconds)

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
   - Adafruit SCD4x
   - Adafruit BusIO

4. Select board: Tools -> Board -> ESP32 Arduino -> Adafruit Feather ESP32-S3 Reverse TFT

5. Copy `src/main.cpp` content and paste into a new Arduino sketch

6. Upload to board

## Configuration

Edit the `#define` values at the top of `src/main.cpp` to customize:

```cpp
// CO2 Thresholds (in ppm)
#define CO2_THRESHOLD_GREEN   800
#define CO2_THRESHOLD_YELLOW  1000
#define CO2_THRESHOLD_ORANGE  1500

// Temperature thresholds (in Celsius)
#define TEMP_THRESHOLD_COLD   18.0
#define TEMP_THRESHOLD_COOL   20.0
#define TEMP_THRESHOLD_WARM   25.0
#define TEMP_THRESHOLD_HOT    28.0

// Humidity thresholds (in %)
#define HUMID_THRESHOLD_DRY   30.0
#define HUMID_THRESHOLD_LOW   40.0
#define HUMID_THRESHOLD_HIGH  60.0
#define HUMID_THRESHOLD_WET   70.0

// Timing
#define SENSOR_READ_INTERVAL  5000      // Read sensor every 5 seconds
#define GRAPH_SAMPLE_INTERVAL 720000    // Graph point every 12 minutes
#define DISPLAY_TIMEOUT       10000     // Warn mode display timeout
```

## Display Layout

### Number View
```
+------------------------+
|        1234            |  <- Big colored number
|         ppm            |
|                        |
| Temp: 23.5C  Humid: 45%|  <- Small secondary values
|[ALWAYS ON] [NUM] [CO2] |  <- Status bar
+------------------------+
```

### Graph View
```
+------------------------+
| CO2 - 24h History 1234 |
| +------------------+   |
| |    ~~~graph~~~   |   |
| +------------------+   |
| -24h             now   |
|[ALWAYS ON] [GRAPH][CO2]|
+------------------------+
```

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

## License

MIT License - feel free to modify and use for your projects.
