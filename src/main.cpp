/**
 * ESP32-S3 CO2 Monitor Firmware
 *
 * Hardware:
 *   - Adafruit ESP32-S3 Reverse TFT Feather
 *   - Adafruit SCD-41 CO2 Sensor (I2C)
 *
 * Features:
 *   - Real-time CO2, Temperature, and Humidity display
 *   - Color-coded thresholds (configurable)
 *   - 24-hour history graph
 *   - Multiple display modes: Day, Night, ECO
 *   - Hyperdrive mode: faster readings when CO2 changes rapidly
 *   - Power saving: WiFi/BT disabled, single-shot sensor, unused GPIO management
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <driver/gpio.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_MAX1704X.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

// =============================================================================
// CONFIGURATION - Adjust these values to customize behavior
// =============================================================================

// Firmware Version
#define VERSION                 "1.0"

// CO2 Thresholds (in ppm)
#define CO2_COLOR_LOW           COLOR_GREEN   // Excellent
#define CO2_COLOR_MODERATE      COLOR_YELLOW  // Acceptable
#define CO2_COLOR_HIGH          COLOR_ORANGE  // Poor
#define CO2_COLOR_CRITICAL      COLOR_RED     // Unhealthy
#define CO2_THRESHOLD_LOW       800
#define CO2_THRESHOLD_MODERATE  1000
#define CO2_THRESHOLD_HIGH      1200
#define CO2_THRESHOLD_CRITICAL  1400

// Temperature thresholds (in Celsius)
#define TEMP_COLOR_COLD         COLOR_BLUE    // Too cold
#define TEMP_COLOR_COOL         COLOR_CYAN    // Cool
#define TEMP_COLOR_OPTIMAL      COLOR_WHITE   // Optimal
#define TEMP_COLOR_WARM         COLOR_YELLOW  // Warm
#define TEMP_COLOR_HOT          COLOR_ORANGE  // Too hot
#define TEMP_THRESHOLD_COLD     16.0
#define TEMP_THRESHOLD_COOL     20.0
#define TEMP_THRESHOLD_OPTIMAL  24.0
#define TEMP_THRESHOLD_WARM     28.0


// Humidity thresholds (in %)
#define HUMID_COLOR_DRY         COLOR_ORANGE  // Too dry
#define HUMID_COLOR_LOW         COLOR_YELLOW  // Low
#define HUMID_COLOR_OPTIMAL     COLOR_WHITE   // Optimal
#define HUMID_COLOR_HIGH        COLOR_CYAN    // High
#define HUMID_COLOR_WET         COLOR_BLUE    // Too wet
#define HUMID_THRESHOLD_DRY     30.0
#define HUMID_THRESHOLD_LOW     40.0
#define HUMID_THRESHOLD_OPTIMAL 60.0
#define HUMID_THRESHOLD_HIGH    70.0

// Timing Configuration
#define SENSOR_READ_INTERVAL       300000 // Read sensor every 5 minutes (normal mode)
#define SENSOR_READ_INTERVAL_FAST  60000  // Read sensor every 1 minute (hyperdrive mode)
#define GRAPH_SAMPLE_INTERVAL      300000 // Store graph point every 5 minutes
#define DISPLAY_TIMEOUT            10000  // Display timeout in Power Save mode (10 seconds)
#define DEBOUNCE_DELAY             200    // Button debounce time (ms)

// Hyperdrive Configuration
#define HYPERDRIVE_THRESHOLD       100    // CO2 change (ppm) to trigger hyperdrive
#define HYPERDRIVE_STABLE_COUNT    3      // Consecutive stable readings to exit hyperdrive

// Display Brightness (PWM values 0-255)
#define BRIGHTNESS_DAY             200    // Day mode brightness
#define BRIGHTNESS_NIGHT           80     // Night mode brightness
#define BRIGHTNESS_ECO             30     // ECO mode brightness (minimal)

// Deep Sleep Configuration
#define DEEP_SLEEP_US              300000000ULL  // 5 minutes in microseconds
#define RTC_DATA_MAGIC             0xC02CAFE     // Magic number for RTC data validation
#define CPU_FREQ_ACTIVE            160           // MHz when display is on
#define CPU_FREQ_SENSOR            80            // MHz for sensor-only wakes

// Battery Protection (cell voltage in volts, measured by MAX17048)
#define BATTERY_SHUTDOWN_VOLTAGE   3.30   // Shut down below this voltage to protect the LiPo
#define BATTERY_RECOVER_VOLTAGE    3.60   // Resume only above this voltage (hysteresis, prevents boot loops)
#define BATTERY_VALID_MIN_VOLTAGE  2.50   // Readings below this are treated as invalid (no battery / I2C error)
#define BATTERY_CHECK_INTERVAL     60000  // Check battery every minute while running
#define BATTERY_CHECK_SAMPLES      3      // Consecutive low readings required to shut down
#define LOW_BATTERY_SLEEP_US       1800000000ULL  // Re-check battery every 30 minutes while shut down

// EMA Filter Configuration (alpha: 0=max smoothing, 1=no smoothing)
#define EMA_ALPHA_CO2              0.5    // CO2 changes slowly, moderate smoothing
#define EMA_ALPHA_TEMPERATURE      0.5    // Temperature is stable, more smoothing
#define EMA_ALPHA_HUMIDITY         0.5    // Humidity is stable, more smoothing

// Trend Detection Thresholds (change required to show trend arrow)
#define TREND_THRESHOLD_CO2        20     // ppm change for CO2 trend
#define TREND_THRESHOLD_TEMP       0.3    // °C change for temperature trend
#define TREND_THRESHOLD_HUMIDITY   2.0    // % change for humidity trend

// SCD41 Sensor Configuration
#define SCD41_TEMPERATURE_OFFSET   4.5    // Temperature offset in °C (positive values only, subtracted from reading)
#define SCD41_ALTITUDE             525    // Altitude in meters (0 = sea level, set for your location)

// Graph Configuration
#define GRAPH_POINTS          288         // Number of data points in graph (24h at 5-min intervals)
#define GRAPH_HEIGHT          80          // Height of graph area in pixels
#define GRAPH_WIDTH           200         // Width of graph area in pixels
#define GRAPH_CO2_MIN         400         // CO2 graph minimum (ppm)
#define GRAPH_CO2_MAX         2500        // CO2 graph maximum (ppm)
#define GRAPH_TEMP_MIN        10.0        // Temperature graph minimum (°C)
#define GRAPH_TEMP_MAX        30.0        // Temperature graph maximum (°C)
#define GRAPH_HUMIDITY_MIN    0.0         // Humidity graph minimum (%)
#define GRAPH_HUMIDITY_MAX    100.0       // Humidity graph maximum (%)

// Hardware Pins (ESP32-S3 Reverse TFT Feather)
// TFT pins are defined by the board support package: TFT_CS, TFT_DC, TFT_RST, TFT_BACKLITE, TFT_I2C_POWER
#define BUTTON_D0             0           // Boot button - pulled HIGH, goes LOW when pressed
#define BUTTON_D1             1           // User button - pulled LOW, goes HIGH when pressed
#define BUTTON_D2             2           // User button - pulled LOW, goes HIGH when pressed

// =============================================================================
// COLOR DEFINITIONS (RGB565 format)
// =============================================================================

#define COLOR_BLACK       0x0000
#define COLOR_WHITE       0xFFFF
#define COLOR_RED         0xF800
#define COLOR_ORANGE      0xFD20
#define COLOR_YELLOW      0xFFE0
#define COLOR_GREEN       0x07E0
#define COLOR_CYAN        0x07FF
#define COLOR_BLUE        0x001F
#define COLOR_DARK_GRAY   0x4208
#define COLOR_LIGHT_GRAY  0xC618

// =============================================================================
// ENUMS AND TYPES
// =============================================================================

enum DisplayValue {
  DISPLAY_CO2,
  DISPLAY_TEMPERATURE,
  DISPLAY_HUMIDITY,
  DISPLAY_VALUE_COUNT
};

enum ViewMode {
  VIEW_NUMBER,
  VIEW_GRAPH,
  VIEW_MODE_COUNT
};

enum DisplayMode {
  MODE_DAY,         // Full brightness, always on
  MODE_NIGHT,       // Reduced brightness, always on
  MODE_ECO,         // Display off, turns on when threshold exceeded or button pressed
  DISPLAY_MODE_COUNT
};

// =============================================================================
// RTC PERSISTENT STATE (survives deep sleep)
// =============================================================================

struct RtcState {
  uint32_t magic;

  // Sensor readings (raw)
  uint16_t rawCO2;
  float rawTemperature;
  float rawHumidity;

  // Filtered readings (EMA smoothed)
  uint16_t currentCO2;
  float currentTemperature;
  float currentHumidity;
  bool emaInitialized;

  // Graph history (circular buffer)
  uint16_t historyCO2[GRAPH_POINTS];
  float historyTemperature[GRAPH_POINTS];
  float historyHumidity[GRAPH_POINTS];
  int historyIndex;
  int historyCount;

  // UI state
  DisplayValue currentDisplayValue;
  ViewMode currentViewMode;
  DisplayMode currentDisplayMode;

  // Trend tracking
  uint16_t trendCO2;
  float trendTemperature;
  float trendHumidity;
  int8_t co2Trend;
  int8_t tempTrend;
  int8_t humidTrend;

  // Hyperdrive state
  bool hyperdriveActive;
  uint16_t lastHyperdriveCO2;
  int hyperdriveStableCount;
};

RTC_DATA_ATTR RtcState rtc;

// Set while shut down for low battery. Kept outside RtcState so it survives
// the cold boot memset; cleared automatically on power-on and brownout resets.
RTC_DATA_ATTR bool lowBatteryShutdown = false;

// Aliases so existing code compiles without renaming every reference
#define rawCO2              rtc.rawCO2
#define rawTemperature      rtc.rawTemperature
#define rawHumidity         rtc.rawHumidity
#define currentCO2          rtc.currentCO2
#define currentTemperature  rtc.currentTemperature
#define currentHumidity     rtc.currentHumidity
#define emaInitialized      rtc.emaInitialized
#define historyCO2          rtc.historyCO2
#define historyTemperature  rtc.historyTemperature
#define historyHumidity     rtc.historyHumidity
#define historyIndex        rtc.historyIndex
#define historyCount        rtc.historyCount
#define currentDisplayValue rtc.currentDisplayValue
#define currentViewMode     rtc.currentViewMode
#define currentDisplayMode  rtc.currentDisplayMode
#define trendCO2            rtc.trendCO2
#define trendTemperature    rtc.trendTemperature
#define trendHumidity       rtc.trendHumidity
#define co2Trend            rtc.co2Trend
#define tempTrend           rtc.tempTrend
#define humidTrend          rtc.humidTrend
#define hyperdriveActive    rtc.hyperdriveActive
#define lastHyperdriveCO2   rtc.lastHyperdriveCO2
#define hyperdriveStableCount rtc.hyperdriveStableCount

// =============================================================================
// GLOBAL STATE (re-initialized each wake cycle, not persisted)
// =============================================================================

// Hardware objects
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
SensirionI2CScd4x scd4x;
Adafruit_MAX17048 batteryMonitor;
bool batteryMonitorAvailable = false;

// Wake-cycle state
bool displayOn = true;
bool sensorSingleShotMode = false;
bool forceDisplayRefresh = true;
unsigned long lastSensorRead = 0;
unsigned long lastGraphSample = 0;
unsigned long lastDisplayActivity = 0;
unsigned long lastBatteryCheck = 0;
unsigned long lastButtonPress[3] = {0, 0, 0};

// Previous values for display update optimization (reset each wake)
uint16_t prevCO2 = 0;
float prevTemperature = 0.0;
float prevHumidity = 0.0;
DisplayValue prevDisplayValue = DISPLAY_CO2;
ViewMode prevViewMode = VIEW_NUMBER;
DisplayMode prevDisplayMode = MODE_DAY;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

void initDisplay();
void initSensor();
void readSensor();
void updateHistory();
void updateDisplay();
void drawNumberView();
void drawGraphView();
void drawSmallValues();
void drawGraph();
void drawStatusBar();
uint16_t getCO2Color(uint16_t co2);
uint16_t getTemperatureColor(float temp);
uint16_t getHumidityColor(float humidity);
void drawThresholdLine(int graphX, int graphY, int graphWidth, int graphHeight,
                       float threshold, float minVal, float maxVal, uint16_t color);
void handleButtons();
void setDisplayPower(bool on);
void setDisplayBrightness(int brightness);
const char* getDisplayValueName(DisplayValue val);
const char* getDisplayModeName(DisplayMode mode);
void updateTrends();
void drawTrendArrow(int x, int y, int8_t trend, uint16_t color);
int getBatteryPercent();
void initPowerSaving();
void updateHyperdrive();
void setSensorMode(bool singleShot);
void enterDeepSleep();
void enableButtonWake();
float readBatteryVoltage();
bool isBatteryBelow(float threshold);
void enterLowBatterySleep();
void initDisplayWarm();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
  bool warmBoot = (rtc.magic == RTC_DATA_MAGIC);
  bool buttonWake = (wakeReason == ESP_SLEEP_WAKEUP_EXT1);

  // CPU: 80 MHz for sensor-only wakes, 160 MHz when display is needed
  setCpuFrequencyMhz((warmBoot && !buttonWake) ? CPU_FREQ_SENSOR : CPU_FREQ_ACTIVE);

  // Release GPIO hold from previous deep sleep
  gpio_hold_dis((gpio_num_t)TFT_I2C_POWER);
  gpio_deep_sleep_hold_dis();

  Serial.begin(115200);
  delay(warmBoot ? 100 : 1000);

  // Initialize power saving (disable WiFi, Bluetooth, unused GPIOs)
  initPowerSaving();

  // Initialize buttons
  pinMode(BUTTON_D0, INPUT_PULLUP);
  pinMode(BUTTON_D1, INPUT_PULLDOWN);
  pinMode(BUTTON_D2, INPUT_PULLDOWN);

  // Ensure I2C power is on
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // Initialize battery monitor
  if (batteryMonitor.begin()) {
    batteryMonitorAvailable = true;
    Serial.println("MAX17048 battery monitor initialized");
  } else {
    Serial.println("MAX17048 not found, battery display disabled");
  }

  // Battery protection: check before display and sensor draw any current.
  // After a low battery shutdown or a brownout reset the battery must reach
  // the higher recover voltage first, otherwise the load of booting would
  // drop the voltage again and the device would boot loop.
  bool needsRecovery = lowBatteryShutdown || (esp_reset_reason() == ESP_RST_BROWNOUT);
  if (isBatteryBelow(needsRecovery ? BATTERY_RECOVER_VOLTAGE : BATTERY_SHUTDOWN_VOLTAGE)) {
    enterLowBatterySleep();
    // Never reached
  }
  if (lowBatteryShutdown) {
    // Battery recovered: sensor was powered off, so start over with a cold boot
    Serial.println("Battery recovered, restarting");
    lowBatteryShutdown = false;
    rtc.magic = 0;
    warmBoot = false;
    setCpuFrequencyMhz(CPU_FREQ_ACTIVE);
  }

  if (!warmBoot) {
    // ==================== COLD BOOT ====================
    Serial.printf("ESP32-S3 CO2 Monitor v%s Cold Start (CPU: %d MHz)\n",
                  VERSION, getCpuFrequencyMhz());

    // Zero out RTC state
    memset(&rtc, 0, sizeof(rtc));
    currentDisplayMode = MODE_DAY;
    currentDisplayValue = DISPLAY_CO2;
    currentViewMode = VIEW_NUMBER;
    hyperdriveActive = true;

    // Initialize display (full init with splash screen)
    initDisplay();

    // Initialize sensor (full config)
    initSensor();

    // Wait for first valid sensor reading
    Serial.println("Waiting for first sensor reading...");
    while (currentCO2 == 0) {
      readSensor();
      delay(500);
    }
    updateHistory();
    lastHyperdriveCO2 = currentCO2;
    rtc.magic = RTC_DATA_MAGIC;

    Serial.println("Cold boot setup complete!");
    forceDisplayRefresh = true;

  } else {
    // ==================== WARM BOOT (from deep sleep) ====================
    Serial.printf("ESP32-S3 CO2 Monitor v%s Warm Boot (%s, CPU: %d MHz)\n",
                  VERSION, buttonWake ? "button" : "timer", getCpuFrequencyMhz());

    // Re-initialize I2C bus and sensor (bus state lost during deep sleep)
    sensorSingleShotMode = true;  // Always in ECO mode during deep sleep
    Wire.begin();
    scd4x.begin(Wire);

    // Read sensor (single-shot)
    readSensor();
    updateHistory();

    if (buttonWake) {
      // Button wake: turn on display, enter active loop
      setCpuFrequencyMhz(CPU_FREQ_ACTIVE);
      initDisplayWarm();
      displayOn = true;
      lastDisplayActivity = millis();
      forceDisplayRefresh = true;
      Serial.println("Display activated by button press");
    } else {
      // Timer wake: sensor read done, go back to sleep immediately
      Serial.println("Timer wake: sensor read complete, returning to sleep");
      enterDeepSleep();
      // Never reached
    }
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  // Handle button input
  handleButtons();

  // Battery protection: shut down before the battery is fully drained
  if (millis() - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    lastBatteryCheck = millis();
    if (isBatteryBelow(BATTERY_SHUTDOWN_VOLTAGE)) {
      if (displayOn) {
        tft.fillScreen(COLOR_BLACK);
        tft.setTextColor(COLOR_RED);
        tft.setTextSize(2);
        tft.setCursor(54, 60);
        tft.print("Battery low");
        delay(3000);
      }
      enterLowBatterySleep();
      // Never reached
    }
  }

  // ECO mode: display timeout -> deep sleep
  if (currentDisplayMode == MODE_ECO && displayOn) {
    if (millis() - lastDisplayActivity >= DISPLAY_TIMEOUT) {
      Serial.println("ECO display timeout, entering deep sleep");
      setDisplayPower(false);
      enterDeepSleep();
      // Never reached
    }
  }

  // Sensor reads (DAY/NIGHT only - ECO reads happen in setup() on wake)
  if (currentDisplayMode != MODE_ECO) {
    unsigned long currentInterval = hyperdriveActive ?
      SENSOR_READ_INTERVAL_FAST : SENSOR_READ_INTERVAL;

    if (millis() - lastSensorRead >= currentInterval) {
      readSensor();
      lastSensorRead = millis();
      forceDisplayRefresh = true;
      updateHyperdrive();
    }
  }

  // History update (DAY/NIGHT only - ECO updates in setup() on wake)
  if (currentDisplayMode != MODE_ECO) {
    if (millis() - lastGraphSample >= GRAPH_SAMPLE_INTERVAL) {
      updateHistory();
      lastGraphSample = millis();
    }
  }

  // Update display (smart refresh - only redraws when values change)
  if (displayOn) {
    updateDisplay();
    forceDisplayRefresh = false;
  }

  delay(100);
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

void initDisplay() {
  // Enable power to the TFT and I2C
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // Enable backlight with PWM (start at full brightness)
  pinMode(TFT_BACKLITE, OUTPUT);
  analogWrite(TFT_BACKLITE, BRIGHTNESS_DAY);

  // Initialize ST7789 240x135 display
  tft.init(135, 240);
  tft.setRotation(3);  // Landscape mode, USB port on the left
  tft.fillScreen(COLOR_BLACK);

  // Show startup message (centered)
  // Display is 240x135 pixels
  tft.setTextColor(COLOR_WHITE);

  // "CO₂ Monitor" with subscript 2
  // "CO" size 2 = 2 chars * 12px = 24px
  // "2" size 1 = 6px (subscript)
  // " Monitor" size 2 = 8 chars * 12px = 96px
  // Total width approx: 24 + 6 + 96 = 126px
  int startX = (240 - 126) / 2;
  int baseY = 52;

  tft.setTextSize(2);
  tft.setCursor(startX, baseY);
  tft.print("CO");

  // Subscript "2" - smaller and lower
  tft.setTextSize(1);
  tft.setCursor(startX + 24, baseY + 8);  // Lower position for subscript
  tft.print("2");

  // Continue with " Monitor"
  tft.setTextSize(2);
  tft.setCursor(startX + 30, baseY);
  tft.print(" Monitor");

  // Version - size 1, 4 chars * 6px = 24px wide
  tft.setTextColor(COLOR_LIGHT_GRAY);
  tft.setTextSize(1);
  tft.setCursor((240 - 24) / 2, 75);
  tft.print("v" VERSION);

  displayOn = true;
  lastDisplayActivity = millis();
}

void initSensor() {
  Wire.begin();

  // SCD41 default I2C address is 0x62
  scd4x.begin(Wire);

  // Wake sensor first - it may be in powerDown state from a previous run
  // (reflashing the ESP32 doesn't cut power to the sensor)
  scd4x.wakeUp();
  delay(30);

  // Stop any previous measurements (required before configuration)
  uint16_t error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.println("Error stopping measurement");
  }
  delay(500);

  // Check sensor serial number to verify communication
  uint16_t serial0, serial1, serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    Serial.println("SCD4x sensor not found!");
    tft.fillScreen(COLOR_BLACK);
    tft.setTextColor(COLOR_RED);
    tft.setTextSize(2);
    tft.setCursor(20, 50);
    tft.println("Sensor Error!");
    tft.setTextSize(1);
    tft.setCursor(20, 80);
    tft.println("Check SCD-41 connection");
    while (1) delay(1000);
  }

  Serial.printf("SCD-41 Serial: %04X%04X%04X\n", serial0, serial1, serial2);

  // === SCD41 Best Practices Configuration ===

  // Set temperature offset to compensate for sensor self-heating
  // The SCD41 generates heat which affects temperature readings
  error = scd4x.setTemperatureOffset(SCD41_TEMPERATURE_OFFSET);
  if (error) {
    Serial.println("Error setting temperature offset");
  } else {
    Serial.printf("Temperature offset set to %.1f C\n", SCD41_TEMPERATURE_OFFSET);
  }

  // Set sensor altitude for CO2 pressure compensation
  // This improves CO2 accuracy based on local atmospheric pressure
  if (SCD41_ALTITUDE > 0) {
    error = scd4x.setSensorAltitude(SCD41_ALTITUDE);
    if (error) {
      Serial.println("Error setting sensor altitude");
    } else {
      Serial.printf("Sensor altitude set to %d m\n", SCD41_ALTITUDE);
    }
  }

  // Enable automatic self-calibration (ASC)
  // ASC assumes exposure to ~400ppm fresh air at least once per week
  // This is enabled by default, but we explicitly enable it to be sure
  error = scd4x.setAutomaticSelfCalibration(1);
  if (error) {
    Serial.println("Error enabling automatic self-calibration");
  } else {
    Serial.println("Automatic self-calibration enabled");
  }

  // Start periodic measurements
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.println("Failed to start measurements!");
  }

  Serial.println("SCD-41 sensor initialized");

  // Wait for first measurement and discard it (recommended by Sensirion)
  // The first reading after power-on may be less accurate
  Serial.println("Discarding first sensor reading...");
  delay(5000);
  uint16_t co2;
  float temp, hum;
  scd4x.readMeasurement(co2, temp, hum);  // Discard first reading

  tft.fillScreen(COLOR_BLACK);
}

// =============================================================================
// SENSOR FUNCTIONS
// =============================================================================

void readSensor() {
  uint16_t co2 = 0;
  float temp = 0.0f;
  float humidity = 0.0f;
  uint16_t error;

  if (sensorSingleShotMode) {
    // Single shot mode: trigger measurement and wait for result
    // Wake sensor from power down if needed
    scd4x.wakeUp();
    delay(30);  // Wait for sensor to wake

    // Trigger single shot measurement
    error = scd4x.measureSingleShot();
    if (error) {
      Serial.println("Error triggering single shot measurement");
      return;
    }

    // Wait for measurement to complete (5 seconds typical)
    delay(5000);

    // Read the measurement
    error = scd4x.readMeasurement(co2, temp, humidity);
    if (error) {
      Serial.println("Error reading single shot measurement");
      return;
    }

    // Put sensor back to sleep to save power
    scd4x.powerDown();

  } else {
    // Periodic mode: check if data is ready
    bool dataReady = false;
    error = scd4x.getDataReadyFlag(dataReady);

    if (error) {
      Serial.println("Error checking data ready status");
      return;
    }

    if (!dataReady) {
      return;  // No data available yet
    }

    error = scd4x.readMeasurement(co2, temp, humidity);
    if (error) {
      Serial.println("Error reading measurement");
      return;
    }
  }

  // Only update if we got valid data (CO2 = 0 means invalid)
  if (co2 > 0) {
    // Store raw readings
    rawCO2 = co2;
    rawTemperature = temp;
    rawHumidity = humidity;

    // Apply EMA filter
    if (!emaInitialized) {
      // First valid reading: initialize filtered values directly
      currentCO2 = co2;
      currentTemperature = temp;
      currentHumidity = humidity;
      emaInitialized = true;
      Serial.println("EMA filter initialized with first reading");
    } else {
      // Apply exponential moving average filter
      // Formula: filtered = alpha * new + (1 - alpha) * filtered
      currentCO2 = (uint16_t)(EMA_ALPHA_CO2 * co2 + (1.0 - EMA_ALPHA_CO2) * currentCO2);
      currentTemperature = EMA_ALPHA_TEMPERATURE * temp + (1.0 - EMA_ALPHA_TEMPERATURE) * currentTemperature;
      currentHumidity = EMA_ALPHA_HUMIDITY * humidity + (1.0 - EMA_ALPHA_HUMIDITY) * currentHumidity;
    }

    // Update trends (based on filtered values)
    updateTrends();

    Serial.printf("Raw:      CO2=%d ppm, Temp=%.1f C, Humidity=%.1f %%\n",
                  rawCO2, rawTemperature, rawHumidity);
    Serial.printf("Filtered: CO2=%d ppm, Temp=%.1f C, Humidity=%.1f %%\n",
                  currentCO2, currentTemperature, currentHumidity);
  }
}

void updateTrends() {
  // CO2 trend
  if (currentCO2 > trendCO2 + TREND_THRESHOLD_CO2) {
    co2Trend = 1;
  } else if (currentCO2 < trendCO2 - TREND_THRESHOLD_CO2) {
    co2Trend = -1;
  } else {
    co2Trend = 0;
  }
  trendCO2 = currentCO2;

  // Temperature trend
  if (currentTemperature > trendTemperature + TREND_THRESHOLD_TEMP) {
    tempTrend = 1;
  } else if (currentTemperature < trendTemperature - TREND_THRESHOLD_TEMP) {
    tempTrend = -1;
  } else {
    tempTrend = 0;
  }
  trendTemperature = currentTemperature;

  // Humidity trend
  if (currentHumidity > trendHumidity + TREND_THRESHOLD_HUMIDITY) {
    humidTrend = 1;
  } else if (currentHumidity < trendHumidity - TREND_THRESHOLD_HUMIDITY) {
    humidTrend = -1;
  } else {
    humidTrend = 0;
  }
  trendHumidity = currentHumidity;
}

void updateHistory() {
  historyCO2[historyIndex] = currentCO2;
  historyTemperature[historyIndex] = currentTemperature;
  historyHumidity[historyIndex] = currentHumidity;

  historyIndex = (historyIndex + 1) % GRAPH_POINTS;
  if (historyCount < GRAPH_POINTS) {
    historyCount++;
  }
}

// =============================================================================
// DISPLAY FUNCTIONS
// =============================================================================

void updateDisplay() {
  // Check if anything actually changed
  bool modeChanged = (currentDisplayValue != prevDisplayValue) ||
                     (currentViewMode != prevViewMode) ||
                     (currentDisplayMode != prevDisplayMode);

  bool valuesChanged = (currentCO2 != prevCO2) ||
                       (abs(currentTemperature - prevTemperature) > 0.05) ||
                       (abs(currentHumidity - prevHumidity) > 0.5);

  // Only redraw if something changed or forced
  if (!modeChanged && !valuesChanged && !forceDisplayRefresh) {
    return;
  }

  // Full screen clear only on mode change
  if (modeChanged) {
    tft.fillScreen(COLOR_BLACK);
  }

  switch (currentViewMode) {
    case VIEW_NUMBER:
      drawNumberView();
      break;
    case VIEW_GRAPH:
      drawGraphView();
      break;
    default:
      break;
  }

  drawStatusBar();

  // Update previous values
  prevDisplayValue = currentDisplayValue;
  prevViewMode = currentViewMode;
  prevDisplayMode = currentDisplayMode;
  prevCO2 = currentCO2;
  prevTemperature = currentTemperature;
  prevHumidity = currentHumidity;
}

void drawNumberView() {
  // Screen layout (240x135):
  // - Status bar: top (y: 0-17)
  // - Main value + unit: left-aligned (y: 25-75)
  // - Trend arrow: right-aligned
  // - Secondary values: bottom (y: 95-125)

  // Clear main display area (below status bar)
  tft.fillRect(0, 18, 240, 117, COLOR_BLACK);

  // Draw the main big number left-aligned
  char valueStr[16];
  char unitStr[8];
  uint16_t color;
  int8_t trend;

  // Use consistent font size 7 for all values
  int textSize = 7;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      snprintf(valueStr, sizeof(valueStr), "%d", currentCO2);
      snprintf(unitStr, sizeof(unitStr), "ppm");
      color = getCO2Color(currentCO2);
      trend = co2Trend;
      break;
    case DISPLAY_TEMPERATURE:
      snprintf(valueStr, sizeof(valueStr), "%.1f", currentTemperature);
      snprintf(unitStr, sizeof(unitStr), "C");
      color = getTemperatureColor(currentTemperature);
      trend = tempTrend;
      break;
    case DISPLAY_HUMIDITY:
      snprintf(valueStr, sizeof(valueStr), "%.0f", currentHumidity);
      snprintf(unitStr, sizeof(unitStr), "%%");
      color = getHumidityColor(currentHumidity);
      trend = humidTrend;
      break;
    default:
      return;
  }

  // Left-aligned main value with more top spacing (below status bar)
  int startX = 10;
  int valueY = 32;

  // Adafruit GFX font: 7 pixels tall at size 1, 6 pixels wide
  int valueHeight = 7 * textSize;  // Height of big value in pixels

  // Draw main value
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.setCursor(startX, valueY);
  tft.print(valueStr);

  // Calculate where value ends
  int charWidth = 6 * textSize;
  int valueWidth = strlen(valueStr) * charWidth;

  // Draw unit on same line, aligned with bottom of big number
  int unitSize = 2;
  int unitHeight = 7 * unitSize;  // Height of unit text
  int unitY = valueY + valueHeight - unitHeight;  // Align bottoms
  tft.setTextSize(unitSize);
  tft.setCursor(startX + valueWidth + 8, unitY);
  tft.print(unitStr);

  // Draw trend arrow right-aligned, aligned with TOP of big value
  int arrowX = 215;  // Right side of screen
  int arrowY = valueY + 12;  // Align with top of big value
  drawTrendArrow(arrowX, arrowY, trend, color);

  // Draw secondary values at bottom with more spacing
  drawSmallValues();
}

void drawTrendArrow(int x, int y, int8_t trend, uint16_t color) {
  // Equilateral triangle: height = width * √3/2 ≈ width * 0.866
  int arrowWidth = 14;
  int arrowHeight = 12;  // 14 * 0.866 ≈ 12

  if (trend == 0) {
    // No change - draw a rectangular block
    tft.fillRect(x, y - arrowHeight/4, arrowWidth, arrowHeight/2, color);
    return;
  }

  if (trend > 0) {
    // Up arrow
    tft.fillTriangle(
      x + arrowWidth/2, y - arrowHeight/2,      // Top point
      x, y + arrowHeight/2,                      // Bottom left
      x + arrowWidth, y + arrowHeight/2,         // Bottom right
      color
    );
  } else {
    // Down arrow
    tft.fillTriangle(
      x + arrowWidth/2, y + arrowHeight/2,      // Bottom point
      x, y - arrowHeight/2,                      // Top left
      x + arrowWidth, y - arrowHeight/2,         // Top right
      color
    );
  }
}

void drawSmallValues() {
  // Draw secondary readings: left-aligned on left, right-aligned on right
  // Position lower with more spacing from big value
  int yPos = 100;
  int leftX = 10;
  int rightMargin = 230;  // Right edge for right-aligned text

  char leftValue[16];
  char leftLabel[16];
  char rightValue[16];
  char rightLabel[16];
  uint16_t leftColor, rightColor;

  // Determine which two values to show (the ones not currently selected as main)
  if (currentDisplayValue == DISPLAY_CO2) {
    // Main is CO2, show Temp left, Humidity right
    snprintf(leftValue, sizeof(leftValue), "%.1f C", currentTemperature);
    snprintf(leftLabel, sizeof(leftLabel), "Temp");
    leftColor = getTemperatureColor(currentTemperature);

    snprintf(rightValue, sizeof(rightValue), "%.0f%%", currentHumidity);
    snprintf(rightLabel, sizeof(rightLabel), "Humidity");
    rightColor = getHumidityColor(currentHumidity);
  } else if (currentDisplayValue == DISPLAY_TEMPERATURE) {
    // Main is Temp, show CO2 left, Humidity right
    snprintf(leftValue, sizeof(leftValue), "%d", currentCO2);
    snprintf(leftLabel, sizeof(leftLabel), "CO2 ppm");
    leftColor = getCO2Color(currentCO2);

    snprintf(rightValue, sizeof(rightValue), "%.0f%%", currentHumidity);
    snprintf(rightLabel, sizeof(rightLabel), "Humidity");
    rightColor = getHumidityColor(currentHumidity);
  } else {
    // Main is Humidity, show CO2 left, Temp right
    snprintf(leftValue, sizeof(leftValue), "%d", currentCO2);
    snprintf(leftLabel, sizeof(leftLabel), "CO2 ppm");
    leftColor = getCO2Color(currentCO2);

    snprintf(rightValue, sizeof(rightValue), "%.1f C", currentTemperature);
    snprintf(rightLabel, sizeof(rightLabel), "Temp");
    rightColor = getTemperatureColor(currentTemperature);
  }

  // Draw left value (left-aligned)
  tft.setTextSize(2);
  tft.setTextColor(leftColor);
  tft.setCursor(leftX, yPos);
  tft.print(leftValue);
  tft.setTextSize(1);
  tft.setCursor(leftX, yPos + 18);
  tft.print(leftLabel);

  // Draw right value (right-aligned)
  tft.setTextSize(2);
  int rightValueWidth = strlen(rightValue) * 12;
  tft.setTextColor(rightColor);
  tft.setCursor(rightMargin - rightValueWidth, yPos);
  tft.print(rightValue);
  tft.setTextSize(1);
  int rightLabelWidth = strlen(rightLabel) * 6;
  tft.setCursor(rightMargin - rightLabelWidth, yPos + 18);
  tft.print(rightLabel);
}

void drawGraphView() {
  // Clear display area (below status bar at top)
  tft.fillRect(0, 18, 240, 117, COLOR_BLACK);

  // No header text - maximize graph area
  drawGraph();
}

void drawGraph() {
  // Graph area - use full width, no border, more space under status bar
  int graphX = 5;
  int graphY = 26;         // Start below status bar with padding
  int graphWidth = 230;    // Use full width (labels inside)
  int graphHeight = 106;   // Fill remaining vertical space

  // Find min/max for scaling
  float minVal = 0, maxVal = 0;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      minVal = GRAPH_CO2_MIN;
      maxVal = GRAPH_CO2_MAX;
      for (int i = 0; i < historyCount; i++) {
        if (historyCO2[i] > maxVal) maxVal = historyCO2[i];
      }
      break;
    case DISPLAY_TEMPERATURE:
      minVal = GRAPH_TEMP_MIN;
      maxVal = GRAPH_TEMP_MAX;
      break;
    case DISPLAY_HUMIDITY:
      minVal = GRAPH_HUMIDITY_MIN;
      maxVal = GRAPH_HUMIDITY_MAX;
      break;
    default:
      break;
  }

  // Draw threshold lines
  if (currentDisplayValue == DISPLAY_CO2) {
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, CO2_THRESHOLD_LOW, minVal, maxVal, CO2_COLOR_LOW);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, CO2_THRESHOLD_MODERATE, minVal, maxVal, CO2_COLOR_MODERATE);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, CO2_THRESHOLD_HIGH, minVal, maxVal, CO2_COLOR_HIGH);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, CO2_THRESHOLD_CRITICAL, minVal, maxVal, CO2_COLOR_CRITICAL);
  } else if (currentDisplayValue == DISPLAY_TEMPERATURE) {
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, TEMP_THRESHOLD_COLD, minVal, maxVal, TEMP_COLOR_COLD);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, TEMP_THRESHOLD_COOL, minVal, maxVal, TEMP_COLOR_COOL);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, TEMP_THRESHOLD_OPTIMAL, minVal, maxVal, TEMP_COLOR_WARM);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, TEMP_THRESHOLD_WARM, minVal, maxVal, TEMP_COLOR_HOT);
  } else if (currentDisplayValue == DISPLAY_HUMIDITY) {
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, HUMID_THRESHOLD_DRY, minVal, maxVal, HUMID_COLOR_DRY);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, HUMID_THRESHOLD_LOW, minVal, maxVal, HUMID_COLOR_LOW);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, HUMID_THRESHOLD_OPTIMAL, minVal, maxVal, HUMID_COLOR_HIGH);
    drawThresholdLine(graphX, graphY, graphWidth, graphHeight, HUMID_THRESHOLD_HIGH, minVal, maxVal, HUMID_COLOR_WET);
  }

  // Draw the graph line in white (always a line, no dots)
  if (historyCount > 1) {
    float xStep = (float)graphWidth / (GRAPH_POINTS - 1);

    for (int i = 1; i < historyCount; i++) {
      // Get indices in circular buffer order (oldest to newest)
      int idx0 = (historyIndex - historyCount + i - 1 + GRAPH_POINTS) % GRAPH_POINTS;
      int idx1 = (historyIndex - historyCount + i + GRAPH_POINTS) % GRAPH_POINTS;

      float val0, val1;

      switch (currentDisplayValue) {
        case DISPLAY_CO2:
          val0 = historyCO2[idx0];
          val1 = historyCO2[idx1];
          break;
        case DISPLAY_TEMPERATURE:
          val0 = historyTemperature[idx0];
          val1 = historyTemperature[idx1];
          break;
        case DISPLAY_HUMIDITY:
          val0 = historyHumidity[idx0];
          val1 = historyHumidity[idx1];
          break;
        default:
          val0 = val1 = 0;
          break;
      }

      // Scale to graph coordinates
      int x0 = graphX + (int)((i - 1) * xStep);
      int x1 = graphX + (int)(i * xStep);
      int y0 = graphY + graphHeight - (int)(((val0 - minVal) / (maxVal - minVal)) * graphHeight);
      int y1 = graphY + graphHeight - (int)(((val1 - minVal) / (maxVal - minVal)) * graphHeight);

      // Clamp to graph area
      y0 = constrain(y0, graphY, graphY + graphHeight);
      y1 = constrain(y1, graphY, graphY + graphHeight);

      // Always draw white line
      tft.drawLine(x0, y0, x1, y1, COLOR_WHITE);
    }
  }

  // Draw scale labels INSIDE the graph (top-right and bottom-right corners)
  tft.setTextColor(COLOR_DARK_GRAY);
  tft.setTextSize(1);

  // Max value label (top-right inside graph)
  char maxLabel[10];
  snprintf(maxLabel, sizeof(maxLabel), "%.0f", maxVal);
  int maxLabelWidth = strlen(maxLabel) * 6;
  tft.setCursor(graphX + graphWidth - maxLabelWidth - 3, graphY + 3);
  tft.print(maxLabel);

  // Min value label (bottom-right inside graph)
  char minLabel[10];
  snprintf(minLabel, sizeof(minLabel), "%.0f", minVal);
  int minLabelWidth = strlen(minLabel) * 6;
  tft.setCursor(graphX + graphWidth - minLabelWidth - 3, graphY + graphHeight - 11);
  tft.print(minLabel);

  // Datapoint counter (bottom-left inside graph)
  char countLabel[16];
  snprintf(countLabel, sizeof(countLabel), "n=%d", historyCount);
  tft.setCursor(graphX + 3, graphY + graphHeight - 11);
  tft.print(countLabel);
}

void drawStatusBar() {
  // Status bar at TOP (y: 0-17, height: 17px)
  int yPos = 0;
  int barHeight = 17;

  // Black background (no border)
  tft.fillRect(0, yPos, 240, barHeight, COLOR_BLACK);

  tft.setTextSize(1);
  int textY = yPos + 5;

  // Left: Colored dot + metric label (white text)
  uint16_t metricColor;
  const char* metricLabel;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      metricColor = getCO2Color(currentCO2);
      metricLabel = "CO2";
      break;
    case DISPLAY_TEMPERATURE:
      metricColor = getTemperatureColor(currentTemperature);
      metricLabel = "TEMP";
      break;
    case DISPLAY_HUMIDITY:
      metricColor = getHumidityColor(currentHumidity);
      metricLabel = "HUMID";
      break;
    default:
      metricColor = COLOR_WHITE;
      metricLabel = "";
      break;
  }

  // Draw colored dot indicator
  tft.fillCircle(10, yPos + barHeight/2, 4, metricColor);

  // Draw metric label in white
  tft.setTextColor(COLOR_WHITE);
  tft.setCursor(20, textY);
  tft.print(metricLabel);

  // Center: Mode icon + display mode + hyperdrive indicator
  const char* modeLabel;
  switch (currentDisplayMode) {
    case MODE_DAY:   modeLabel = "DAY"; break;
    case MODE_NIGHT: modeLabel = "NIGHT"; break;
    case MODE_ECO:   modeLabel = "ECO"; break;
    default:         modeLabel = ""; break;
  }

  // Calculate total width: icon (12px) + gap (5px) + text
  int iconWidth = 12;
  int gap = 5;
  int textWidth = strlen(modeLabel) * 6;
  int totalWidth = iconWidth + gap + textWidth;
  int startX = 120 - totalWidth / 2;

  uint16_t modeColor = COLOR_LIGHT_GRAY;
  int iconCenterY = yPos + barHeight / 2;

  // Draw mode icon (lightning bolt if hyperdrive active)
  if (hyperdriveActive) {
    // Lightning bolt icon
    tft.fillTriangle(
      startX + 5, iconCenterY - 5,   // Top point
      startX + 1, iconCenterY,       // Middle left
      startX + 5, iconCenterY,       // Middle center
      modeColor
    );
    tft.fillTriangle(
      startX + 4, iconCenterY,       // Middle center
      startX + 8, iconCenterY,       // Middle right
      startX + 4, iconCenterY + 5,   // Bottom point
      modeColor
    );
  } else {
    switch (currentDisplayMode) {
      case MODE_DAY:
        // Sun icon: small filled circle with visible rays
        tft.fillCircle(startX + 4, iconCenterY, 2, modeColor);
        // Cardinal rays (short lines)
        tft.drawFastVLine(startX + 4, iconCenterY - 6, 2, modeColor);  // Top
        tft.drawFastVLine(startX + 4, iconCenterY + 5, 2, modeColor);  // Bottom
        tft.drawFastHLine(startX - 2, iconCenterY, 2, modeColor);      // Left
        tft.drawFastHLine(startX + 9, iconCenterY, 2, modeColor);      // Right
        // Diagonal rays
        tft.drawPixel(startX, iconCenterY - 4, modeColor);
        tft.drawPixel(startX + 8, iconCenterY - 4, modeColor);
        tft.drawPixel(startX, iconCenterY + 4, modeColor);
        tft.drawPixel(startX + 8, iconCenterY + 4, modeColor);
        break;
      case MODE_NIGHT:
        // Moon icon: crescent shape
        tft.fillCircle(startX + 4, iconCenterY, 4, modeColor);
        tft.fillCircle(startX + 6, iconCenterY - 1, 3, COLOR_BLACK);  // Cut out crescent
        break;
      case MODE_ECO:
        // Simple 4-pointed star (celestial theme: sun → moon → star)
        {
          int cx = startX + 5;
          int cy = iconCenterY;
          // Vertical line with pointed ends
          tft.drawFastVLine(cx, cy - 5, 11, modeColor);
          // Horizontal line with pointed ends
          tft.drawFastHLine(cx - 5, cy, 11, modeColor);
          // Center dot for emphasis
          tft.fillRect(cx - 1, cy - 1, 3, 3, modeColor);
          // Diagonal accents
          tft.drawPixel(cx - 2, cy - 2, modeColor);
          tft.drawPixel(cx + 2, cy - 2, modeColor);
          tft.drawPixel(cx - 2, cy + 2, modeColor);
          tft.drawPixel(cx + 2, cy + 2, modeColor);
        }
        break;
      default:
        break;
    }
  }

  // Draw mode text
  tft.setTextColor(modeColor);
  tft.setCursor(startX + iconWidth + gap, textY);
  tft.print(modeLabel);

  // Right: Battery icon + percentage
  int batteryPercent = getBatteryPercent();
  char batteryStr[8];

  if (batteryPercent >= 0) {
    snprintf(batteryStr, sizeof(batteryStr), "%d%%", batteryPercent);
  } else {
    // No battery / USB powered
    snprintf(batteryStr, sizeof(batteryStr), "USB");
  }

  uint16_t batteryColor = COLOR_LIGHT_GRAY;

  // Draw battery icon (small rectangle with nub)
  int batteryIconX = 190;
  int batteryIconY = yPos + 4;
  int batteryIconW = 14;
  int batteryIconH = 8;

  // Battery outline
  tft.drawRect(batteryIconX, batteryIconY, batteryIconW, batteryIconH, batteryColor);
  // Battery nub (positive terminal)
  tft.fillRect(batteryIconX + batteryIconW, batteryIconY + 2, 2, 4, batteryColor);

  // Fill based on percentage
  if (batteryPercent >= 0) {
    int fillWidth = (batteryIconW - 2) * batteryPercent / 100;
    if (fillWidth > 0) {
      tft.fillRect(batteryIconX + 1, batteryIconY + 1, fillWidth, batteryIconH - 2, batteryColor);
    }
  }

  // Draw battery text
  tft.setTextColor(batteryColor);
  tft.setCursor(210, textY);
  tft.print(batteryStr);
}

int getBatteryPercent() {
  if (batteryMonitorAvailable) {
    float percent = batteryMonitor.cellPercent();
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return (int)percent;
  }
  return -1;  // No battery monitor available
}

float readBatteryVoltage() {
  // Returns NAN if no plausible reading is available (no monitor, no battery,
  // I2C error, or MAX17048 not finished with its first conversion after reset)
  if (!batteryMonitorAvailable) return NAN;
  for (int attempt = 0; attempt < 10; attempt++) {
    float voltage = batteryMonitor.cellVoltage();
    if (!isnan(voltage) && voltage >= BATTERY_VALID_MIN_VOLTAGE) return voltage;
    delay(25);
  }
  return NAN;
}

bool isBatteryBelow(float threshold) {
  // Requires several consecutive valid readings below the threshold, so a
  // single voltage dip or a failed reading never triggers a shutdown
  for (int i = 0; i < BATTERY_CHECK_SAMPLES; i++) {
    float voltage = readBatteryVoltage();
    if (isnan(voltage) || voltage >= threshold) return false;
    delay(50);
  }
  return true;
}

// =============================================================================
// GRAPH HELPER FUNCTIONS
// =============================================================================

void drawThresholdLine(int graphX, int graphY, int graphWidth, int graphHeight,
                       float threshold, float minVal, float maxVal, uint16_t color) {
  int y = graphY + graphHeight - (int)(((threshold - minVal) / (maxVal - minVal)) * graphHeight);
  if (y > graphY && y < graphY + graphHeight) {
    tft.drawFastHLine(graphX, y, graphWidth, color);
  }
}

// =============================================================================
// COLOR FUNCTIONS
// =============================================================================

uint16_t getCO2Color(uint16_t co2) {
  if (co2 < CO2_THRESHOLD_LOW) return CO2_COLOR_LOW;
  if (co2 < CO2_THRESHOLD_MODERATE) return CO2_COLOR_MODERATE;
  if (co2 < CO2_THRESHOLD_HIGH) return CO2_COLOR_HIGH;
  return CO2_COLOR_CRITICAL;
}

uint16_t getTemperatureColor(float temp) {
  if (temp < TEMP_THRESHOLD_COLD) return TEMP_COLOR_COLD;
  if (temp < TEMP_THRESHOLD_COOL) return TEMP_COLOR_COOL;
  if (temp < TEMP_THRESHOLD_OPTIMAL) return TEMP_COLOR_OPTIMAL;
  if (temp < TEMP_THRESHOLD_WARM) return TEMP_COLOR_WARM;
  return TEMP_COLOR_HOT;
}

uint16_t getHumidityColor(float humidity) {
  if (humidity < HUMID_THRESHOLD_DRY) return HUMID_COLOR_DRY;
  if (humidity < HUMID_THRESHOLD_LOW) return HUMID_COLOR_LOW;
  if (humidity < HUMID_THRESHOLD_OPTIMAL) return HUMID_COLOR_OPTIMAL;
  if (humidity < HUMID_THRESHOLD_HIGH) return HUMID_COLOR_HIGH;
  return HUMID_COLOR_WET;
}

// =============================================================================
// BUTTON HANDLING
// =============================================================================

void handleButtons() {
  unsigned long now = millis();

  // Static flags to track if button action was already handled (require release before next action)
  static bool button0Handled = false;
  static bool button1Handled = false;
  static bool button2Handled = false;

  bool button0Pressed = (digitalRead(BUTTON_D0) == LOW);
  bool button1Pressed = (digitalRead(BUTTON_D1) == HIGH);
  bool button2Pressed = (digitalRead(BUTTON_D2) == HIGH);

  // ECO mode with display off: ANY button press wakes display (first press only wakes)
  if (currentDisplayMode == MODE_ECO && !displayOn) {
    if (button0Pressed || button1Pressed || button2Pressed) {
      lastDisplayActivity = now;
      setDisplayPower(true);
      button0Handled = true;
      button1Handled = true;
      button2Handled = true;
      Serial.println("ECO: Display woken by button");
      return;
    }
  }

  // Button D0 - Cycle through display values (CO2 -> Temp -> Humidity)
  if (button0Pressed) {
    if (!button0Handled && (now - lastButtonPress[0] >= DEBOUNCE_DELAY)) {
      lastButtonPress[0] = now;
      button0Handled = true;
      currentDisplayValue = (DisplayValue)((currentDisplayValue + 1) % DISPLAY_VALUE_COUNT);
      lastDisplayActivity = now;
      forceDisplayRefresh = true;
      Serial.printf("Display value: %s\n", getDisplayValueName(currentDisplayValue));
    }
  } else {
    button0Handled = false;
  }

  // Button D1 - Toggle between number and graph view
  if (button1Pressed) {
    if (!button1Handled && (now - lastButtonPress[1] >= DEBOUNCE_DELAY)) {
      lastButtonPress[1] = now;
      button1Handled = true;
      currentViewMode = (ViewMode)((currentViewMode + 1) % VIEW_MODE_COUNT);
      lastDisplayActivity = now;
      forceDisplayRefresh = true;
      const char* viewNames[] = {"Number", "Graph"};
      Serial.printf("View mode: %s\n", viewNames[currentViewMode]);
    }
  } else {
    button1Handled = false;
  }

  // Button D2 - Cycle through display modes (Day -> Night -> ECO)
  if (button2Pressed) {
    if (!button2Handled && (now - lastButtonPress[2] >= DEBOUNCE_DELAY)) {
      lastButtonPress[2] = now;
      button2Handled = true;

      DisplayMode previousMode = currentDisplayMode;
      currentDisplayMode = (DisplayMode)((currentDisplayMode + 1) % DISPLAY_MODE_COUNT);
      lastDisplayActivity = now;
      forceDisplayRefresh = true;
      Serial.printf("Display mode: %s\n", getDisplayModeName(currentDisplayMode));

      // Set brightness based on new mode
      if (currentDisplayMode == MODE_DAY) {
        setDisplayBrightness(BRIGHTNESS_DAY);
      } else if (currentDisplayMode == MODE_NIGHT) {
        setDisplayBrightness(BRIGHTNESS_NIGHT);
      } else {
        setDisplayBrightness(BRIGHTNESS_ECO);
      }

      // Switch sensor mode when entering/exiting ECO mode
      if (currentDisplayMode == MODE_ECO && previousMode != MODE_ECO) {
        setSensorMode(true);  // Enter single shot mode
      } else if (currentDisplayMode != MODE_ECO && previousMode == MODE_ECO) {
        setSensorMode(false); // Return to periodic mode
      }
    }
  } else {
    button2Handled = false;
  }
}

// =============================================================================
// DISPLAY POWER MANAGEMENT
// =============================================================================

void setDisplayPower(bool on) {
  if (on == displayOn) return;

  displayOn = on;

  if (on) {
    // Wake ST7789 from sleep mode first
    tft.enableSleep(false);
    delay(120);  // ST7789 requires 120ms after sleep out

    // Clear screen and set brightness based on mode
    tft.fillScreen(COLOR_BLACK);
    int brightness;
    if (currentDisplayMode == MODE_NIGHT) {
      brightness = BRIGHTNESS_NIGHT;
    } else if (currentDisplayMode == MODE_ECO) {
      brightness = BRIGHTNESS_ECO;
    } else {
      brightness = BRIGHTNESS_DAY;
    }
    analogWrite(TFT_BACKLITE, brightness);
    lastDisplayActivity = millis();  // Reset timeout when display turns on
    forceDisplayRefresh = true;
  } else {
    // Turn off backlight first, then put ST7789 to sleep
    analogWrite(TFT_BACKLITE, 0);
    tft.enableSleep(true);  // Reduces ST7789 from 6mA to 30µA
  }

  // IMPORTANT: Reset button debounce timers after display state change
  // GPIO 0 (boot button) is a strapping pin prone to noise during power transitions
  // This prevents spurious button triggers from electrical noise
  unsigned long now = millis();
  lastButtonPress[0] = now;
  lastButtonPress[1] = now;
  lastButtonPress[2] = now;

  Serial.printf("Display: %s\n", on ? "ON" : "OFF");
}

void setDisplayBrightness(int brightness) {
  if (displayOn) {
    analogWrite(TFT_BACKLITE, brightness);
  }
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

const char* getDisplayValueName(DisplayValue val) {
  switch (val) {
    case DISPLAY_CO2: return "CO2";
    case DISPLAY_TEMPERATURE: return "Temperature";
    case DISPLAY_HUMIDITY: return "Humidity";
    default: return "Unknown";
  }
}

const char* getDisplayModeName(DisplayMode mode) {
  switch (mode) {
    case MODE_DAY: return "DAY";
    case MODE_NIGHT: return "NIGHT";
    case MODE_ECO: return "ECO";
    default: return "Unknown";
  }
}

// =============================================================================
// POWER MANAGEMENT FUNCTIONS
// =============================================================================

void initPowerSaving() {
  Serial.println("Initializing power saving...");

  // Disable WiFi
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_wifi_deinit();
  Serial.println("  WiFi disabled");

  // Disable Bluetooth
  esp_bt_controller_disable();
  Serial.println("  Bluetooth disabled");

  // Configure unused GPIO pins to save power
  // ESP32-S3 Reverse TFT Feather pins in use:
  // - GPIO 0, 1, 2: Buttons
  // - GPIO 3, 4: I2C (SDA, SCL) for sensor and battery monitor
  // - GPIO 7: TFT_I2C_POWER
  // - GPIO 33: NEOPIXEL, GPIO 21: NEOPIXEL_POWER
  // - GPIO 35, 36, 37, 40, 41, 42, 45: TFT display (MOSI, SCK, MISO, DC, RST, CS, BACKLITE)
  //
  // Unused pins should be set to INPUT with no pull resistors
  // to minimize current leakage
  const int unusedPins[] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 38, 39};
  for (int i = 0; i < sizeof(unusedPins) / sizeof(unusedPins[0]); i++) {
    gpio_reset_pin((gpio_num_t)unusedPins[i]);
    gpio_set_direction((gpio_num_t)unusedPins[i], GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)unusedPins[i], GPIO_FLOATING);
  }
  Serial.println("  Unused GPIOs configured");

  // Disable onboard NeoPixel to save power
  pinMode(PIN_NEOPIXEL, OUTPUT);
  digitalWrite(PIN_NEOPIXEL, LOW);
  Serial.println("  NeoPixel disabled");

  Serial.println("Power saving initialized");
}

void setSensorMode(bool singleShot) {
  if (singleShot == sensorSingleShotMode) return;

  uint16_t error;

  if (singleShot) {
    // Switch to single shot mode: stop periodic measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
      Serial.println("Error stopping periodic measurement");
      return;
    }
    delay(500);  // Wait for sensor to stop

    // Put sensor to sleep until next measurement
    scd4x.powerDown();

    sensorSingleShotMode = true;
    Serial.println("Sensor: Single shot mode (low power)");
  } else {
    // Switch to periodic mode: wake sensor and start periodic measurement
    scd4x.wakeUp();
    delay(30);

    error = scd4x.startPeriodicMeasurement();
    if (error) {
      Serial.println("Error starting periodic measurement");
      return;
    }

    sensorSingleShotMode = false;
    Serial.println("Sensor: Periodic measurement mode");
  }
}

void updateHyperdrive() {
  // Calculate CO2 change since last hyperdrive check
  int co2Change = abs((int)currentCO2 - (int)lastHyperdriveCO2);

  if (!hyperdriveActive) {
    // Check if we should enter hyperdrive
    if (co2Change >= HYPERDRIVE_THRESHOLD) {
      hyperdriveActive = true;
      hyperdriveStableCount = 0;
      Serial.printf("Hyperdrive ACTIVATED (CO2 change: %d ppm)\n", co2Change);
    }
  } else {
    // Check if we should exit hyperdrive
    if (co2Change < HYPERDRIVE_THRESHOLD) {
      hyperdriveStableCount++;
      Serial.printf("Hyperdrive stable reading %d/%d\n", hyperdriveStableCount, HYPERDRIVE_STABLE_COUNT);

      if (hyperdriveStableCount >= HYPERDRIVE_STABLE_COUNT) {
        hyperdriveActive = false;
        hyperdriveStableCount = 0;
        Serial.println("Hyperdrive DEACTIVATED (values stabilized)");
      }
    } else {
      // Still changing rapidly, reset stable count
      hyperdriveStableCount = 0;
    }
  }

  // Update baseline for next comparison
  lastHyperdriveCO2 = currentCO2;
}

// =============================================================================
// DEEP SLEEP FUNCTIONS
// =============================================================================

void enterDeepSleep() {
  // Ensure display is off
  if (displayOn) {
    setDisplayPower(false);
  }

  // Mark RTC data as valid
  rtc.magic = RTC_DATA_MAGIC;

  // Configure wake sources
  // Timer: 5 minutes for periodic sensor reads
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_US);
  enableButtonWake();

  // Hold I2C power HIGH during sleep so sensor retains calibration state
  gpio_hold_en((gpio_num_t)TFT_I2C_POWER);
  gpio_deep_sleep_hold_en();

  Serial.println("Entering deep sleep...");
  Serial.flush();

  esp_deep_sleep_start();
  // Execution stops here. Next code to run is setup() after wake.
}

void enableButtonWake() {
  // EXT1: GPIO 1 (D1) and GPIO 2 (D2), wake on HIGH level
  // GPIO 0 excluded - strapping pin prone to spurious wakes from noise
  uint64_t ext1_mask = (1ULL << BUTTON_D1) | (1ULL << BUTTON_D2);
  esp_sleep_enable_ext1_wakeup(ext1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

  // Configure RTC pull-downs for wake pins
  // Digital pull-downs from pinMode() are lost during deep sleep — only RTC domain stays active
  rtc_gpio_pulldown_en((gpio_num_t)BUTTON_D1);
  rtc_gpio_pullup_dis((gpio_num_t)BUTTON_D1);
  rtc_gpio_pulldown_en((gpio_num_t)BUTTON_D2);
  rtc_gpio_pullup_dis((gpio_num_t)BUTTON_D2);
}

void enterLowBatterySleep() {
  // Battery protection shutdown: everything off, wake only to re-check the
  // battery (timer or button). setup() resumes with a cold boot once the
  // battery is above BATTERY_RECOVER_VOLTAGE, otherwise it sleeps again
  // before display or sensor are powered.
  Serial.printf("Battery low (%.2f V), shutting down\n", readBatteryVoltage());

  // Backlight off (display may not be initialized yet, so no SPI commands)
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, LOW);

  lowBatteryShutdown = true;
  rtc.magic = 0;  // Sensor loses power, force full re-init on recovery

  esp_sleep_enable_timer_wakeup(LOW_BATTERY_SLEEP_US);
  enableButtonWake();

  // Cut power to display, sensor and I2C pull-ups, hold LOW during sleep
  digitalWrite(TFT_I2C_POWER, LOW);
  gpio_hold_en((gpio_num_t)TFT_I2C_POWER);
  gpio_deep_sleep_hold_en();

  Serial.flush();
  esp_deep_sleep_start();
}

void initDisplayWarm() {
  // Minimal display init after deep sleep (no splash screen)
  // I2C power already enabled in setup()
  pinMode(TFT_BACKLITE, OUTPUT);
  analogWrite(TFT_BACKLITE, 0);  // Start dark

  // SPI state is lost during deep sleep, must re-init
  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(COLOR_BLACK);

  // Wake display and set ECO brightness
  tft.enableSleep(false);
  delay(120);  // ST7789 requires 120ms after sleep out
  analogWrite(TFT_BACKLITE, BRIGHTNESS_ECO);
}
