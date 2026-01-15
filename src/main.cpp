/**
 * ESP32-S3 CO2 Monitor Firmware
 *
 * Hardware:
 *   - Adafruit ESP32-S3 Reverse TFT Feather
 *   - Adafruit SCD-41 CO2 Sensor (I2C)
 *
 * Features:
 *   - Real-time CO2, Temperature, and Humidity display
 *   - Color-coded thresholds (Green/Yellow/Orange/Red)
 *   - 24-hour history graph
 *   - Multiple display modes via buttons
 *   - Warn mode (display only when threshold exceeded)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_MAX1704X.h>

// =============================================================================
// CONFIGURATION - Adjust these values to customize behavior
// =============================================================================

// CO2 Thresholds (in ppm)
#define CO2_THRESHOLD_GREEN   800   // Excellent
#define CO2_THRESHOLD_YELLOW  1000  // Acceptable
#define CO2_THRESHOLD_ORANGE  1400  // Poor
#define CO2_THRESHOLD_RED     1600  // Unhealthy

// Temperature thresholds (in Celsius)
#define TEMP_THRESHOLD_COLD   18.0  // Too cold
#define TEMP_THRESHOLD_COOL   20.0  // Cool
#define TEMP_THRESHOLD_WARM   25.0  // Warm
#define TEMP_THRESHOLD_HOT    28.0  // Hot

// Humidity thresholds (in %)
#define HUMID_THRESHOLD_DRY   30.0  // Too low
#define HUMID_THRESHOLD_LOW   40.0  // Low
#define HUMID_THRESHOLD_HIGH  60.0  // High
#define HUMID_THRESHOLD_WET   70.0  // Too high

// Timing Configuration
#define SENSOR_READ_INTERVAL  300000      // Read sensor every 5 minutes
#define GRAPH_SAMPLE_INTERVAL 720000      // Store graph point every 12 minutes (24h / 120 points)
#define DISPLAY_TIMEOUT       10000       // Display timeout in warn mode (10 seconds)
#define DEBOUNCE_DELAY        200         // Button debounce time (ms)
#define GRAPH_START_HOUR      6           // Graph starts at 6:00 AM

// Graph Configuration
#define GRAPH_POINTS          120         // Number of data points in graph
#define GRAPH_HEIGHT          80          // Height of graph area in pixels
#define GRAPH_WIDTH           200         // Width of graph area in pixels

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
#define COLOR_PURPLE      0x780F
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
  MODE_ALWAYS_ON,
  MODE_WARN,
  DISPLAY_MODE_COUNT
};

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

// TFT_CS, TFT_DC, TFT_RST are defined by the ESP32-S3 Reverse TFT board package
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
SensirionI2CScd4x scd4x;
Adafruit_MAX17048 batteryMonitor;
bool batteryMonitorAvailable = false;

// =============================================================================
// GLOBAL STATE
// =============================================================================

// Current readings
uint16_t currentCO2 = 0;
float currentTemperature = 0.0;
float currentHumidity = 0.0;

// Graph history (circular buffer)
uint16_t historyC02[GRAPH_POINTS] = {0};
float historyTemperature[GRAPH_POINTS] = {0};
float historyHumidity[GRAPH_POINTS] = {0};
int historyIndex = 0;
int historyCount = 0;

// UI State
DisplayValue currentDisplayValue = DISPLAY_CO2;
ViewMode currentViewMode = VIEW_NUMBER;
DisplayMode currentDisplayMode = MODE_ALWAYS_ON;
bool displayOn = true;

// Timing
unsigned long lastSensorRead = 0;
unsigned long lastGraphSample = 0;
unsigned long lastDisplayActivity = 0;
unsigned long lastButtonPress[3] = {0, 0, 0};
bool forceDisplayRefresh = true;  // Force refresh on startup and mode changes

// Previous values for display update optimization
uint16_t prevCO2 = 0;
float prevTemperature = 0.0;
float prevHumidity = 0.0;
DisplayValue prevDisplayValue = DISPLAY_CO2;
ViewMode prevViewMode = VIEW_NUMBER;
DisplayMode prevDisplayMode = MODE_ALWAYS_ON;

// Trend tracking (for arrow indicators)
uint16_t trendCO2 = 0;
float trendTemperature = 0.0;
float trendHumidity = 0.0;
int8_t co2Trend = 0;      // -1 = down, 0 = stable, 1 = up
int8_t tempTrend = 0;
int8_t humidTrend = 0;

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
void drawBigNumber(int value, const char* unit, uint16_t color);
void drawBigNumberFloat(float value, const char* unit, uint16_t color);
void drawSmallValues();
void drawGraph();
void drawStatusBar();
uint16_t getCO2Color(uint16_t co2);
uint16_t getTemperatureColor(float temp);
uint16_t getHumidityColor(float humidity);
void handleButtons();
void setDisplayPower(bool on);
bool shouldDisplayBeOn();
const char* getDisplayValueName(DisplayValue val);
const char* getDisplayModeName(DisplayMode mode);
void updateTrends();
void drawTrendArrow(int x, int y, int8_t trend, uint16_t color);
int getBatteryPercent();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 CO2 Monitor Starting...");

  // Initialize buttons
  // D0 is pulled HIGH internally, goes LOW when pressed
  pinMode(BUTTON_D0, INPUT_PULLUP);
  // D1 and D2 are pulled LOW internally, go HIGH when pressed
  pinMode(BUTTON_D1, INPUT_PULLDOWN);
  pinMode(BUTTON_D2, INPUT_PULLDOWN);

  // Initialize display
  initDisplay();

  // Initialize sensor
  initSensor();

  // Initialize battery monitor (MAX17048)
  if (batteryMonitor.begin()) {
    batteryMonitorAvailable = true;
    Serial.println("MAX17048 battery monitor initialized");
  } else {
    batteryMonitorAvailable = false;
    Serial.println("MAX17048 not found, battery display disabled");
  }

  // Initial sensor read to display values immediately
  Serial.println("Waiting for first sensor reading...");
  while (currentCO2 == 0) {
    readSensor();
    delay(500);
  }
  updateHistory();  // Store first data point

  Serial.println("Setup complete!");
  forceDisplayRefresh = true;
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  unsigned long currentMillis = millis();

  // Handle button input
  handleButtons();

  // Read sensor at regular intervals
  if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readSensor();
    lastSensorRead = currentMillis;
    forceDisplayRefresh = true;  // Refresh display when new data arrives
  }

  // Update history for graph at regular intervals
  if (currentMillis - lastGraphSample >= GRAPH_SAMPLE_INTERVAL) {
    updateHistory();
    lastGraphSample = currentMillis;
  }

  // Handle display power in warn mode
  if (currentDisplayMode == MODE_WARN) {
    bool shouldBeOn = shouldDisplayBeOn();
    if (shouldBeOn != displayOn) {
      setDisplayPower(shouldBeOn);
    }
  }

  // Update display (smart refresh - only redraws when values change)
  if (displayOn) {
    updateDisplay();
    forceDisplayRefresh = false;
  }

  delay(100); // Delay to prevent CPU hogging
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

void initDisplay() {
  // Enable power to the TFT and I2C
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // Enable backlight
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

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

  // "v0.1" - size 1, 4 chars * 6px = 24px wide
  tft.setTextColor(COLOR_LIGHT_GRAY);
  tft.setTextSize(1);
  tft.setCursor((240 - 24) / 2, 75);
  tft.print("v0.2");

  displayOn = true;
  lastDisplayActivity = millis();
}

void initSensor() {
  Wire.begin();

  // SCD41 default I2C address is 0x62
  scd4x.begin(Wire);

  // Stop any previous measurements
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

  // Start periodic measurements
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.println("Failed to start measurements!");
  }

  Serial.println("SCD-41 sensor initialized");

  // Wait for first measurement (SCD41 measures every 5 seconds)
  delay(5000);
  tft.fillScreen(COLOR_BLACK);
}

// =============================================================================
// SENSOR FUNCTIONS
// =============================================================================

void readSensor() {
  bool dataReady = false;
  uint16_t error = scd4x.getDataReadyFlag(dataReady);

  if (error) {
    Serial.println("Error checking data ready status");
    return;
  }

  if (dataReady) {
    uint16_t co2 = 0;
    float temp = 0.0f;
    float humidity = 0.0f;

    error = scd4x.readMeasurement(co2, temp, humidity);
    if (error) {
      Serial.println("Error reading measurement");
      return;
    }

    // Only update if we got valid data (CO2 = 0 means invalid)
    if (co2 > 0) {
      currentCO2 = co2;
      currentTemperature = temp;
      currentHumidity = humidity;

      // Update trends
      updateTrends();

      Serial.printf("CO2: %d ppm, Temp: %.1f C, Humidity: %.1f %%\n",
                    currentCO2, currentTemperature, currentHumidity);
    }
  }
}

void updateTrends() {
  // CO2 trend (threshold: 20 ppm change)
  if (currentCO2 > trendCO2 + 20) {
    co2Trend = 1;
  } else if (currentCO2 < trendCO2 - 20) {
    co2Trend = -1;
  } else {
    co2Trend = 0;
  }
  trendCO2 = currentCO2;

  // Temperature trend (threshold: 0.3°C change)
  if (currentTemperature > trendTemperature + 0.3) {
    tempTrend = 1;
  } else if (currentTemperature < trendTemperature - 0.3) {
    tempTrend = -1;
  } else {
    tempTrend = 0;
  }
  trendTemperature = currentTemperature;

  // Humidity trend (threshold: 2% change)
  if (currentHumidity > trendHumidity + 2.0) {
    humidTrend = 1;
  } else if (currentHumidity < trendHumidity - 2.0) {
    humidTrend = -1;
  } else {
    humidTrend = 0;
  }
  trendHumidity = currentHumidity;
}

void updateHistory() {
  historyC02[historyIndex] = currentCO2;
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

  if (currentViewMode == VIEW_NUMBER) {
    drawNumberView();
  } else {
    drawGraphView();
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
  if (trend == 0) {
    // No change - draw horizontal line/dash
    tft.drawFastHLine(x, y, 20, COLOR_DARK_GRAY);
    return;
  }

  // Arrow size
  int arrowHeight = 16;
  int arrowWidth = 12;

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

void drawBigNumber(int value, const char* unit, uint16_t color) {
  // Legacy function - now handled in drawNumberView
  (void)value; (void)unit; (void)color;
}

void drawBigNumberFloat(float value, const char* unit, uint16_t color) {
  // Legacy function - now handled in drawNumberView
  (void)value; (void)unit; (void)color;
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
      minVal = 400;  // CO2 rarely goes below 400
      maxVal = 2500; // Cap at 2500 for scale
      for (int i = 0; i < historyCount; i++) {
        if (historyC02[i] > maxVal) maxVal = historyC02[i];
      }
      break;
    case DISPLAY_TEMPERATURE:
      minVal = 15;
      maxVal = 35;
      break;
    case DISPLAY_HUMIDITY:
      minVal = 0;
      maxVal = 100;
      break;
    default:
      break;
  }

  // Always draw threshold lines (thin, colored)
  if (currentDisplayValue == DISPLAY_CO2) {
    int yGreen = graphY + graphHeight - (int)(((CO2_THRESHOLD_GREEN - minVal) / (maxVal - minVal)) * graphHeight);
    int yYellow = graphY + graphHeight - (int)(((CO2_THRESHOLD_YELLOW - minVal) / (maxVal - minVal)) * graphHeight);
    int yOrange = graphY + graphHeight - (int)(((CO2_THRESHOLD_ORANGE - minVal) / (maxVal - minVal)) * graphHeight);
    int yRed = graphY + graphHeight - (int)(((CO2_THRESHOLD_RED - minVal) / (maxVal - minVal)) * graphHeight);

    if (yGreen > graphY && yGreen < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yGreen, graphWidth, COLOR_GREEN);
    }
    if (yYellow > graphY && yYellow < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yYellow, graphWidth, COLOR_YELLOW);
    }
    if (yOrange > graphY && yOrange < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yOrange, graphWidth, COLOR_ORANGE);
    }
    if (yRed > graphY && yRed < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yRed, graphWidth, COLOR_RED);
    }
  } else if (currentDisplayValue == DISPLAY_TEMPERATURE) {
    // Temperature thresholds
    int yCool = graphY + graphHeight - (int)(((TEMP_THRESHOLD_COOL - minVal) / (maxVal - minVal)) * graphHeight);
    int yWarm = graphY + graphHeight - (int)(((TEMP_THRESHOLD_WARM - minVal) / (maxVal - minVal)) * graphHeight);
    int yHot = graphY + graphHeight - (int)(((TEMP_THRESHOLD_HOT - minVal) / (maxVal - minVal)) * graphHeight);

    if (yCool > graphY && yCool < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yCool, graphWidth, COLOR_CYAN);
    }
    if (yWarm > graphY && yWarm < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yWarm, graphWidth, COLOR_YELLOW);
    }
    if (yHot > graphY && yHot < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yHot, graphWidth, COLOR_ORANGE);
    }
  } else if (currentDisplayValue == DISPLAY_HUMIDITY) {
    // Humidity thresholds
    int yLow = graphY + graphHeight - (int)(((HUMID_THRESHOLD_LOW - minVal) / (maxVal - minVal)) * graphHeight);
    int yHigh = graphY + graphHeight - (int)(((HUMID_THRESHOLD_HIGH - minVal) / (maxVal - minVal)) * graphHeight);

    if (yLow > graphY && yLow < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yLow, graphWidth, COLOR_YELLOW);
    }
    if (yHigh > graphY && yHigh < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yHigh, graphWidth, COLOR_CYAN);
    }
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
          val0 = historyC02[idx0];
          val1 = historyC02[idx1];
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
      metricColor = COLOR_WHITE;
      metricLabel = "TEMP";
      break;
    case DISPLAY_HUMIDITY:
      metricColor = COLOR_CYAN;
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

  // Center: Power mode (light grey)
  const char* modeLabel;
  if (currentDisplayMode == MODE_ALWAYS_ON) {
    modeLabel = "ALWAYS ON";
  } else {
    modeLabel = "WARN";
  }

  int modeWidth = strlen(modeLabel) * 6;
  tft.setTextColor(COLOR_LIGHT_GRAY);
  tft.setCursor(120 - modeWidth/2, textY);
  tft.print(modeLabel);

  // Right: Battery icon + percentage
  int batteryPercent = getBatteryPercent();
  char batteryStr[8];
  uint16_t batteryColor;

  if (batteryPercent >= 0) {
    snprintf(batteryStr, sizeof(batteryStr), "%d%%", batteryPercent);
    // Color based on battery level
    if (batteryPercent > 50) {
      batteryColor = COLOR_GREEN;
    } else if (batteryPercent > 20) {
      batteryColor = COLOR_YELLOW;
    } else {
      batteryColor = COLOR_RED;
    }
  } else {
    // No battery / USB powered
    snprintf(batteryStr, sizeof(batteryStr), "USB");
    batteryColor = COLOR_LIGHT_GRAY;
  }

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
  tft.setTextColor(COLOR_WHITE);
  tft.setCursor(210, textY);
  tft.print(batteryStr);
}

int getBatteryPercent() {
  // Use MAX17048 battery fuel gauge if available
  if (batteryMonitorAvailable) {
    float percent = batteryMonitor.cellPercent();
    // Clamp to 0-100
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return (int)percent;
  }

  // No battery monitor available - return -1 to indicate USB power
  return -1;
}

// =============================================================================
// COLOR FUNCTIONS
// =============================================================================

uint16_t getCO2Color(uint16_t co2) {
  if (co2 < CO2_THRESHOLD_GREEN) return COLOR_GREEN;
  if (co2 < CO2_THRESHOLD_YELLOW) return COLOR_YELLOW;
  if (co2 < CO2_THRESHOLD_ORANGE) return COLOR_ORANGE;
  return COLOR_RED;
}

uint16_t getTemperatureColor(float temp) {
  // Diverging scale: blue (cold) -> cyan -> WHITE (optimal) -> yellow -> orange -> red (hot)
  if (temp < TEMP_THRESHOLD_COLD) return COLOR_BLUE;
  if (temp < TEMP_THRESHOLD_COOL) return COLOR_CYAN;
  if (temp <= TEMP_THRESHOLD_WARM) return COLOR_WHITE;  // Optimal range
  if (temp <= TEMP_THRESHOLD_HOT) return COLOR_YELLOW;
  return COLOR_ORANGE;
}

uint16_t getHumidityColor(float humidity) {
  // Diverging scale: orange (dry) -> yellow -> WHITE (optimal) -> cyan -> blue (wet)
  if (humidity < HUMID_THRESHOLD_DRY) return COLOR_ORANGE;
  if (humidity < HUMID_THRESHOLD_LOW) return COLOR_YELLOW;
  if (humidity <= HUMID_THRESHOLD_HIGH) return COLOR_WHITE;  // Optimal range
  if (humidity <= HUMID_THRESHOLD_WET) return COLOR_CYAN;
  return COLOR_BLUE;
}

// =============================================================================
// BUTTON HANDLING
// =============================================================================

void handleButtons() {
  unsigned long currentMillis = millis();

  // Button D0 - Cycle through display values (CO2 -> Temp -> Humidity)
  // D0 is pulled HIGH, goes LOW when pressed
  if (digitalRead(BUTTON_D0) == LOW) {
    if (currentMillis - lastButtonPress[0] >= DEBOUNCE_DELAY) {
      currentDisplayValue = (DisplayValue)((currentDisplayValue + 1) % DISPLAY_VALUE_COUNT);
      lastDisplayActivity = currentMillis;
      lastButtonPress[0] = currentMillis;
      forceDisplayRefresh = true;

      Serial.printf("Display value: %s\n", getDisplayValueName(currentDisplayValue));

      // Wake display if in warn mode
      if (currentDisplayMode == MODE_WARN && !displayOn) {
        setDisplayPower(true);
      }
    }
  }

  // Button D1 - Toggle between number and graph view
  // D1 is pulled LOW, goes HIGH when pressed
  if (digitalRead(BUTTON_D1) == HIGH) {
    if (currentMillis - lastButtonPress[1] >= DEBOUNCE_DELAY) {
      currentViewMode = (ViewMode)((currentViewMode + 1) % VIEW_MODE_COUNT);
      lastDisplayActivity = currentMillis;
      lastButtonPress[1] = currentMillis;
      forceDisplayRefresh = true;

      Serial.printf("View mode: %s\n", currentViewMode == VIEW_NUMBER ? "Number" : "Graph");

      // Wake display if in warn mode
      if (currentDisplayMode == MODE_WARN && !displayOn) {
        setDisplayPower(true);
      }
    }
  }

  // Button D2 - Toggle between always-on and warn mode
  // D2 is pulled LOW, goes HIGH when pressed
  if (digitalRead(BUTTON_D2) == HIGH) {
    if (currentMillis - lastButtonPress[2] >= DEBOUNCE_DELAY) {
      currentDisplayMode = (DisplayMode)((currentDisplayMode + 1) % DISPLAY_MODE_COUNT);
      lastDisplayActivity = currentMillis;
      forceDisplayRefresh = true;
      lastButtonPress[2] = currentMillis;

      Serial.printf("Display mode: %s\n", getDisplayModeName(currentDisplayMode));

      // Ensure display is on when changing modes
      if (!displayOn) {
        setDisplayPower(true);
      }
    }
  }
}

// =============================================================================
// DISPLAY POWER MANAGEMENT
// =============================================================================

void setDisplayPower(bool on) {
  if (on == displayOn) return;

  displayOn = on;
  digitalWrite(TFT_BACKLITE, on ? HIGH : LOW);

  if (on) {
    lastDisplayActivity = millis();
    // Force full redraw
    prevViewMode = (ViewMode)255;
  }

  Serial.printf("Display: %s\n", on ? "ON" : "OFF");
}

bool shouldDisplayBeOn() {
  unsigned long currentMillis = millis();

  // Check if threshold is exceeded (CO2 only for warn mode)
  bool thresholdExceeded = currentCO2 >= CO2_THRESHOLD_YELLOW;

  // Check if recent button activity
  bool recentActivity = (currentMillis - lastDisplayActivity) < DISPLAY_TIMEOUT;

  return thresholdExceeded || recentActivity;
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
    case MODE_ALWAYS_ON: return "ALWAYS ON";
    case MODE_WARN: return "WARN MODE";
    default: return "Unknown";
  }
}
