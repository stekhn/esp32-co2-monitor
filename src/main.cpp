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

// =============================================================================
// CONFIGURATION - Adjust these values to customize behavior
// =============================================================================

// CO2 Thresholds (in ppm)
#define CO2_THRESHOLD_GREEN   800   // Excellent air quality
#define CO2_THRESHOLD_YELLOW  1000  // Acceptable, consider ventilating
#define CO2_THRESHOLD_ORANGE  1500  // Poor, should ventilate
// Above ORANGE = Red (unhealthy)

// Temperature thresholds (in Celsius)
#define TEMP_THRESHOLD_COLD   18.0  // Too cold (blue)
#define TEMP_THRESHOLD_COOL   20.0  // Cool (cyan)
#define TEMP_THRESHOLD_WARM   25.0  // Warm (yellow)
#define TEMP_THRESHOLD_HOT    28.0  // Hot (orange)
// Above HOT = Red

// Humidity thresholds (in %)
#define HUMID_THRESHOLD_DRY   30.0  // Too dry (orange)
#define HUMID_THRESHOLD_LOW   40.0  // Low (yellow)
#define HUMID_THRESHOLD_HIGH  60.0  // High (cyan)
#define HUMID_THRESHOLD_WET   70.0  // Too wet (blue)
// Above WET = Purple

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

  Serial.println("Setup complete!");
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

  // Show startup message
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(40, 50);
  tft.println("CO2 Monitor");
  tft.setTextSize(1);
  tft.setCursor(60, 80);
  tft.println("Initializing...");

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

      Serial.printf("CO2: %d ppm, Temp: %.1f C, Humidity: %.1f %%\n",
                    currentCO2, currentTemperature, currentHumidity);
    }
  }
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
  // - Main value: centered, large font (y: 5-75)
  // - Secondary values: bottom area with labels (y: 80-110)
  // - Status bar: very bottom (y: 120-135)

  // Clear main display area (not status bar)
  tft.fillRect(0, 0, 240, 118, COLOR_BLACK);

  // Draw the main big number centered
  char valueStr[16];
  char unitStr[8];
  uint16_t color;
  int textSize;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      snprintf(valueStr, sizeof(valueStr), "%d", currentCO2);
      snprintf(unitStr, sizeof(unitStr), "ppm");
      color = getCO2Color(currentCO2);
      textSize = 7;
      break;
    case DISPLAY_TEMPERATURE:
      snprintf(valueStr, sizeof(valueStr), "%.1f", currentTemperature);
      snprintf(unitStr, sizeof(unitStr), "C");
      color = getTemperatureColor(currentTemperature);
      textSize = 6;
      break;
    case DISPLAY_HUMIDITY:
      snprintf(valueStr, sizeof(valueStr), "%.0f", currentHumidity);
      snprintf(unitStr, sizeof(unitStr), "%%");
      color = getHumidityColor(currentHumidity);
      textSize = 7;
      break;
    default:
      return;
  }

  // Calculate centering for main value
  tft.setTextSize(textSize);
  int charWidth = 6 * textSize;  // Base char width is 6 pixels
  int valueWidth = strlen(valueStr) * charWidth;

  tft.setTextSize(2);
  int unitWidth = strlen(unitStr) * 12;

  int totalWidth = valueWidth + 8 + unitWidth;
  int startX = (240 - totalWidth) / 2;
  int valueY = 15;

  // Draw main value
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.setCursor(startX, valueY);
  tft.print(valueStr);

  // Draw unit (vertically centered with the number)
  tft.setTextSize(2);
  int unitY = valueY + (textSize * 8) / 2 - 8;
  tft.setCursor(startX + valueWidth + 8, unitY);
  tft.print(unitStr);

  // Draw secondary values at bottom
  drawSmallValues();
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
  // Draw secondary readings in a nice two-column layout
  int yPos = 85;
  int leftX = 20;
  int rightX = 130;

  tft.setTextSize(2);

  // Determine which two values to show (the ones not currently selected as main)
  if (currentDisplayValue != DISPLAY_CO2) {
    // Show CO2 on the left or right depending on what else is shown
    if (currentDisplayValue == DISPLAY_TEMPERATURE) {
      // Main is Temp, show CO2 left, Humidity right
      tft.setTextColor(getCO2Color(currentCO2));
      tft.setCursor(leftX, yPos);
      tft.printf("%d", currentCO2);
      tft.setTextSize(1);
      tft.setCursor(leftX, yPos + 18);
      tft.print("CO2 ppm");

      tft.setTextSize(2);
      tft.setTextColor(getHumidityColor(currentHumidity));
      tft.setCursor(rightX, yPos);
      tft.printf("%.0f%%", currentHumidity);
      tft.setTextSize(1);
      tft.setCursor(rightX, yPos + 18);
      tft.print("Humidity");
    } else {
      // Main is Humidity, show CO2 left, Temp right
      tft.setTextColor(getCO2Color(currentCO2));
      tft.setCursor(leftX, yPos);
      tft.printf("%d", currentCO2);
      tft.setTextSize(1);
      tft.setCursor(leftX, yPos + 18);
      tft.print("CO2 ppm");

      tft.setTextSize(2);
      tft.setTextColor(getTemperatureColor(currentTemperature));
      tft.setCursor(rightX, yPos);
      tft.printf("%.1f C", currentTemperature);
      tft.setTextSize(1);
      tft.setCursor(rightX, yPos + 18);
      tft.print("Temp");
    }
  } else {
    // Main is CO2, show Temp left, Humidity right
    tft.setTextColor(getTemperatureColor(currentTemperature));
    tft.setCursor(leftX, yPos);
    tft.printf("%.1f C", currentTemperature);
    tft.setTextSize(1);
    tft.setCursor(leftX, yPos + 18);
    tft.print("Temp");

    tft.setTextSize(2);
    tft.setTextColor(getHumidityColor(currentHumidity));
    tft.setCursor(rightX, yPos);
    tft.printf("%.0f%%", currentHumidity);
    tft.setTextSize(1);
    tft.setCursor(rightX, yPos + 18);
    tft.print("Humidity");
  }
}

void drawGraphView() {
  // Clear display area (except status bar)
  tft.fillRect(0, 0, 240, 118, COLOR_BLACK);

  // Header: "CO2" on left, current value on right
  tft.setTextSize(1);
  tft.setTextColor(COLOR_LIGHT_GRAY);
  tft.setCursor(5, 4);
  tft.print(getDisplayValueName(currentDisplayValue));
  tft.print(" - 24h (from 6:00)");

  // Draw current value on right side - clear area first and use proper formatting
  char valueStr[16];
  uint16_t valueColor;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      snprintf(valueStr, sizeof(valueStr), "%d ppm", currentCO2);
      valueColor = getCO2Color(currentCO2);
      break;
    case DISPLAY_TEMPERATURE:
      snprintf(valueStr, sizeof(valueStr), "%.1f C", currentTemperature);
      valueColor = getTemperatureColor(currentTemperature);
      break;
    case DISPLAY_HUMIDITY:
      snprintf(valueStr, sizeof(valueStr), "%.0f %%", currentHumidity);
      valueColor = getHumidityColor(currentHumidity);
      break;
    default:
      valueStr[0] = '\0';
      valueColor = COLOR_WHITE;
      break;
  }

  // Right-align the current value
  tft.setTextSize(2);
  int valueWidth = strlen(valueStr) * 12;
  tft.setTextColor(valueColor);
  tft.setCursor(235 - valueWidth, 2);
  tft.print(valueStr);

  drawGraph();
}

void drawGraph() {
  // Graph area - adjusted to fit with status bar
  int graphX = 22;
  int graphY = 20;
  int graphWidth = 195;
  int graphHeight = 75;

  // Draw graph border
  tft.drawRect(graphX - 1, graphY - 1, graphWidth + 2, graphHeight + 2, COLOR_DARK_GRAY);

  // Find min/max for scaling
  float minVal = 0, maxVal = 0;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      minVal = 400;  // CO2 rarely goes below 400
      maxVal = 2000; // Cap at 2000 for scale
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

  // Draw threshold lines for CO2
  if (currentDisplayValue == DISPLAY_CO2) {
    int yGreen = graphY + graphHeight - (int)(((CO2_THRESHOLD_GREEN - minVal) / (maxVal - minVal)) * graphHeight);
    int yYellow = graphY + graphHeight - (int)(((CO2_THRESHOLD_YELLOW - minVal) / (maxVal - minVal)) * graphHeight);
    int yOrange = graphY + graphHeight - (int)(((CO2_THRESHOLD_ORANGE - minVal) / (maxVal - minVal)) * graphHeight);

    if (yGreen > graphY && yGreen < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yGreen, graphWidth, COLOR_GREEN);
    }
    if (yYellow > graphY && yYellow < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yYellow, graphWidth, COLOR_YELLOW);
    }
    if (yOrange > graphY && yOrange < graphY + graphHeight) {
      tft.drawFastHLine(graphX, yOrange, graphWidth, COLOR_ORANGE);
    }
  }

  // Draw the graph line (always draw what we have, even if just 1 point)
  if (historyCount > 0) {
    float xStep = (float)graphWidth / (GRAPH_POINTS - 1);

    for (int i = 1; i < historyCount; i++) {
      // Get indices in circular buffer order (oldest to newest)
      int idx0 = (historyIndex - historyCount + i - 1 + GRAPH_POINTS) % GRAPH_POINTS;
      int idx1 = (historyIndex - historyCount + i + GRAPH_POINTS) % GRAPH_POINTS;

      float val0, val1;
      uint16_t color;

      switch (currentDisplayValue) {
        case DISPLAY_CO2:
          val0 = historyC02[idx0];
          val1 = historyC02[idx1];
          color = getCO2Color((uint16_t)val1);
          break;
        case DISPLAY_TEMPERATURE:
          val0 = historyTemperature[idx0];
          val1 = historyTemperature[idx1];
          color = getTemperatureColor(val1);
          break;
        case DISPLAY_HUMIDITY:
          val0 = historyHumidity[idx0];
          val1 = historyHumidity[idx1];
          color = getHumidityColor(val1);
          break;
        default:
          val0 = val1 = 0;
          color = COLOR_WHITE;
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

      tft.drawLine(x0, y0, x1, y1, color);
    }

    // If we only have one point, draw it as a dot
    if (historyCount == 1) {
      int idx = (historyIndex - 1 + GRAPH_POINTS) % GRAPH_POINTS;
      float val;
      uint16_t color;

      switch (currentDisplayValue) {
        case DISPLAY_CO2:
          val = historyC02[idx];
          color = getCO2Color((uint16_t)val);
          break;
        case DISPLAY_TEMPERATURE:
          val = historyTemperature[idx];
          color = getTemperatureColor(val);
          break;
        case DISPLAY_HUMIDITY:
          val = historyHumidity[idx];
          color = getHumidityColor(val);
          break;
        default:
          val = 0;
          color = COLOR_WHITE;
          break;
      }

      int y = graphY + graphHeight - (int)(((val - minVal) / (maxVal - minVal)) * graphHeight);
      y = constrain(y, graphY, graphY + graphHeight);
      tft.fillCircle(graphX, y, 2, color);
    }
  }

  // Draw scale labels on the right
  tft.setTextColor(COLOR_LIGHT_GRAY);
  tft.setTextSize(1);
  tft.setCursor(graphX + graphWidth + 3, graphY);
  tft.printf("%.0f", maxVal);
  tft.setCursor(graphX + graphWidth + 3, graphY + graphHeight - 8);
  tft.printf("%.0f", minVal);

  // Draw time labels - 6:00 to 6:00 (24h period)
  tft.setCursor(graphX, graphY + graphHeight + 3);
  tft.print("6:00");
  tft.setCursor(graphX + graphWidth - 24, graphY + graphHeight + 3);
  tft.print("6:00");

  // Draw data point count indicator
  tft.setTextColor(COLOR_DARK_GRAY);
  tft.setCursor(graphX + graphWidth/2 - 15, graphY + graphHeight + 3);
  tft.printf("%d pts", historyCount);
}

void drawStatusBar() {
  // Status bar at bottom (y: 118-135, height: 17px)
  int yPos = 118;
  int barHeight = 17;

  // Draw background
  tft.fillRect(0, yPos, 240, barHeight, COLOR_DARK_GRAY);

  // Draw a subtle top border
  tft.drawFastHLine(0, yPos, 240, COLOR_LIGHT_GRAY);

  // Three sections: Mode | View | Value
  // Each section is ~80px wide

  tft.setTextSize(1);
  int textY = yPos + 5;

  // Section 1: Display mode (Always On / Warn)
  if (currentDisplayMode == MODE_ALWAYS_ON) {
    tft.setTextColor(COLOR_GREEN);
    tft.setCursor(8, textY);
    tft.print("ALWAYS ON");
  } else {
    tft.setTextColor(COLOR_YELLOW);
    tft.setCursor(8, textY);
    tft.print("WARN MODE");
  }

  // Draw separator
  tft.drawFastVLine(80, yPos + 3, barHeight - 6, COLOR_LIGHT_GRAY);

  // Section 2: View mode (Number / Graph)
  tft.setTextColor(COLOR_WHITE);
  tft.setCursor(95, textY);
  if (currentViewMode == VIEW_NUMBER) {
    tft.print("VALUE");
  } else {
    tft.print("GRAPH");
  }

  // Draw separator
  tft.drawFastVLine(160, yPos + 3, barHeight - 6, COLOR_LIGHT_GRAY);

  // Section 3: Current measurement type with color indicator
  int indicatorX = 172;
  uint16_t indicatorColor;
  const char* label;

  switch (currentDisplayValue) {
    case DISPLAY_CO2:
      indicatorColor = getCO2Color(currentCO2);
      label = "CO2";
      break;
    case DISPLAY_TEMPERATURE:
      indicatorColor = COLOR_WHITE;
      label = "TEMP";
      break;
    case DISPLAY_HUMIDITY:
      indicatorColor = COLOR_CYAN;
      label = "HUMID";
      break;
    default:
      indicatorColor = COLOR_WHITE;
      label = "";
      break;
  }

  // Draw color dot indicator
  tft.fillCircle(indicatorX, yPos + barHeight/2, 4, indicatorColor);

  // Draw label
  tft.setTextColor(COLOR_WHITE);
  tft.setCursor(indicatorX + 10, textY);
  tft.print(label);
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
  (void)temp;  // Temperature always displayed in white
  return COLOR_WHITE;
}

uint16_t getHumidityColor(float humidity) {
  (void)humidity;  // Humidity always displayed in blue
  return COLOR_CYAN;
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
