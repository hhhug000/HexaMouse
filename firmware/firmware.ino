#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// --- Hardware Constants ---
constexpr int N = 3;
Adafruit_MLX90393 mags[N];
const uint8_t ADDRESSES[N] = {0x0C, 0x0D, 0x0E};

// --- NeoPixel Configuration ---
constexpr int LED_PIN = D3;       
constexpr int LED_COUNT = 3;      
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const float ALPHA = 0.03f;

// --- Recalibration Deadzone & Noise Settings ---
const float RECALIB_DEADZONE = 0.9f;  
const float OUTPUT_DEADZONE  = 0.15f; 
const unsigned long IDLE_TIMEOUT_MS = 1500; 
unsigned long stableStartTime = 0;
bool isCurrentlyStable = false;

// --- State Variables ---
float runningBase[N][3] = {0};
bool initialized = false;

// --- Binary Protocol Struct ---
#pragma pack(push, 1)
struct SensorPacket {
  uint8_t magic = 0xAA;
  float posX;
  float posY;
  float posZ;
  float tiltX;
  float tiltY;
  float yaw;
  uint8_t checksum;
};
#pragma pack(pop)

// --- Function Declarations ---
void initHardware();
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void triggerRecalibration(bool manual);
void checkCalibrationTriggers(float curr[N][3]);
bool readSensorData(float curr[N][3]);
void updateBaselineAndGetDeltas(float curr[N][3], float dBx[N], float dBy[N], float dBz[N]);
void computeKinematics(float dBx[N], float dBy[N], float dBz[N], float &Tx, float &Ty, float &Tz, float &Rx, float &Ry, float &Rz);
void sendSensorPacket(float Tx, float Ty, float Tz, float Rx, float Ry, float Rz);
float applyDeadzone(float val, float deadzone);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  initHardware();
}

void loop() {
  // Check for manual serial recalibration command ('n' or 'N')
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'n' || c == 'N') {
      triggerRecalibration(true); // Manual trigger
    }
  }

  float curr[N][3];
  if (!readSensorData(curr)) return;

  // Check for idle auto-recalibration deadzone trigger
  checkCalibrationTriggers(curr);

  float dBx[N], dBy[N], dBz[N];
  updateBaselineAndGetDeltas(curr, dBx, dBy, dBz);

  float Tx, Ty, Tz, Rx, Ry, Rz;
  computeKinematics(dBx, dBy, dBz, Tx, Ty, Tz, Rx, Ry, Rz);

  sendSensorPacket(Tx, Ty, Tz, Rx, Ry, Rz);

  delay(15);
}

// --- Implementation ---

void initHardware() {
  strip.begin();
  strip.setBrightness(40); 
  setLedColor(0, 255, 0);   // Green on boot

  for (int i = 0; i < N; i++) {
    if (!mags[i].begin_I2C(ADDRESSES[i], &Wire)) {
      while (1) {
        setLedColor(255, 0, 0); // Flash Red on fatal error
        delay(500);
        setLedColor(0, 0, 0);
        delay(500);
      }
    }
    mags[i].setGain(MLX90393_GAIN_1X);
    mags[i].setOversampling(MLX90393_OSR_1);
    mags[i].setFilter(MLX90393_FILTER_3);
  }
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void triggerRecalibration(bool manual) {
  setLedColor(0, 0, 255); // Flash Blue for re-zeroing acknowledgement
  initialized = false;    // Forces baseline recapture on next loop iteration
  if (manual) {
    delay(150);
  } else {
    delay(100);
  }
  setLedColor(0, 255, 0); // Return to Green
}

bool readSensorData(float curr[N][3]) {
  for (int i = 0; i < N; i++) {
    if (!mags[i].readData(&curr[i][0], &curr[i][1], &curr[i][2])) {
      return false;
    }
  }
  return true;
}

void checkCalibrationTriggers(float curr[N][3]) {
  if (!initialized) return;

  // Evaluate if all sensors are staying within the recalibration deadzone band
  bool withinRecalibZone = true;
  for (int i = 0; i < N; i++) {
    for (int axis = 0; axis < 3; axis++) {
      if (fabs(curr[i][axis] - runningBase[i][axis]) > RECALIB_DEADZONE) {
        withinRecalibZone = false;
        break;
      }
    }
    if (!withinRecalibZone) break;
  }

  unsigned long now = millis();
  if (withinRecalibZone) {
    if (!isCurrentlyStable) {
      isCurrentlyStable = true;
      stableStartTime = now;
    } else if (now - stableStartTime >= IDLE_TIMEOUT_MS) {
      // Lingered in the recalibration deadzone too long: fire auto-recalibration
      for (int i = 0; i < N; i++) {
        for (int axis = 0; axis < 3; axis++) {
          runningBase[i][axis] = curr[i][axis];
        }
      }
      stableStartTime = now;
      triggerRecalibration(false); // Auto trigger
    }
  } else {
    isCurrentlyStable = false;
    setLedColor(0, 255, 0); // Normal operation: Green
  }
}

void updateBaselineAndGetDeltas(float curr[N][3], float dBx[N], float dBy[N], float dBz[N]) {
  if (!initialized) {
    for (int i = 0; i < N; i++) {
      for (int axis = 0; axis < 3; axis++) {
        runningBase[i][axis] = curr[i][axis];
      }
    }
    initialized = true;
    setLedColor(0, 255, 0);
  }

  for (int i = 0; i < N; i++) {
    for (int axis = 0; axis < 3; axis++) {
      if (!isCurrentlyStable) {
        runningBase[i][axis] = (1.0f - ALPHA) * runningBase[i][axis] + ALPHA * curr[i][axis];
      }
    }
    dBx[i] = curr[i][0] - runningBase[i][0];
    dBy[i] = curr[i][1] - runningBase[i][1];
    dBz[i] = curr[i][2] - runningBase[i][2];
  }
}

float applyDeadzone(float val, float deadzone) {
  if (fabs(val) < deadzone) return 0.0f;
  return (val > 0.0f) ? (val - deadzone) : (val + deadzone);
}

void computeKinematics(float dBx[N], float dBy[N], float dBz[N], 
                       float &Tx, float &Ty, float &Tz, 
                       float &Rx, float &Ry, float &Rz) {
  float rawTx = (dBx[0] + dBx[1] + dBx[2]) / 3.0f;
  float rawTy = (dBy[0] + dBy[1] + dBy[2]) / 3.0f;
  float rawTz = (dBz[0] + dBz[1] + dBz[2]) / 3.0f;

  float rawRx = 0.577350269f * (dBz[1] + dBz[2] - 2.0f * dBz[0]);
  float rawRy = dBz[2] - dBz[1];

  const float px[N] = {0.0f, -0.5f, 0.5f};
  const float py[N] = {-0.5f,  0.5f,  0.5f};
  float rawRz = 0.0f;
  for (int i = 0; i < N; i++) {
    rawRz += (px[i] * dBy[i] - py[i] * dBx[i]);
  }

  // Mapped physical axes
  float mappedTx = -rawTy;         
  float mappedTy = rawTz;          
  float mappedTz = -rawTx;         

  // Apply output deadzone clamping
  Tx = applyDeadzone(mappedTx, OUTPUT_DEADZONE);
  Ty = applyDeadzone(mappedTy, OUTPUT_DEADZONE);
  Tz = applyDeadzone(mappedTz, OUTPUT_DEADZONE);
  Rx = applyDeadzone(rawRx,    OUTPUT_DEADZONE);
  Ry = applyDeadzone(rawRy,    OUTPUT_DEADZONE);
  Rz = applyDeadzone(rawRz,    OUTPUT_DEADZONE);
}

void sendSensorPacket(float Tx, float Ty, float Tz, float Rx, float Ry, float Rz) {
  SensorPacket pkt;
  pkt.posX = Tx;
  pkt.posY = Ty;
  pkt.posZ = Tz;
  pkt.tiltX = Rx;
  pkt.tiltY = Ry;
  pkt.yaw = Rz;

  uint8_t* ptr = (uint8_t*)&pkt.posX;
  uint8_t crc = 0;
  for (int i = 0; i < 24; i++) {
    crc ^= ptr[i];
  }
  pkt.checksum = crc;

  Serial.write((uint8_t*)&pkt, sizeof(SensorPacket));
}