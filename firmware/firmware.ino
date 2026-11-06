#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <math.h>

constexpr int N = 3;
Adafruit_MLX90393 mag0, mag1, mag2;
Adafruit_MLX90393* mags[N] = {&mag0, &mag1, &mag2};
const uint8_t addresses[N] = {0x0C, 0x0D, 0x0E};

// Running baseline tracker for dynamic centering across all axes (Bx, By, Bz)
float runningBase[N][3] = {0};
bool initialized = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  for (int i = 0; i < N; i++) {
    if (!mags[i]->begin_I2C(addresses[i], &Wire)) {
      while (1) delay(1000);
    }
    mags[i]->setGain(MLX90393_GAIN_1X);
    mags[i]->setOversampling(MLX90393_OSR_1);
    mags[i]->setFilter(MLX90393_FILTER_3);
  }
}

void loop() {
  float curr[N][3]; // [sensor][0:Bx, 1:By, 2:Bz]
  
  for (int i = 0; i < N; i++) {
    if (!mags[i]->readData(&curr[i][0], &curr[i][1], &curr[i][2])) return;
  }

  // Initialize dynamic baselines instantly on first read
  if (!initialized) {
    for (int i = 0; i < N; i++) {
      for (int axis = 0; axis < 3; axis++) {
        runningBase[i][axis] = curr[i][axis];
      }
    }
    initialized = true;
  }

  // Alpha filter for dynamic DC bias removal
  float alpha = 0.03f;
  float dBx[N], dBy[N], dBz[N];

  for (int i = 0; i < N; i++) {
    for (int axis = 0; axis < 3; axis++) {
      runningBase[i][axis] = (1.0f - alpha) * runningBase[i][axis] + alpha * curr[i][axis];
    }
    dBx[i] = curr[i][0] - runningBase[i][0];
    dBy[i] = curr[i][1] - runningBase[i][1];
    dBz[i] = curr[i][2] - runningBase[i][2];
  }

  // 1. Average horizontal and vertical shifts
  float rawX = 0.0f;
  float rawY = 0.0f;
  float totalLift = 0.0f;

  for (int i = 0; i < N; i++) {
    rawX += dBx[i];
    rawY += dBy[i];
    totalLift += dBz[i];
  }
  rawX /= 3.0f;
  rawY /= 3.0f;
  totalLift /= 3.0f;

  // --- DEVICE-SIDE AXIS REMAPPING ---
  // Screen X (Left/Right slide) = -rawY
  // Screen Y (Up/Down lift)     = -totalLift
  // World Z (Depth forward/back) = rawX
  float posX = -rawY;
  float posY = -totalLift;
  float posZ = rawX;

  // 2. Differential Tilt
  float tiltX = dBz[1] - dBz[2];
  float tiltY = (2.0f * dBz[0] - dBz[1] - dBz[2]) * 0.57735f;

  // Stream pre-mapped values directly
  Serial.print("X:"); Serial.print(posX, 2); Serial.print("\t");
  Serial.print("Y:"); Serial.print(posY, 2); Serial.print("\t");
  Serial.print("Z:"); Serial.print(posZ, 2); Serial.print("\t");
  Serial.print("TiltX:"); Serial.print(tiltX, 2); Serial.print("\t");
  Serial.print("TiltY:"); Serial.println(tiltY, 2);

  delay(15);
}