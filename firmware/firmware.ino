#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <math.h>

constexpr int N = 3;
Adafruit_MLX90393 mag0, mag1, mag2;
Adafruit_MLX90393* mags[N] = {&mag0, &mag1, &mag2};
const uint8_t addresses[N] = {0x0C, 0x0D, 0x0E};

const float sensorAngles[N] = { 
  0.0f,               // 0°
  2.0943951f,         // 120°
  4.1887902f          // 240°
};

float runningBase[N][3] = {0};
bool initialized = false;

// Strict binary packet structure matching Python unpacker
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
  // Check for manual reset command from host
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'n' || c == 'N') {
      initialized = false;
    }
  }

  float curr[N][3]; 
  for (int i = 0; i < N; i++) {
    if (!mags[i]->readData(&curr[i][0], &curr[i][1], &curr[i][2])) return;
  }

  if (!initialized) {
    for (int i = 0; i < N; i++) {
      for (int axis = 0; axis < 3; axis++) {
        runningBase[i][axis] = curr[i][axis];
      }
    }
    initialized = true;
  }

  // Alpha filter for dynamic DC bias cancellation
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

  // 1. Translation (Horizontal Bx/By shifts + Lift)
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

  float posX = -rawY;
  float posY = -totalLift;
  float posZ = rawX;

  // 2. Orthogonal Tilt Projection (120° spatial derivative)
  float tiltX = 0.0f;
  float tiltY = 0.0f;
  for (int i = 0; i < N; i++) {
    tiltX += dBz[i] * cosf(sensorAngles[i]);
    tiltY += dBz[i] * sinf(sensorAngles[i]);
  }
  tiltX *= (2.0f / 3.0f);
  tiltY *= (2.0f / 3.0f);

  // 3. Twist / Yaw (Tangential horizontal curl)
  float yaw = 0.0f;
  for (int i = 0; i < N; i++) {
    yaw += (-dBx[i] * sinf(sensorAngles[i]) + dBy[i] * cosf(sensorAngles[i]));
  }
  yaw /= 3.0f;
  yaw = -yaw; // Inverted to match correct physical rotation sense

  // Pack binary payload
  SensorPacket pkt;
  pkt.posX = posX;
  pkt.posY = -posY;
  pkt.posZ = posZ;
  pkt.tiltX = tiltX;
  pkt.tiltY = tiltY;
  pkt.yaw = yaw;

  // Calculate XOR Checksum over the 24 payload bytes
  uint8_t* ptr = (uint8_t*)&pkt.posX;
  uint8_t crc = 0;
  for (int i = 0; i < 24; i++) {
    crc ^= ptr[i];
  }
  pkt.checksum = crc;

  // Stream raw binary frame
  Serial.write((uint8_t*)&pkt, sizeof(SensorPacket));

  delay(15);
}