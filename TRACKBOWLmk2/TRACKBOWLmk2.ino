// Jacek Fedorynski <jfedor@jfedor.org>
// http://www.jfedor.org/

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <PDM.h>
int32_t mic;
 
extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read
 
#define ROTATE_Z_PIN 10
#define Z_ROTATION_FILENAME "/trackbowl_z"

Adafruit_LSM6DS33 lsm6ds;
Adafruit_LIS3MDL lis3mdl;
Adafruit_Sensor_Calibration_SDFat cal;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

Adafruit_NXPSensorFusion filter;
//Adafruit_Madgwick filter;
//Adafruit_Mahony filter;

BLEDis bledis;
BLEHidAdafruit blehid;
BLEBas blebas;

uint32_t prev_millis;
float prevquat[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float rotationZ = 0.0f;
float x = 0.0f;
float y = 0.0f;
bool needToPersistRotationZ = false;
uint32_t lastRotationZAdjustment;
float battery = 0;
uint32_t lastBatteryReport;

void setup() {
//********************
Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
Serial.println("Trackbowl Mouse is active");

  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);


//*********************
  
  pinMode(ROTATE_Z_PIN, INPUT_PULLUP);
  // for battery level readings
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  // assuming we followed the magnetometer calibration procedure at
  // https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions
  cal.begin();
  cal.loadCalibration();

  lsm6ds.begin_I2C();
  lis3mdl.begin_I2C();
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  filter.begin(66); // Hz

  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
  Bluefruit.autoConnLed(false);
  Bluefruit.begin();
  Bluefruit.setName("TRACKBOWL");

  // Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Feather Sense");
  bledis.begin();

  blehid.begin();

  blebas.begin();
  battery = getBatteryLevel();
  blebas.write(battery);

  Bluefruit.Periph.setConnInterval(12, 12); // 12*1.25ms=15ms

  // if "Z rotation" is on during startup, clear bonding information
  if (digitalRead(ROTATE_Z_PIN) == LOW) {
    Bluefruit.clearBonds();
  }

  startAdvertising();

  // not sure if needed
  Wire.setClock(400000);

  InternalFS.begin();
  loadZRotation();
}

void startAdvertising(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loadZRotation() {
  Adafruit_LittleFS_Namespace::File file(InternalFS);
  if (file.open(Z_ROTATION_FILENAME, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
    file.read((uint8_t*) &rotationZ, sizeof(rotationZ));
    file.close();
  }
}

void saveZRotation() {
  Adafruit_LittleFS_Namespace::File file(InternalFS);
  if (file.open(Z_ROTATION_FILENAME, Adafruit_LittleFS_Namespace::FILE_O_WRITE)) {
    file.seek(0);
    file.write((uint8_t*) &rotationZ, sizeof(rotationZ));
    file.close();
  }
  needToPersistRotationZ = false; // even if failed to actually persist
}

float getBatteryLevel() {
  int raw = analogRead(PIN_VBAT);
  // board-specific and dependent on resolution and reference
  float mvolts = raw * 2.0f * 3000.0f / 4096.0f;
  if (mvolts < 3300) {
    return 0;
  }
  if (mvolts < 3600) {
    mvolts -= 3300;
    return mvolts / 30;
  }
  mvolts -= 3600;
  return 10 + (mvolts * 0.15f);
}

// quaternion multiplication
void quatmul(float* result, float* r, float* q) {
  result[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
  result[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
  result[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
  result[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
}
//****************************


int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;
 
  while (samples > 0) {
    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}
 
void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();
 
  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);
 
  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

//*********************************

void loop() {
  // we target 66 Hz
  uint32_t current_millis = millis();
  if ((current_millis - prev_millis) < 15) {
    return;
  }

  

  //************************

     samplesRead = 0;
  mic = getPDMwave(400);
  //Serial.println(mic);
  if(mic > 1000){
    blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
    blehid.mouseButtonRelease();
    Serial.println("Mouse Clicked");
    }

 
    //Serial.print("Mic: ");
  
  //***********************
  prev_millis = current_millis;

  if (needToPersistRotationZ && (lastRotationZAdjustment + 3000 < current_millis)) {
    saveZRotation();
  }

  // the reading is noisy so we keep a running average
  battery = 0.995f * battery + 0.005f * getBatteryLevel();
  // report battery level every 30 seconds
  if (lastBatteryReport + 30000 < current_millis) {
    blebas.notify(battery);
    lastBatteryReport = current_millis;
  }

  bool zRotationButtonState = digitalRead(ROTATE_Z_PIN) == LOW;

  sensors_event_t accel, gyro, mag;

  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz,
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  float quat[4];
  filter.getQuaternion(&quat[0], &quat[1], &quat[2], &quat[3]);

  for (int i = 1; i < 4; i++) {
    prevquat[i] *= -1;
  }
  float dquat[4];
  quatmul(dquat, prevquat, quat);

  for (int i = 0; i < 4; i++) {
    prevquat[i] = quat[i];
  }

  // convert quaternion to axis-angle
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

  float s = sqrt(1 - dquat[0] * dquat[0]);

  float angle;
  if (!isnan(s) && s > 0.001) {
    angle = 2.0 * acos(dquat[0]);
    // we want the angle between -180 and 180 degrees
    if (angle > PI) {
      angle = angle - 2 * PI;
    }

    float rotationX = angle * dquat[1] / s;
    float rotationY = angle * dquat[2] / s;
    if (zRotationButtonState) {
      rotationZ += angle * dquat[3] / s;
      needToPersistRotationZ = true;
      lastRotationZAdjustment = current_millis;
    }

    // rotate around Z axis using the accumulated angle to let the user set the "up" direction
    float sinz = sin(rotationZ);
    float cosz = cos(rotationZ);
    // rotation around X axis moves the cursor along Y axis
    y += rotationX * cosz - rotationY * sinz;
    x += rotationX * sinz + rotationY * cosz;
  }

  int8_t int_x = (int8_t) max(-127, min(127, (255 * x)));
  int8_t int_y = (int8_t) max(-127, min(127, (255 * y)));

  if (int_x != 0 || int_y != 0) {
    blehid.mouseMove(int_x, int_y);
    x = 0.0f;
    y = 0.0f;
  }
}
