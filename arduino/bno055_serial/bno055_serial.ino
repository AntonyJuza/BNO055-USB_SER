/*
 * BNO055 IMU Serial Driver for Arduino Uno
 *
 * Reads BNO055 sensor data over I2C and sends it via USB serial
 * to be consumed by a ROS2 node on Raspberry Pi / Linux host.
 *
 * Wiring (BNO055 -> Arduino Uno):
 *   VIN -> 5V  |  GND -> GND  |  SDA -> A4  |  SCL -> A5
 *
 * ─────────────────── Serial Protocol (115200 baud) ────────────────────────
 *
 *  HOST → ARDUINO:
 *    LOAD_CAL,accel_off_x,accel_off_y,accel_off_z,
 *             mag_off_x,mag_off_y,mag_off_z,
 *             gyro_off_x,gyro_off_y,gyro_off_z,
 *             accel_radius,mag_radius
 *
 *  ARDUINO → HOST:
 *    BNO,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,ex,ey,ez,temp,
 *        cal_sys,cal_gyro,cal_accel,cal_mag
 *       (22 fields total, index 0 = "BNO")
 *
 *    CAL_STATUS,sys,gyro,accel,mag
 *    CAL_COMPLETE
 *    CAL_OFFSETS,ax,ay,az,mx,my,mz,gx,gy,gz,accel_radius,mag_radius
 *    CAL_LOADED
 *
 * Libraries required:
 *   - Adafruit BNO055
 *   - Adafruit Unified Sensor
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// BNO055 sensor instance (I2C address 0x28 default)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Timing
const unsigned long SEND_INTERVAL_MS = 20;  // ~50 Hz
unsigned long lastSendTime = 0;

// Calibration tracking
uint8_t prevCalSys = 0, prevCalGyro = 0, prevCalAccel = 0, prevCalMag = 0;
bool fullyCalibrated = false;

// ─── Helpers ────────────────────────────────────────────────────────────────

void broadcastOffsets() {
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);

  Serial.print("CAL_OFFSETS,");
  Serial.print(offsets.accel_offset_x); Serial.print(",");
  Serial.print(offsets.accel_offset_y); Serial.print(",");
  Serial.print(offsets.accel_offset_z); Serial.print(",");
  Serial.print(offsets.mag_offset_x);   Serial.print(",");
  Serial.print(offsets.mag_offset_y);   Serial.print(",");
  Serial.print(offsets.mag_offset_z);   Serial.print(",");
  Serial.print(offsets.gyro_offset_x);  Serial.print(",");
  Serial.print(offsets.gyro_offset_y);  Serial.print(",");
  Serial.print(offsets.gyro_offset_z);  Serial.print(",");
  Serial.print(offsets.accel_radius);   Serial.print(",");
  Serial.println(offsets.mag_radius);
}

bool applyOffsetsFromSerial(String line) {
  // Expected: LOAD_CAL,ax,ay,az,mx,my,mz,gx,gy,gz,ar,mr
  int parts[11];
  int start = line.indexOf(',') + 1;  // skip "LOAD_CAL"

  for (int i = 0; i < 11; i++) {
    int comma = line.indexOf(',', start);
    if (comma == -1) comma = line.length();
    parts[i] = line.substring(start, comma).toInt();
    start = comma + 1;
  }

  adafruit_bno055_offsets_t offsets;
  offsets.accel_offset_x = parts[0];
  offsets.accel_offset_y = parts[1];
  offsets.accel_offset_z = parts[2];
  offsets.mag_offset_x   = parts[3];
  offsets.mag_offset_y   = parts[4];
  offsets.mag_offset_z   = parts[5];
  offsets.gyro_offset_x  = parts[6];
  offsets.gyro_offset_y  = parts[7];
  offsets.gyro_offset_z  = parts[8];
  offsets.accel_radius   = parts[9];
  offsets.mag_radius     = parts[10];

  bno.setSensorOffsets(offsets);
  Serial.println("CAL_LOADED");
  return true;
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("BNO055 IMU Serial Driver v2");
  Serial.println("============================");
  Serial.println("Initializing BNO055...");

  // Start in CONFIG mode so stored offsets can be written before NDOF
  if (!bno.begin(OPERATION_MODE_CONFIG)) {
    Serial.println("BNO_ERROR");
    Serial.println("ERROR: No BNO055 detected. Check wiring:");
    Serial.println("  VIN -> 5V | GND -> GND | SDA -> A4 | SCL -> A5");
    while (1) {
      delay(1000);
      if (bno.begin(OPERATION_MODE_CONFIG)) {
        Serial.println("BNO055 connected after retry!");
        break;
      }
      Serial.println("BNO_ERROR: Retrying...");
    }
  }

  bno.setExtCrystalUse(true);
  delay(100);

  // Display sensor info
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.print("Sensor: ");    Serial.println(sensor.name);
  Serial.print("Driver Ver: "); Serial.println(sensor.version);
  Serial.println("");

  Serial.println("=== CALIBRATION GUIDE ===");
  Serial.println("Gyroscope:     Place sensor still on flat surface");
  Serial.println("Accelerometer: Rotate sensor to 6 positions (each axis up/down)");
  Serial.println("Magnetometer:  Move sensor in figure-8 pattern");
  Serial.println("System:        Auto-calibrates when others are calibrated");
  Serial.println("Values: 0=uncalibrated, 3=fully calibrated");
  Serial.println("=========================");
  Serial.println("");

  // Signal ROS2 node: ready and waiting for optional offset restore
  Serial.println("BNO_READY");

  // Wait up to 3 s for host to send stored calibration offsets
  unsigned long waitStart = millis();
  bool offsetsLoaded = false;
  while (millis() - waitStart < 3000) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.startsWith("LOAD_CAL,")) {
        offsetsLoaded = applyOffsetsFromSerial(line);
        if (offsetsLoaded) {
          Serial.println("Offsets restored from host.");
          fullyCalibrated = true;
        }
        break;
      }
    }
    delay(10);
  }

  if (!offsetsLoaded) {
    Serial.println("No stored offsets — starting fresh calibration.");
  }

  // Switch to full NDOF fusion mode
  bno.setMode(OPERATION_MODE_NDOF);
  delay(100);
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
  // Handle runtime commands from host
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("LOAD_CAL,")) {
      applyOffsetsFromSerial(line);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = currentTime;

  // ── Calibration ──────────────────────────────────────────────────────────
  uint8_t calSys, calGyro, calAccel, calMag;
  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

  if (calSys != prevCalSys || calGyro != prevCalGyro ||
      calAccel != prevCalAccel || calMag != prevCalMag) {

    Serial.print("CAL_STATUS,");
    Serial.print(calSys);   Serial.print(",");
    Serial.print(calGyro);  Serial.print(",");
    Serial.print(calAccel); Serial.print(",");
    Serial.println(calMag);

    prevCalSys   = calSys;
    prevCalGyro  = calGyro;
    prevCalAccel = calAccel;
    prevCalMag   = calMag;

    if (calSys == 3 && calGyro == 3 && calAccel == 3 && calMag == 3) {
      if (!fullyCalibrated) {
        Serial.println("CAL_COMPLETE");
        broadcastOffsets();
        fullyCalibrated = true;
      }
    } else {
      fullyCalibrated = false;
    }
  }

  // ── Fusion data ───────────────────────────────────────────────────────────
  imu::Quaternion quat  = bno.getQuat();
  imu::Vector<3>  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3>  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3>  mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3>  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int8_t          temp  = bno.getTemp();

  // ── CSV data line ─────────────────────────────────────────────────────────
  // Format: BNO,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,ex,ey,ez,temp,
  //             cal_sys,cal_gyro,cal_accel,cal_mag    (22 fields)
  Serial.print("BNO,");

  // Quaternion (w, x, y, z)
  Serial.print(quat.w(), 4); Serial.print(",");
  Serial.print(quat.x(), 4); Serial.print(",");
  Serial.print(quat.y(), 4); Serial.print(",");
  Serial.print(quat.z(), 4); Serial.print(",");

  // Gyroscope (rad/s)
  Serial.print(gyro.x(), 4); Serial.print(",");
  Serial.print(gyro.y(), 4); Serial.print(",");
  Serial.print(gyro.z(), 4); Serial.print(",");

  // Linear acceleration (m/s²)
  Serial.print(accel.x(), 4); Serial.print(",");
  Serial.print(accel.y(), 4); Serial.print(",");
  Serial.print(accel.z(), 4); Serial.print(",");

  // Magnetometer (µT)
  Serial.print(mag.x(), 2); Serial.print(",");
  Serial.print(mag.y(), 2); Serial.print(",");
  Serial.print(mag.z(), 2); Serial.print(",");

  // Euler angles (heading, roll, pitch in degrees)
  Serial.print(euler.x(), 2); Serial.print(",");
  Serial.print(euler.y(), 2); Serial.print(",");
  Serial.print(euler.z(), 2); Serial.print(",");

  // Temperature (°C)
  Serial.print(temp); Serial.print(",");

  // Calibration status
  Serial.print(calSys);   Serial.print(",");
  Serial.print(calGyro);  Serial.print(",");
  Serial.print(calAccel); Serial.print(",");
  Serial.println(calMag);
}
