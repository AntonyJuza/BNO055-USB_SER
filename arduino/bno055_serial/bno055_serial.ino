/*
 * BNO055 IMU Serial Driver for Arduino Uno
 * 
 * Reads BNO055 sensor data over I2C and sends it via USB serial
 * to be consumed by a ROS2 node on Raspberry Pi 4.
 * 
 * Wiring (BNO055 -> Arduino Uno):
 *   VIN -> 5V
 *   GND -> GND
 *   SDA -> A4
 *   SCL -> A5
 * 
 * Serial Protocol (115200 baud):
 *   BNO,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyro,cal_accel,cal_mag
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
const unsigned long SEND_INTERVAL_MS = 20; // ~50 Hz
unsigned long lastSendTime = 0;

// Calibration tracking
uint8_t prevCalSys = 0, prevCalGyro = 0, prevCalAccel = 0, prevCalMag = 0;
bool fullyCalibrated = false;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("BNO055 IMU Serial Driver");
  Serial.println("========================");
  Serial.println("Initializing BNO055...");
  
  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("BNO_ERROR");
    Serial.println("ERROR: No BNO055 detected. Check wiring:");
    Serial.println("  VIN -> 5V");
    Serial.println("  GND -> GND");
    Serial.println("  SDA -> A4");
    Serial.println("  SCL -> A5");
    while (1) {
      delay(1000);
      // Retry initialization
      if (bno.begin()) {
        Serial.println("BNO055 connected after retry!");
        break;
      }
      Serial.println("BNO_ERROR: Retrying...");
    }
  }
  
  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  // Set to NDOF mode (all sensors fused) — this is the default
  // NDOF = Nine Degrees Of Freedom: accel + gyro + mag all fused
  
  delay(100);
  
  // Display sensor info
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.print("Sensor: ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver: ");
  Serial.println(sensor.version);
  
  Serial.println("");
  Serial.println("=== CALIBRATION GUIDE ===");
  Serial.println("Gyroscope:     Place sensor still on flat surface");
  Serial.println("Accelerometer: Rotate sensor to 6 positions (each axis up/down)");
  Serial.println("Magnetometer:  Move sensor in figure-8 pattern");
  Serial.println("System:        Auto-calibrates when others are calibrated");
  Serial.println("Values: 0=uncalibrated, 3=fully calibrated");
  Serial.println("=========================");
  Serial.println("");
  
  Serial.println("BNO_READY");
  delay(500);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = currentTime;
    
    // Read calibration status
    uint8_t calSys, calGyro, calAccel, calMag;
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
    
    // Print calibration changes to help user during calibration
    if (calSys != prevCalSys || calGyro != prevCalGyro || 
        calAccel != prevCalAccel || calMag != prevCalMag) {
      Serial.print("CAL_STATUS,");
      Serial.print(calSys);   Serial.print(",");
      Serial.print(calGyro);  Serial.print(",");
      Serial.print(calAccel); Serial.print(",");
      Serial.println(calMag);
      
      prevCalSys = calSys;
      prevCalGyro = calGyro;
      prevCalAccel = calAccel;
      prevCalMag = calMag;
      
      // Check if fully calibrated
      if (calSys == 3 && calGyro == 3 && calAccel == 3 && calMag == 3) {
        if (!fullyCalibrated) {
          Serial.println("CAL_COMPLETE");
          fullyCalibrated = true;
        }
      } else {
        fullyCalibrated = false;
      }
    }
    
    // Read quaternion orientation (fused output)
    imu::Quaternion quat = bno.getQuat();
    
    // Read angular velocity (gyroscope) in rad/s
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // Read linear acceleration (without gravity) in m/s^2
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    // Send data as CSV line
    // Format: BNO,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyro,cal_accel,cal_mag
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
    
    // Linear acceleration (m/s^2)
    Serial.print(accel.x(), 4); Serial.print(",");
    Serial.print(accel.y(), 4); Serial.print(",");
    Serial.print(accel.z(), 4); Serial.print(",");
    
    // Calibration status
    Serial.print(calSys);   Serial.print(",");
    Serial.print(calGyro);  Serial.print(",");
    Serial.print(calAccel); Serial.print(",");
    Serial.println(calMag);
  }
}
