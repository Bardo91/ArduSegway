///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


struct vec3{
  float x, y, z;
};

struct quaternion{
  float x, y, z, w;
};

class IMU{
public: // Public Interface
  /// Init communication with the MPU6050 and configure offsets.
  bool init();

  /// Returns true if everything was configured properly during the initialization.
  bool isReady();

  // Get accelerations on device coordinates.
  vec3 accel();

  // Get acceleration on world coordinates.
  vec3 accelWorld();
  
  vec3 gyro();

  // Get Quaternion.
  Quaternion quaternion();
  
  /// Get Yaw, pitch and roll
  vec3 ypr();
  
private:  // Private interface
  void updateData();
private:  // members
  MPU6050 mMpu;

  // MPU control/status vars
  bool mDmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t mDevStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t mPacketSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t mFifoCount;     // count of all bytes currently in FIFO
  uint8_t mFifoBuffer[64]; // FIFO storage buffer

  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================
  volatile bool mMpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {
      mMpuInterrupt = true;
  }

};

