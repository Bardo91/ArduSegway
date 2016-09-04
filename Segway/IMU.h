///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ARDUSEGWAY_IMU_H_
#define _ARDUSEGWAY_IMU_H_

struct Quaternion;
class MPU6050;

struct vec3{
  float x, y, z;
};

class IMU{
public: // Public Interface
  /// Init communication with the MPU6050 and configure offsets.
  static bool create();
  static IMU *get();


  bool init();
  
  /// Returns true if everything was configured properly during the initialization.
  bool isReady();

  // Get accelerations on device coordinates.
  vec3 accel();

  // Get acceleration on world coordinates.
  vec3 accelWorld();
  
  vec3 gyro();

  vec3 euler();

  // Get Quaternion.
  Quaternion quaternion();
  
  /// Get Yaw, pitch and roll
  vec3 ypr();

  void dmpDataReady();
  
private:  // Private interface
  IMU();
  void updateData();
  
private:  // members
  MPU6050 *mMpu;

  // MPU control/status vars
  bool mDmpReady = false;  // set true if DMP init was successful
  unsigned char mMpuIntStatus;   // holds actual interrupt status byte from MPU
  unsigned char mDevStatus;      // return status after each device operation (0 = success, !0 = error)
  unsigned int mPacketSize;    // expected DMP packet size (default is 42 bytes)
  unsigned int mFifoCount;     // count of all bytes currently in FIFO
  unsigned char mFifoBuffer[64]; // FIFO storage buffer

  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================
  volatile bool mMpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  // Singletone
  static IMU *mInstance;

};

void callbackIMU();

#endif  //  _ARDUSEGWAY_IMU_H_
