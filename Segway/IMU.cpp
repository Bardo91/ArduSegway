///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "IMU.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// Static data initialization
static IMU* IMU::mInstance = nullptr;

//-------------------------------------------------------------------------------------------------------------------
static bool IMU::create(){
  if(mInstance==nullptr){
    mInstance = new IMU();
    return true;
  }else{
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------------------
IMU* IMU::get(){
  return mInstance;
}

//-------------------------------------------------------------------------------------------------------------------
bool IMU::init(){
    mMpu = new MPU6050();
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    mMpu->initialize();
    mDevStatus = mMpu->dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mMpu->setXGyroOffset(20);
    mMpu->setYGyroOffset(-35);
    mMpu->setZGyroOffset(20);
    mMpu->setZAccelOffset(920); // 1688 factory default for my test chip
    mMpu->setXAccelOffset(-1000);
    mMpu->setYAccelOffset(-2380);
    // make sure it worked (returns 0 if so)
    
    if (mDevStatus == 0) {
        mMpu->setDMPEnabled(true);
        attachInterrupt(0, callbackIMU, RISING);
        mMpuIntStatus = mMpu->getIntStatus();
        mDmpReady = true;
        mPacketSize = mMpu->dmpGetFIFOPacketSize();
        return true;
    } else {
       return false;
    }
}

//-------------------------------------------------------------------------------------------------------------------
bool IMU::isReady(){
  return mDmpReady;
}

//-------------------------------------------------------------------------------------------------------------------
vec3 IMU::accel(){
  updateData();
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  mMpu->dmpGetQuaternion(&q, mFifoBuffer);
  mMpu->dmpGetAccel(&aa, mFifoBuffer);
  mMpu->dmpGetGravity(&gravity, &q);
  mMpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

//-------------------------------------------------------------------------------------------------------------------
vec3 IMU::accelWorld(){
  updateData();
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  mMpu->dmpGetQuaternion(&q, mFifoBuffer);
  mMpu->dmpGetAccel(&aa, mFifoBuffer);
  mMpu->dmpGetGravity(&gravity, &q);
  mMpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mMpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}
  
//-------------------------------------------------------------------------------------------------------------------
vec3 IMU::gyro(){
  
}

vec3 IMU::euler(){
  updateData();
  Quaternion q;           // [w, x, y, z]         quaternion container
  float euler[3];         // [psi, theta, phi]    Euler angle container
  mMpu->dmpGetQuaternion(&q, mFifoBuffer);
  mMpu->dmpGetEuler(euler, &q);
  vec3 angles = {euler[0], euler[1], euler[2]};
  return angles;
}

//-------------------------------------------------------------------------------------------------------------------
Quaternion IMU::quaternion(){
  updateData();
  Quaternion q;           // [w, x, y, z]         quaternion container
  mMpu->dmpGetQuaternion(&q, mFifoBuffer);
  return q;
}

//-------------------------------------------------------------------------------------------------------------------
vec3 IMU::ypr(){
  updateData();
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  mMpu->dmpGetQuaternion(&q, mFifoBuffer);
  mMpu->dmpGetGravity(&gravity, &q);
  mMpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
  vec3 angles = {ypr[0], ypr[1], ypr[2]};
  return angles;
}

//-------------------------------------------------------------------------------------------------------------------
void IMU::updateData(){
  // wait for MPU interrupt or extra packet(s) available
    while (!mMpuInterrupt && mFifoCount < mPacketSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mMpuInterrupt = false;
    mMpuIntStatus = mMpu->getIntStatus();

    // get current FIFO count
    mFifoCount = mMpu->getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mMpuIntStatus & 0x10) || mFifoCount == 1024) {
        // reset so we can continue cleanly
        mMpu->resetFIFO();
    } else if (mMpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (mFifoCount < mPacketSize) mFifoCount = mMpu->getFIFOCount();

        // read a packet from FIFO
        mMpu->getFIFOBytes(mFifoBuffer, mPacketSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        mFifoCount -= mPacketSize;
    }
}

//--------------------------------------------------------------------------------------------------------------
IMU::IMU(){
}

//--------------------------------------------------------------------------------------------------------------
void IMU::dmpDataReady() {
    mMpuInterrupt = true;
}

//--------------------------------------------------------------------------------------------------------------
void callbackIMU() {
      IMU::get()->dmpDataReady();
}

