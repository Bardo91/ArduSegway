///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "IMU.h"

class ArduSegway{
public:
  bool init();

  bool isReady();

  IMU &imu();
  
private:
  IMU mImu;
  bool mIsReady = false;
};

