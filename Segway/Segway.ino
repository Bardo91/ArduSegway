///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "IMU.h"

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
IMU *imu;
void setup() {
    Serial.begin(9600);
    IMU::create();
    imu = IMU::get();
    if (!imu->init()){
      Serial.println("Error IMU not initialized");
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if(imu->isReady()){
      vec3 ypr = imu->ypr();
      Serial.println("");
      Serial.print(ypr.x);
      Serial.print(", ");
      Serial.print(ypr.y);
      Serial.print(", ");
      Serial.print(ypr.z);
    }
}
