///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "IMU.h"

#include <PID_v1.h>

//Define Variables we'll be connecting to
double input, output;
int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;    

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
IMU *imu;

double setpoint = 0.04;
 PID myPID(&input, &output, &setpoint,20,0.01,15, DIRECT);
void setup() {
    //Serial.begin(9600);
    IMU::create();
    imu = IMU::get();
    if (!imu->init()){
      //Serial.println("Error IMU not initialized");
    }
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255,255);
     pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT); 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if(imu->isReady()){
      vec3 ypr = imu->ypr();
      //Serial.println("");
      //Serial.print("--- Input: ");
      //Serial.print(ypr.y);
      //Serial.print(" ---> Output: ");
      input = ypr.y;
      myPID.Compute();
      //Serial.print(output);
      if(output < 0){
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
      }else{
        digitalWrite(M1,LOW);   
        digitalWrite(M2, LOW);
      }
      double val = 32*sqrt(abs(output));
      
      analogWrite(E1, val);   //PWM Speed Control
      analogWrite(E2, val);   //PWM Speed Control
    }
}
