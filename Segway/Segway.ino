///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ArduSegway
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ArduSegway.h"
ArduSegway segway;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    segway.init();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!segway.isReady()) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // READ VALUES
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
