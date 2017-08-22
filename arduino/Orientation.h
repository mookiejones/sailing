#ifndef _ORIENTATION_H_
#define _ORIENTATION_H_
#include "Arduino.h"
#include "CurieIMU.h"


class Orientation {
  private:
    // The boards orientation
    int orientation;
    // string for printing description of orientation
    String orientation_string;

  public :
   /*
    The orientations of the board:
    0: flat, processor facing up
    1: flat, processor facing down
    2: landscape, analog pins down
    3: landscape, analog pins up
    4: portrait, USB connector up
    5: portrait, USB connector down
  */
    static int Y_UP;
    static int Y_DOWN;
    static int X_UP;
    static int X_DOWN;
    static int Z_UP;
    static int Z_DOWN;
    Orientation();
    void Listen();

};

Orientation::Orientation(){
  {
    orientation=-1;
    X_UP=0;
    X_DOWN=1;
    Y_UP=2;
    Y_DOWN=3;
    Z_UP=4;
    Z_DOWN=5;

    Serial.begin(9600); // initialize Serial communication
    while (!Serial);    // wait for the serial port to open
    
        // initialize device
    Serial.println("Initializing IMU device...");
    CurieIMU.begin();

    // Set the accelerometer range to 2G
    CurieIMU.setAccelerometerRange(2);
  }   


  void Orientation::Listen(){
  
      // initialize device
      Serial.println("listening");
  }
}
#endif

