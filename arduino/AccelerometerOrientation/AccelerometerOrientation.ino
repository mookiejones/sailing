// Includes
#include "CurieIMU.h"
#include "CurieBLE.h"

int ax, ay, az; // accelerometer values
int gx, gy, gz; // gyrometer values

BLEPeripheral blePeripheral;         // BLE Peripheral Device (the board you're programming)
BLEService heartRateService("180D"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic heartRateChar("2A37",                  // standard 16-bit characteristic UUID
                                BLERead | BLENotify, 2); // remote clients will be able to get notifications if this characteristic changes
                                                         // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                                                         // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

int oldHeartRate = 0;    // last heart rate reading from analog input
long previousMillis = 0; // last time the heart rate was checked, in ms

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

void setup()
{

  Serial.begin(9600); // initialize Serial communication
  while (!Serial)
    ; // wait for the serial port to open
  setupHeartBeat();
  setupGyroscope();
  // initialize device


}


void setupHeartBeat(){
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected
  
    /* Set a local name for the BLE device
       This name will appear in advertising packets
       and can be used by remote devices to identify this BLE device
       The name can be changed but maybe be truncated based on space left in advertisement packet */
    blePeripheral.setLocalName("HeartRateSketch");
    blePeripheral.setAdvertisedServiceUuid(heartRateService.uuid());  // add the service UUID
    blePeripheral.addAttribute(heartRateService);   // Add the BLE Heart Rate service
    blePeripheral.addAttribute(heartRateChar); // add the Heart Rate Measurement characteristic
  
    /* Now activate the BLE device.  It will start continuously transmitting BLE
       advertising packets and will be visible to remote BLE central devices
       until it receives a new connection */
    blePeripheral.begin();
    Serial.println("Bluetooth device active, waiting for connections...");
}
void setupGyroscope(){
  writeMessage("Initializing Gyroscope...");
  // verify connection
  writeMessage("Testing device connections...");

  String begin = CurieIMU.begin() ? "CurieIMU connection successful" : "CurieIMU connection failed";
  writeMessage(begin);

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1)
  {

    writeMessage("Internal sensor offsets BEFORE calibration...");
    String x = String(CurieIMU.getAccelerometerOffset(X_AXIS));
    String y = String(CurieIMU.getAccelerometerOffset(Y_AXIS));
    String z = String(CurieIMU.getAccelerometerOffset(Z_AXIS));

    writeMessage("x:" + x + "\ty:" + y + "\tz" + z);

    String rx = String(CurieIMU.getGyroOffset(X_AXIS));
    String ry = String(CurieIMU.getGyroOffset(Y_AXIS));
    String rz = String(CurieIMU.getGyroOffset(Z_AXIS));
    writeMessage("rx:" + rx + "\try:" + ry + "\trz" + rz);

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
    //CurieIMU.setGyroOffset(X_AXIS,7.869);
    //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
    //CurieIMU.setGyroOffset(Z_AXIS,15.494);

    writeMessage("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    writeMessage("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    writeMessage(" Done");

    writeMessage("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    writeMessage(" Done");

    /*
    writeMessage("Internal sensor offsets AFTER calibration...");
    writeMessage(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
    */
  }

  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);
  /// Set the accelerometer range to 250 degrees/second
  //  CurieIMU.setAccelerometerRange(360);
}
String format(int x, int y, int z, int rx, int ry, int rz)
{
  String _x = String(x);
  String _y = String(y);
  String _z = String(z);
  String _rx = String(rx);
  String _ry = String(ry);
  String _rz = String(rz);
  String time = String(millis() >> 9);

  String json = String("{ \"time\":" + time + ",\"x\":" + _x + ",\"y\":" + _y + ",\"z\":" + _z + ",\"rx\":" + _rx + ",\"ry\":" + _ry + ",\"rz\":" + _rz + "}");
  return json;
}

void writeToPC(int x, int y, int z, int rx, int ry, int rz)
{
  String output = format(x, y, z, rx, ry, rz);
  Serial.print(output);
  Serial.println();
}

void blink()
{
  if (blinkState == true)
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  else
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  delay(200);                       // wait for a second
  blinkState = !blinkState;
}
void loop()
{
  checkGyroscope();
  checkHeartBeat();
}

void checkGyroscope()
{

  // read raw accel/gyro measurements from device
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  // these methods (and a few others) are also available
  blink();
  //CurieIMU.readAcceleration(ax, ay, az);
  //CurieIMU.readRotation(gx, gy, gz);

  //ax = CurieIMU.readAccelerometer(X_AXIS);
  //ay = CurieIMU.readAccelerometer(Y_AXIS);
  //az = CurieIMU.readAccelerometer(Z_AXIS);
  //gx = CurieIMU.readGyro(X_AXIS);
  //gy = CurieIMU.readGyro(Y_AXIS);
  //gz = CurieIMU.readGyro(Z_AXIS);

  writeToPC(gx, gy, gz, ax, ay, az);
  // read gyro measurements from device, scaled to the configured range
  // CurieIMU.readGyroScaled(gx, gy, gz);

  //  writeToPC(gx,gy,gz);
}

void updateHeartRate()
{
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.
  */
  int heartRateMeasurement = analogRead(A0);
  int heartRate = map(heartRateMeasurement, 0, 1023, 0, 100);
  if (heartRate != oldHeartRate)
  {                                      // if the heart rate has changed
    Serial.print("Heart Rate is now: "); // print it
    Serial.println(heartRate);
    const unsigned char heartRateCharArray[2] = {0, (char)heartRate};
    heartRateChar.setValue(heartRateCharArray, 2); // and update the heart rate measurement characteristic
    oldHeartRate = heartRate;                      // save the level for next comparison
  }
}

void checkHeartBeat()
{
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected())
    {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200)
      {
        previousMillis = currentMillis;
        updateHeartRate();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void writeMessage(String message)
{
  String msg = "{\"message\":\"" + message + "\"}";
  Serial.println(msg);
}

