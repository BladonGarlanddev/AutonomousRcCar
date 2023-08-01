//4-11-2023:
//Changes:
//cleaned up code
//added debug statements 

//4-12-2023:
//Changes:
//removed instances of old mag code
//added code to utilize new mag

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>
#include <RC_LOGGER_418.h>
#include <LiquidCrystal.h>
#include <RC_GPS_418.h>
#include <RC_418.h>

#define EEPROM_OFFSET 0

void setup() {

  
 
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);
  lcd.clear();
  mtr.attach(3);
  servo.attach(2);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  mtr.writeMicroseconds(1000);
  Serial.println("SparkFun u-blox Example");
  lcd.clear();
  //check for mag and gps i2c 
  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    lcd.print("GPS failed freezing");
    while (1);
  }
  Serial.println("GPS good");

  if (!bno.begin())
  {
    Serial.println("Failed to initialize BNO055 sensor!");
    lcd.print("IMU failed freezing");
    while (1);
  }
  Serial.println("Mag good");
  lcd.print("both sensors good");
  //gps settings
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA messages to processNMEA
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);

  //mag settings
  bno.setMode(OPERATION_MODE_NDOF);
  //this line sets clock to external crystal. may crash hardware
  bno.setExtCrystalUse(true);
  // Load calibration data from EEPROM
  EEPROM.get(EEPROM_OFFSET, calibrationData);
  // Set calibration data to the BNO055 sensor
  
  //bno.setSensorOffsets(calibrationData);
  //wait for good gps vals
  gps.init();
  lcd.clear();
  lcd.print("GPS ready");
  //get obstacles
  enterPrepMode(obstacles);

  //log obstacle position data to eeprom
  /*
  for (int i = 0; i < 8; i++) {
    char buffer[16];
    dtostrf(obstacle[i][0], 15, 6, buffer);
    String obstacleLat = String(buffer);
    buffer[16];
    dtostrf(obstacle[i][1], 15, 6, buffer);
    String obstacleLon = String(buffer);
    log(obstacleLat);
    log(obstacleLon);
  }
  */
  delay(300);
  while(digitalRead(46) == 1) {
    delay(500);
  }
  Serial.println("set up complete"); 
  gps.readCoord();
  car.lastHeading = readIMU();
}

void loop() {
  
  for (int i = 0; i < obstacles; i++) {
    lcd.clear();
    gps.read();
    car.obstacleID = i; 
    while (gps.distanceTo(i) > 40) {
      gps.read();
      delay(50);
      car.faceTowards(gps.getHeading(i));
      delay(50);
      //lcd.clear();
      //lcd.print(gps.distanceTo(i));
    }
  }
  Serial.println("obstacle course complete");
  lcd.clear();
  lcd.print("Course complete");
  mtr.writeMicroseconds(1000);
  servo.write(25);
  while(1);
  delay(50);
  //if (analogRead(A1) < 100) {
  //  Serial.println("loging");
  //  readLog();
  //}
}


