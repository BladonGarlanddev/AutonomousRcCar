#include "Constants.h"
#include "RC.h"

static int lastIMURead = 0;

class Memory {
public:
  int memoryStored = 0;
  int writeAddress = 0;
  int logLength = 0;
  String something = "";
  void writeToMemory(String message) {
    message += "\n";
    writeAddress = memoryStored; // starting address in EEPROM // string to be stored in EEPROM
    logLength = message.length(); // length of string in bytes
    memoryStored += message.length();
  // write string to EEPROM
    for (int i = 0; i < logLength; i++) {
      char c = message.charAt(i); // get character from string
      EEPROM.write(writeAddress + i, c); // write character to EEPROM
    }
  }

  void readLog() {
    String readString = "";
    for(int i = 0; i < memoryStored; i++){
      char c = EEPROM.read(0 + i);
      if(c == '\n'){
        Serial.println(readString);
        readString = "";
      } else {
        readString += c;
      }
    }
  }
};
Memory memory;

class GPS {
public:
    double lat;
    double lon;
    double velocity;
    double lastLat;
    double lastLon;
    int lastReadTime;

    bool read() {
        myGNSS.checkUblox();
        if(nmea.isValid() == true) {
            lat = nmea.getLatitude() / 1000000.0000;
            lon = nmea.getLongitude() / 1000000.0000;
            velocity = distanceTo(lastLat, lastLon) / ((millis()-lastReadTime)/1000);
            lastLat = lat;
            lastLon = lon;
            lastReadTime = millis();
            nmea.clear();
            return true;
        } return false;
    }

    bool readCoord() {
        myGNSS.checkUblox();
        while(nmea.isValid() == false) {
            myGNSS.checkUblox();
        }
        lat = nmea.getLatitude() / 1000000.0000;
        lon = nmea.getLongitude() / 1000000.0000;
        nmea.clear();
        return true;
    }
    

    double distanceTo(int i) {
    // convert to radians
    double currentLatRad = convert_to_radians(this->lat);
    double currentLonRad = convert_to_radians(this->lon);
    double desiredLatRad = convert_to_radians(obstacle[i][0]);
    double desiredLonRad = convert_to_radians(obstacle[i][1]);

    // Haversine formula
    double delta_lat = desiredLatRad - currentLatRad;
    double delta_lon = desiredLonRad - currentLonRad;
    double a = pow(sin(delta_lat / 2), 2) + cos(currentLatRad) * cos(desiredLatRad) * pow(sin(delta_lon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance_km = 6371.0 * c;

    return distance_km * 10000;
    }

    double distanceTo(float lastLat, float lastLon) {
    // convert to radians
    double currentLatRad = convert_to_radians(this->lat);
    double currentLonRad = convert_to_radians(this->lon);
    double desiredLatRad = convert_to_radians(lastLat);
    double desiredLonRad = convert_to_radians(lastLon);

    // Haversine formula
    double delta_lat = desiredLatRad - currentLatRad;
    double delta_lon = desiredLonRad - currentLonRad;
    double a = pow(sin(delta_lat / 2), 2) + cos(currentLatRad) * cos(desiredLatRad) * pow(sin(delta_lon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance_km = 6371.0 * c * 10000;

    return distance_km;
    }

    double getHeading(int i) {
        double desLatCon = convert_to_radians(obstacle[i][0]);
        double desLonCon = convert_to_radians(obstacle[i][1]);
        double latCon = convert_to_radians(this->lat);
        double lonCon = convert_to_radians(this->lon);
        double heading = atan2(desLatCon - latCon, lonCon - desLonCon) * 180.0 / pi;
        heading -= 90;
    // Adjust heading for northern/southern hemisphere
        if (heading < 0) {
            heading += 360;
        } else {
            if (heading > 360) {
            heading -= 360;                                                                                                                                                                                                               
            }
        }
    return heading;
    }   

};
static GPS gps;

class Car {
public:
  float diff; 
  int obstacleID;
  float heading;
  float lastHeading;
  float lastDiff;
  float derivative;
  float integral;
  const int zeroAngle = 30;
  int lastCalculate;
  int calcPeriod = 100;
  float Ki = .0001;
  float Kd = 5;
  const int bufferSize = 30;
  float integral_array[30];
  int bufferIndex = 0;
  int bufferCount = 0;
  int steeringUnit;

  void findIntegralDerivative() {
    if(millis() - lastCalculate > calcPeriod) {
      derivative = constrain(Kd * (lastDiff - diff), -10, 10);
      if(abs(diff) < 2) {
        diff = 0;
      }
      integral_array[bufferIndex] = Ki * diff * calcPeriod;
      bufferIndex = (bufferIndex + 1) % bufferSize;
      bufferCount = min(bufferCount + 1, bufferSize);
      integral = 0;
      for(int i = 0; i < bufferCount; i++) {
        integral += integral_array[i];
      }
      integral = constrain(integral, -4, 4);
      lastDiff = diff;
      lastCalculate = millis();
    }
  }

  void faceTowards(float desDeg) {
    findIntegralDerivative();
    mtr.writeMicroseconds(1260);
    lcd.clear();

    if(360 - desDeg + heading < abs(desDeg - heading)) {
      diff = -1*(360 - desDeg + heading);
      lcd.print("CC");
    } 
    else if (360 - heading + desDeg < abs(desDeg - heading)) 
    {
      diff = 360 - heading + desDeg;
      lcd.print("CW");
    } 
    else 
    {
      diff = desDeg - heading;
      if(diff > 0) {
        lcd.print("right");
      } else lcd.print("left");
    }

    Serial.print("Arr: {");
    for(int i = 0; i < 20; i++) {
      Serial.print(integral_array[i]);
      Serial.print(", ");
    }
    Serial.println("}");
    setSteeringUnit();
    servo.write(zeroAngle + steeringUnit);
    lcd.setCursor(0,0);
    lcd.print(desDeg);
    lcd.setCursor(0,1);
    lcd.print(heading);
  }

  void predictPosition() {
    int time = (millis() - lastIMURead) / 1000;
    float referenceAngle = heading - 180;
    if(referenceAngle < 0) {
      referenceAngle += 360;
    }
    referenceAngle = referenceAngle*(pi/180);
    const float degreesPerTurnUnit = .75;
    float wheelDegrees = (abs(steeringUnit) * degreesPerTurnUnit)*(pi/180);
    //wheelbase * sin of degrees of steering
    float turningRadius = 24 / tan(wheelDegrees);
    float arcLength = ((heading-lastHeading) * turningRadius + time*gps.velocity)/2;
    //angle between two points in radians * turnRadius
    float angleOfTwoPoints = arcLength/turningRadius;
    float deltaX = -(steeringUnit/abs(steeringUnit))*(turningRadius*cos(angleOfTwoPoints)-turningRadius);
    float deltaY = turningRadius*sin(angleOfTwoPoints);
    float delta_lat = -1*((-deltaX * sin(referenceAngle)) + (deltaY * cos(referenceAngle)));
    float delta_lon = -1*((deltaX * cos(referenceAngle)) + (deltaY * sin(referenceAngle)));
    gps.lat += delta_lon / 111320;
    gps.lon += delta_lat / 111320 * cos(gps.lat*(pi/180));
  }

  void setSteeringUnit()  
  {
    steeringUnit = constrain((diff/abs(diff))*(43.0 / (1 + exp(-0.06 * abs(diff) +1))-11) + derivative + integral, -30, 30);
  }

  int desSpeed(){
    int distance_speed = constrain(exp((.015*gps.distanceTo(obstacleID))+3) + 1260, 1270, 1400);
    int turn_speed = constrain(-1*pow((.6*steeringUnit) + 2, 2) + 1600, 1270, 1400);
    return (distance_speed + turn_speed*3) / 4;
  }  
};
static Car car; 

void initializeCar() 
{   
    lcd.clear();
    mtr.attach(3);
    servo.attach(2);
    pinMode(3, OUTPUT);
    pinMode(2, OUTPUT);
    servo.write(car.zeroAngle);
    mtr.writeMicroseconds(1000);

    if (myGNSS.begin() == false) {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        lcd.print("GPS failed freezing");
        while (1);
    }
    lcd.print("Sensors good");

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
    lcd.setCursor(0,1);
    lcd.print("GPS initializing");
    Serial.println("GPS initializing");
    while(gps.read() == false)
    {
        if(millis() > 100000)
        {
            Serial.println("GPS too cold or no good readings... Freezing");
            while(true);
        }
    }
    Serial.println("GPS initialized");       
}

void enterDiagnosticMode() {
    Serial.println("diagnostic mode entered");
    String inputString = "";
    while (!Serial.available()) {
    // Do nothing
    }

  // Read user input
    while (Serial.available()) {
      inputString += (char)Serial.read();
      delay(5);
    }

    inputString.trim();
    inputString.toLowerCase();

  // Check if input string matches expected string
    if (inputString == "log") {
      memory.readLog();
    } else {
      Serial.println("command unknown");
    }
}

double convert_to_radians(double degrees) {
        return degrees * 3.14159265358979323846 / 180;
}  