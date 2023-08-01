#ifndef CONSTANTS
#define CONSTANTS

#include <Wire.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MicroNMEA.h>
#include <EEPROM.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define EEPROM_OFFSET 0
#define BNO055_SAMPLERATE_DELAY_MS (100)

static const int rs = 43, en = 41, d4 = 29, d5 = 31, d6 = 33, d7 = 35;
static LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

static double obstacle[10][2];
static int num_of_obstacles = 10;

static Servo mtr;
static Servo servo;  

static SFE_UBLOX_GNSS myGNSS;
static char nmeaBuffer[100];
static MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

static const double pi = 3.14159265358979323846;

static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
static adafruit_bno055_offsets_t calibrationData;

#endif