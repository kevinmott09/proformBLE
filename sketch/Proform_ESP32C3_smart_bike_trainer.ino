#include "button.h"
#include <ArduinoBLE.h>
#include <HardwareSerial.h>

HardwareSerial MySerial1(1);  //instantiate HardwareSerial instance
BLEDevice central;            //instantiate BLE instance
//instantiate button instances for debounce library
Button rDnBtn;  //rearDownButton
Button rUpBtn;
Button fDnBtn;  //frontDownButton
Button fUpBtn;

//set to true to print messages to the console via serial0 (usb)
boolean debugging = false;
boolean powerDebug = false;
boolean debuggingCad = false;
boolean adjustBikeTilt = false;

#define FU_B 8    //front gears shift up button, front chainring
#define FD_B 20   //front gears shift down button
#define RU_B 21   //rear gears shift up button, rear cassette
#define RD_B 7    //rear gears shift down button
#define ENABLE 5  //write enable for RS485
#define LED 9     //led attached to this pin to indicate activity
#define RXD2 4    //pins for rx to proform
#define TXD2 3    //pins for tx from proform

// The Fitness Machine Control Point data type structure
#define FMCP_DATA_SIZE 19  // Control point consists of 1 opcode (byte) and maximum 18 bytes as parameters

// This fmcp_data_t structure represents the control point data. The first octet represents the opcode of the request
// followed by a parameter array of maximum 18 octects
typedef struct __attribute__((packed)) {
  uint8_t OPCODE;
  uint8_t OCTETS[FMCP_DATA_SIZE - 1];
} fmcp_data_t;

typedef union  // The union type automatically maps the bytes member array to the fmcp_data_t structure member values
{
  fmcp_data_t values;
  uint8_t bytes[FMCP_DATA_SIZE];
} fmcp_data_ut;

fmcp_data_ut fmcpData;
short fmcpValueLength;


BLEService fitnessMachineService("1826");
BLECharacteristic fitnessMachineFeatureCharacteristic("2ACC", BLERead, 8);                                   // Fitness Machine Feature, mandatory, read
BLECharacteristic indoorBikeDataCharacteristic("2AD2", BLENotify, 8);                                        // Indoor Bike Data, optional, notify
BLECharacteristic trainingStatusCharacteristic("2AD3", BLENotify | BLERead, 20);                             // Training Status, optional, read & notify
BLECharacteristic supportedResistanceLevelRangeCharacteristic("2AD6", BLERead, 4);                           // Supported Resistance Level, read, optional
BLECharacteristic fitnessMachineControlPointCharacteristic("2AD9", BLEWrite | BLEIndicate, FMCP_DATA_SIZE);  // Fitness Machine Control Point, optional, write & indicate
BLECharacteristic fitnessMachineStatusCharacteristic("2ADA", BLENotify, 2);                                  // Fitness Machine Status, mandatory, notify

// Buffers used to write to the characteristics and initial values
unsigned char ftmsFeatureBuf[4] = { 0b00000011, 0b01000000, 0, 0 };  //, 0, 0, 0, 0};                        // Features: 0 (Avg speed), 1 (Cadence), 2 (Total distance), 7 (Resistance level), 10 (Heart rate measurement), 14 (Power measurement)
unsigned char indoorBikeDataBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char ftmsControlPointBuf[20];

// Indoor Bike Data characteristic variables

const uint16_t flagMoreData = 1; //0000 0001
const uint16_t flagAverageSpeed = 2; //0000 0010
const uint16_t flagInstantaneousCadence = 4; //0000 0100
const uint16_t flagAverageCadence = 8; //0000 1000
const uint16_t flagTotalDistance = 16;
const uint16_t flagResistanceLevel = 32;
const uint16_t flagIntantaneousPower = 64;
const uint16_t flagAveragePower = 128;
const uint16_t flagExpendedEnergy = 256;
const uint16_t flagHeartRate = 512;
const uint16_t flagMetabolicEquivalent = 1024;
const uint16_t flagElapsedTime = 2048;
const uint16_t flagRemainingTime = 4096;

// Fitness Machine Control Point opcodes

const uint8_t fmcpRequestControl = 0x00;
const uint8_t fmcpReset = 0x01;
const uint8_t fmcpSetTargetSpeed = 0x02;
const uint8_t fmcpSetTargetInclination = 0x03;
const uint8_t fmcpSetTargetResistanceLevel = 0x04;
const uint8_t fmcpSetTargetPower = 0x05;
const uint8_t fmcpSetTargetHeartRate = 0x06;
const uint8_t fmcpStartOrResume = 0x07;
const uint8_t fmcpStopOrPause = 0x08;
const uint8_t fmcpSetTargetedExpendedEngery = 0x09;
const uint8_t fmcpSetTargetedNumberOfSteps = 0x0A;
const uint8_t fmcpSetTargetedNumberOfStrided = 0x0B;
const uint8_t fmcpSetTargetedDistance = 0x0C;
const uint8_t fmcpSetTargetedTrainingTime = 0x0D;
const uint8_t fmcpSetTargetedTimeInTwoHeartRateZones = 0x0E;
const uint8_t fmcpSetTargetedTimeInThreeHeartRateZones = 0x0F;
const uint8_t fmcpSetTargetedTimeInFiveHeartRateZones = 0x10;
const uint8_t fmcpSetIndoorBikeSimulationParameters = 0x11;
const uint8_t fmcpSetWheelCircumference = 0x12;
const uint8_t fmcpSetSpinDownControl = 0x13;
const uint8_t fmcpSetTargetedCadence = 0x14;
const uint8_t fmcpResponseCode = 0x80;



// *** change these variable to match your fitness/environment ***

float ftpWatts = 160;  // base FTP to use for sim calculations
float losses = 1;      // losses multiplier, account for losses in drive train

// Data for the resistance calculation in sim mode

float wind_speed = 0;  // meters per second, resolution 0.001
float grade = 0;       // percentage, resolution 0.01
float oldGrade = 0;
float crr = 0;  // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;   // Wind resistance Kg/m, resolution 0.01;

// power variables

unsigned int instantaneous_cadence = 0;  // Global variable to hold the calculated cadence in RPM
unsigned int instantaneous_power = 0;    // Global variable to hold the calculated power in watts
unsigned int instantaneous_speed = 0;    // Global variable to hold the calculated speed in m/s
unsigned int current_gear = 0;

float Watts = 0.0;
//constants for power equation
float a = 1900;
float b = -0.3;
float c = 0.5;
float d = 0;

unsigned int resistanceSetting = 0;
unsigned int oldGearSetting = 0;
unsigned int setWatts = 0;
boolean OVERRIDE = false;
bool commandWaiting = false;

unsigned long previousMillis = 0UL;
unsigned long interval = 1000UL;
unsigned long oldACKtime = 0UL;
unsigned long ACKtime = 20UL;

int loopCounter = 0;

int current_incline = 30;  //(levels from -15 to +15 by half increments. Index for zero is 30)
int last_incline = 30;
int gear = 0;
int frontGear = 0;
int rearGear = 0;
int gearOffset = 0;

//initialize gear command array and later fill via code
byte gearCommand1[130];
byte gearCommand2[130];
byte gearCommand3[130];
byte gearCommand4[130];
//these are the bytes for incline changes. note that "0" is number 31 below and changes are 0.5 degree increments on the bike
//                           1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,   24,   25,   26,   27,   28,   29,   30,   31,   32,   33,   34,   35,   36,   37,   38,   39,   40,   41,   42,   43,   44,   45,   46,   47,   48,   49,   50,   51,   52,   53,   54,   55,   56,   57,   58,   59,   60,   61
byte inclineCommand1[] = { 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB2, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3, 0xB3 };
byte inclineCommand2[] = { 0xC6, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xC1, 0xC2 };
byte inclineCommand3[] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2, 0xC2, 0xC2, 0xC2, 0xC2, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB7, 0xB7, 0xB7 };
byte inclineCommand4[] = { 0xB9, 0xB8, 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xB9, 0xB8, 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xB9, 0xB8, 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xB9, 0xB8, 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xC6, 0xC5, 0xC4 };

//variables for reading MySerial1 and calculating cadence
bool waitACK = false;
byte incoming = 0;
bool responseData = false;
bool cadenceRequest = false;
int count = 0;
int byte14 = 0;
int byte15 = 0;


void InitializeGears() {
  //fill array with gear/resistance ratios.  There are four bytes for each gear level, each byte starts at a different position, some count up, some count down.
  //this random seeming code creates the array of all possible resistance levels
  unsigned char code[] = { 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6 };
  unsigned char oneMask[] = { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
  unsigned char threeMask[] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
  int initialOffset2 = 15;
  int initialOffset3 = 8;
  int initialOffset4 = 11;
  int one, two, three, four;
  int countloop = 0;
  int maxp = 8;
  for (int p = 0; p < maxp; p++) {
    for (unsigned int k = 0; k < 16; k++) {
      one = (oneMask[k]);
      gearCommand1[countloop] = (code[one + p]);
      two = (k + initialOffset2) % 16;
      gearCommand2[countloop] = (code[two]);
      three = (initialOffset3 - threeMask[k]);
      gearCommand3[countloop] = (code[three - (p % maxp)]);
      four = (16 - initialOffset4 - k) % 16;
      gearCommand4[countloop] = (code[four]);
      countloop++;
    }
  }
}

// Set the correct resistance level on the physical trainer
//15 degrees of incline in 0.5 degree steps.  30 levels.  Max grade in Zwift is 18%?  Most not above 14%.
//but, we could say that 15% is the max resistance setting.  Works out to about 8 gear steps per degree (128/15=8.5333)
//incline is given in 0.01degree increments, so the gear change should be rounded to nearest
int SetTrainerResistance(float wind_speed, float grade, float crr, float cw) {
  current_incline = (int)(30 + (int)(grade * 2));  //set the incline of the trainer

//return (int)(grade * 8);  //this is crazy harsh and stops you in your tracks on the steeper climbs
  return (int)(grade * 4);  //this is returning a gear setting.  (grade*8) is realistic but harsh in zwift
}

void SetIncline(int incline) {
  if (adjustBikeTilt) {  //this feature is a bit annoying, so I turned it off
    //given in 0.01 degree increments.
    digitalWrite(ENABLE, HIGH);
    MySerial1.write(0xBA);
    MySerial1.write(0xB4);
    MySerial1.write(0xB1);
    MySerial1.write(0xB0);
    MySerial1.write(0xB6);
    MySerial1.write(0xB0);
    MySerial1.write(0xB0);
    MySerial1.write(0xB0);
    MySerial1.write(0xB1);
    MySerial1.write(0xB0);
    MySerial1.write(0xB0);
    MySerial1.write(inclineCommand1[incline]);
    MySerial1.write(inclineCommand2[incline]);
    MySerial1.write(inclineCommand3[incline]);
    MySerial1.write(inclineCommand4[incline]);
    MySerial1.write(0x8D);
    MySerial1.write(0x8A);
    MySerial1.flush();
    digitalWrite(ENABLE, LOW);
    waitACK = true;  //we need to wait for response
    //write the incline value to the control board
  }
}

void NextGear(byte shift) {  //get 1,2,4 or 8. shift rear down, shift r up, shift f down, shift f up
  int offset = 0;
  if (shift == 1) {
    offset = -4;
    rearGear--;
    if (rearGear < 0) {
      rearGear = 0;  //can't go lower than 1 or no power will be generated
      offset = 0;    //no shift happens
    }
  }
  if (shift == 2) {
    offset = 4;
    rearGear++;
    if (rearGear > 10) {
      rearGear = 10;  //eleven speed freewheel, 0-10
      offset = 0;
    }
  }
  if (shift == 4) {
    offset = -15;
    frontGear--;
    if (frontGear < 0) {
      frontGear = 0;
      offset = 0;  //already lowest ring, no shift happens
    }
  }  //what is the offset for this really?
  if (shift == 8) {
    offset = 15;
    frontGear++;
    if (frontGear > 2) {
      frontGear = 2;  //three ring chainring
      offset = 0;
    }
  }
  delay(50);                         //do we want a shift delay here?
  gearOffset = gearOffset + offset;  //should we call SetGear directly?
  if (debugging) {
    Serial.print("Gearoffset: ");
    Serial.println(gearOffset);
  }
  SetGear(0);
}

void SetGear(int setting) {
  if (debugging) {
    Serial.print("Old gear: ");
    Serial.println(current_gear);  //the gear hasn't changed yet
  }
  current_gear = setting + gearOffset;
  if (current_gear < 0) { current_gear = 0; }
  if (current_gear > 127) { current_gear = 127; }
  if (debugging) {
    Serial.print("New gear: ");
    Serial.println(current_gear);  //the gear has now changed
  }
}

void UpdateResistance() {
  digitalWrite(ENABLE, HIGH);
  MySerial1.write(0xBA);
  MySerial1.write(0xB6);
  MySerial1.write(0xB1);
  MySerial1.write(0xB0);
  MySerial1.write(0xB6);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xB5);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(gearCommand1[current_gear]);
  MySerial1.write(gearCommand2[current_gear]);
  MySerial1.write(gearCommand3[current_gear]);
  MySerial1.write(gearCommand4[current_gear]);
  MySerial1.write(0x8D);
  MySerial1.write(0x8A);
  MySerial1.flush();
  digitalWrite(ENABLE, LOW);
  waitACK = true;
}


void RequestCadence() {  //do we need to write values to get cadence back?
  if (debugging) { Serial.println("Requesting cadence."); }
  digitalWrite(ENABLE, HIGH);
  MySerial1.write(0xBA);
  MySerial1.write(0xB5);
  MySerial1.write(0xB1);
  MySerial1.write(0xB0);
  MySerial1.write(0xB3);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xB2);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xB0);
  MySerial1.write(0xC1);
  MySerial1.write(0xC1);
  MySerial1.write(0x8D);
  MySerial1.write(0x8A);
  MySerial1.flush();
  digitalWrite(ENABLE, LOW);
  waitACK = true;
}

int CalcPower() {
  //instantaneous_cadence = 80;
  //how to calc power. not linear, depends on gear and cadence
  //instantaneous_power = round(((float)instantaneous_power)*losses); // account for losses, fudge factor...
  //int power = (int)(instantaneous_cadence * (current_gear + 1) / 30);  //very rough estimate for now!!!
  int power = (int)((instantaneous_cadence * current_gear * current_gear / a) + (instantaneous_cadence * c) + d + (current_gear * b));
  if (power < 0) {
    power = 0;
  }
  if (powerDebug) {
    Serial.print("Power: ");
    Serial.print(power);
    Serial.print(" Gear: ");
    Serial.println(current_gear);
  }
  return power;
}

int CalcValue(byte message) {
  int value;
  if (message <= 0xB9) {
    value = message & 0xf;
  } else (value = (message & 0xf) + 10);
  return value;
}

void ReadButton()
{
      bool rearGearDown = digitalRead(RD_B);
      bool rearGearUp = digitalRead(RU_B);
      bool frontGearDown = digitalRead(FD_B);
      bool frontGearUp = digitalRead(FU_B);

      if (rDnBtn.debounce()) {
        if (debugging) { Serial.println("Rear down button pressed."); }
        NextGear(1);
      }
      if (rUpBtn.debounce()) {
        NextGear(2);
      }
      if (fDnBtn.debounce()) {
        NextGear(4);
      }
      if (fUpBtn.debounce()) {
        NextGear(8);
      }
}

void setup() {
  MySerial1.begin(38400, SERIAL_8N1, RXD2, TXD2);  //using serial2 to connect to the proform RS485 bus. Pins defined above
  if (true) {
    Serial.begin(38400);  //USB serial communication
    Serial.println("Setting up serial comm with DPS, bluetooth device and BLE Fitness Machine Service");
  }

  InitializeGears();
  rDnBtn.begin(RD_B);
  rUpBtn.begin(RU_B);
  fDnBtn.begin(FD_B);
  fUpBtn.begin(FU_B);

  pinMode(ENABLE, OUTPUT);
  pinMode(LED, OUTPUT);

  if (!BLE.begin()) {  // Error starting the bluetooth module
    if (debugging) {
      Serial.println("Error initiliazing bluetooth module, please restart");
    }
    while (1)
      ;
  }

  BLE.setDeviceName("Proform Le Tour de France");
  BLE.setLocalName("Proform1");
  BLE.setAdvertisedService(fitnessMachineService);
  fitnessMachineService.addCharacteristic(fitnessMachineFeatureCharacteristic);
  fitnessMachineService.addCharacteristic(indoorBikeDataCharacteristic);
  fitnessMachineService.addCharacteristic(fitnessMachineControlPointCharacteristic);
  BLE.addService(fitnessMachineService);

  fitnessMachineFeatureCharacteristic.writeValue(ftmsFeatureBuf, 4);
  indoorBikeDataCharacteristic.writeValue(indoorBikeDataBuf, 8);

  // start advertising
  BLE.advertise();
  if (debugging) {
    Serial.println("BLE advertisement started");
  }

  if (debugging) {
    Serial.println("Proform bike BLE started.");
  }
}

void loop() {

ReadButton();//we want to read the buttons even if we are not connected to bluetooth app
UpdateResistance();
waitACK = false;

  BLE.poll();

  central = BLE.central();
  if (central) {

    digitalWrite(LED, HIGH);

    if (debugging) {
      Serial.println("Connected to central: ");
      Serial.print("Address: ");
      Serial.println(central.address());
    }

    while (central.connected()) {

      ReadButton(); //read the buttons for gear shifts
      if (MySerial1.available() > 0) {  //serial read rs485
        incoming = MySerial1.read();
        if ((incoming == 0xB5) && (count == 0))  //response or request data
        {
          count = 1;
        }
        count++;
        if (count == 7)  //response from lower board or console request
        {
          if (incoming == 0xB2) { responseData = true; }   //this is response from the lower control board
          if (incoming == 0xB0) { responseData = false; }  //this is a request from the console
        }
        if (count == 9) {
          if (incoming == 0xB1) {  //lower board response
          }
        }
        if ((count == 14) && (responseData == true)) {
          if (debuggingCad) {
            Serial.print(" b14: ");
            Serial.print(incoming, HEX);  //print all the commands received
          }
          byte14 = CalcValue(incoming);
        }
        if ((count == 15) && (responseData == true)) {
          byte15 = CalcValue(incoming);
          if (debuggingCad) {
            Serial.print("b15: ");
            Serial.print(incoming, HEX);  //second cadence
            Serial.print(" Cadence:");
            Serial.print(byte14 * 16 + byte15);
          }
          int old_cadence = instantaneous_cadence;
          instantaneous_cadence = (byte14 * 16 + byte15);
        }
        if ((incoming == 0x8A) && (responseData == true))  //this is the end marker for a command
        {
          count = 0;
          waitACK = false;
        }
      }

      if (fitnessMachineControlPointCharacteristic.written()) {
        fmcpValueLength = fitnessMachineControlPointCharacteristic.valueLength();
        memset(fmcpData.bytes, 0, sizeof(fmcpData.bytes));
        fitnessMachineControlPointCharacteristic.readValue(fmcpData.bytes, fmcpValueLength);
        if (debugging) {
          Serial.println("Control point received");
          //Serial.print("OpCode: ");
          //Serial.println(fmcpData.values.OPCODE, HEX);
          //Serial.println("Values: ");
          //for (int i = 0; i < fmcpValueLength - 1; i++) Serial.println(fmcpData.values.OCTETS[i], HEX);
          //Serial.println();
        }
        ftmsControlPointBuf[0] = fmcpResponseCode;
        ftmsControlPointBuf[1] = fmcpData.values.OPCODE;
        ftmsControlPointBuf[2] = 0x02;  // OpCode not supported for now
        switch (fmcpData.values.OPCODE) {
          case fmcpRequestControl:
            {
              // Always allow control
              if (debugging) {
                Serial.println("Request Control");
              }
              ftmsControlPointBuf[2] = 0x01;  // OpCode supported
              break;
            }
          case fmcpStartOrResume:
            {
              if (debugging) {
                Serial.println("Start the training");
              }
              ftmsControlPointBuf[2] = 0x01;  // OpCode supported
              break;
            }
          case fmcpStopOrPause:
            {
              if (debugging) {
                Serial.println("Stop or pause training");
              }
              break;
            }
          case fmcpSetIndoorBikeSimulationParameters:
            {
              if (debugging) {
                Serial.println("Set indoor bike simulation parameters received");
              }
              int16_t ws = (int16_t)((fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0]);  // Short is 16 bit signed, so the windspeed is converted from two bytes to signed value. Highest bit is sign bit
              wind_speed = ws / 1000.0;
              int16_t gr = (int16_t)((fmcpData.values.OCTETS[3] << 8) + fmcpData.values.OCTETS[2]);  // Short is 16 bit signed, so a negative grade is correctly converted from two bytes to signed value. Highest bit is sign bit
              grade = gr / 100;                                                                      //this grade is in percent.  Close to what we want
              crr = fmcpData.values.OCTETS[4] / 10000.0;
              cw = fmcpData.values.OCTETS[5] / 100.0;
              if (debugging) {  // Remember, if debugging with Zwift, that these values are divided by 2 if in normal settings!
                //Serial.print("Wind speed (1000): ");
                //Serial.println(wind_speed);  // seems to always be zero in Zwift
                Serial.print("Grade (100): ");
                Serial.println(grade);
                //Serial.print("Crr (10000): ");
                //Serial.println(crr);  // seems to always be 0.00 in Zwift
                //Serial.print("Cw (100): ");
                //Serial.println(cw);  // seems to always be 0.51 in Zwift
              }
              // check override flag - currently not working on the PROFORM
              if (OVERRIDE) {
                if (debugging) {
                  Serial.println("OVERRIDE flag is on, not using SIM settings");
                }
              } else {
                //this needs a rewrite to work with the proform//
                if (debugging) {
                  Serial.print("Set sim target power request: ");
                }
              }


              ftmsControlPointBuf[2] = 0x01;
              break;
            }
          case fmcpReset:
            {
              if (debugging) {
                Serial.println("Reset request");
              }
              ftmsControlPointBuf[2] = 0x01;
              break;
            }
          case fmcpSetTargetResistanceLevel: //not working on proform
            {
              if (debugging) {
                Serial.println("Set target resistance level");
              }
              short resistance = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0];
              ftmsControlPointBuf[2] = 0x01;
              break;
            }
          case fmcpSetTargetPower:
            {  //ERG mode.  Not sure how to do -- some PID feedback needed so we don't death spiral
               //this isn't really implemented yet by the proform
              if (debugging) {
                Serial.println("Set target power");
              }
              setWatts = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0];
              //write a target resistance as erg mode

              if (debugging) {
                Serial.print("Set target power request: ");
                Serial.println(setWatts);
              }
              ftmsControlPointBuf[2] = 0x01;
              break;
            }
        }
        fitnessMachineControlPointCharacteristic.writeValue(ftmsControlPointBuf, 3);
      }

      unsigned long ACKMillis = millis();

      if (ACKMillis - oldACKtime > (ACKtime)) {
        if (debugging) { Serial.println("ACK timeout"); }
        waitACK = false;
      }
      resistanceSetting = SetTrainerResistance(wind_speed, grade, crr, cw);  //returns a gear setting
      SetGear(resistanceSetting);
      if (!waitACK) {
        if (current_gear != oldGearSetting) {
          UpdateResistance();
          oldGearSetting = current_gear;
        }
        if (current_incline != last_incline) {
          SetIncline(current_incline);
          last_incline = current_incline;
        }
      }
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis > (interval / 2))  //loop through every 1000ms
      {
        if(!waitACK){
          RequestCadence();
        }
        //build response data to send to computer head unit
        indoorBikeDataBuf[0] = 0x00 | flagInstantaneousCadence | flagIntantaneousPower;  // More Data = 0 (instantaneous speed present), bit 2: instantaneous cadence present
        indoorBikeDataBuf[1] = 0;
        int current_speed = current_gear * instantaneous_cadence / 200;  //made up!! ha!                                
        indoorBikeDataBuf[2] = current_speed & 0xFF;                      // Instantaneous Speed, uint16
        indoorBikeDataBuf[3] = (current_speed >> 8) & 0xFF;
        int doubleCadence = (int)round(instantaneous_cadence * 2);  //cadence sent to zwift always seems to be 1/2. 
        indoorBikeDataBuf[4] = (int)round(doubleCadence) & 0xFF;    // Instantaneous Cadence, uint16
        indoorBikeDataBuf[5] = ((int)round(doubleCadence) >> 8) & 0xFF;
        instantaneous_power = CalcPower();
        indoorBikeDataBuf[6] = (instantaneous_power)&0xFF;  // Instantaneous Power, uint16
        indoorBikeDataBuf[7] = (instantaneous_power >> 8) & 0xFF;
        indoorBikeDataCharacteristic.writeValue(indoorBikeDataBuf, 8);
        previousMillis = currentMillis;
        if (debugging) {
          Serial.print("Bike data written. Power: ");
          Serial.println(instantaneous_power);
        }
      }
    }

    digitalWrite(LED, LOW);

    if (debugging) {
      Serial.println("Disconnected from central... ");
      Serial.print("Address: ");
      Serial.println(central.address());
    }
  }
}
