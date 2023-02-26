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
boolean debuggingERG = false;

boolean gearChanged = false;     //track gear changes
boolean setTargetPower = false;  //ERG power is active
boolean adjustBikeTilt = false;  //turns off the physical tilt setting of the bike

#define FU_B 8    //front gears shift up button, front chainring
#define FD_B 20   //front gears shift down button
#define RU_B 21   //rear gears shift up button, rear cassette
#define RD_B 7    //rear gears shift down button
#define ENABLE 5  //write enable for RS485
#define LED 9     //led attached to this pin to indicate BLE connection
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

const uint16_t flagMoreData = 1;              //0000 0001
const uint16_t flagAverageSpeed = 2;          //0000 0010
const uint16_t flagInstantaneousCadence = 4;  //0000 0100
const uint16_t flagAverageCadence = 8;        //0000 1000
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

//variables for calculating resistance.  Zwift only uses grade but other programs (not tested) might(?) use others
float wind_speed = 0;  // meters per second, resolution 0.001
float grade = 0;       // percentage, resolution 0.01
float oldGrade = 0;
float crr = 0;  // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;   // Wind resistance Kg/m, resolution 0.01;

unsigned int instantaneous_cadence = 0;  // Global variable to hold the calculated cadence in RPM
unsigned int instantaneous_power = 0;  // Global variable to hold the calculated power in watts
unsigned int instantaneous_speed = 0;  // Global variable to hold the calculated speed in m/s
unsigned int baseResistance = 0;
unsigned int currentResistance = 0;

//constants for power equation
float a = 1900;
float b = -0.3;
float c = 0.5;
float d = 0;

unsigned int resistanceSetting = 0;
unsigned int oldResistance = 0;
unsigned int oldGear = 0;
unsigned int setWatts = 0;
boolean OVERRIDE = false;

unsigned long previousMillis = 0UL;
unsigned long interval = 500UL; //update interval for BLE transmission and cadence request

unsigned long oldACKtime = 0UL;
unsigned long ACKtime = 30UL;  //for the timeout of the ACK - we read the ACK but don't act, just wait for it to pass so we don't collide.
unsigned long ergMillis = 0UL;
unsigned long ergInterval = 50UL;  //50ms cooldown for ERG so we don't spiral out of control and don't collide with other commands
int oldResistanceERG = 0;
bool newCadence = false;

int currentIncline = 30;  //(levels from -15 to +15 by half increments. Index for zero is 30)
int lastIncline = 30;
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
bool cadenceData = false;
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
  currentIncline = (int)(30 + (int)(grade * 2));  //set the incline of the trainer
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
  gearOffset = gearOffset + offset;
  gearChanged = true;
  SetResistance(resistanceSetting);  //now update the resistance setting
}

void SetResistance(int setting) {
  //Serial.print("Setting: ");
  //Serial.println(setting);
  currentResistance = setting + gearOffset;
  if (currentResistance < 0) { currentResistance = 0; }
  if (currentResistance > 127) { currentResistance = 127; }
}

void ChangeResistance() {
  //Serial.println("ChgResist");
  gearChanged = false;
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
  MySerial1.write(gearCommand1[currentResistance]);
  MySerial1.write(gearCommand2[currentResistance]);
  MySerial1.write(gearCommand3[currentResistance]);
  MySerial1.write(gearCommand4[currentResistance]);
  MySerial1.write(0x8D);
  MySerial1.write(0x8A);
  MySerial1.flush();
  digitalWrite(ENABLE, LOW);
  waitACK = true;
}

void RequestCadence() {  //write values to get cadence back as a response from lower control board
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
  //should we add 1 to the currentResistance?  Because the lowest gear is 0 index and power was measured assuming it was gear 1
  int power = (int)((instantaneous_cadence * currentResistance * currentResistance / a) + (instantaneous_cadence * c) + d + (currentResistance * b));
  if (power < 0) {
    power = 0;
  }
  if (powerDebug) {
    Serial.print("Pow: ");
    Serial.print(power);
    Serial.print(" Gr: ");
    Serial.println(currentResistance);
  }
  return power;
}

int CalcPowerTheoretical(int _change) {
  int _resistance = _change;
  int _power = (int)((instantaneous_cadence * _resistance * _resistance / a) + (instantaneous_cadence * c) + d + (_resistance * b));
  if (_power < 0) {
    _power = 0;
  }
  return _power;
}

int CalcValue(byte message) {
  int value;
  if (message <= 0xB9) {
    value = message & 0xf;
  } else (value = (message & 0xf) + 10);
  return value;
}

void ReadButton() {
  bool rearGearDown = digitalRead(RD_B);
  bool rearGearUp = digitalRead(RU_B);
  bool frontGearDown = digitalRead(FD_B);
  bool frontGearUp = digitalRead(FU_B);

  if (rDnBtn.debounce()) {
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
}

void loop() {

  ReadButton();                             //we want to read the buttons even if we are not connected to bluetooth app
  if (gearChanged) { ChangeResistance(); }  //check gear has changed before updating

  waitACK = false;  //in the non-BLE loop we don't ask for cadence so collisions from ACK don't matter?

  BLE.poll();

  central = BLE.central();
  if (central) {  //main BLE loop - now connected to BLE

    digitalWrite(LED, HIGH);  //status light for BLE connection.  More useful than I initially thought

    if (debugging) {
      Serial.println("Connected to central: ");
      Serial.print("Address: ");
      Serial.println(central.address());
    }

    while (central.connected()) {  //main loop for the BLE connection

      ReadButton();  //read the buttons for gear shifts

      if (MySerial1.available() > 0) {  //serial read rs485
        incoming = MySerial1.read();
        if ((incoming == 0xBA))  //response to resistance or cadence or incline
        {
          count = 0;
        }
        //the second byte send would be: 0xB4 is incline, 0xB5 is cadence, 0xB6 is resistance.  Don't really need to read it?

        if (count == 6)  //7th byte: response from lower board or console request
        {
          if (incoming == 0xB2) { responseData = true; }   //this is response from the lower control board
          if (incoming == 0xB0) { responseData = false; }  //this is a request from the console
          if (incoming == 0xB1) { responseData = true; }  //this is a response from the lower board, ACK
        }
        if (count == 8) {
          if (incoming == 0xB1) {  //lower board response for cadence
            cadenceData = true;
          }
          if (incoming != 0xB1) { //lower board incline or resistance ACK
            cadenceData = false;
          }
        }
        if ((count == 13) && (responseData == true) && (cadenceData == true)) { //first cadence value
          byte14 = CalcValue(incoming);
        }
        if ((count == 14) && (responseData == true) && (cadenceData == true)) {
          byte15 = CalcValue(incoming);
          instantaneous_cadence = (byte14 * 16 + byte15);
        }
        count++;

        if ((incoming == 0x8A))  //this is the end marker for a command.  always, 8a doesn't otherwise appear
        {
          MySerial1.flush();  //flush remaining bytes
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
                Serial.println("Request Control");  //at start of connection
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
                if (debuggingERG) {
                  Serial.println("OVERRIDE flag is on, not using SIM settings");
                }
              } else {  //this seems to trigger when erg mode disabled by zwift
                //this needs a rewrite to work with the proform//
                if (debuggingERG) {
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
          case fmcpSetTargetResistanceLevel:  //not working on proform, zwift doesn't seem to use this
            {
              if (debuggingERG) {
                Serial.println("Set target resistance level");
              }
              //short resistance = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0];
              //ftmsControlPointBuf[2] = 0x01;
              break;
            }
          case fmcpSetTargetPower:
            {  //ERG mode.  
              setWatts = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0];  //get requested power from Zwift
              setTargetPower = true;                                                    //got a target power, now need to try to achieve it
              ftmsControlPointBuf[2] = 0x01;
              break;
            }
        }
        fitnessMachineControlPointCharacteristic.writeValue(ftmsControlPointBuf, 3);
      }

      unsigned long ACKMillis = millis();
      if (ACKMillis - oldACKtime > (ACKtime)) //in case our command is never acknowledged
      {
        waitACK = false;
      }

      if (!waitACK) {  //send commands, update resistance, incline, gears if not waiting for ACK.

        //not ERG mode so get resistance from grade
        if (!setTargetPower) {
          bool changeResist = false;
          resistanceSetting = SetTrainerResistance(wind_speed, grade, crr, cw);  //returns a gear setting
          if (oldResistance != resistanceSetting) {                              //check it is changed before we do anything
            SetResistance(resistanceSetting);
            oldResistance = resistanceSetting;
            changeResist = true;
          }
          if (gearOffset != oldGear) {
            oldGear = gearOffset;
            changeResist = true;
          }
          if (currentIncline != lastIncline) {
            SetIncline(currentIncline);
            lastIncline = currentIncline;
          }
          if (changeResist) {
            ChangeResistance();
          }
        }

        /*************************************************************************/
        //attempt to set ERG power
        /*************************************************************************/

        unsigned long currentMillis = millis();
        if (currentMillis - ergMillis > ergInterval) {//collisions in RS485 are a killer and lockup the program.  So try to avoid them
          if ((setTargetPower) && (instantaneous_cadence > 40) && (newCadence)) {
            newCadence = false;

            int resistanceChange = 0;
            int diff = 1000;
            for (int i = 0; i < 128; i++) {             //try all the different resistance levels to find the closest to target watts
              int tempPower = CalcPowerTheoretical(i);  //send change in power level to calculate a new power if we increased the current resistance level
              int calcDiff = tempPower - setWatts;
              int absDiff = abs(calcDiff);
              if (absDiff < diff) {
                diff = absDiff;
                resistanceChange = i;  //eventually get the best level for target watts at current cadence
              }
            }

            if(oldResistanceERG != resistanceChange) //update resistance only if resistance has changed.  
            {
              oldResistanceERG = resistanceChange;
              SetResistance(resistanceChange);
              ChangeResistance();  //update resistance
            }
          }
        }
      }

      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis > (interval))  //loop through every 500ms
      {
        previousMillis = currentMillis;
        if (!waitACK) {
          RequestCadence();   //updates cadence and triggers a wait for ACK
          newCadence = true;  //new cadence data available
        }
        //build response data to send to computer head unit
        indoorBikeDataBuf[0] = 0x00 | flagInstantaneousCadence | flagIntantaneousPower;  // More Data = 0 (instantaneous speed present), bit 2: instantaneous cadence present
        indoorBikeDataBuf[1] = 0;
        int current_speed = resistanceSetting * instantaneous_cadence / 200;  //made up!! ha!
        indoorBikeDataBuf[2] = current_speed & 0xFF;                          // Instantaneous Speed, uint16
        indoorBikeDataBuf[3] = (current_speed >> 8) & 0xFF;
        int doubleCadence = (int)round(instantaneous_cadence * 2);  //cadence sent to zwift always seems to be 1/2.
        indoorBikeDataBuf[4] = (int)round(doubleCadence) & 0xFF;    // Instantaneous Cadence, uint16
        indoorBikeDataBuf[5] = ((int)round(doubleCadence) >> 8) & 0xFF;
        instantaneous_power = CalcPower();
        indoorBikeDataBuf[6] = (instantaneous_power)&0xFF;  // Instantaneous Power, uint16
        indoorBikeDataBuf[7] = (instantaneous_power >> 8) & 0xFF;
        Serial.println("Wrt buf");
        indoorBikeDataCharacteristic.writeValue(indoorBikeDataBuf, 8);
        if (debugging) {
          Serial.print("C: ");
          Serial.print(instantaneous_cadence);
          Serial.print("  P: ");
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
