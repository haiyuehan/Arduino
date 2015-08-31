//Includes (like "import" for Python, but you can't include a soul)
#include <Wire.h>
#include <utility/Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <math.h>

//===================Global definitions=====================

//Module voltage scalers
#define V1_SCALE 1
#define V2_SCALE 1.62         // = 1/(100k/(100k+62k))  and then precision calibrated with DMM and voltrage source
#define V3_SCALE 2.422925086  // = 1/(100k/(100k+140k)) and then precision calibrated with DMM and voltrage source
#define V4_SCALE 3.159857568  // = 1/(100k/(100k+220k)) and then precision calibrated with DMM and voltrage source

//Thermistor constants
#define B 3950
#define Ro 10000
#define Rknown 10000
#define To 25
#define Rmin 3000  //Max temp is actually 65C, which is 2072 ohms. We're using 3000 ohms, which is around 55C
#define Rmax 25580 //Min temp is actually -20C, which is 98880 ohms. We're using 25580 ohms, which is around 5C

//Cell voltage limits
#define Vmin 2.8      //Actual limit is 2.8V
#define Vmax 3.6    //Actual limit is 3.6V
#define VChargeComplete 13.8  //3.45V per cell
#define VCanChargeAgain 13.2  //3.3V per cell

//Cell current limits
#define MaxCurrent 110  //Actual continuous limit is 120A, with pulse limit of 400A

//Pin definitions
#define LED_R_PIN 7
#define LED_O_PIN 6
#define LED_Y_PIN 5
#define LED_G_PIN 4
#define LED_B_PIN 3
#define LED_V_PIN 2

#define V1_BALANCE_PIN 14
#define V2_BALANCE_PIN 15
#define V3_BALANCE_PIN 16
#define V4_BALANCE_PIN 17

#define BATTERY_CONTACTOR_PIN 18
#define ARRAY_CONTACTOR_PIN 19

#define CONTACTOR_DRY_CONTACT_PIN 93

#define ARRAY_CURRENT_PIN A5
#define VOLTAGE_REFERENCE_PIN A6
#define AMBIENT_TEMPERATURE_PIN A7
#define V1_PIN A8
#define V2_PIN A9
#define V3_PIN A10
#define V4_PIN A11
#define THERMISTOR_V12_PIN A12
#define THERMISTOR_V23_PIN A13
#define THERMISTOR_V34_PIN A14
#define BATTERY_CURRENT_PIN A15

#define ONBOARD_LED 13

//LCD defines
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

//Analog input definitions
#define FILTER_ORDER 20
#define VOLTAGE_REFERENCE 512 //512=round(3.000/5*1023) wtf????

//===================Global declarations=====================

//Inputs
float ModuleTemp[3] = {0,0,0};
float ModuleTempR[3] = {0,0,0};
float ModuleVoltage[4] = {0,0,0,0};
float ArrayCurrent = 0;
float PackCurrent = 0;
float PackVoltage = 0;
float AmbientTemp = 0;

//Charging
bool ChargeComplete = false;

//Input data
int ModuleTempRaw[3] = {0,0,0};
int ModuleVoltageRaw[4] = {0,0,0,0};
int SolarCurrentRaw = 0;
int BatteryCurrentRaw = 0;
int ReferenceVoltageRaw = 0;
int AmbientTempRaw = 0;

//Input filter arrays
int ModuleTempArray[3][FILTER_ORDER];
int ModuleVoltageArray[4][FILTER_ORDER];
int SolarCurrentArray[FILTER_ORDER];
int BatteryCurrentArray[FILTER_ORDER];
int ReferenceVoltageArray[FILTER_ORDER];
int AmbientTempArray[FILTER_ORDER];

//Filtered input data
int ModuleTempFiltered[3] = {0,0,0};
int ModuleVoltageFiltered[4] = {0,0,0,0};
int BatteryCurrentFiltered = 0;
int SolarCurrentFiltered = 0;
int ReferenceVoltageFiltered = 0;
int AmbientTempFiltered = 0;

//Initial boot
bool isBootComplete = 0;
int numberOfIterations = 0;

//Quiescient current sensor calibrations
float battCurrentQuiescient = 2.5;
float solarCurrentQuiescient = 0.5;

//LCD
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

//===================FUNCTIONS: Read raw data======================
void readADCs(){
  ModuleTempRaw[0] = analogRead(THERMISTOR_V12_PIN);
  ModuleTempRaw[1] = analogRead(THERMISTOR_V23_PIN);
  ModuleTempRaw[2] = analogRead(THERMISTOR_V34_PIN);
  
  ModuleVoltageRaw[0] = analogRead(V1_PIN);
  ModuleVoltageRaw[1] = analogRead(V2_PIN);
  ModuleVoltageRaw[2] = analogRead(V3_PIN);
  ModuleVoltageRaw[3] = analogRead(V4_PIN);
  
  SolarCurrentRaw = analogRead(ARRAY_CURRENT_PIN);
  BatteryCurrentRaw = analogRead(BATTERY_CURRENT_PIN);
  
  ReferenceVoltageRaw = analogRead(VOLTAGE_REFERENCE_PIN); // Should correlate to 2.5 volts and should use TMP to account for temperature
  AmbientTempRaw = analogRead(AMBIENT_TEMPERATURE_PIN); 
}

void initializeAnalogFilters(){
  for (int x = 0; x < (FILTER_ORDER-1); x++){
    ModuleTempArray[0][x] = ModuleTempRaw[0];
    ModuleTempArray[1][x] = ModuleTempRaw[1];
    ModuleTempArray[2][x] = ModuleTempRaw[2];

    ModuleVoltageArray[0][x] = ModuleVoltageRaw[0];
    ModuleVoltageArray[1][x] = ModuleVoltageRaw[1];
    ModuleVoltageArray[2][x] = ModuleVoltageRaw[2];
    ModuleVoltageArray[3][x] = ModuleVoltageRaw[3];

    SolarCurrentArray[x] = SolarCurrentRaw;
    BatteryCurrentArray[x] = BatteryCurrentRaw;
    ReferenceVoltageArray[x] = ReferenceVoltageRaw;
    AmbientTempArray[x] = AmbientTempRaw;
  }
}

void CalculateAnalogValues(){
  //Declarations
  float AnalogScalingFactor = 0;
  float Rthermistor = 0;
  float temp = 0;
  
  //Calculates the scaling factor - multiply ADC value by this number to get voltage reference corrected voltage
  AnalogScalingFactor = (float)((float)5/1024)*((float)VOLTAGE_REFERENCE/ReferenceVoltageFiltered);
  
  //Calculates the module voltages 
  ModuleVoltage[0] = (float)ModuleVoltageFiltered[0] * AnalogScalingFactor * V1_SCALE;
  ModuleVoltage[1] = (float)ModuleVoltageFiltered[1] * AnalogScalingFactor * V2_SCALE - (float)ModuleVoltageFiltered[0] * AnalogScalingFactor * V1_SCALE;
  ModuleVoltage[2] = (float)ModuleVoltageFiltered[2] * AnalogScalingFactor * V3_SCALE - (float)ModuleVoltageFiltered[1] * AnalogScalingFactor * V2_SCALE;
  ModuleVoltage[3] = (float)ModuleVoltageFiltered[3] * AnalogScalingFactor * V4_SCALE - (float)ModuleVoltageFiltered[2] * AnalogScalingFactor * V3_SCALE;

  //Pack voltage
  PackVoltage = (float)ModuleVoltageFiltered[3] * AnalogScalingFactor * V4_SCALE;
  
  //Calculates the module temperatures
  ModuleTempR[0] = (((float)ModuleTempFiltered[0]*AnalogScalingFactor)*10000)/(5-(float)ModuleTempFiltered[0]*AnalogScalingFactor);
  ModuleTempR[1] = (((float)ModuleTempFiltered[1]*AnalogScalingFactor)*10000)/(5-(float)ModuleTempFiltered[1]*AnalogScalingFactor);
  ModuleTempR[2] = (((float)ModuleTempFiltered[2]*AnalogScalingFactor)*10000)/(5-(float)ModuleTempFiltered[2]*AnalogScalingFactor);
  //temp = B/log(Rthermistor/(Ro*exp(-1*B/To)));
  
  //Calculates the battery current
  PackCurrent = (float)80*((float)BatteryCurrentFiltered * AnalogScalingFactor - battCurrentQuiescient);
  
  //Calculates the array current
  ArrayCurrent = ((float)20/4.5)*((float)SolarCurrentFiltered * AnalogScalingFactor - solarCurrentQuiescient);
 
}

bool getMainContactorDryContactState(){
  
}

void getPushButtonState(){
}

// Runs the filter for the analog inputs
void runADCFilters(){
  //Declarations
  int x = 0;
  long ModuleTempSum[3] = {0,0,0};
  long ModuleVoltageSum[4] = {0,0,0,0};
  long SolarCurrentSum = 0;
  long BatteryCurrentSum = 0;
  long ReferenceVoltageSum = 0;
  long AmbientTempSum = 0;
  
  //Shift all values by 1
  for (x = (FILTER_ORDER-1); x == 0; x--){
    ModuleTempArray[0][x+1] = ModuleTempArray[0][x];
    ModuleTempArray[1][x+1] = ModuleTempArray[1][x];
    ModuleTempArray[2][x+1] = ModuleTempArray[2][x];

    ModuleVoltageArray[0][x+1] = ModuleVoltageArray[0][x];
    ModuleVoltageArray[1][x+1] = ModuleVoltageArray[1][x];
    ModuleVoltageArray[2][x+1] = ModuleVoltageArray[2][x];
    ModuleVoltageArray[3][x+1] = ModuleVoltageArray[3][x];

    SolarCurrentArray[x+1] = SolarCurrentArray[x];
    BatteryCurrentArray[x+1] = BatteryCurrentArray[x];
    
    ReferenceVoltageArray[x+1] = ReferenceVoltageArray[x];
    AmbientTempArray[x+1] = AmbientTempArray[x];
  }

  //Replace the beginning of the array with the current value
  ModuleTempArray[0][0] = ModuleTempRaw[0];
  ModuleTempArray[1][0] = ModuleTempRaw[1];
  ModuleTempArray[2][0] = ModuleTempRaw[2];
  
  ModuleVoltageArray[0][0] = ModuleVoltageRaw[0];
  ModuleVoltageArray[1][0] = ModuleVoltageRaw[1];
  ModuleVoltageArray[2][0] = ModuleVoltageRaw[2];
  ModuleVoltageArray[3][0] = ModuleVoltageRaw[3];
 
  SolarCurrentArray[0] = SolarCurrentRaw;
  BatteryCurrentArray[0] = BatteryCurrentRaw;
  
  ReferenceVoltageArray[0] = ReferenceVoltageRaw;
  AmbientTempArray[0] = AmbientTempRaw;

  //Performs addition
  for (x = 0; x < (FILTER_ORDER); x++){
    ModuleTempSum[0] = ModuleTempSum[0] + ModuleTempArray[0][x];
    ModuleTempSum[1] = ModuleTempSum[1] + ModuleTempArray[1][x];
    ModuleTempSum[2] = ModuleTempSum[2] + ModuleTempArray[2][x];

    ModuleVoltageSum[0] = ModuleVoltageSum[0] + ModuleVoltageArray[0][x];
    ModuleVoltageSum[1] = ModuleVoltageSum[1] + ModuleVoltageArray[1][x];
    ModuleVoltageSum[2] = ModuleVoltageSum[2] + ModuleVoltageArray[2][x];
    ModuleVoltageSum[3] = ModuleVoltageSum[3] + ModuleVoltageArray[3][x];

    SolarCurrentSum = SolarCurrentSum + SolarCurrentArray[x];
    BatteryCurrentSum = BatteryCurrentSum + BatteryCurrentArray[x];
    
    ReferenceVoltageSum = ReferenceVoltageSum + ReferenceVoltageArray[x];
    AmbientTempSum = AmbientTempSum + AmbientTempArray[x];
  }

  //Performs averaging
  ModuleTempFiltered[0] = (int)(ModuleTempSum[0] / FILTER_ORDER);
  ModuleTempFiltered[1] = (int)(ModuleTempSum[1] / FILTER_ORDER);
  ModuleTempFiltered[2] = (int)(ModuleTempSum[2] / FILTER_ORDER);
  
  ModuleVoltageFiltered[0] = (int)(ModuleVoltageSum[0] / FILTER_ORDER);
  ModuleVoltageFiltered[1] = (int)(ModuleVoltageSum[1] / FILTER_ORDER);
  ModuleVoltageFiltered[2] = (int)(ModuleVoltageSum[2] / FILTER_ORDER);
  ModuleVoltageFiltered[3] = (int)(ModuleVoltageSum[3] / FILTER_ORDER);

  SolarCurrentFiltered = (int)(SolarCurrentSum / FILTER_ORDER);
  BatteryCurrentFiltered = (int)(BatteryCurrentSum / FILTER_ORDER);
  
  ReferenceVoltageFiltered = (int)(ReferenceVoltageSum / FILTER_ORDER);
  AmbientTempFiltered = (int)(AmbientTempSum / FILTER_ORDER);
}
//===================FUNCTIONS: BMS checks======================

bool isVoltBad(){
  bool badVoltage = false;
  for (int x = 0; x < 4; x++){
    if ((ModuleVoltage[x] > Vmax) || (ModuleVoltage[x] < Vmin)){
      Serial.println(x);
      Serial.println(ModuleVoltage[x]);
      badVoltage = true;
    }
  }
  return badVoltage;
}

bool isCurrentBad(){
  if ((PackCurrent > MaxCurrent) || (PackCurrent < -1*MaxCurrent)){
    return true;
  }
  else{
    return false;
  }
}

bool isTempBad(){
  bool badTemp = false;
  for (int x = 0; x < 3; x++){
    if ((ModuleTempR[x] > Rmax) || (ModuleTempR[x] < Rmin)){
      badTemp = true;
    }
  }
  return badTemp; 
}

bool isContactorFused(){
}

bool ChargeBalanceNeeded(){
}

bool isChargeComplete(){
  if (PackVoltage > VChargeComplete){
    return true;
  }
  else {
    return false;
  }
}

bool canChargeAgain(){
  if (PackVoltage < VCanChargeAgain){
    return true;
  }
  else {
    return false;
  }
}

bool isPackBricked(){
}

bool isLowSOC(){
}

//===================FUNCTIONS: SOC calculation===================

float getSOC(){
  
}

float updateSocOcv(){
}

float updateSocCurrent(float currentSOC){
}

//===================FUNCTIONS: Contactor =======================

void MainContactor(bool Close){
  digitalWrite(BATTERY_CONTACTOR_PIN, Close);
}

void ArrayContactor(bool Close){
  digitalWrite(ARRAY_CONTACTOR_PIN, Close);
}

bool isMainContactorClosed(){
}

bool isArrayContactorClosed(){
}

//===================FUNCTIONS: Display =======================
void DisplayPage(int PageNumber){
}

void InitLCD (){
  //Set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  
  //Sets the LCD's backlight to green
  lcd.setBacklight(WHITE);
}

//========================TIMER 1  ===========================
//Hardware timer to run code (no use of "delay" code)
//Instructions at: http://www.instructables.com/id/Arduino-Timer-Interrupts/?ALLSTEPS
//============================================================

//Set Timer1 interrupt at 10Hz 
void initTimer(){
  //**Hardware timer speed = 16MHz (clock rate) / Prescaler / Compare Match Register
   
  //Stop interrupts - must do this before configuring timers
  cli();

  //Clears Timer 1 control registers
  TCCR1A = 0;
  TCCR1B = 0;

  //Initialize counter value to 0
  TCNT1  = 0;
  
  //Set compare match register for 10Hz increments
  OCR1A = 1562;   // =(16*10^6) / (1024*Timer_Speed) - 1 (must be <65535)
  
  // Turn on CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);
  
  // Set CS10 and CS12 bits for 1024 prescaler (refer to table at http://www.instructables.com/file/F3TTIKLH3WSA4V7)
  TCCR1B |= (1 << CS12) | (1 << CS10);
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //Allow interrupts - must enable to allow timers to work
  sei();
}


//****************************************************
//This function runs forever at 10Hz, and after setup()
//Put main code here to execute repeatedly at 10Hz
//****************************************************
ISR(TIMER1_COMPA_vect){
  //Reads values from the ADCs
  readADCs();

  //Runs filters to filter data from ADCs
  runADCFilters();

  //Caculate analog values
  CalculateAnalogValues();

  //Checks for initial startup
  if (numberOfIterations > (FILTER_ORDER*2)){
    //Checks for bad voltage
    if (isVoltBad() == true){
      Serial.println("Bad Voltage!!!!");
      MainContactor(false);
    }
  
    //Checks for bad current
    if (isCurrentBad() == true){
      Serial.println("Bad Current!!!!");
      MainContactor(false);
    }
  
    //Checks for bad temperature
    if (isTempBad() == true){
      Serial.println("Bad Temperature!!!!");
      MainContactor(false);
    }
  
    //Checks for charge complete
    if (ChargeComplete == false){
      if (isChargeComplete()){
        ChargeComplete = true;
        ArrayContactor(false);
      }
    }
    else{
      if (canChargeAgain){
        ChargeComplete = false;
        ArrayContactor(true);
      }
    }
  }
  else{
    //Iterate iternations
    numberOfIterations++;
  }

  //DEBUG
  Serial.print("V1: ");
  Serial.print(ModuleVoltage[0],4);
  
  Serial.print(" V2: ");
  Serial.print(ModuleVoltage[1],4);
  
  Serial.print(" V3: ");
  Serial.print(ModuleVoltage[2],4);
  
  Serial.print(" V4: ");
  Serial.print(ModuleVoltage[3],4);

  Serial.print(" T1: ");
  Serial.print(ModuleTempR[0]);
  
  Serial.print(" T2: ");
  Serial.print(ModuleTempR[1]);
  
  Serial.print(" T3: ");
  Serial.print(ModuleTempR[2]);
  
  Serial.print(" B: ");
  Serial.print(PackCurrent);

  Serial.print(" BR: ");
  Serial.print(PackCurrent);

  Serial.print(" S: ");
  Serial.println(ArrayCurrent);
}

//================Functions: Setup==================
void initIO(){
  //LED
  pinMode(ONBOARD_LED, OUTPUT);

  //Contactors
  pinMode(BATTERY_CONTACTOR_PIN, OUTPUT);
  pinMode(ARRAY_CONTACTOR_PIN, OUTPUT);
}

//This function runs at startup
void setup() {
  //Initializes all our I/O
  initIO();
  
  //Initializes the LCD
  InitLCD();

  //Closes all contactors
  MainContactor(true);
  ArrayContactor(true);

  //Initializes the ADC filters
  initializeAnalogFilters();

  //Initalizes debug serial communication
  Serial.begin(9600);

  //Initializes and enables Timer1, which will run
  //all our code at 10Hz
  initTimer();  
}

//==================MAIN LOOP=======================
//This function runs forever, and after setup()
//==================================================
void loop() {
  // We really don't want to use this function due 
  // to poor timing - no code here
}
