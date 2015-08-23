//Includes (like "import" for Python, but you can't include a soul)


//Global declarations

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

//Analog input definitions
#define FILTER_ORDER 4
#define VOLTAGE_REFERENCE 614 //614=round(3.000/5*1023)

//Inputs
float ModuleTemp[3];
float ModuleVoltage[4];
float ArrayCurrent = 0;
float PackCurrent = 0;

//Input data
int ModuleTempRaw[3];
int ModuleVoltageRaw[4];
int SolarCurrentRaw = 0;
int BatteryCurrentRaw = 0;
int ReferenceVoltageRaw = 0;

//Input filter arrays
int ModuleTempArray[3][FILTER_ORDER];
int ModuleVoltageArray[4][FILTER_ORDER];
int SolarCurrentArray[FILTER_ORDER];
int BatteryCurrentArray[FILTER_ORDER];
int ReferenceVoltageArray[FILTER_ORDER];

//Filtered input data
int ModuleTempFiltered[3];
int ModuleVoltageFiltered[4];
int BatteryCurrentFiltered = 0;
int SolarCurrentFiltered = 0;
int ReferenceVoltageFiltered = 0;

//===================FUNCTIONS: Read raw data======================
void readADCs(){
  ModuleTempRaw[0] = analogRead(THERMISTOR_V12_PIN);
  ModuleTempRaw[1] = analogRead(THERMISTOR_V23_PIN);
  ModuleTempRaw[3] = analogRead(THERMISTOR_V34_PIN);
  
  ModuleVoltageRaw[0] = analogRead(V1_PIN);
  ModuleVoltageRaw[1] = analogRead(V2_PIN);
  ModuleVoltageRaw[2] = analogRead(V3_PIN);
  ModuleVoltageRaw[4] = analogRead(V4_PIN);
  
  SolarCurrentRaw = analogRead(ARRAY_CURRENT_PIN);
  BatteryCurrentRaw = analogRead(BATTERY_CURRENT_PIN);
  
  ReferenceVoltageRaw = analogRead(VOLTAGE_REFERENCE_PIN);
}

void CalculateAnalogValues(){
  //Declarations
  float AnalogScalingFactor = 0;
  
  //Calculates the scaling factor - multiply ADC value by this number to get voltage reference corrected voltage
  AnalogScalingFactor = (5/1024)*(VOLTAGE_REFERENCE/ReferenceVoltageFiltered);
  
  //Calculates the module voltages
  
  //Calculates the module temperatures
  
  //Calculates the battery current
  
  //Calculates the array current
}

bool getMainContactorDryContactState(){
  
}

void getPushButtonState(){
}

// Runs the filter for the analog inputs
void runInputFilters(){
  //Declarations
  int x = 0;
  long ModuleTempSum[3];
  long ModuleVoltageSum[4];
  long SolarCurrentSum = 0;
  long BatteryCurrentSum = 0;
  long ReferenceVoltageSum = 0;
  
  //Shift all values by 1
  for (x = 0; x < (FILTER_ORDER-1); x++){
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
}
//===================FUNCTIONS: BMS checks======================

bool isVoltBad(){
}

bool isCurrentBad(){
  
}

bool isTempBad(){
}

bool isContactorFused(){
}

bool ChargeBalanceNeeded(){
}

bool isChargeComplete(){
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
}

void ArrayContactor(bool Close){
}

bool isMainContactorClosed(){
}

bool isArrayContactorClosed(){
}

//===================FUNCTIONS: Display =======================
void DisplayPage(int PageNumber){
}

//====================SETUP=========================
//This function runs at startup
//==================================================
void setup() {
  // put your setup code here, to run once:

}


//==================MAIN LOOP=======================
//This function runs forever, and after setup()
//==================================================
void loop() {
  // put your main code here, to run repeatedly:

}
