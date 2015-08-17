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
#define THERMISTOR_V12 A12
#define THERMISTOR_V23 A13
#define THERMISTOR_V34 A14
#define BATTERY_CURRENT_PIN A15

#define ONBOARD_LED 13

//Filter order
#define FILTER_ORDER       4

//Inputs
float ModuleTemp[3];
float ModuleVoltage[4];
float ArrayCurrent = 0;
float PackCurrent = 0;

//Input filter arrays
int ModuleTempRaw[3][FILTER_ORDER];
int ModuleVoltage[4][FILTER_ORDER];
int ArrayCurrent[FILTER_ORDER]


//===================FUNCTIONS: Read raw data======================
void updateTemp(){
  
}

void updateVolts(){
}

void updateCurrent(){
}

bool getMainContactorDryContactState(){
  
}

void getPushButtonState(){
}

void runInputFilters(){

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
