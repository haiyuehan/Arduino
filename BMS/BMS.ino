//Includes (like "import" for Python, but you can't include a soul)


//Global declarations

//Filter order
#define FILTER_ORDER 4

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
