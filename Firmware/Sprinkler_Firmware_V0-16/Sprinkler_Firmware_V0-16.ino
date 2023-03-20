// Firmware for Lawn-Sprinkler "Sprouter" by DB3JHF in 02/2023
// Hardware: ESP32 Board (almost any will do), genuine PS3-Controller (Bluetooth) - most likely: clones will not work
// Software: Compilation tested with Arduino IDE V.1.8.19, ESP32 Board-Driver V.1.0.4, libs -> see include section
//
// History of change:
// -----------------------------------------------------------------------------------------
// Version 0.16 : Basic functionalities to set the servos to the required position, PS3 controller support
// Future Features (not implemented yet):  Hard- and Software for Battery voltage monitoring
//
//
// -----------------------------------------------------------------------------------------
//
// CREDITS: The making of this Firmware was inspired by the work and examples of following programmers:
// -> Ujwal Nandanwar https://github.com/un0038998/Robot_Arm_PS3_Controller
// -> Jeffrey van Pernis https://github.com/jvpernis/esp32-ps3
// -> ESP32-Servo https://github.com/madhephaestus/ESP32ServoServer
// -> SPIFFS Filesystem for Arduino IDE https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/

#include <Wire.h>
#include <ESP32Servo.h>    // V 0.12.1
#include <Ps3Controller.h> // V 1.0.0
#include "SPIFFS.h"


unsigned long myTimerAZ = 0;
unsigned long myTimerEL = 0;
unsigned long myTimerW = 0;
const long intervalAZ = 30;  // Delay for Servo movement
const long intervalEL = 30;  // Delay for Servo movement
const long intervalW = 10;   // Delay for Servo movement

Servo servo1;
Servo servo2;
Servo servo3;

int minUs = 500;
int maxUs = 2500;

int servo1Pin = 5;    // AZ
int servo2Pin = 17;   // EL
int servo3Pin = 16;   // W

// -------------------------------------------------------------
// User defined parameters here:

const long sprinkleTime = 10000;   // Time in milliseconds for watering one sector (1s = 1000ms), Standard value = 10000 ms
const long intervalELw = 1000;     // Elevation delay in milliseconds for auto-watering (1s = 1000ms)

//------------------ Configuration of PS3 elements ----------

int player = 1;
int battery = 0;

//------------------ Configuration of device specific variables ----------

int limMinAZ = 1;   // will cover full 360° circle by 2°steps, should be 0° by book but 0 is causing problems when saving
int limMaxAZ = 181; // will cover full 360° circle by 2°steps, should be 0° by book but is shifted +1 because 0° isn't used.

int limMinEL = 0;   // (0°)
int limMaxEL = 90;  // (90°)

int limMinW = 5;    // (0° to 5°)
int limMaxW = 85;   // (85° to 90°)

int centerAZ = ((limMinAZ - limMaxAZ) *(-1));
int centerEL = ((limMinEL - limMaxEL) *(-1));
int centerW = ((limMinW - limMaxW) *(-1));

int posAZ = 0;  // Servo position AZ
int posEL = 0;  // Servo position EL
int posW  = 0;  // Servo position W

int  saveAZ = 0;    // placeholder for preferences
int  saveELup = 0;  // placeholder for preferences
int  saveELdn = 0;  // placeholder for preferences
int  saveW = 0;     // placeholder for preferences



int ps3AZ = 0; // Buffer controller position readout
int ps3EL = 0; // Buffer controller position readout
int ps3W = 0;  // Buffer controller position readout

int progStage = 1;  // Flag for stage of programming  1=Start programming, 2=AZ, 2=ELup, 3=ELdn, 4=Save
int autoStop = 0;  // Flag for start/stop automatic watering 0=run 1=stop

int progMode = 0; // Flag for Programming Mode
int manMode = 0;  // Flag for manual control
int standbyMode = 1; // Flag for auto mode (maybe not neccessary)
int incDecAZ = 0; // FLag for direction change AZ
int incDecEL = 0; // Flag for direction change AZ
int incDecW = 0;  // Flag for direction change Water-Valve

int buttonUp = 0; // Flag for Button down/up
int buttonStart = 0; // Flag for START button
int buttonDStart = 0; // Flag for button "Select"
int twist = 0;  // storage for exchanging stored EL values
unsigned long sprinkleMillis = 0;
unsigned long currentMillis = 0;
unsigned long timeLeft = 0;
int sprinkleDirection = 0; // Flag for changing EL direction

int newELup = 0;  //for reading saved data
int newELdn = 0;  //for reading saved data
int newW = 0;     //for reading saved data

int array_ELup [182];
int array_ELdn [182];
int array_W    [182];


static const int servosPins[3] = {5, 17, 16}; // GPIO 5 -> AZ, GPIO 17 ->EL, GPIO 16 -> W

Servo servos[3];

void onConnect()
{
  Serial.println("PS3 Remote Control is now connected.");
}

void onDisConnect()
{
  Serial.println("PS3 Remote Control disconnected.");
}




void setup()
{
  Serial.begin(115200);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);

  servo1.attach(servo1Pin, minUs, maxUs);
  servo2.attach(servo2Pin, minUs, maxUs);
  servo3.attach(servo3Pin, minUs, maxUs);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);

// ================ Enter your PS3 controller BT address here ================

  
  Ps3.begin("b8:27:eb:0b:53:d8"); // address of the used, genuine, PS3 controller


// ===========================================================================
  
  Serial.println("Ready.");
  Ps3.setPlayer(player);  // activate player LED

  // Servos to initial positions for autoSprinkle
  posAZ = limMinAZ;  // Initial Position for Azimut (0°)
  posEL = limMinEL;  // Initial Position for Elevation (0°)
  posW  = limMinW;   // Initial Position for Water-Valve (close position) (0° - closed)

  Serial.println("Sprouter started...");
  Serial.println("Firmware V0.16 ");
  Serial.println(" ");
  Serial.print("Elevation Up: ");
  Serial.println(posAZ);
  Serial.print("Elevation Dn: ");
  Serial.println(posEL);
  Serial.print("Water-Valve : ");
  Serial.println(posW);
  

  // Launch the SPIFFS file system  
  if(!SPIFFS.begin()){ 
  Serial.println("When Mounting SPIFFS something went wrong."); 
  }

  loadArrays();    //Load stored data for auto sprinkle

  Serial.print("");
  Serial.println("Please Connect your PS3-Controller by pressing the PS-Button.");
  Serial.print(" ");
 }
void loop()
{
  modeSelect();  // select between auto-, manual-, and program-mode
  // notify();    // Battery status of PS3-Controller
  servoOutput(); // send data to servos

  serialData();  // use serial interface for testing commands

}



void notify() {

  //---------------------- Battery events ---------------------
  if ( battery != Ps3.data.status.battery ) {
    battery = Ps3.data.status.battery;
    Serial.print("The controller battery is ");
    if ( battery == ps3_status_battery_charging )      Serial.println("CHARGING");
    else if ( battery == ps3_status_battery_full )     Serial.println("FULL");
    else if ( battery == ps3_status_battery_high )     Serial.println("HIGH");
    else if ( battery == ps3_status_battery_low)       Serial.println("LOW");
    else if ( battery == ps3_status_battery_dying )    Serial.println("DYING");
    else if ( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
    else Serial.println("UNDEFINED");
  }
}


void manualControl() {

  ps3AZ = Ps3.data.analog.stick.lx, DEC; // Read PS3 input status
  ps3EL = Ps3.data.analog.stick.ly, DEC; // Read PS3 input status
  ps3W  = Ps3.data.analog.stick.ry, DEC; // Read PS3 input status

  // ------------------ Azimuth -------------------------


  if ((ps3AZ > 30) && (posAZ < limMaxAZ) && (millis() > intervalAZ + myTimerAZ )) {
    posAZ = posAZ + 1;
    myTimerAZ = millis();
    Serial.print("AZ:");
    Serial.println(posAZ);
  }
  else if ((ps3AZ < -30) && (posAZ > limMinAZ) && (millis() > intervalAZ + myTimerAZ )) {
    posAZ = posAZ - 1;
    myTimerAZ = millis();
    Serial.print("AZ:");
    Serial.println(posAZ);
  }

  // ------------------ Elevation -----------------------


  if ((ps3EL < -30) && (posEL < limMaxEL) && (millis() > intervalEL + myTimerEL )) {
    posEL = posEL + 1;
    myTimerEL = millis();
    Serial.print("EL:");    Serial.println(posEL);
  }
  else if ((ps3EL > 30) && (posEL > limMinEL) && (millis() > intervalEL + myTimerEL )) {
    posEL = posEL - 1;
    myTimerEL = millis();
    Serial.print("EL:");
    Serial.println(posEL);
  }

  // ------------------ Water-Valve ---------------------

  if ((ps3W < -30) && (posW < limMaxW) && (millis() > intervalW + myTimerW )) {
    posW = posW + 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }
  else if ((ps3W > 30) && (posW > limMinW) && (millis() > intervalW + myTimerW )) {
    posW = posW - 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }

}

void servoOutput() {

  servo1.write(posAZ);             // tell servo to go to position in variable 'posAZ'
  servo2.write(posEL);             // tell servo to go to position in variable 'posEL'
  servo3.write(posW);              // tell servo to go to position in variable 'posW'

}

void modeSelect() {


  // ----------------- Define PS3 Controller keys ------------------

  if ((progMode == 0) && (manMode  == 0) && (standbyMode  != 1)) {
    standbyMode = 1;
    Serial.println("Standby ON.");
    player = 1;  // activate player 1 LED
    Ps3.setPlayer(player); // update player LED
  }

  if ((Ps3.event.button_down.circle) && (standbyMode == 1) && (progMode  != 1)) {
    progMode = 1;
    standbyMode = 0;
    Serial.println("ProgMode ON.");
    Serial.print("Select the Azimuth Vector to be programmed...");
    player = 4;  // activate player 4 LED
    Ps3.setPlayer(player); // update player LED
  }

  if ((Ps3.event.button_down.square) && (standbyMode == 1) && (manMode  != 1)) {
    manMode = 1;
    standbyMode = 0;
    Serial.println("ManualMode ON.");
    player = 2;  // activate player 2 LED
    Ps3.setPlayer(player); // update player LED
  }

  if ((Ps3.event.button_down.cross) && (standbyMode != 1)) {
    manMode = 0;
    progMode = 0;
    standbyMode = 1;
    progStage = 1;
    Serial.println("CANCELLING... Back to standby.");
    player = 1;  // activate player 1 LED
    Ps3.setPlayer(player); // update player LED
    homeSprinkler();
  }

  if ((Ps3.event.button_down.start) && (buttonStart == 0)) {
    buttonStart = 1;
  }

  if ((Ps3.event.button_up.start) && (buttonStart == 1 )) {
        buttonStart = 0;
  }

  if(( Ps3.event.button_down.select) && (buttonDStart == 0)){
        buttonDStart = 1;}
  
  if(( Ps3.event.button_up.select) && (buttonDStart == 1)){
        buttonDStart = 2;}

  if ((Ps3.event.button_down.triangle) && (buttonUp == 0)) {
        buttonUp = 1;  // Flag when button was pressed
  }
  if ((Ps3.event.button_up.triangle) && (buttonUp == 1)) {
        buttonUp = 2;  // Flag when button has been released
  }

  // ----------------- Arrange sub-program routines ------------------

  
  if (manMode == 1) {
    manualControl();   // start the manual control section
  }

  if ((progMode == 1) && (progStage == 1)) {
    learnAZ();       // enter the first programming section (for Azimuth)
  }

  if ((progMode == 1) && (progStage == 2)) {
    learnELup();       // enter the first programming section (for ELup)
  }

  if ((progMode == 1) && (progStage == 3)) {
    learnELdn();       // enter the first programming section (for ELdn)
  }

  if ((progMode == 1) && (progStage == 4)) {
    saveVector();       // save the collected vector data
  }

  if ((standbyMode == 1) && (buttonStart == 1)) {
    autoSprinkleInit();       // start auto sprinkle
  }

}


void learnAZ() {

  ps3AZ = Ps3.data.analog.stick.lx, DEC; // Read PS3 input status
  ps3W  = Ps3.data.analog.stick.ry, DEC; // Read PS3 input status

  // Select the Azimuth Vector to be programmed... progStage = 1

  if ((ps3AZ < -30) && (posAZ < limMaxAZ) && (millis() > intervalAZ + myTimerAZ )) {
    posAZ = posAZ + 1;
    myTimerAZ = millis();
    Serial.print("AZ:");
    Serial.println(posAZ);
  }
  else if ((ps3AZ > 30) && (posAZ > limMinAZ) && (millis() > intervalAZ + myTimerAZ )) {
    posAZ = posAZ - 1;
    myTimerAZ = millis();
    Serial.print("AZ:");
    Serial.println(posAZ);
  }

  if ((ps3W < -30) && (posW < limMaxW) && (millis() > intervalW + myTimerW )) {
    posW = posW + 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }
  else if ((ps3W > 30) && (posW > limMinW) && (millis() > intervalW + myTimerW )) {
    posW = posW - 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }

  if ((progMode == 1) && (progStage == 1)  && (buttonUp == 2)) {
    buttonUp = 0; // Remove flag for button release
    saveAZ = posAZ;
    Serial.print("Value for Azimuth saved as: ");
    Serial.println(saveAZ);
    Serial.print("Select upper EL position for this vector... ");
    player = 7;  // activate player 5 LED
    Ps3.setPlayer(player); // update player LED
    progStage = 2;  // switch to the next progStage
  }
}


void learnELup() {
  
  ps3EL = Ps3.data.analog.stick.ly, DEC; // Read PS3 input status
  ps3W  = Ps3.data.analog.stick.ry, DEC; // Read PS3 input status


  // Select the Maximum value for Elevation to be programmed...  progStage = 2

  if ((ps3EL < -30) && (posEL < limMaxEL) && (millis() > intervalEL + myTimerEL )) {
    buttonUp = 1; // Set flag for button release
    posEL = posEL + 1;
    myTimerEL = millis();
    Serial.print("EL:");
    Serial.println(posEL);
  }
  else if ((ps3EL > 30) && (posEL > limMinEL) && (millis() > intervalEL + myTimerEL )) {
    posEL = posEL - 1;
    myTimerEL = millis();
    Serial.print("EL:");
    Serial.println(posEL);
  }

  if ((ps3W < -30) && (posW < limMaxW) && (millis() > intervalW + myTimerW )) {
    posW = posW + 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }
  else if ((ps3W > 30) && (posW > limMinW) && (millis() > intervalW + myTimerW )) {
    posW = posW - 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }

  if ((progMode == 1) && (progStage == 2)  && (buttonUp == 2)) {
    buttonUp = 0; // Remove flag for button release
    saveELup = posEL;
    learnELdn();
    Serial.print("Upper EL position saved as: ");
    Serial.println(saveELup);
    Serial.print("Select the Minimum value for Elevation and Value for Water-Valve to be programmed...");
    player = 9;  // activate player 61 LED
    Ps3.setPlayer(player); // update player LED
    progStage = 3;  // switch to the next progStage
  }
}

void learnELdn() {
  
  ps3EL = Ps3.data.analog.stick.ly, DEC; // Read PS3 input status
  ps3W  = Ps3.data.analog.stick.ry, DEC; // Read PS3 input status

  // Select the Minimum value for Elevation and Value for Water-Valve to be programmed... progStage = 3



  if ((ps3EL < -30) && (posEL < limMaxEL) && (millis() > intervalEL + myTimerEL )) {
    posEL = posEL + 1;
    myTimerEL = millis();
    Serial.print("EL:");
    Serial.println(posEL);
  }
  else if ((ps3EL > 30) && (posEL > limMinEL) && (millis() > intervalEL + myTimerEL )) {
    posEL = posEL - 1;
    myTimerEL = millis();
    Serial.print("EL:");
    Serial.println(posEL);
  }

  if ((ps3W < -30) && (posW < limMaxW) && (millis() > intervalW + myTimerW )) {
    posW = posW + 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }
  else if ((ps3W > 30) && (posW > limMinW) && (millis() > intervalW + myTimerW )) {
    posW = posW - 1;
    myTimerW = millis();
    Serial.print("W:");
    Serial.println(posW);
  }

  if ((progMode == 1) && (progStage == 3)  && (buttonUp == 2)) {
    buttonUp = 0; // Remove flag for button release
    saveELdn = posEL;
    saveW = posW;
    Serial.print("Lower Value for lower elevation saved as: ");
    Serial.println(saveELdn);
    Serial.print("Value for Water Valve saved as: ");
    Serial.println(saveW);
    player = 4;  // activate player 4 LED
    Ps3.setPlayer(player); // update player LED
    progStage = 4;  // switch to the next progStage
  }
}

void serialData() {
  if (Serial.available() > 0) { //check if any data was received
    char RXdata = Serial.read();
//    Serial.print(RXdata, DEC);

    switch (RXdata) {

      case '-':{
        saveAZ = posAZ;  // select current AZ position as to be requested fom NVM
        Serial.print("posAZ: ");
        Serial.println(posAZ);

        newELup = 0;
        newELdn = 0;
        newW    = 0;
        
        loadVector();}
        break;

      case '0':{
        Serial.println("Servos will be moved to assembly positions...");
        delay (1000);
        posAZ = centerAZ;  // Initial Position for Azimut
        posEL = limMinEL;  // Initial Position for Elevation
        posW  = limMinW;   // Initial Position for Water-Valve (close position)
        }
        break;

       case '+':{
       
        for (saveAZ = limMinAZ ; saveAZ < 181; saveAZ++){
        newELup = 0;
        newELdn = 0;
        newW    = 0;
        
        newELup = array_ELup[saveAZ];
        newELdn = array_ELdn[saveAZ];
        newW    = array_W[saveAZ];

        Serial.println ("Currently loaded values: ");
        Serial.print("AZ: ");
        Serial.println(saveAZ);
        Serial.print("ELup: ");
        Serial.println(newELup);
        Serial.print("ELdn: ");
        Serial.println(newELdn);
        Serial.print("Water-Valve: ");
        Serial.println(newW);
        }}
        break;

       case '7':{
        File fileELup = SPIFFS.open("/file_ELup.txt",FILE_READ);
        if(!fileELup){
        Serial.println("Failed to open file for reading");
        //return;
        }
        Serial.println("− read from file:");
        while(fileELup.available()){
        Serial.write(fileELup.read());
        }
        fileELup.close();}
        break;

       case '8':{
        Serial.print("Saving arrays to SPIFFS...");
        saveArrays();
        Serial.print("...saving completed.");}
        break;

       case '9':{
        Serial.print("Loading arrays from SPIFFS...");
        loadArrays();
        Serial.print("...loading completed.");}
        break;
    
    }
  }
}


// ------------ Section for auto sprinkling --------------

void autoSprinkleInit() {
  
  buttonStart = 2; // Auto-Sprinkle active
  Serial.println("Starting Auto-Watering...");
  
  // Servos to initial positions for autoSprinkle
  posAZ = limMinAZ;  // Initial Position for Azimut
  posEL = limMinEL;  // Initial Position for Elevation
  posW  = limMinW;   // Initial Position for Water-Valve (close position)

  servo1.write(posAZ);  // tell servo to go to position in variable 'posAZ'
  servo2.write(posEL);  // tell servo to go to position in variable 'posEL'
  servo3.write(posW);   // tell servo to go to position in variable 'posW'

  autoSprinkleLoad();
}

void autoSprinkleLoad() {

    saveAZ = posAZ;  // make sure the right data is loaded

    loadVector(); // --- Load stored data for first vector
    posW = newW;
    for (;posAZ < limMaxAZ ;){
    
    if (autoStop ==1) break;
    
    saveAZ = posAZ;
    saveW = posW;
    loadVector();



    autoSprinkleMove(); // start watering the vector by moving EL up/down
    Serial.println("Sector Watered.");
    
    posAZ++;
    servo1.write(posAZ); // move azimuth servo
    servo3.write(newW); // move water valve servo
    Serial.print("Next AZ: "); 
    Serial.println(posAZ); 

    }

    Serial.println("Auto watering finished.");
    homeSprinkler();
    buttonStart = 0; // Reset Flag for starting auto watering
    autoStop = 0; // Reset flag for auto stop
  }


void autoSprinkleMove() {

  // exchange upper and lower elevation limits when twisted

    sprinkleMillis = millis();  // set millis
    currentMillis  = millis();  // set millis

    for ( ; sprinkleMillis + sprinkleTime > currentMillis ;) {

    timeLeft = (((sprinkleMillis + sprinkleTime) - currentMillis)/1000);

    //Serial.println("Remaining time for sector: ");
    //Serial.println(timeLeft);
    stopAutoWatering(); // check for user inpot to stop auto-watering

    if ((posEL >= newELup) && (sprinkleDirection != 1)) {
      sprinkleDirection = 1;  // Upper limit is reached, set flag for downwards movement
      Serial.println("Changing EL direction to DOWN");
    }

    if ((posEL <= newELdn) && (sprinkleDirection != 0)) {
      sprinkleDirection = 0; // Lower limit is reached, set flag for upwards movement
      Serial.println("Changing EL direction to UP");
    }

    if ((posEL < newELup) && (sprinkleDirection == 0) && (currentMillis > intervalELw + myTimerEL )) {
      sprinkleUp();
    }

    if ((posEL > newELdn) && (sprinkleDirection == 1) && (currentMillis > intervalELw + myTimerEL )) {
      sprinkleDn();
    }

    currentMillis = millis();  // set millis
  }
}

void sprinkleUp() {
  posEL = posEL + 10;
  myTimerEL = millis();
  servo2.write(posEL);             // tell servo to go to position in variable 'posEL'
  
}

void sprinkleDn() {
  posEL = posEL - 10;
  myTimerEL = millis();
  servo2.write(posEL);             // tell servo to go to position in variable 'posEL'
  
}


void saveVector() {

  saveAZ = posAZ;

  Serial.print("The following data will be saved for the current vector: ");
    
    Serial.print("AZ: ");
    Serial.println(saveAZ);
    Serial.print("ELup: ");
    Serial.println(saveELup);
    Serial.print("ELdn: ");
    Serial.println(saveELdn);
    Serial.print("Water-Valve: ");
    Serial.println(saveW);

  array_ELup [saveAZ] = saveELup;
  array_ELdn [saveAZ] = saveELdn;
  array_W    [saveAZ] = saveW;
  
  Serial.print("Leaving ProgMode! ");
  progMode = 0;
  progStage = 1; // reset programming stage for next programming
  saveArrays();  // saving data to SPIFFS
}

void loadVector(){

    // load saved vector data

    newELup = array_ELup[saveAZ];
    newELdn = array_ELdn[saveAZ];
    newW    = array_W[saveAZ];

    // Check upper / lower limits for EL and correct order if required
    if (newELup < newELdn) {
    twist = newELup;
    newELup = newELdn;
    newELdn = twist;
  }
    
    Serial.print("Loaded Azimuth:");
    Serial.println(saveAZ);
    Serial.print("ELup: ");
    Serial.println(newELup);
    Serial.print("ELdn: ");
    Serial.println(newELdn);
    Serial.print("Water-Valve: ");
    Serial.println(newW);
  }

  void saveArrays(){

    // ------------- save Array to SPIFFS ----------------

  File fileELup = SPIFFS.open("/file_ELup.txt",FILE_WRITE);
  if(!fileELup){
      Serial.println("− failed to open fileELup for writing");
      return;}
  for (int n = 0; n <= limMaxAZ; n++){
    fileELup.println(array_ELup[n]);
    }
  fileELup.close();

  File fileELdn = SPIFFS.open("/file_ELdn.txt",FILE_WRITE);
  if(!fileELdn){
      Serial.println("− failed to open fileELdn for writing");
      return;}
  for (int n = 0; n <= limMaxAZ; n++){
    fileELdn.println(array_ELdn[n]);
    }
  fileELdn.close();

    File fileW = SPIFFS.open("/file_W.txt",FILE_WRITE);
    if(!fileW){
      Serial.println("− failed to open fileW for writing");
      return;}
  for (int n = 0; n <= limMaxAZ; n++){
    fileW.println(array_W[n]);
    }
  fileW.close();
   
   }


  void loadArrays(){

String string_ELup;
String string_ELdn;
String string_W;
int counter = 0;

  File fileELup = SPIFFS.open("/file_ELup.txt",FILE_READ);
  if(!fileELup){
    Serial.println("Failed to open file for reading");
    return;
  }
  while
  (fileELup.available()){string_ELup = fileELup.readStringUntil('\r');
  array_ELup[counter] = string_ELup.toInt();
  counter++;
  }
  Serial.print("Loaded settings for elevation UP: ");
  Serial.println(counter); 
  fileELup.close();

  File fileELdn = SPIFFS.open("/file_ELdn.txt",FILE_READ);
   if(!fileELdn){
    Serial.println("Failed to open file for reading");
    return;
  }
  counter = 0;
  while
  (fileELdn.available()){string_ELdn = fileELdn.readStringUntil('\r');
  array_ELdn[counter] = string_ELdn.toInt();
  counter++;
  }
  Serial.print("Loaded settings for elevation DOWN: ");
  Serial.println(counter);   
  fileELdn.close();

  File fileW    = SPIFFS.open("/file_W.txt",FILE_READ);
   if(!fileW){
    Serial.println("Failed to open file for reading");
    return;
  }
  counter = 0;
  while
  (fileW.available()){string_W = fileW.readStringUntil('\r');
  array_W[counter] = string_W.toInt();
  counter++;
  }
  Serial.print("Loaded settings for Water-Valve: ");
  Serial.println(counter); 
  fileW.close();
   
 }


 void stopAutoWatering(){
    // ---- stop option by pressing SELECT button
      if (Ps3.event.button_down.select){
      sprinkleMillis = 0; // Stop auto watering time immediately
      myTimerEL = 0;
      Serial.print("Auto watering stopped by user command!");
      Serial.println("Back to standby.");
      buttonDStart = 0;
      buttonStart = 0; // Reset Flag for starting auto watering
      autoStop = 1;
      posW  = limMinW;   // Shut Water-Valve (close position)
      

      }
  }

  void homeSprinkler(){
  
  // Servos to initial positions for autoSprinkle
  Serial.println("Homing sprinkler.");

    while ( posW > limMinW){
    posW--;
    servo3.write(posW);
    delay(10); // slow down movement
    }
    
  while ( posAZ > limMinAZ){
    posAZ--;
    servo1.write(posAZ);
    delay(30); // slow down movement
    }
    
  while ( posEL > limMinEL){
    posEL--;
    servo2.write(posEL);
    delay(30); // slow down movement
    }
   }
 
