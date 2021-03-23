#include <Arduino.h>
#include <U8g2lib.h>

#include <ArduinoJson.h>
#include "SdFat.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS

#define error(s) sd.errorHalt(&Serial, F(s))

//defining the pins

// LCD module pins
const uint8_t LCDclockPin = 23;  // LCD clock pin
const uint8_t LCDdataPin = 17;  // LCD data pin
const uint8_t LCDcsPin = 16;  // LCD cs pin
const uint8_t BTNselectPin = 35;  // menu selection button pin
const uint8_t BTNnextPin = 33;  // menu next/down button pin
const uint8_t BTNprevPin = 31;  // menu previous/up button pin
const uint8_t StopPin = 43;  // stop button pin - unused
const uint8_t BeepPin = 37;  // beeper pin - unused
const uint8_t chipSelect = 4;  // SD card selection chip pin

// Stepper motors pins
const uint8_t XenabPin = 24;  // translation stepper enable pin
const uint8_t RenabPin = 30;  // rotation stepper enable pin
const uint8_t XstepPin = 26;  // translation stepper step pin
const uint8_t RstepPin = 36;  // rotation stepper step pin 
const uint8_t XdirPin = 28;  // translation stepper direction pin
const uint8_t RdirPin = 34;  // rotation stepper step pin

// Interrupt pins
const uint8_t XplusInterrPin = 19;  // translation + end switch pin
const uint8_t XminusInterrPin = 18;  // translation - end switch pin
const uint8_t BTNinterrPin = 2;  // menu selection button winding interrupt pin







//variables

char *programList;  // user's winding program name list (read from SD)


long Rposition = 0; // rotation stepper position [steps]
long Xposition = 0; // translation stepper position [steps]
bool Rdir = 1; // rotation stepper rotation direction
bool Xdir = 1;  // translation stepper rotation direction
bool XRpos = 0; // flag to interrupt function know which stepper to adjust to 0 position
 bool toggle = 0; // debug toggle boll to flash led when adjusting 0 position // TODO




const uint8_t maxArrLen = 10;  // maximum number of winding sections on a coil

float Aprofile[maxArrLen];  // constant values to calculate translation stepper step interval
float Bprofile[maxArrLen];  // acceleration component values to calculate translation stepper step interval
double Lprofile[maxArrLen];  // nr. of steps in winding sections for rotation stepper

float lenghts[maxArrLen];  // lengths of coil winding sections [mm] - user programmable
float startK[maxArrLen];  // coil pitch at the start of winding section [mm] - user programmable
float endK[maxArrLen];  // coil pitch at the end of winding section [mm] - user programmable
uint8_t dly = 5;  // stepper motor driver minimum on/off time interval
uint8_t profileLength = 0;  // nr. of coil winding sections - user programmable

unsigned long Tstart; // variable to measure time between steps - time at start of winding

float Rspeed = 1.2;  // rotation stepper max rotation speed [rot/s]
float XbackOffDist = 2.0;  // back-off distance at end switch interrupt [mm]
uint64_t Rsteps = 200 * 8;  // nr. of steps to perform 1 revolution for rotation stepper
uint64_t Xsteps = 200 * 8;  // nr. of steps to perform 1 revolution for translation stepper
float Xd = 4.0;  // translation stepper linear screw pitch [mm]
float r = 15.0 / 28.0; // stepper pulley / winding axis pulley ratio
unsigned int fullRotSteps = Rsteps / r;  // nr. of steps to perform 1 revolution for winding axis
int XbackOffSteps = XbackOffDist/Xd*Xsteps;  // nr. of steps to perform in case of end switch interrupt

uint8_t velPercent = 100;  // percentage of Rspeed that will be used to wind the coils
uint8_t retVelPercent = 100;  // percentage speed of translation stepper that will be used to return to 0 position

uint8_t prog_selection = 1;  // nr. of curent user program that is selected
uint8_t current_selection = 1;  // nr. of curent menu item that is selected


float returnSpeed = 10.0;  // translation unit returning (to position 0) speed [mm/s] - only approximately!
int retStepTime = Xsteps/(Xd*returnSpeed);  // translation stepper return step interval [us] 
int stepTime = 1e8 / (Rspeed*velPercent*fullRotSteps);  // time beetween steps of rottation stepper [us]
float initAcc = sq(Rspeed*velPercent / 100.0) * fullRotSteps / 4;  // initial acceleration of rotation stepper (devided by 2 to decrease computing time in running mode)
int accSteps = sq(Rspeed*velPercent / 100.0 * fullRotSteps) / (4 * initAcc);  // nr. of steps intended for accelaration of rotation stepper

volatile bool interruptFlag;  // interrupt flag


// U8g2lib object for controlling the LCD screen
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ LCDclockPin, /* data=*/ LCDdataPin, /* CS=*/ LCDcsPin, /* reset=*/ U8X8_PIN_NONE); 


// Config object for reading users winding settings (programs) from SD card
struct Config {
  uint8_t pLength;
  bool dir;
  float K;
  float lengths[maxArrLen];
  float startK[maxArrLen];
  float endK[maxArrLen];
};




Config config;  // global configuration object
SdFat sd;  // SdFat object for SD card operations
File settingsFile;  // File object for reading files
File settingsDir;  // File object for reading directories

// main menu items
const char *mainMenu_list =
  "Izberi program\n"
  "Zacni navijanje\n"
  "Pozicija 0,0\n"
  "Nastavitve";
  
// function for calculating new winding parameters if user changes velPercent variable
void calcSpeedPar(void) {
  stepTime = 1e8 / (Rspeed * velPercent * fullRotSteps);
  accSteps = sq(Rspeed * velPercent / 100.0 * fullRotSteps) / (4 * initAcc);
}


void setup(void) {
  Serial.begin(9600); // begin serial comumnication
  
  u8g2.begin(/*Select=*/ BTNselectPin, /*Right/Next=*/ BTNnextPin, /*Left/Prev=*/ BTNprevPin, /*Up=*/ BTNprevPin, /*Down=*/ BTNnextPin, /*Home/Cancel=*/ U8X8_PIN_NONE);  // setup pins for controlling the UI
  u8g2.setFont(u8g2_font_6x12_tr);  // set LCD font
  //set pins as outputs
  pinMode(13, OUTPUT);
  pinMode(XenabPin, OUTPUT);
  pinMode(RenabPin, OUTPUT);
  pinMode(XstepPin, OUTPUT);
  pinMode(RstepPin, OUTPUT);
  pinMode(XdirPin, OUTPUT);
  pinMode(RdirPin, OUTPUT);

  pinMode(XplusInterrPin, INPUT);
  pinMode(XminusInterrPin, INPUT);
  pinMode(BTNinterrPin, INPUT);

  // Initialize Timer2
  cli();  // stop interrupts
  TCCR2A = 0;  // set entire TCCR2A register to 0
  TCCR2B = 0;  // same for TCCR2B
  TCNT2  = 0;  // initialize counter value to 0
  OCR2A = 255;  // set compare match register for 490hz increments = (16*10^6) / (490*128) - 1 (must be <256)
  TCCR2A |= (1 << WGM21);  // turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);  // Set CS22 and CS20 bits for 128 prescaler
  sei();  // allow interrupts
//  lenghts[0] = 25;
//  startK[0] = 0.4; //začetni koraki odseka upora v mm
//  endK[0] = 0.4;
//  profileLength = 1;
//  transformProfile();
  disableMotors();  // disable stepper motor drivers
}






void loop(void) {
  mainMenu();
}


//#############################################GUI#####################################

// main GUI menu
void mainMenu(void) {
  current_selection = u8g2.userInterfaceSelectionList(
                        "Glavni meni",
                        current_selection,
                        mainMenu_list);
  switch (current_selection) {
    case 0:
      current_selection = 1;
      mainMenu();
      break;
    case 1:
      current_selection = 1;
      programMenu();
      break;
    case 2:
      windingMenu();
      break;
    case 3:
      resetingPosMenu();
      break;
    case 4:
      settingsMenu();
      break;
  }
}


// user program selection menu
void programMenu(void) {
  char programList[200];  // initialize char array for display of available user programs - max 200 char (10x20char)
  readProgramNames(programList);  // read program names from the SD and populate programList
  if (programList[5] == '\0') {  // if there is no program name resturned show SD error menu
    u8g2.userInterfaceMessage(
      "Kartica ni vstavljena",
      "ali pa je prazna.",
      "",
      " OK ");
  } else {  // if there is some program name returned show program selection menu
    current_selection = u8g2.userInterfaceSelectionList(
                          "Izberi program",
                          current_selection,
                          programList);
  }
  free(programList);  // free programList memory
  if (current_selection > 1) { // if user selected program (not exit option) load selected program
    prog_selection = current_selection - 1;
    loadProgram(config);
  }
  transformProfile();  // transform coil profile to software frendly form
  current_selection = 1;  // set main menu item selection back to 1
}


// start winding menu
void windingMenu(void) {
  // show warning before start of winding
  current_selection = u8g2.userInterfaceMessage( 
                        "!!!Pozor!!! ",
                        "Zecetek navijanja",
                        "Nadaljujem?",
                        " PREKLICI \n OK ");
  switch (current_selection) {
    case 1:  // if user select "Cancel" do nothing
      break;
    case 2:  // if user select "OK" start winding
      if (Rposition | Xposition) {  // if Rposition or Xposition not at 0 go to notInInitPosWarnMenu
        notInInitPosWarnMenu();
      } else {  // if Rposition and Xposition at 0 start winding
        runProgram();
        // when program finishes show returning into initial position dialogue
        u8g2.userInterfaceMessage(
          "!!!Pozor!!! ",
          "Vrni v pozicjo 0,0",
          "Nadaljujem?",
          "OK ");
        goToInitPos();
      }
      break;
  }
  current_selection = 2;  // set main menu item selection back to 2
}


// not in initial position warning menu
void notInInitPosWarnMenu(void) {
  // display warning message
  current_selection = u8g2.userInterfaceMessage(
                        "!!!Pozor!!! ",
                        "Naprava ni v zac. poz.",
                        "Postavim v zac poz.?",
                        " PREKLICI \n OK ");

  switch (current_selection) {
    case 1:  // if user selects "Cancel" do nothing
      break;
    case 2:  // if user selects "OK" go to initial position
      goToInitPos();
  }
}

// return to initial position menu
void resetingPosMenu(void) {
  // display warning message before stepper start
  current_selection = u8g2.userInterfaceMessage(
                        "!!!Pozor!!! ",
                        "Vrni v pozicjo 0,0",
                        "Nadaljujem?",
                        " PREKLICI \n OK ");                   
  switch (current_selection) {
    case 1:  // if user selects "Cancel" do nothing
      break;
    case 2:  // if user selects "OK" go to initial position
      goToInitPos();
      break;
  }
  current_selection = 3;  // set main menu item selection back to 3
}


// settings menu
void settingsMenu(void) {
  current_selection = 1;  // set settings menu item selection to 1
  // define menu items
  const char *menuItems =
    "Hitrost vretena\n"
    "Hitrost pomika v 0,0\n"
    "Pozicija 0 pomik\n"
    "Pozicija 0 rotac.\n"
    "Izhod";
  while (current_selection != 5) { // untill user select "Exit" option stay in menu
    // show settings menu 
    current_selection = u8g2.userInterfaceSelectionList(
                          "Nastavitve naprave",
                          current_selection,
                          menuItems);
    switch (current_selection) {
      case 1:  // if user selected to set winding speed show setWindingSpeed dialogue
        setWindingSpeed();
        current_selection = 1;  // set settings menu item selection back to 1
        break;
      case 2:  // if user selected to set returning speed show setReturnSpeed dialogue
        setReturnSpeed();
        current_selection = 2;  // set settings menu item selection back to 2
        break;
      case 3:  // if user selected to set translation 0 position show settingXposMenu dialogue
        settingXposMenu();
        current_selection = 3;  // set settings menu item selection back to 3
        break;
      case 4:  // if user selected to set rotation 0 position show settingRposMenu dialogue
        settingRposMenu();
        current_selection = 4;  // set settings menu item selection back to 4
        break;
      case 5:  // if user selected "Exit" do nothing
        break;
    }
  }
  current_selection = 4;  // set main menu item selection back to 4
}


// set winding speed dialogue
void setWindingSpeed(void) {
  u8g2.userInterfaceInputValue("Nast. hitr. navijanja:", "hitrost=", &velPercent, 20, 110, 3, "%");  // show dialogue
  calcSpeedPar();  // calculate new speed parameters
  if(velPercent>100){  // if set speed is above 100% show warning
    u8g2.userInterfaceMessage(
                            "Pri vrednostih ",
                            "nad 100% se lahko",
                            "pojavijo tezave",
                            " OK ");
  }
}

// set returning speed dialogue
void setReturnSpeed(void) {
  u8g2.userInterfaceInputValue("Nast. hitr. vracanja:", "hitrost=", &retVelPercent, 20, 125, 3, "%");  // show dialogue
  if(retVelPercent>100){  // if set speed is above 100% show warning
    u8g2.userInterfaceMessage(
                            "Pri vrednostih ",
                            "nad 100% se lahko",
                            "pojavijo tezave",
                            " OK ");
  }
}

// set translation initial (0) position
void settingXposMenu(void) {
  current_selection = 1;  // set curent menu selection to 1
  XRpos = 1;  // enable translation stepper
  uint8_t sel = 1;  // set menu to show "-" button first
  while (current_selection != 2) {  // while user does not click "OK" stay in the menu
    if (sel) {  // if sel=true show "-" button first
      current_selection = u8g2.userInterfaceMessage( // show menu
                            "Nastavi pozicijo 0",
                            "pomika",
                            "",
                            " - \n OK \n + ");
      switch (current_selection) {
        case 1:  // if user selected "-"
          digitalWrite(XdirPin, LOW);  // set translation stepper direction to "-"
          movingStepperMenu();
          //digitalWrite(13, LOW);
          sel = 1;  // set menu to show "-" button first
          break;
        case 3:  // if user selected "+"
          digitalWrite(XdirPin, HIGH);
          movingStepperMenu();
          //digitalWrite(13, LOW);
          sel = 0;  // set menu to show "+" button first
          break;
      }
    } else { // if sel=false show "+" button first
      current_selection = u8g2.userInterfaceMessage(  // show menu
                            "Nastavi pozicijo 0",
                            "pomika",
                            "",
                            " + \n OK \n - ");
      switch (current_selection) {
        case 1:  // if user selected "+"
          digitalWrite(XdirPin, HIGH);  // set translation stepper direction to "+"
          movingStepperMenu();
          //digitalWrite(13, LOW);
          sel = 0;  // set menu to show "+" button first
          break;
        case 3:  // if user selected "-"
          digitalWrite(XdirPin, LOW);  // set translation stepper direction to "-"
          movingStepperMenu();
          //digitalWrite(13, LOW);
          sel = 1;  // set menu to show "-" button first
          break;
      }
    }
  }
  Xposition = 0;  // set translation position to 0
}

void settingRposMenu(void) {
  current_selection = 1;  // set curent menu selection to 1
  XRpos = 0;  // enable rotation stepper
  uint8_t sel = 1;  // set menu to show "-" button first
  while (current_selection != 2) {  // while user does not click "OK" stay in the menu
    if (sel) { // if sel=true show "-" button first
      current_selection = u8g2.userInterfaceMessage(
                            "Nastavi pozicijo 0",
                            "rotacije",
                            "",
                            " - \n OK \n + ");
      switch (current_selection) {
        case 1:  // if user selected "-"
          digitalWrite(RdirPin, LOW);  // set rotation stepper direction to "-"
          movingStepperMenu();
          sel = 1;  // set menu to show "-" button first
          break;
        case 3:  // if user selected "+"
          digitalWrite(RdirPin, HIGH);  // set rotation stepper direction to "+"
          movingStepperMenu();
          sel = 0;  // set menu to show "+" button first
          break;
      }
    } else { // if sel=false show "+" button first
      current_selection = u8g2.userInterfaceMessage(
                            "Nastavi pozicijo 0",
                            "rotacije",
                            "",
                            " + \n OK \n - ");
      switch (current_selection) {
        case 1:  // if user selected "+"
          digitalWrite(RdirPin, HIGH);  // set rotation stepper direction to "+"
          movingStepperMenu();
          sel = 0;
          break;
        case 3:
          digitalWrite(RdirPin, LOW);
          movingStepperMenu();
          sel = 1;
          break;
      }
    }
  }
  Rposition = 0;
  //mainMenu();
}



void movingStepperMenu(void){
  startMovingStepper();  // start moving enabled stepper
  current_selection = u8g2.userInterfaceMessage(  // show stop stepper dialogue
                        "Nastavi pozicijo 0",
                        "rotacije",
                        "",
                        " STOP ");
  stopMovingStepper();  // when user clicks "STOP" stop moving enabled stepper
}

//#######################################FUNCTIONS#################################

void startMovingStepper(void){
  attachInterrupts();  // enable end swithches interrupts 
  enableMotors();  // enable stepper motors drivers
  TIMSK2 |= (1 << OCIE2A);  // start timer2
}

void stopMovingStepper(void){
  TIMSK2 = 0;  // stop timer2
  disableMotors();  // disable stepper motors drivers
  detachInterrupts();  // disable end swithches interrupts 
}

ISR(TIMER2_COMPA_vect) {  //timer2 executing function
  if (XRpos) {  // if translation stepper is enabled move it
    digitalWrite(XstepPin, HIGH);
    digitalWrite(XstepPin, LOW);
  } else {  // if rotation stepper is enabled move it
    digitalWrite(RstepPin, HIGH);
    digitalWrite(RstepPin, LOW);
  }
}

void runProgram(void) {
  unsigned long currPos = 0;  // current translation motor position
  unsigned long nextPos; // calculated next position for translation motor to move
  unsigned long accTime;  // acceleration time between steps
  //bool runNextProfile = true;
  //unsigned long stepsT = 0;
  //unsigned long loopT = 0;
  
  setSteppersDir();  // set steppers directions
  enableMotors();  // enable stepper drivers
  
  attachInterrupts();  // enable end switches and stop button (selection button)
  Tstart = micros();  // start time for current iterration
  //digitalWrite(RdirPin, config.dir);
  for (int pr = 0; pr < config.pLength; pr++) {  // loop trough coil sections
    for (unsigned long i = 0; i < Lprofile[pr]; i++) {  // loop trough coil section rotation stepper steps
      if (interruptFlag) {  // if interuption has occured stop the loop
        break;
      }
      nextPos = (i * (Aprofile[pr] + Bprofile[pr] * i / 2)) / Rsteps;  // calculate at what position translation stepper should be
      PORTC = PORTC | B00000010;  // set rotation stepper step pin to 1 (move it one step)
      if (nextPos != currPos) {  // if current loop calculated position of 
        PORTA = PORTA | B00010000;  // set translation stepper step pin to 1 (move it one step)
        currPos = nextPos;  // set current translation stepper position
        //TODO implement different position tracking
        if (Xdir) {  // if moving in "+" direction increase Xposition by 1
          Xposition++;
        }
        else {  // if moving in "-" direction decrease Xposition by 1
          Xposition--;
        }
      }
      if (config.dir) {  // if moving in "+" direction increase Rposition by 1
        Rposition++;
      }
      else {  // if moving in "-" direction decrease Rposition by 1
        Rposition--;
      }
      delayMicroseconds(dly);  // delay minimum stepper driver step pin on/off state interval
      PORTC = PORTC & B11111101;  // set rotation stepper step pin to 0 (prepare it for next step)
      PORTA = PORTA & B11101111;  // set translation stepper step pin to 0 (prepare it for next step)
      if (abs(Rposition)+1 > accSteps) {  // if rotation stepper has finished accelerating switch to constant speed "mode"
        if (Tstart + stepTime > micros()) {  // if curent system time has not reached time set for next iteration calculate delay for starting new step
          if (micros() > Tstart) {  // check for overflow
            delayMicroseconds(Tstart + stepTime - micros());
            Tstart = Tstart + stepTime;  // calculate new Tstart for new iteration
          } else {  // if overflow has occured calculate adjust calculation - TODO change to better implementation
            delayMicroseconds(Tstart + stepTime - micros() - ULONG_MAX);
            Tstart = Tstart + stepTime - ULONG_MAX;
          }
        }
      } else {  // stay in rotation stepper acceleration "mode"
        accTime = sqrt((abs(Rposition) + 1) / initAcc) * 1e6;  // calculate acelleration step time
        if (Tstart + accTime > micros()) {  // if curent system time has not reached time set for next iteration calculate delay for starting new step
          if (micros() > Tstart) {  // check for overflow
            delayMicroseconds(Tstart + accTime - micros());
          } else { // if overflow has occured calculate adjust calculation - TODO change to better implementation
            delayMicroseconds(Tstart + accTime - micros() - ULONG_MAX);
          }
          if (abs(Rposition)+1 == accSteps) { // if this is last "acceleration made" iteration correct Tstart
            Tstart += accTime;
          }
        }
      }

    }
    if (interruptFlag) {  // if interrupt has occured stop iteration
      break;
    }
  }
  disableMotors();  // disable stepper motor drivers
  detachInterrupts();  // detach end switch interrupts
}

// "+" end switch interrupt routine
void interruptRoutineEndSwPlus(void){
  stopAnyMovement();  // emergency stop of the motors
  backoffXminusDir();  // move translation stepper for set distance in "-" direction
}

// "-" end switch interrupt routine
void interruptRoutineEndSwMinus(void){
  stopAnyMovement();  // emergency stop of the motors
  backoffXplusDir();  // move translation stepper for set distance in "+" direction
}

// select button interrupt routine
void interruptRoutineBtn(void){
  stopAnyMovement();  // emergency stop of the motors
}

// emergency stop of the motors
void stopAnyMovement(void){
  interruptFlag = true;  // let other functions know that interrupt has happened
  TIMSK2 = 0;  // stop timer2
}

// attach interrupts to end switches and selection button
void attachInterrupts(void){
  interruptFlag = false;  // lower the interrupt flag
  attachInterrupt(digitalPinToInterrupt(XplusInterrPin), interruptRoutineEndSwPlus, LOW);  // attach interrupt to "+" end switch
  attachInterrupt(digitalPinToInterrupt(XminusInterrPin), interruptRoutineEndSwMinus, LOW);  // attach interrupt to "-" end switch
  attachInterrupt(digitalPinToInterrupt(BTNinterrPin), interruptRoutineBtn, LOW);  // attach interrupt to selection button
}

// dettach interrupts to end switches and selection button
void detachInterrupts(void){
  detachInterrupt(digitalPinToInterrupt(XplusInterrPin));  // dettach interrupt to "+" end switch
  detachInterrupt(digitalPinToInterrupt(XminusInterrPin));  // dettach interrupt to "-" end switch
  detachInterrupt(digitalPinToInterrupt(BTNinterrPin));  // dettach interrupt to selection button
  interruptFlag = false;  // lower the interrupt flag
}

// move translation stepper for set distance in "-" direction 
void backoffXminusDir(void){
  digitalWrite(XdirPin, LOW);  // set translation stepper dir pin to low: "-" direction
  backoffX();  // move the stepper for set distance
  Xposition -= XbackOffSteps;  // correct translation motor position
}

// move translation stepper for set distance in "+" direction
void backoffXplusDir(void){
  digitalWrite(XdirPin, HIGH);  // set translation stepper dir pin to high: "+" direction
  backoffX();  // move the stepper for set distance
  Xposition += XbackOffSteps;  // correct translation motor position
}

void backoffX(void){
  enableMotors();  // enable stepper motor drivers
  for(int i=0; i<XbackOffSteps; i++){
    digitalWrite(XstepPin, HIGH);  // set translation stepper step pin to 1 (move it one step)
    digitalWrite(XstepPin, LOW);  // set translation stepper step pin to 0 (prepare it for next step)
    delayMicroseconds(retStepTime / (float)retVelPercent * 100);  // delay for achieve ~return speed
  }
  disableMotors();  // disable stepper motor drivers
}


// transform coil profile to software frendly form 
void transformProfile(void) {
  float l;  // length of coil section
  float ks;  // start pitch of coil section
  float ke;  // end pitch of coil section
  float coil;    // nr. of windings in a coil section
  for (int pr = 0; pr < config.pLength; pr++) {  // loop trough coil sections
    l = config.lengths[pr]*config.K;  // calculate elasticaly adjusted coil section length
    ks = config.startK[pr]*config.K;  // calculate elasticaly adjusted coil start pitch
    ke = config.endK[pr]*config.K;  // calculate elasticaly adjusted coil end pitch
    if (ks == ke) {  // if pitch is constant
      Bprofile[pr] = 0;  // set variable parameter to 0
      coil = l / ks;  // calculate nr. of windings
    } else {
      Bprofile[pr] = Xsteps * sq(r) / (Rsteps * Xd) * (ke - ks) * (ks + ke) / (2 * l); //(2*l*Rsteps);  // calculate variable parameter
      coil = 2 * l / (ks + ke);  // calculate nr. of windings
    }
    Aprofile[pr] = ks * r / Xd * Xsteps;  // calculate constant parameter
    Lprofile[pr] = round(coil * Rsteps / r);  // calculate coil section nr. of steps for rotation motor
  }
}


// go to steppers initial positions
void goToInitPos(void) {
  attachInterrupts();  // attach end switch and select button interrupts
  enableMotors();  // enable stepper motor drivers 
//  Serial.print("Xpos:\t");
//  Serial.println(Xposition);
//  Serial.print("Rpos:\t");
//  Serial.println(Rposition);
  if (Xposition > 0) {  // if translation motor is on "+" side set returning direction to "-"
    digitalWrite(XdirPin, LOW);
  } else {  // if translation motor is on "-" side set returning direction to "+"
    digitalWrite(XdirPin, HIGH);
  }
  
  Rposition = abs(Rposition) % fullRotSteps * sgn(Rposition);  // get relative position of rotation stepper
  if (Rposition >= 0) {
    if (Rposition > fullRotSteps / 2) { // if position of rotation stepper is over half of full "+" rotation 
      Rposition -= fullRotSteps;  // correct its position
      digitalWrite(RdirPin, HIGH);  // set rotation direction to "+"
    } else {
      digitalWrite(RdirPin, LOW); // set rotation direction to "-"
    }
  } else {
    if (Rposition < -1 * (fullRotSteps / 2)) { // if position of rotation stepper is over half of full "-" rotation 
      Rposition += fullRotSteps;  // correct its position
      digitalWrite(RdirPin, LOW);  // set rotation direction to "-"
    } else {
      digitalWrite(RdirPin, HIGH);  // set rotation direction to "+"
    }
  }
//  Serial.println("Rposition:");
//  Serial.println(Rposition);
  Xposition = abs(Xposition);  // set translation motor position to absolute for looping
  Rposition = abs(Rposition);  // set rotation motor position to absolute for looping
  while (Xposition | Rposition) {  // while translation and rotation positions are not 0 stay in loop
    if(interruptFlag){  // if interrupt has occured break loop and stop motors
      break;
    }
    if (Xposition) {  // if traslation motor postition is not 0, move it
      digitalWrite(XstepPin, HIGH);
      digitalWrite(XstepPin, LOW);
      Xposition--;  // corect its position
    }
    if (toggle) {  // toggle condition to run rotation stepper every 2 iterations to move slower (phisical limitations of the used motor)
      if (Rposition) {  // if rotation motor postition is not 0, move it
        digitalWrite(RstepPin, HIGH);
        digitalWrite(RstepPin, LOW);
        Rposition--;  // corect its position
        toggle = false;  // set toggle flag for next loop
      }
    } else {
      toggle = true;  // set toggle flag for next loop
    }
    delayMicroseconds(retStepTime / retVelPercent * 100);  // simple delay for acheiving set ~retrun speed
  }
  disableMotors();  // disable stepper motor drivers 
  detachInterrupts();  // dettach end switch and select button interrupts
}

// function to return sign of a number
static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

// populate programList with user program names from the SD card
void readProgramNames(char* programList) {
  char programL[] = "Izhod";  // set first item in menu to be "Exit" option
  int totLen = strlen(programL);  // total length of used programList array
  int fnLen;  // length of a filename
  for(int i=0; i < totLen; i++){  // populate programList with first menu item
    programList[i] = programL[i];
  }
  char fileName[20];  // filename of a program file
  if (sd.begin(SD_CONFIG)) {  // if SD card is available
    if (!settingsDir.open("programs/")){  
      error("dir.open failed");
    }else{  // if "programs/" directory is avalable on SD
      int nrProg = 0;  // number of available programs
      while (settingsFile.openNext(&settingsDir, O_RDONLY)) {  // iterate trough "programs/" directory content
        if (!settingsFile.isDir()) {  // if settingsFile is not a directory
          programList[totLen] = '\n';  // separate menu items with newline character
          totLen++;  // increase otal length of used programList array by 1
          settingsFile.getName(fileName, sizeof(fileName));  // populate  fileName variable with name of the file
          settingsFile.close();  // close file
          fnLen = strlen(fileName)-4;  // get length of file name without file extension
          for(int i=0; i < fnLen; i++){  // populate programList with file name
            programList[i+totLen] = fileName[i];
          }
          totLen += fnLen;  // adjust total programList length
          nrProg++;  // inrease number of available programs by 1
          if(nrProg==10){ // if number of programs reaches max menu len break loop - can be bigger with increasing programList length
            break;
          }
        }
      }
      settingsDir.close();  // close "programs/" directory
    }
  }
  programList[totLen] = '\0';  // end programList with null terminator
  //programList = programL;
}

// enable stepper motor drivers 
void enableMotors(void) {
  digitalWrite(XenabPin, LOW);
  digitalWrite(RenabPin, LOW);
}

// disanable stepper motor drivers 
void disableMotors(void) {
  digitalWrite(XenabPin, HIGH);
  digitalWrite(RenabPin, HIGH);
}

// set stepper motor directions
void setSteppersDir(void) {
  if (config.dir) {
    digitalWrite(RdirPin, HIGH);
  }
  else {
    digitalWrite(RdirPin, LOW);
  }
  if (Xdir) {
    digitalWrite(XdirPin, HIGH);
  }
  else {
    digitalWrite(XdirPin, LOW);
  }
}

// Loads the configuration from a file
void loadSettings(File setFile, Config &config) {
  // Allocate a temporary JsonDocument
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<610> doc;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, setFile);//file);
 
  if (error){
    Serial.println(F("loadJson: Failed to read file, using default configuration"));
  }
  else{
    Serial.println(F("loadJson: Succeeded to read file"));
  }
  // Copy values from the JsonDocument to the Config
  config.pLength = doc["pLength"] | 0;  // read "pLength" from file
  config.dir = doc["dir"]|0;  // read "dir" from file
  config.K = doc["K"] | 1.0;  // read "K" from file
//  Serial.println("l\tsK\teK");
//  for(int i=0; i<maxArrLen; i++){
//    config.lengths[i] = doc["lengths"][i]|0.00;
//    config.startK[i] = doc["startK"][i]|0.00;
//    config.endK[i] = doc["endK"][i]|0.00;
//    Serial.print(config.lengths[i]);
//    Serial.print('\t');
//    Serial.print(config.startK[i]);
//    Serial.print('\t');
//    Serial.println(config.endK[i]);
//  }
}

// load selected file from SD
void loadProgram(Config &config){
  if (sd.begin(SD_CONFIG)) {  // if SD card is available
    int i = 0;  // variable to count non directory items
    if (!settingsDir.open("programs/")){  // if "programs/" directory is not avalable on SD
      error("dir.open failed");
    }else{
      while (settingsFile.openNext(&settingsDir, O_RDONLY)) {  // loop trough directory items
        if (!settingsFile.isDir()) {  // if item is not directory increase i
          i++;  
        }
        if(i<prog_selection){  // if i is lower than selected program number close file
          settingsFile.close();
        }else{  // if i is equal to selected program number load settings, close settingsFile and settingsDir and exit loop
          loadSettings(settingsFile, config);
          settingsFile.close();
          settingsDir.close();
          break;
        }
      }
    }
  }
}
