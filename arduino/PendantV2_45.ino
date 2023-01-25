
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
#include <Wire.h>
#include <RotaryEncoder.h>
#include <PinButton.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>      // Hardware-specific library

#define EEPROM_SIZE 1

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

BluetoothSerial SerialBT;
RotaryEncoder *encoderA = nullptr;

// I/O pindefinitions
#define PIN_REM1 GPIO_NUM_13
#define PIN_REM2 GPIO_NUM_15
#define PIN_JSX GPIO_NUM_32
#define PIN_JSY GPIO_NUM_33
#define PIN_REMS GPIO_NUM_2
#define PIN_YELLOW GPIO_NUM_25
#define PIN_GREEN GPIO_NUM_26
#define PIN_RED GPIO_NUM_27
#define PIN_JS_VCC GPIO_NUM_17
#define PIN_BATT 34
#define ADC_EN  14

#define TFT_LEFT    1
#define TFT_CENTER  2
#define TFT_RIGHT   3
#define TFT_TRANSP  true
#define TFT_OBLQ  false
#define TFT_LIGHTBLUE 0x86FF
#define TFT_LIGHTRED 0xF9E7
#define TFT_DARKBLUE 0x0018
#define TFT_OVERYELLOW 0xFF35
#define TFT_MEDIUMGREEN 0x04A2
#define TFT_DARKRED 0xA800
#define TFT_MEDIUMBLUE 0x3D5F
#define TFT_DARKORANGE 0xFAC0

#define MAX_JOGSTR 50
#define MAX_STATUSSTR 50
#define POWEROFF_TIME 300
#define LINEHEIGHT_DEF  22

enum machinestate {
  Unknwn,
  Alarm,
  Idle,
  Jog,
  Home,
  Check,
  Run,
  Cycle,
  Hold,
  Door,
  Sleep
};
enum pendantstate {
  Menu,
  MenuConfirm,
  Pendant,
  RotateChoice,
  MenuSpindleSpeed,
  MenuBrightness
};

enum menuOptions {
  poweroff,
  spindlespeed,
  spindleonoff,
  steps,
  quit,
  home,
  setwxy0,
  setwz0,
  probe,
  gawxy0,
  unlock,
  resetgrbl,
  brightness
};

enum rotorDestin  {
  RotXaxis,
  RotYaxis,
  RotZaxis,
  RotSpindlespeed,
  RotJogspeed,
  RotOvfeed,
  RotOvspeed,
  RotOvrapid
};

enum joystickDestin {
  jsXYaxis,
  jsZaxis
};

machinestate mState = Unknwn;
pendantstate pState = Pendant;
menuOptions menuState = quit;
rotorDestin rState = RotJogspeed;
joystickDestin jState = jsXYaxis;

const int maxMenu = 12;
const int jogSpeedMax = 5000; //dit moet gelijk lopen met de settings in config.yaml
static char mStateStr[][11] { "? ?", "ALARM", "IDLE", "JOG", "HOME", "CHECK", "RUN", "CYCLE", "HOLD", "SAFE", "SLEEP" };
const int powerOffTime = 60;
bool powerOffNow = false;

String name = "FluidNC";
//String name = "btgrblesp";
//uint8_t address[6]  = {0xA5, 0xFE, 0x06, 0x00, 0xFF, 0xFF};
//uint8_t address[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t address[6] = {0xC8, 0xC9, 0xA3, 0xD2, 0xA5, 0xFE};
bool btConnected;
const char *ssid = "TWICKEL2.4";
const char *password = "Ahkvdmdp";


char jogStr[MAX_JOGSTR];
char statusStr[MAX_STATUSSTR];
char sbuf[20];
byte menuChoice = 0, backLight = 1;
unsigned long lastJogCmdXYZ = 0, lastStepCmdXYZ = 0;
const int tExec = 1000;
const int updateInterval = 198;
float distX = 0, distY = 0, distZ = 0;
int battVolt;
unsigned long sinceStart, lastGrblState;
int rdX = 0, rdY = 0, rdZ = 0, multXYZ = 0;
int calibrateX, calibrateY;
float mX, mY, mZ;  //Machine positions
float wX, wY, wZ;  //Work positions
RTC_DATA_ATTR int jogSpeed = 1000; //Overleeft een herstart vanuit sleep
int jogSpeedDisp = 0, setSpindleSpeed = 3000, spindleDisp = -1;
int reportedSpindleSpeed = 0;
int ovSpeed = 0, ovFeed = 0, ovRapid = 0;
bool spindleOn = false;
String grblState;
bool smallSteps = false;
int stepXYZ = 0;
PinButton buttonREMS(PIN_REMS);
PinButton buttonGREEN(PIN_GREEN);
PinButton buttonYELLOW(PIN_YELLOW);
PinButton buttonRED(PIN_RED);

void setup() {
  //  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
  analogReadResolution(9);  //minimum is 9 bits dus waarden 0 en 512;
  backLight = EEPROM.read(0) % 16;
  if (backLight < 1) backLight = 1;
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  //zet de helderheid van het backlight
  ledcSetup(0, 5000, 8); // 0-15, 5000, 8
  ledcAttachPin(TFT_BL, 0); // TFT_BL, 0 - 15
  ledcWrite(0, 64); // 0-15, 0-255 (with 8 bit resolution)
  tftPrintSimple(20, 2 * LINEHEIGHT_DEF, "(OTA)");
  tftPrintSimple( 20, 3 * LINEHEIGHT_DEF, "Connecting...");
  if (digitalRead(PIN_RED) == LOW) otaLoop();
  ledcWrite(0, backLight * backLight); // 0-15, 0-255 (with 8 bit resolution)
  pinMode(PIN_JSX, INPUT);
  pinMode(PIN_JSY, INPUT);
  pinMode(PIN_BATT, INPUT);
  pinMode(PIN_REMS, INPUT_PULLUP);
  //  pinMode(PIN_JSS,INPUT_PULLUP);
  pinMode(PIN_JS_VCC, OUTPUT);
  digitalWrite(PIN_JS_VCC, HIGH);
  grblState.reserve(100);
  tftUpdate(true);
  /*
    xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,0);
    vTaskDelay(500);
  */
  encoderA = new RotaryEncoder(PIN_REM1, PIN_REM2, RotaryEncoder::LatchMode::FOUR3);
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_REM1), checkPositionA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_REM2), checkPositionA, CHANGE);
  activeTimer(true);
  // Calibrate Joystick
  for (int n = 100; n; --n) {
    calibrateX += analogRead(PIN_JSX);
    calibrateY += analogRead(PIN_JSY);
    delay(1);
  }
  calibrateX /= 100;
  calibrateY /= 100;
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  SerialBT.begin("ESP32test", true);
  SerialBT.setTimeout(5000);
  SerialBT.register_callback(btCallback);
  if (!SerialBT.connect(name)) {
    tftPrint(TFT_LEFT, TFT_OBLQ, TFT_RED, TFT_BLACK, 20, 2 * LINEHEIGHT_DEF, 0, "No connection");
    tftPrint(TFT_LEFT, TFT_OBLQ, TFT_RED, TFT_BLACK, 20, 3 * LINEHEIGHT_DEF, 0, "Shutting down");
    delay(2000);
    esp_deep_sleep_start();
  }
  Serial.println("Connected Succesfully to " + name);
  getGrblState(false);
}


void otaLoop() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  tft.fillScreen(TFT_BLACK);
  tftPrintSimple( 0, 1 * LINEHEIGHT_DEF, "WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println(WiFi.localIP());
  tftPrintSimple( 0, 2 * LINEHEIGHT_DEF, "OTA");
  ArduinoOTA.setHostname("ESP32-pendant");
  ArduinoOTA.begin();
  while (true) {
    ArduinoOTA.handle();
    delay(1000);
    checkBattery();
    sprintf(sbuf, "Batt: %d", (int) battVolt);
    tftPrintSimple( 100, 4 * LINEHEIGHT_DEF, sbuf);
  }
}

IRAM_ATTR void checkPositionA() {
  encoderA->tick();  // just call tick() to check the state.
}

void forceEndJog() {
  Serial.println("C");
  btWrite(0x85);
  //  Serial.println("cancel");
  delay(100);
  getGrblState(false);
  while (mState == Jog) {
    Serial.println("D");
    btWrite(0x85);
    //    Serial.println("cancel");
    delay(100);
    getGrblState(true);
  }
}

void btWrite(char c) {
  if (btConnected) {
    SerialBT.write(c);
  }
  else mState = Unknwn;
}

void waitEndJog() {
  getGrblState(false);
  while (mState == Jog) {
    delay(100);
    getGrblState(true);
    //    Serial.print("jog wait...");
    //    Serial.println(mState);
  }
}

const int rdRawMin = -30;
const int rdRawMax = 30;
const int hysteresis = 10;

int readJSX() {
  int rdRaw = 0;
  for (int n = 10; n; --n) {
    rdRaw += analogRead(PIN_JSX) - calibrateX;
  }
  rdRaw /= 10;
  // soft hysteresis
  if (rdRaw < rdRawMin + hysteresis && rdRaw > rdRawMin && rdX == -1) rdRaw = rdRawMin;
  else if (rdRaw > rdRawMax - hysteresis && rdRaw < rdRawMax && rdX == 1) rdRaw = rdRawMax;
  return (rdRaw);
}

int readJSY() {
  int rdRaw = 0;
  for (int n = 10; n; --n) {
    rdRaw += analogRead(PIN_JSY) - calibrateY;
  }
  rdRaw /= 10;
  // soft hysteresis
  if (rdRaw < rdRawMin + hysteresis && rdRaw > rdRawMin && rdY == 1) rdRaw = rdRawMin;
  else if (rdRaw > rdRawMax - hysteresis && rdRaw < rdRawMax && rdY == -1) rdRaw = rdRawMax;
  return (rdRaw);
}

int checkJoystick() {
  static int rdXprev = 0, rdYprev = 0, rdZprev = 0;
  static float sX, sY, sZ, speedXYZ;
  /*
    if (readJSY() <= rdRawMin || readJSY() >= rdRawMax ||
        readJSX() <= rdRawMin || readJSX() >= rdRawMax)
      checkConnectBt();
  */
  if (mState != Idle || pState != Pendant || rState == RotSpindlespeed) return 0;

  while (true) {
    sinceStart = millis() + tExec;
    checkRotateA();
    rdX = 0;
    rdY = 0;
    rdZ = 0;
    if (jState == jsZaxis) {
      if (readJSY() <= rdRawMin) rdZ = 1;
      else if (readJSY() >= rdRawMax) rdZ = -1;
    } else {
      if (readJSX() <= rdRawMin) rdX = -1;
      else if (readJSX() >= rdRawMax) rdX = 1;
      if (readJSY() <= rdRawMin) rdY = 1;
      else if (readJSY() >= rdRawMax) rdY = -1;
    }
    if (rdX == 0 && rdY == 0 && rdZ == 0) {
      if (mState == Jog) {
        forceEndJog();
        lastJogCmdXYZ = 0;
      }
      return 0;
    }
    activeTimer(true);
    if (rdX != rdXprev || rdY != rdYprev || rdZ != rdZprev) {
      rdXprev = rdX;
      rdYprev = rdY;
      rdZprev = rdZ;
      btWrite(0x85);
      lastJogCmdXYZ = 0;
    }
    if (sinceStart - lastGrblState > updateInterval) {
      //      Serial.println("A");
      getGrblState(true);
    }
    multXYZ = jogSpeed > jogSpeedMax ? jogSpeedMax : jogSpeed;
    if (sinceStart - lastJogCmdXYZ > tExec) {
      sX = abs(rdX * rdX) * multXYZ;
      sY = abs(rdY * rdY) * multXYZ;
      sZ = abs(rdZ * rdZ) * multXYZ;
      //      if (sX>=sY) sY=0; else sX=0; //alleen via X of via Y bewegen
      if (sZ) speedXYZ = sZ;
      else speedXYZ = sqrt(sX * sX + sY * sY);
      distX = ((jogSpeedMax - sX) / 200000000 + 0.000018) * sX * (rdX ? (rdX < 0 ? -1 : 1) : 0) * tExec;
      distY = ((jogSpeedMax - sY) / 200000000 + 0.000018) * sY * (rdY ? (rdY < 0 ? -1 : 1) : 0) * tExec;
      distZ = ((jogSpeedMax - sZ) / 200000000 + 0.000018) * sZ * (rdZ ? (rdZ < 0 ? -1 : 1) : 0) * tExec;
      snprintf(jogStr, MAX_JOGSTR, "$J=G21G91X%fY%fZ%fF%d", distX, distY, distZ, (int)speedXYZ);
      lastJogCmdXYZ = sinceStart;
      btPrintln(jogStr);
      Serial.println(jogStr);
      delay(10);
      getGrblState(true);
      mState = Jog; //force
    }
  }
}

void checkRotateA() {
  static int posPrev = 0, pos, dir;
  //  encoderA->tick(); // just call tick() to check the state.
  pos = encoderA->getPosition();
  if (posPrev == pos) return;
  activeTimer(true);
  posPrev = pos;
  dir = (int)encoderA->getDirection();
  if (pState == RotateChoice) {
    tftUpdate(true);
    if (dir == 1) {
      if (rState == RotOvrapid) rState = RotOvspeed;
      else if (rState == RotOvspeed) rState = RotOvfeed;
      else if (rState == RotOvfeed) rState = RotJogspeed;
      else if (rState == RotJogspeed) rState = RotSpindlespeed;
      else if (rState == RotSpindlespeed) rState = RotZaxis;
      else if (rState == RotZaxis) rState = RotYaxis;
      else rState = RotXaxis;
    } else {
      if (rState == RotXaxis) rState = RotYaxis;
      else if (rState == RotYaxis) rState = RotZaxis;
      else if (rState == RotZaxis) rState = RotSpindlespeed;
      else if (rState == RotSpindlespeed) rState = RotJogspeed;
      else if (rState == RotJogspeed) rState = RotOvfeed;
      else if (rState == RotOvfeed) rState = RotOvspeed;
      else rState = RotOvrapid;
    }
    tftUpdate(true);
  } else if (pState == Pendant && digitalRead(PIN_GREEN)==HIGH) {
    if (rState < 3 || pState == RotateChoice) { //een van de assen
      static unsigned long lastStepCmd = 0;
      float dist = 0;
      if (dir == 1) dist = 0.001 * (jogSpeed > 5000 ? 5000 : jogSpeed);
      else dist = -0.001 * (jogSpeed > 5000 ? 5000 : jogSpeed);
      switch (rState) {
        case RotZaxis:
          snprintf(jogStr, MAX_JOGSTR, "$J=G21G91X0Y0Z%fF%d", dist, 1000);
          break;
        case RotYaxis:
          snprintf(jogStr, MAX_JOGSTR, "$J=G21G91X0Y%fZ0F%d", dist, 1000);
          break;
        case RotXaxis:
          snprintf(jogStr, MAX_JOGSTR, "$J=G21G91X%fY0Z0F%d", dist, 1000);
          break;
        default:
          break;
      }
      if (pState == RotateChoice) snprintf(jogStr, MAX_JOGSTR, "$J=G21G91X0Y0Z%fF%d", dist, 1000);
      lastStepCmd = sinceStart;
      btPrintln(jogStr);
      Serial.println(jogStr);
      while (sinceStart - lastStepCmd < (20 + (jogSpeed > 5000 ? 5000 : jogSpeed) / 20)) {
        sinceStart = millis() + tExec;
        encoderA->tick();
      }
    }
    else if (rState == RotOvfeed) {
      btWrite(dir == 1 ? 0x91 : 0x92);
    }
    else if (rState == RotOvspeed) {
      btWrite(dir == 1 ? 0x9A : 0x9B);
    }
    else if (rState == RotOvrapid) {
      btWrite(dir == 1 ? (ovRapid < 50 ? 0x96 : 0x95) : (ovRapid > 50 ? 0x96 : 0x97));
    }
    else if (rState == RotSpindlespeed) {
      if (dir == 1) {
        if (setSpindleSpeed < 3000) {
          setSpindleSpeed = 3000;
        }
        else {
          setSpindleSpeed += 1000;
          if (setSpindleSpeed > 24000) setSpindleSpeed = 24000;
        }
      } else {
        setSpindleSpeed -= 1000;
        if (setSpindleSpeed < 3000) {
          setSpindleSpeed = 3000;
        }
      }
      sprintf(sbuf, "M%d S%d", spindleOn ? 3 : 5, setSpindleSpeed);
      btPrintln(sbuf);
      if (!spindleOn) {
        sprintf(sbuf, "M5 S%d", setSpindleSpeed);
        btPrintln(sbuf);
      }
      getGrblState(true);
      reportedSpindleSpeed = setSpindleSpeed; //tot we beter weten
      tftUpdate(false);
    } else { //rState is RotJogspeed
      forceEndJog();
      lastJogCmdXYZ = 0;
      if (dir == 1) {
        if (smallSteps) {
          if (jogSpeed <= 20) ++jogSpeed;
          else jogSpeed *= 1.05;
        } else {
          jogSpeed *= 2.5;
          if (jogSpeed >= 1000) {
            jogSpeed = (jogSpeed / 1000) * 1000;
          } else if (jogSpeed >= 100) {
            jogSpeed = (jogSpeed / 100) * 100;
          } else if (jogSpeed >= 10) {
            jogSpeed = (jogSpeed / 10) * 10;
          }
        }
      } else {
        if (smallSteps) {
          if (jogSpeed <= 20) --jogSpeed;
          else jogSpeed /= 1.05;
        } else if (jogSpeed > 5000) jogSpeed = 5000;
        else {
          jogSpeed /= 2;
          if (jogSpeed >= 1000) {
            jogSpeed = (jogSpeed / 1000) * 1000;
          } else if (jogSpeed >= 100) {
            jogSpeed = (jogSpeed / 100) * 100;
          } else if (jogSpeed >= 10) {
            jogSpeed = (jogSpeed / 10) * 10;
          }
        }
      }
      if (jogSpeed < 2) jogSpeed = 2;
      if (jogSpeed > jogSpeedMax) jogSpeed = jogSpeedMax;
      tftUpdate(false);
    }
  } else if (pState == Menu) {
    if (dir == 1) {
      if (menuChoice > 0) menuChoice--;
    } else if (menuChoice < maxMenu) menuChoice++;
    tftUpdate(false);
  } else if (pState == MenuSpindleSpeed) {
    if (dir == 1) {
      setSpindleSpeed += 1000;
      if (setSpindleSpeed > 24000) setSpindleSpeed = 24000;
    } else {
      setSpindleSpeed -= 1000;
      if (setSpindleSpeed < 3000) setSpindleSpeed = 3000;
    }
  } else if (pState == MenuBrightness) {
    if (dir == 1) {
      if (backLight < 16) {
        backLight++;
      }
    } else if (backLight > 1) {
      backLight--;
    }
    ledcWrite(0, backLight * backLight); // 0-15, 0-255 (with 8 bit resolution)
    tftUpdate(true);
  }
}


void checkConnectBt() {
  static int try_ = 5;
  if (btConnected) {
    try_ = 5;
    return;
  }
  SerialBT.connect();
  if (!SerialBT.connect()) --try_;
  if (try_ == 0) {
    tftPrint(TFT_LEFT, TFT_OBLQ, TFT_RED, TFT_BLACK, 20, 2 * LINEHEIGHT_DEF, 0, "No connection");
    tftPrint(TFT_LEFT, TFT_OBLQ, TFT_RED, TFT_BLACK, 20, 3 * LINEHEIGHT_DEF, 0, "Shutting down");
    delay(2000);
    ESP.restart();
  }
}

void checkBattery() {
  float b = (analogRead(PIN_BATT) - 180) / 1.9;
  if (b > 100) b = 100;
  if (b > battVolt + 1) battVolt += (b - battVolt) / 2;
  if (b < battVolt - 1) battVolt -= (battVolt - b) / 2;
}

void loop() {
  sinceStart = millis() + tExec;
  activeTimer(false);
  checkRotateA();
  if (sinceStart - lastGrblState > updateInterval) {
    checkBattery();
    getGrblState(true);
    checkConnectBt();
    tftUpdate(false);
  }
  checkJoystick();
  if (rdX == 0 && rdY == 0 && rdZ == 0) {
    buttonREMS.update();
    buttonYELLOW.update();
    buttonGREEN.update();
    buttonRED.update();

    if (buttonYELLOW.isClick()) {
      activeTimer(true);
    }
    if (buttonYELLOW.isSingleClick()) {
    }
    if (buttonYELLOW.isLongClick() ) {
      activeTimer(true);
      if (mState == Door && pState == Pendant) { //Alleen resetten vanuit de pemdant-state en niet vanuit menu
        btWrite(24); //Reset
        getGrblState(false);
      } else {
        btWrite(24); //Eerst een reset
        btPrintln((char *)"$X"); //als er een alarm hangt accepteert de controller geen commando's meer, dus dat eerst oplossen met unlock
        btWrite(132); //Door
        pState = Pendant;
        getGrblState(false);
      }
    }
    if (buttonRED.isClick() && rState != RotSpindlespeed) {
      activeTimer(true);
      jState = jsZaxis;
      tftUpdate(true);
    }
    if (buttonRED.isReleased() && rState != RotSpindlespeed) {
      jState = jsXYaxis;
      tftUpdate(true);
    }
    if (buttonRED.isSingleClick()) {
      if (pState != Pendant) {
        pState = Pendant;
        tftUpdate(true);
      }
      else {
        if (mState == Alarm) {
          btPrintln((char *)"$X"); //Unlock
        }
        else if (mState == Run) btWrite('!');
        else if (mState == Hold) btWrite('~');
      }
    }
    if (buttonRED.isLongClick()) {
      if (pState==Menu) powerOffNow=true;
    }
    if (buttonRED.isClick() && pState == Pendant && rState == RotSpindlespeed) {
      sprintf(sbuf, "M%d S%d", spindleOn ? 5 : 3, setSpindleSpeed);
      btPrintln(sbuf);
      getGrblState(true);
    }
    if (buttonGREEN.isLongClick() && pState == Pendant) {
      pState = RotateChoice;
      rState = RotJogspeed;
      tftUpdate(true);
    }
    if (buttonGREEN.isReleased() && pState == RotateChoice) {
      pState = Pendant;
      tftUpdate(true);
    }
    if (buttonREMS.isSingleClick() || (buttonGREEN.isSingleClick())) {
      //Serial.println((String)pState + " " + (String)menuChoice);
      activeTimer(true);
      if (pState == Pendant) {
        pState = Menu;
        menuChoice = quit;
      } else if (pState == Menu) {
        switch (menuChoice) {
          case poweroff:
            powerOffNow=true;
            break;
          case brightness:
            pState = MenuBrightness;
            break;
          case spindleonoff:
            btPrintln(sbuf);
            pState = Pendant;
            break;
          case spindlespeed:
            spindleDisp = setSpindleSpeed;
            pState = MenuSpindleSpeed;
            break;
          case steps:
            smallSteps = !smallSteps;
            pState = Pendant;
            break;
          case unlock: //Unlock
            pState = Pendant;
            btPrintln((char *)"$X");
            break;
          case home: //Home
            pState = Pendant;
            btPrintln((char *)"$H");
            break;
          case resetgrbl: //Reset
            pState = Pendant;
            btWrite(24);
            break;
          case setwxy0:  //XY=0
            pState = MenuConfirm;
            break;
          case setwz0:  //Z=0
            pState = MenuConfirm;
            break;
          case probe:  //probe
            btPrintln((char *)"G21G91");
            btPrintln((char *)"G38.2 Z-10F100");
            btPrintln((char *)"G0Z0.3");
            btPrintln((char *)"G38.2Z-2F10");
            btPrintln((char *)"G10 L20 P1 Z21.35");
            btPrintln((char *)"G91");
            btPrintln((char *)"G0 Z8.65");
            btPrintln((char *)"G90");
            pState = Pendant;
            break;
          case gawxy0:  //naar wXY=0
            //            Serial.println("Restore origin");
            pState = Pendant;
            tftUpdate(true);
            btPrintln((char *)"$J=G53Z0F1000");  //Spindle optrekken
            delay(100);
            waitEndJog();
            btPrintln((char *)"$J=X0 Y0 F5000");
            delay(100);
            getGrblState(false);
            delay(100);
            waitEndJog();
            break;
          default:
          case quit:
            pState = Pendant;
            break;
        }
        //        tft.fillScreen(TFT_BLACK);
      } else if (pState == MenuConfirm) {
        if (menuChoice == setwxy0) {
          btPrintln((char *)"G10 P1 L20 X0 Y0");
        } else if (menuChoice == setwz0) {
          btPrintln((char *)"G10 P1 L20 Z0");
        }
        getGrblState(false);
        pState = Pendant;
      } else if (pState == MenuSpindleSpeed) {
        sprintf(sbuf, "S %d", setSpindleSpeed);
        btPrintln(sbuf);
        pState = Pendant;
      } else if (pState == MenuBrightness) {
        EEPROM.write(0, backLight);
        EEPROM.commit();
        pState = Pendant;
      }
      tftUpdate(true);
    }
  } else {
  }
  //  Serial.print("."); //Check of er puntjes overblijven in de loop
}

void printAxisCoords(float m, float w, byte row) {
  tft.fillRect(18, row * LINEHEIGHT_DEF + 2, 164, LINEHEIGHT_DEF, TFT_BLACK); //clear to eol
  if (m > 0) tftPrint(TFT_LEFT, TFT_TRANSP, TFT_LIGHTBLUE, TFT_BLACK, 18, row * LINEHEIGHT_DEF, 0, "+");
  else if (m < 0) tftPrint(TFT_LEFT, TFT_TRANSP, TFT_LIGHTBLUE, TFT_BLACK, 20, row * LINEHEIGHT_DEF, 0, "-");
  sprintf(sbuf, "%03d.%03d", (int)abs(m) % 1000, (int)(fabsf(m) * 1000) % 1000);
  tftPrint(TFT_LEFT, TFT_TRANSP, TFT_LIGHTBLUE, TFT_BLACK, 27, row * LINEHEIGHT_DEF + 2, 0, sbuf);
  if (w > 0) tftPrint(TFT_LEFT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 98, row * LINEHEIGHT_DEF, 0, "+");
  else if (w < 0) tftPrint(TFT_LEFT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 100, row * LINEHEIGHT_DEF, 0, "-");
  sprintf(sbuf, "%03d.%03d", (int)abs(w) % 1000, (int)(fabsf(w) * 1000) % 1000);
  tftPrint(TFT_LEFT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 108, row * LINEHEIGHT_DEF + 2, 0, sbuf);
}

void btPrintln(char *s) {
  if (btConnected) {
    SerialBT.println(s);
  }
  else mState = Unknwn;
}

void printCircleBlue (byte x, byte y) {
  tft.fillCircle(x, y, 8, pState == RotateChoice ? TFT_MEDIUMBLUE : TFT_DARKBLUE);
}

void tftPrintAllButtons() {
  if (pState != RotateChoice) {
    if (mState != Door) tftPrintButtonOrange("Abort");
    else if (pState == Pendant) tftPrintButtonOrange("Reset");
  }
  if (pState == Pendant) {
    tftPrintButtonGreen("Mnu/Rot");
    if ((mState == Idle || mState == Jog) && rState != RotSpindlespeed) {
      tftPrintButtonRed("XYZ");
    }
    if (mState == Alarm) tftPrintButtonRed("Unlock");
    else if (rState == RotSpindlespeed) tftPrintButtonRed("Spindle");
    else if (mState == Run) tftPrintButtonRed("Hold");
    else if (mState == Hold) tftPrintButtonRed("Resume");
  }
  else if (pState == MenuBrightness) {
    tftPrintButtonGreen("Save");
    tftPrintButtonRed("Exit");
  }
  else if (pState == Menu || pState == MenuConfirm) {
    tftPrintButtonGreen("OK");
    tftPrintButtonRed("Cancel");
  }
}

void tftPrintButtonOrange(String txt) {
  tftPrintButton(TFT_ORANGE, TFT_WHITE, 161, 112, 79, txt);
}

void tftPrintButtonGreen(String txt) {
  tftPrintButton(TFT_DARKGREEN, TFT_WHITE, 81, 112, 78, txt);
}

void tftPrintButtonRed(String txt) {
  tftPrintButton(TFT_RED, TFT_WHITE, 0, 112, 79, txt);
}

void tftPrintButton(int bc, int tc, byte col, byte row, byte len, String txt) {
  tft.fillRect(col, row, len, LINEHEIGHT_DEF + 3, TFT_BLACK);
  tft.fillRoundRect(col, row, len, LINEHEIGHT_DEF + 1, 3, bc);
  tft.fillRoundRect(col + 2, row + 2, len - 4, LINEHEIGHT_DEF - 3, 3, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(tc);
  tft.setFreeFont(&FreeSans9pt7b);
  //  tft.setFreeFont(&FreeSansBold9pt7b);
  //  tft.setFreeFont(&FreeSans12pt7b);
  tft.drawString(txt, col + len / 2, row + LINEHEIGHT_DEF / 2 - 2);
  tft.setTextDatum(TL_DATUM);
}


void tftUpdate(bool force) {
  static float mXdisp = 0, mYdisp = 0, mZdisp = 0;  //Getoonde machine positions
  static float wXdisp = 0, wYdisp = 0, wZdisp = 0;  //Getoonde work positions
  static byte menuChoiceDisp = -1, mStateDisp = -1, blDisp = 0;
  static int reportedSpindleSpeedDisp = 0, setSpindleSpeedDisp = 0, battVoltDisp = 0;
  static int ovSpeedDisp = 0, ovFeedDisp = 0, ovRapidDisp = 0;
  static bool btConnDisp = false, spindleOnDisp = false;
  static int pStateButton = -1, mStateButton = -1, rStateButton = -1, menuChoiceButton = -1;
  if (force) tft.fillScreen(TFT_BLACK);
  if (pState == Pendant || pState == RotateChoice) {
    if (mXdisp != mX || wXdisp != wX || mYdisp != mY || wYdisp != wY || mZdisp != mZ || wZdisp != wZ) activeTimer(true);
    if (mXdisp != mX || wXdisp != wX || force) {
      if (rState == RotXaxis) printCircleBlue(8, LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_LEFT, TFT_TRANSP, (jState == jsXYaxis && rState != RotSpindlespeed) ? TFT_LIGHTRED : TFT_WHITE, TFT_BLACK, 2, 0 * LINEHEIGHT_DEF + 2, 0, "X");
      printAxisCoords(mX, mX - wX, 0);
      mXdisp = mX;
      wXdisp = wX;
    }
    if (mYdisp != mY || wYdisp != wY || force) {
      if (rState == RotYaxis) printCircleBlue(8, LINEHEIGHT_DEF + LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_LEFT, TFT_TRANSP, (jState == jsXYaxis && rState != RotSpindlespeed) ? TFT_LIGHTRED : TFT_WHITE, TFT_BLACK, 2, 1 * LINEHEIGHT_DEF + 2, 0, "Y");
      printAxisCoords(mY, mY - wY, 1);
      mYdisp = mY;
      wYdisp = wY;
    }
    if (mZdisp != mZ || wZdisp != wZ || force) {
      if (rState == RotZaxis) printCircleBlue(8, LINEHEIGHT_DEF * 2 + LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_LEFT, TFT_TRANSP, (jState == jsZaxis && rState != RotSpindlespeed) ? TFT_LIGHTRED : TFT_WHITE, TFT_BLACK, 2, 2 * LINEHEIGHT_DEF + 2, 0, "Z");
      printAxisCoords(mZ, mZ - wZ, 2);
      mZdisp = mZ;
      wZdisp = wZ;
    }
    if (reportedSpindleSpeed != reportedSpindleSpeedDisp || setSpindleSpeedDisp != setSpindleSpeed || spindleOnDisp != spindleOn || force) {
      if (rState == RotSpindlespeed) printCircleBlue(8, LINEHEIGHT_DEF * 3 + LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_LEFT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 2, 3 * LINEHEIGHT_DEF + 2, 0, "S");
      sprintf(sbuf, "%03d", spindleOn ? (int)(reportedSpindleSpeed) : setSpindleSpeed);
      tftPrint(TFT_LEFT, TFT_OBLQ, spindleOn ? TFT_DARKORANGE : TFT_WHITE, TFT_BLACK, 27, 3 * LINEHEIGHT_DEF + 2, 55, sbuf);
      if (rState == RotSpindlespeed) {
        //        tftPrintButton(TFT_RED,TFT_WHITE, 80, 3 * LINEHEIGHT_DEF+2, 100, spindleOn?"Off":"On");
      }
      reportedSpindleSpeedDisp = reportedSpindleSpeed;
      setSpindleSpeedDisp = setSpindleSpeed;
      spindleOnDisp = spindleOn;
    }
    if (jogSpeedDisp != jogSpeed || force) {
      if (rState == RotJogspeed) printCircleBlue(232, 3 * LINEHEIGHT_DEF + LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_RIGHT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 228, 3 * LINEHEIGHT_DEF + 2, 10, "J");
      if (jogSpeed < 1000) snprintf(sbuf, 20, "%d/.%03d ", ((int)(jogSpeed > jogSpeedMax ? jogSpeedMax : jogSpeed)) % 10000, (int)(jogSpeed) % 1000);
      else snprintf(sbuf, 20, "%d/%d", ((int)(jogSpeed > jogSpeedMax ? jogSpeedMax : jogSpeed)) % 10000, (int)(jogSpeed > 5000 ? 5000 : jogSpeed) / 1000);
      sbuf[6] = 0;
      tftPrint(TFT_RIGHT, TFT_OBLQ, TFT_WHITE, TFT_BLACK, 218, 3 * LINEHEIGHT_DEF + 2, 60, sbuf);
      jogSpeedDisp = jogSpeed;
    }
    if (ovFeedDisp != ovFeed || force) {
      if (rState == RotOvfeed) printCircleBlue(231, 2 * LINEHEIGHT_DEF + LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_RIGHT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 228, 2 * LINEHEIGHT_DEF + 2, 10, "F");
      sprintf(sbuf, "%d", ovFeed);
      tftPrint(TFT_RIGHT, TFT_OBLQ, TFT_OVERYELLOW, TFT_BLACK, 218, 2 * LINEHEIGHT_DEF + 2, 30, sbuf);
      ovFeedDisp = ovFeed;
    }
    if (ovSpeedDisp != ovSpeed || force) {
      if (rState == RotOvspeed) printCircleBlue(231, LINEHEIGHT_DEF + LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_RIGHT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 227, LINEHEIGHT_DEF + 2, 10, "S");
      sprintf(sbuf, "%d", ovSpeed);
      tftPrint(TFT_RIGHT, TFT_OBLQ, TFT_OVERYELLOW, TFT_BLACK, 218, 1 * LINEHEIGHT_DEF + 2, 30, sbuf);
      ovSpeedDisp = ovSpeed;
    }
    if (ovRapidDisp != ovRapid || force) {
      if (rState == RotOvrapid) printCircleBlue(231, LINEHEIGHT_DEF / 2 - 1);
      tftPrint(TFT_RIGHT, TFT_TRANSP, TFT_WHITE, TFT_BLACK, 227, 2, 10, "R");
      sprintf(sbuf, "%d", ovRapid);
      tftPrint(TFT_RIGHT, TFT_OBLQ, TFT_OVERYELLOW, TFT_BLACK, 218, 2, 30, sbuf);
      ovRapidDisp = ovRapid;
    }
    if (btConnDisp != btConnected || force) {
      //      tftPrintLen(btConnected ? TFT_WHITE : TFT_RED, 190, 3 * LINEHEIGHT_DEF+2, 15, "B");
      btConnDisp = btConnected;
    }
    if (battVoltDisp != battVolt || force) {
      sprintf(sbuf, "%d", (int) battVolt);
      tftPrint(TFT_RIGHT, TFT_OBLQ, TFT_WHITE, TFT_BLACK, 218, 4 * LINEHEIGHT_DEF + 2, 20, sbuf);
      battVoltDisp = battVolt;
    }
    if (mStateDisp != mState || force) {
      sprintf(sbuf, "%s", mStateStr[mState]);
      //      tftClearAt(185,4,8);
      if (mState == Alarm || mState == Door || mState == Unknwn)
        tftPrintBig(TFT_DARKRED, TFT_BLACK, 120, 4 * LINEHEIGHT_DEF + 13, 70, sbuf);
      else if (mState == Hold)
        tftPrintBig(TFT_ORANGE, TFT_BLACK, 120, 4 * LINEHEIGHT_DEF + 13, 70, sbuf);
       else if (mState == Run || mState == Jog)
        tftPrintBig(TFT_BLACK, TFT_ORANGE, 120, 4 * LINEHEIGHT_DEF + 13, 70, sbuf);
      else tftPrintBig(TFT_BLACK, TFT_MEDIUMGREEN, 120, 4 * LINEHEIGHT_DEF + 13, 70, sbuf);
      mStateDisp = mState;
    }
  } else if (pState == Menu) {
    if (menuChoiceDisp != menuChoice || force) {
      menuChoiceDisp = menuChoice;
      tft.fillScreen(TFT_BLACK);
      static byte first = quit - 1;
      first = menuChoice > first + 4 ? menuChoice - 4 : first;
      first = menuChoice < first ? menuChoice : first;
      for (byte n = first; n < (first + 5); n++) {
        tftPrint(TFT_LEFT, TFT_OBLQ, TFT_GREEN, TFT_BLACK, 0, (menuChoice - first)*LINEHEIGHT_DEF, 0, ">");
        switch (n) {
          case poweroff:
            tftPrintColor(TFT_RED, 30, (n - first)*LINEHEIGHT_DEF, "Power off");
            break;
          case brightness:
            tftPrintSimple(30, (n - first)*LINEHEIGHT_DEF, "Brightness");
            break;
          case spindleonoff:
            sprintf(sbuf, "Spindle %s", spindleOn ? "off" : "on");
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, sbuf);
            break;
          case spindlespeed:
            sprintf(sbuf, "Spindle %d", setSpindleSpeed);
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, sbuf);
            break;
          case unlock:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Unlock");
            break;
          case home:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Home");
            break;
          case resetgrbl:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Reset Grbl");
            break;
          case setwxy0:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Set Wxy=0");
            break;
          case setwz0:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Set Wz=0");
            break;
          case probe:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Probe Z");
            break;
          case gawxy0:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Ga naar Wxy=0");
            break;
          case steps:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Steps ");
            tftPrintSimple( 150, (n - first)*LINEHEIGHT_DEF, smallSteps ? "++" : "--");
            break;
          case quit:
          default:
            tftPrintSimple( 30, (n - first)*LINEHEIGHT_DEF, "Exit");
            break;
        }
      }
    }
  } else if (pState == MenuSpindleSpeed) {
    if (spindleDisp != setSpindleSpeed || force) {
      tft.fillScreen(TFT_BLACK);
      tftPrintSimple( 30, 2 * LINEHEIGHT_DEF, "rpm:");
      sprintf(sbuf, "%d", setSpindleSpeed);
      tftPrintSimple( 30, 3 * LINEHEIGHT_DEF, sbuf);
      spindleDisp = setSpindleSpeed;
    }
  } else if (pState == MenuConfirm) {
    if (force) {
      tftPrintSimple( 80, 3 * LINEHEIGHT_DEF, "Confirm..");
    }
  } else if (pState == MenuBrightness) {
    if (blDisp != backLight || force) {
      tft.fillScreen(TFT_BLACK);
      tftPrintSimple( 60, 2 * LINEHEIGHT_DEF, "Brightness:");
      sprintf(sbuf, "%d", backLight);
      tftPrintSimple( 100, 4 * LINEHEIGHT_DEF, sbuf);
      blDisp = backLight;
    }
  }
  if (pStateButton != pState || mStateButton != mState || rStateButton != rState || menuChoiceButton != menuChoice || force)
    tftPrintAllButtons();
  pStateButton = pState;
  mStateButton = mState;
  rStateButton = rState;
  menuChoiceButton = menuChoice;
}

void tftPrint(int align, bool transp, int fg, int bg, byte col, byte row, byte len, String txt) {
  tft.setFreeFont(&FreeSans9pt7b);
  if (transp) tft.setTextColor(fg);
  else {
    tft.setTextColor(fg, bg);
    if (align == TFT_RIGHT) {
      if (len) tft.fillRect(col - len, row, len, LINEHEIGHT_DEF, TFT_BLACK);
      tft.setTextDatum(TR_DATUM);
    }
    else if (align == TFT_CENTER) {
      if (len) tft.fillRect(col - len / 2, row, len, LINEHEIGHT_DEF, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
    }
    else {
      if (len) tft.fillRect(col, row, len, LINEHEIGHT_DEF, TFT_BLACK);
      tft.setTextDatum(TL_DATUM);
    }
  }
  tft.drawString(txt, col, row);
  tft.setTextDatum(TL_DATUM);
}

void tftPrintBig(int bg, int fg, byte col, byte row, byte len, String txt) {
  tft.setTextDatum(MC_DATUM);
  tft.fillRect(col - len / 2, row - LINEHEIGHT_DEF - 10, len, 2 * LINEHEIGHT_DEF - 10 , bg);
  tft.setTextColor(fg);
  //&FreeMonoBold9pt7b
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.drawString(txt, col, row - 17);
  tft.setTextDatum(TL_DATUM);
}
void tftPrintColor(int color, byte col, byte row, String txt) {
  tftPrint(TFT_LEFT, TFT_OBLQ, color, TFT_BLACK, col, row, 0, txt);  
}

void tftPrintSimple(byte col, byte row, String txt) {
  tftPrint(TFT_LEFT, TFT_OBLQ, TFT_WHITE, TFT_BLACK, col, row, 0, txt);
}

void getGrblState(bool full) {
  char a;
  if (!btConnected) {
    mState = Unknwn;
    return;
  }
  //lees de buffer leeg
  btWrite('?');
  grblState = "";
  while (SerialBT.available() && btConnected) {
    a = SerialBT.read();
    Serial.write(a);
    if (a == '\n') {
      if (grblState.indexOf("<") >= 0) {
        grblState.remove(0, grblState.indexOf("<"));
        //Serial.println(grblState);
        if (grblState.charAt(1) == 'J') mState = Jog;
        else if (grblState.charAt(3) == 'm') mState = Home;
        else if (grblState.charAt(4) == 'd') mState = Hold;
        else if (grblState.charAt(1) == 'D') mState = Door;
        else if (grblState.charAt(1) == 'R') mState = Run;
        else if (grblState.charAt(1) == 'A') mState = Alarm;
        else if (grblState.charAt(1) == 'I') mState = Idle;
        else mState = Unknwn;
        if (full) {
          if (grblState.indexOf("MPos:") >= 0) {
            grblState.remove(0, grblState.indexOf("MPos:") + 5);
            convertPos(true, grblState);
          }
          if (grblState.indexOf("|FS:") >= 0) {
            grblState.remove(0, grblState.indexOf("|FS:") + 4);
            grblState.remove(0, grblState.indexOf(",") + 1);
            reportedSpindleSpeed = grblState.toInt();
          }
          if (grblState.indexOf("WCO:") >= 0) {
            grblState.remove(0, grblState.indexOf("WCO:") + 4);
            convertPos(false, grblState);
          }
          else if (grblState.indexOf("|Ov:") >= 0) {
            grblState.remove(0, grblState.indexOf("|Ov:") + 4);
            convertOverride(grblState);
            if (grblState.indexOf("|A:") >= 0) {
              grblState.remove(0, grblState.indexOf("|A:") + 3);
              //            Serial.println(grblState);
              if (grblState.indexOf("S") >= 0 && grblState.indexOf("S") < 3) spindleOn = true;
              else spindleOn = false;
            }
            else spindleOn = false;
          }
        }
      }
    } else {
      grblState.concat(a);
    }
  }
  if (full) tftUpdate(false);
  lastGrblState = sinceStart;
  //  Serial.println(mX);

  //  Serial.println("Grbl State: " + (String)mState);
}

void convertPos(bool isM, String s) {

  // Serial.println(s);
  static float x, y, z;
  x = s.substring(0, s.indexOf(",")).toFloat();
  s.remove(0, s.indexOf(",") + 1);
  y = s.substring(0, s.indexOf(",")).toFloat();
  s.remove(0, s.indexOf(",") + 1);
  if (s.indexOf("|") < 7) {
    z = s.substring(0, s.indexOf("|")).toFloat();
  } else {
    z = s.substring(0, s.indexOf(">")).toFloat();
  }
  if (isM) {
    mX = x;
    mY = y;
    mZ = z;
  } else {
    wX = x;
    wY = y;
    wZ = z;
  }
  //  Serial.println(wX);
  //  Serial.println(wY);
  //  Serial.println(wZ);
}

void convertOverride(String s) {
  static int feed, spd, rpd;
  feed = s.substring(0, s.indexOf(",")).toFloat();
  s.remove(0, s.indexOf(",") + 1);
  rpd = s.substring(0, s.indexOf(",")).toFloat();
  s.remove(0, s.indexOf(",") + 1);
  if (s.indexOf("|") < 7) {
    spd = s.substring(0, s.indexOf("|")).toFloat();
  } else {
    spd = s.substring(0, s.indexOf(">")).toFloat();
  }
  ovRapid = rpd;
  ovFeed = feed;
  ovSpeed = spd;
}

void activeTimer(bool reset) {
  static unsigned long aTimer = 0;
  if (reset) {
    aTimer = sinceStart;
    powerOffNow = false;
  }
  if ((sinceStart - aTimer > (POWEROFF_TIME - 10) * 1000 && battVolt < 65) || powerOffNow) {
    tft.fillScreen(TFT_BLACK);
    tftPrintSimple( 30, 4 * LINEHEIGHT_DEF, "Poweroff");
    delay(1000);
    tftUpdate(true);
    delay(1000);
    if ((sinceStart - aTimer > (POWEROFF_TIME) * 1000 && battVolt < 65) || powerOffNow) {
      tft.fillScreen(TFT_BLACK);
      digitalWrite(TFT_BL, LOW);
      tft.writecommand(TFT_DISPOFF);
      tft.writecommand(TFT_SLPIN);
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      pinMode(PIN_JS_VCC, INPUT);
      //      esp_sleep_enable_ext0_wakeup(PIN_RED, 0);
      delay(200);
      esp_deep_sleep_start();
    }
  }
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {

  switch (event)
  {
    case ESP_SPP_CLOSE_EVT://Client connection closed
      Serial.println("ESP_SPP_CLOSE_EVT");
      btConnected = false;
      break;

    case ESP_SPP_CONG_EVT://connection congestion status changed
      btConnected = false;
      Serial.println("ESP_SPP_CONG_EVT");
      break;

    case ESP_SPP_OPEN_EVT://Client connection open
      Serial.println("ESP_SPP_OPEN_EVT");
      btConnected = true;
      break;

    case ESP_SPP_DATA_IND_EVT://connection received data
    case ESP_SPP_WRITE_EVT://write operation completed
      btConnected = true;
      break;

    default:
      Serial.print("ESP_SPP_EVT: ");
      Serial.println(event);
      break;
  }
}
