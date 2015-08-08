
#include <EEPROM.h>

#define SERIAL_LOGGING 0

const int rollPin     = 0;
const int pitchPin    = 1;
const int yawPin      = 3;
const int throttlePin = 2;

const int axisPins[4] = {A0, A1, A2, A3};

const int calibrationPin = 0;
const int statusLed = 13;

bool isCalibrating = false;
bool isChangingMode = false;

int16_t axes[4];
int16_t axesMin[4];
int16_t axesMax[4];
int16_t lastCalibratedReading[4];

char msgBuf[256];

#define NUM_BUTTONS 7

int buttonPins[NUM_BUTTONS] = {5, 6, 7, 8, 9, 10, 11};
int buttonTimers[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0};
bool buttonStates[NUM_BUTTONS];
const int debounceLimit = 30;

void setup() {
  for (int i=0; i<NUM_BUTTONS; ++i) pinMode(buttonPins[i], INPUT_PULLUP);
  pinMode(calibrationPin, INPUT_PULLUP);
  pinMode(statusLed, OUTPUT);

  loadCalibrationData();

#if SERIAL_LOGGING
  Serial.begin(9600);
  while (!Serial) ;
  Serial.println("ready");
  printCalibrationData();
#endif

  // initialize joystick 
  Gamepad.begin();
}

void loop() {
  readAxes();
  readButtons();
  determineJoystickMode();
  if (isCalibrating) {
    for (int i=0; i<4; ++i) {
      int val = axes[i];
      axesMin[i] = min(val, axesMin[i]);
      axesMax[i] = max(val, axesMax[i]);
    }
    
  } else {
    int16_t newReading;
    for (int i=0; i<4; ++i) {
      newReading = calibratedAxis(i);
      lastCalibratedReading[i] = newReading;
    }
    
    Gamepad.releaseAll();
    Gamepad.xAxis(lastCalibratedReading[0]);
    Gamepad.yAxis(lastCalibratedReading[1]);
    Gamepad.rxAxis(lastCalibratedReading[2]);
    Gamepad.ryAxis(lastCalibratedReading[3]);
    for (int i=0; i<NUM_BUTTONS; ++i) {
      if(buttonStates[i]) {
        Gamepad.press(i+1);
      }
    }
    Gamepad.write();
    
//#if SERIAL_LOGGING
//      sprintf(msgBuf, "x:%d y:%d rx:%d ry:%d", 
//        lastCalibratedReading[0], 
//        lastCalibratedReading[1], 
//        lastCalibratedReading[2], 
//        lastCalibratedReading[3]);
//      Serial.println(msgBuf);
//#endif
  }
}

void readAxes() {
  static int samplingAxis = 0;
  analogRead(axisPins[samplingAxis]);
  axes[samplingAxis] = analogRead(axisPins[samplingAxis]);
  samplingAxis = (samplingAxis+1) % 4;
}

void readButtons() {
  int c = 0;
  memset(msgBuf, 0, sizeof(msgBuf));
  
  int now = millis();
  for (int i=0; i<NUM_BUTTONS; ++i) {
    if (digitalRead(buttonPins[i]) == LOW) {
      if (buttonTimers[i] == 0) {
        buttonTimers[i] = now;
      }
    } else {
      buttonTimers[i] = 0;
    }

    buttonStates[i] = buttonTimers[i] && ((now-buttonTimers[i])>debounceLimit);

    sprintf(msgBuf+2*c++, "%d ", buttonStates[i]);
  }


//Serial.println(msgBuf);
  
}

void determineJoystickMode() {
  bool calibrationButtonPushed = digitalRead(calibrationPin) == LOW;
  if (calibrationButtonPushed) {
    if (!isChangingMode) {
      isChangingMode = true;
      isCalibrating = !isCalibrating;
      if (isCalibrating) {
#if SERIAL_LOGGING
        Serial.println("starting calibration...");
#endif
        for (int i=0; i<4; ++i) {
          axesMin[i] = axes[i];
          axesMax[i] = axes[i];
        }
      } else {
#if SERIAL_LOGGING
        Serial.println("calibration done");
        printCalibrationData();
#endif
        saveCalibrationData();
      }
      digitalWrite(statusLed, isCalibrating ? HIGH : LOW);
      delay(100);
    }
  } else {
    if (isChangingMode) {
      delay(100);
    }
    isChangingMode = false;
  }
}

void saveCalibrationData() {
  int writeAddr = 0;
  uint8_t *writePtr = (uint8_t*)&axesMin;
  for (writeAddr=0; writeAddr<8; ++writeAddr) {
    EEPROM.write(writeAddr, *writePtr);
    writePtr++;
  }
  writePtr = (uint8_t*)&axesMax;
  for (writeAddr=8; writeAddr<16; ++writeAddr) {
    EEPROM.write(writeAddr, *writePtr);
    writePtr++;
  }
}

void loadCalibrationData() {
  int readAddr = 0;
  uint8_t *readPtr = (uint8_t*)&axesMin;
  for (readAddr=0; readAddr<8; ++readAddr) {
    *readPtr = EEPROM.read(readAddr);
    readPtr++;
  }
  readPtr = (uint8_t*)&axesMax;
  for (readAddr=8; readAddr<16; ++readAddr) {
    *readPtr = EEPROM.read(readAddr);
    readPtr++;
  }
}

void printCalibrationData() {
  for (int i=0; i<4; ++i) {
    sprintf(msgBuf, "a%d: %d-(%d)-%d", i, axesMin[i], axes[i], axesMax[i]);
    Serial.println(msgBuf);
  }
}

int16_t calibratedAxis(int i) {
  float axisRange = (float)(axesMax[i] - axesMin[i]);
  float alignedAxis = (float)max(0, axes[i] - axesMin[i]);
  float calibratedAxis = min(1.0, alignedAxis / axisRange);
  calibratedAxis = 2*(calibratedAxis-0.5) * 0x7fff;
  return round(calibratedAxis);
}

