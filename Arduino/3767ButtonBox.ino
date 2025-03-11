// Total Teensy Buttons 42

const int climbPin = 23;
const int climbLedPin = 22;
const int algaePin = 7;
const int algaeLedPin = 6;
const int sourcePin = 5;


const int aPin = 13;
const int aLedPin = 41;
const int bPin = 30;
const int bLedPin = 29;
const int cPin = 28;
const int cLedPin = 27;
const int dPin = 26;
const int dLedPin = 25;
const int ePin = 24;
const int eLedPin = 12;
const int fPin = 11;
const int fLedPin = 10;
const int gPin = 9;
const int gLedPin = 8;
const int hPin = 40;
const int hLedPin = 39;
const int iPin = 21;
const int iLedPin = 20;
const int jPin = 19;
const int jLedPin = 18;
const int kPin = 17;
const int kLedPin = 16;
const int lPin = 15;
const int lLedPin = 14;

const int l1Pin = 32;
const int l1LedPin = 31;
const int l2Pin = 34;
const int l2LedPin = 33;
const int l3Pin = 36;
const int l3LedPin = 35;
const int l4Pin = 38;
const int l4LedPin = 37;


const int reefButtonPins[12] = { aPin, bPin, cPin, dPin, ePin, fPin, gPin, hPin, iPin, jPin, kPin, lPin };
const int reefLedPins[12] = { aLedPin, bLedPin, cLedPin, dLedPin, eLedPin, fLedPin, gLedPin, hLedPin, iLedPin, jLedPin, kLedPin, lLedPin };
const int reefJoystick[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

const int levelButtonPins[4] = { l1Pin, l2Pin, l3Pin, l4Pin };
const int levelLedPins[4] = { l1LedPin, l2LedPin, l3LedPin, l4LedPin };
const int levelJoystick[4] = { 13, 14, 15, 16 };

const int algeaButtonPin = algaePin;
const int algeaLedPin = algaeLedPin;
const int algeaJoystick = 17;

const int sourceButtonPin = sourcePin;
const int sourceJoystick = 18;

const int endGameButtonPin = climbPin;
const int endGameLedPin = climbLedPin;
const int endGameJoystick = 19;

// Variables to track the active button
int activeReefButton = 0;
int activeLevelButton = 3;
bool isClimbActive = false;

bool endGameLedState = false;         // Variable to track LED state
bool endGameButtonState = false;      // Last button state
bool endGameLastButtonState = false;  // Previous button state

void setup() {
  for (int i = 0; i < 12; i++) {
    pinMode(reefButtonPins[i], INPUT_PULLUP);  // Enable internal pull-up resistor
    pinMode(reefLedPins[i], OUTPUT);
    digitalWrite(reefLedPins[i], HIGH);  // Ensure LEDs are off initially
  }

  for (int i = 0; i < 4; i++) {
    pinMode(levelButtonPins[i], INPUT_PULLUP);  // Enable internal pull-up resistor
    pinMode(levelLedPins[i], OUTPUT);
    digitalWrite(levelLedPins[i], HIGH);  // Ensure LEDs are off initially
  }

  pinMode(algeaButtonPin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(algeaLedPin, OUTPUT);
  digitalWrite(algeaLedPin, HIGH);  // Ensure LEDs are off initially

  pinMode(sourceButtonPin, INPUT_PULLUP);  // Enable internal pull-up resistor

  pinMode(endGameButtonPin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(endGameLedPin, OUTPUT);
  digitalWrite(endGameLedPin, HIGH);  // Ensure LEDs are off initially



  Joystick.useManualSend(true);
}

void loop() {
  for (int i = 0; i < 12; i++) {
    bool reading = digitalRead(reefButtonPins[i]);

    if (!reading || i == activeReefButton) {
      digitalWrite(reefLedPins[i], LOW);
      Joystick.button(reefJoystick[i], true);
      activeReefButton = i;
    } else {
      digitalWrite(reefLedPins[i], HIGH);
      Joystick.button(reefJoystick[i], false);
    }
  }

  for (int i = 0; i < 4; i++) {
    bool reading = digitalRead(levelButtonPins[i]);
    if (!reading || activeLevelButton == i) {
      digitalWrite(levelLedPins[i], LOW);
      Joystick.button(levelJoystick[i], true);
      activeLevelButton = i;
    } else {
      digitalWrite(levelLedPins[i], HIGH);
      Joystick.button(levelJoystick[i], false);
    }
  }

  if (!digitalRead(algeaButtonPin)) {
    digitalWrite(algeaLedPin, LOW);
    Joystick.button(algeaJoystick, true);
  } else {
    digitalWrite(algeaLedPin, HIGH);
    Joystick.button(algeaJoystick, false);
  }

  endGameButtonState = digitalRead(endGameButtonPin);

  if (endGameButtonState != endGameLastButtonState) {
    if (endGameLastButtonState) {
      digitalWrite(endGameLedPin, LOW);
      Joystick.button(endGameJoystick, true);
      endGameLastButtonState = false;
    } else {
      digitalWrite(endGameLedPin, HIGH);
      Joystick.button(endGameJoystick, false);
      endGameLastButtonState = true;
    }
  }


  if (!digitalRead(sourceButtonPin)) {
    Joystick.button(sourceJoystick, true);
  } else {
    Joystick.button(sourceJoystick, false);
  }


  Joystick.send_now();
}
