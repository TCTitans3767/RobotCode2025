// input pin can be any digital input
const byte inputPin = 2;
static unsigned long lastPeriodic = 0;
boolean isHigh = false;

boolean inputState = false;
boolean lastInputState = false;
boolean run = false;
long count = 0L;

unsigned long previousCountMillis = millis();
const long countMillis = 500L;

void setInputState() {
  inputState = digitalRead(inputPin);
}

/////////////////////////////////////////////////////////////
// the start up fundtion
void startUp() {
  // power on
  delay(2000);
  digitalWrite(9, HIGH);
  delay(2000);
  digitalWrite(9, LOW);
  delay(3000);
  // inital 6 second pulse
  digitalWrite(9, HIGH);
  delay(6000);
  digitalWrite(9, LOW);
  delay(1000);
  // 4 pulses every half second
  // 1
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
  // 2
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
  // 3
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
  // 4
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
}
///////////////////////////////////////////////////

///////////////////////////////////////////////////
void setup() {
  pinMode(inputPin, INPUT);
  pinMode(9, OUTPUT);

  Serial.begin(115200);
  startUp();
}
//////////////////////////////////////////////////////

// determinds step
void logic() {
  if (millis() >= 30000) { run = true; }  // The number here is how long to mute timer after startup
  // tickler timer

  if (millis() - lastPeriodic >= 20000 && count <= 4 && run) {
    /*
    when true, set current time
    on loop, check if x time has passed (amount of high time) and if high is true
    if high is true and x time has passed, set to low and set isHigh to false
    */
    lastPeriodic = millis();
    isHigh = true;
    digitalWrite(9, HIGH);
    Serial.println("tickling in progress");
  }

  if (millis() - lastPeriodic >= 500 && isHigh == true) {
    isHigh = false;
    digitalWrite(9, LOW);
  }

  // step 1
  if (count >= 6 && count <= 9) {
    digitalWrite(3, HIGH);
  } else {
    digitalWrite(3, LOW);
  }
  // step 2
  if (count >= 10 && count <= 11) {
    digitalWrite(4, HIGH);
  } else {
    digitalWrite(4, LOW);
  }
  // step 3
  if (count >= 12 && count <= 19) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }
  //step 4
  if (count >= 20 && count <= 30) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW);
  }
  // step 5
  if (count > 31) {
    digitalWrite(10, HIGH);
  } else {
    digitalWrite(10, LOW);
  }

  if (count >= 6) {
    lastPeriodic = millis();
  }

}

void loop() {
  // runs every time thru the loop
  setInputState();

  // count every transision HIGH<->LOW
  if (inputState != lastInputState) {
    count++;
    lastInputState = inputState;
  }

  // ------- every half second, count is equal to Hz.---------------
  if (millis() - previousCountMillis >= countMillis) {
    previousCountMillis += countMillis;
    logic();
    // show Hz on Serial too if available
    Serial.print(count);
    Serial.println(" Hz");

    // reset to zero for the next half second's sample
    count = 0L;
  }
}