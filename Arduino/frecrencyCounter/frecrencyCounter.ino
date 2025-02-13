// input pin can be any digital input
const byte inputPin = 2;

boolean inputState = false;
boolean lastInputState = false;
long count = 0L;

unsigned long previousCountMillis = millis();
const long countMillis = 500L;

void setInputState() {
  inputState = digitalRead(inputPin);
}

void setup() {
  pinMode(inputPin, INPUT);

  Serial.begin(115200);
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

    // show Hz on Serial too if available
    Serial.print(count); 
    Serial.println(" Hz");

    // reset to zero for the next half second's sample
    count = 0L;
  }
}