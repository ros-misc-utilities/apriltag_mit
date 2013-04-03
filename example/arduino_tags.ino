/*
  ArduinoTag
  Turns on an LED whenever Raspberry Pi sees an Apriltag.
 
  Michael Kaess 04/13
 */
 
int led = 9;
int tagId = 0;

// the setup routine runs once when you press reset:
void setup() {
  // open serial port
  Serial.begin(9600);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  // check if new data is available
  if (Serial.available() > 0) {
    tagId = Serial.parseInt();
    Serial.read(); // ignore newline character
    if (tagId >= 0) {
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
  }
  // wait for 20ms before checking again for new data
  delay(20);
}

