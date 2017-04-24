// Hello World by Vincent Claes - Hogeschool PXL


int pushButton = PUSH2;
int greenPin= GREEN_LED;
int redPin= RED_LED;
int yellowPin= YELLOW_LED;

void setup() {
  Serial.begin(9600);
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(greenPin,OUTPUT);
  pinMode(redPin,OUTPUT);
  pinMode(yellowPin,OUTPUT);
}

void loop() {
  int buttonState = digitalRead(pushButton);
  Serial.println(buttonState);
  digitalWrite(greenPin, buttonState);
  digitalWrite(redPin, buttonState);
  digitalWrite(yellowPin, buttonState);
  delay(500);
}



