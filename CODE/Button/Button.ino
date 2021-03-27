int RedIn = 22;    // pushbutton connected to digital pin 7
int GreenIn = 23;    // pushbutton connected to digital pin 7
int OrangeIn = 24;    // pushbutton connected to digital pin 7
int PurpleIn = 25;    // pushbutton connected to digital pin 7


void setup() {
  Serial.begin(9600);
  pinMode(RedIn, INPUT);    // sets the digital pin 7 as input
  pinMode(GreenIn, INPUT);    // sets the digital pin 7 as input
  pinMode(OrangeIn, INPUT);    // sets the digital pin 7 as input
  pinMode(PurpleIn, INPUT);    // sets the digital pin 7 as input
}

void loop() {

  int inputs = ((digitalRead(RedIn) << 3) |  (digitalRead(GreenIn) << 2) | (digitalRead(OrangeIn) << 1) | digitalRead(PurpleIn));

  switch (inputs) {
    case 0: //no button
      Serial.println("NO INPUT");
      break;
    case 1: //purple
      Serial.println("PURPLE HIGH");
      break;
    case 2:
      Serial.println("ORANGE HIGH");
      break;
    case 4:
      Serial.println("GREEN HIGH");
      break;
    case 8:
      Serial.println("RED HIGH");
      break;
    default:
      Serial.println("NO INPUT");
      break;
  }
}
