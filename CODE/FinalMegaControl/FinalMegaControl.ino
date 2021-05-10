//______________________________________________Motor Definitions____

#include <AccelStepper.h>
#include <MultiStepper.h>

// Define step constants
#define FULLSTEP 4 //start pin for FeedMotor
#define FULLSTEP 8 //start pin for GearMotor

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper GearMotor (FULLSTEP, 8, 10, 9, 11);
AccelStepper FeedMotor (FULLSTEP, 4, 6, 5, 7);

//______________________________________________Button definitions____

int RedIn = 22;    // pushbutton connected to digital pin 22
int GreenIn = 23;    // pushbutton connected to digital pin 23
int OrangeIn = 24;    // pushbutton connected to digital pin 24
int PurpleIn = 25;    // pushbutton connected to digital pin 25
int inputs = 0;
//______________________________________________Colour definitions____

#include <Adafruit_TCS34725.h>
#include <Wire.h>

#define redpin 1 // Pick outputs
#define greenpin 2
#define bluepin 3

// set to false if using a common cathode LED
#define commonAnode false

// our RGB -> eye-recognized gamma color
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int SweetColour = 0; //0=base, 1=purple, 2=orange, 3=green, 4=red

bool purpleSearch = false;
bool redSearch = false;
bool orangeSearch = false;
bool greenSearch = false;

//______________________________________________Robot Definitions____
bool correctColour = false;
bool buttonPressed = false;

unsigned long referenceTime = millis();


int SendPin = 12;

//______________________________________________End Definitions____

void setup() {

  //______________________________________________Button Setup____

  Wire.begin();
  Serial.begin(9600);
  pinMode(RedIn, INPUT);    // sets the digital pin as input
  pinMode(GreenIn, INPUT);    // sets the digital pin as input
  pinMode(OrangeIn, INPUT);    // sets the digital pin as input
  pinMode(PurpleIn, INPUT);    // sets the digital pin as input

  //______________________________________________Motor Setup____

  // set the maximum speed, acceleration factor,
  // initial speed and the target position for motor 1
  GearMotor.setMaxSpeed(400.0);   // (steps per second) this is a good speed for the gear motor, need to refine how many steps i want it to take.
  GearMotor.setAcceleration(20.0); // the gear motor running on 6V D batteries is way better and capable
  GearMotor.setSpeed(200);

  // set the same for motor 2
  FeedMotor.setMaxSpeed(300.0); // 
  FeedMotor.setAcceleration(100.0);
  FeedMotor.setSpeed(300);

  //______________________________________________Colour Setup____

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // use these three pins to drive an LED
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }
  //______________________________________________Robot Setup____


  pinMode(SendPin, OUTPUT);
  digitalWrite(SendPin, LOW);

  //__________________________________________
  GearMotor.moveTo(GearMotor.currentPosition() + 200);
}


void ReadColour()
{
  Serial.println("ReadColour Running");
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  //delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED
#if defined(ARDUINO_ARCH_ESP32)   //make LED shine the colour it scans
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
#else
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
#endif

  if ( (red < 90 && red > 82) && (green < 92 && green > 82) && (blue < 104 && blue > 94)) //PURPLE
  {
    Serial.println("PURPLE");
    Serial.println("R:\t"); Serial.print(int(red));
    Serial.print("\tG:\t"); Serial.print(int(green));
    Serial.print("\tB:\t"); Serial.print(int(blue));
    SweetColour = 1;
  }
  else if ( (red < 135 && red > 117) && (green < 87 && green > 76) && (blue < 84 && blue > 64)) //ORANGE
  {
    Serial.println("ORANGE");
    Serial.println("R:\t"); Serial.print(int(red));
    Serial.print("\tG:\t"); Serial.print(int(green));
    Serial.print("\tB:\t"); Serial.print(int(blue));
    SweetColour = 2;
  }
  else if ( (red < 87 && red > 81) && (green < 110 && green > 98) && (blue < 84 && blue > 75)) //GREEN
  {
    Serial.println("GREEN");
    Serial.println("R:\t"); Serial.print(int(red));
    Serial.print("\tG:\t"); Serial.print(int(green));
    Serial.print("\tB:\t"); Serial.print(int(blue));
    SweetColour = 3;
  }
  else if ( (red < 124 && red > 112) && (green < 84 && green > 75) && (blue < 84 && blue > 75)) //RED
  {
    Serial.println("RED");
    Serial.println("R:\t"); Serial.print(int(red));
    Serial.print("\tG:\t"); Serial.print(int(green));
    Serial.print("\tB:\t"); Serial.print(int(blue));
    SweetColour = 4;
  }
  else
  {
    Serial.println("No Sweet Detected");
    SweetColour = 0;
  }
}

void ColourMatch() //function dictates actions that occur once colour under sensor matches user choice.
{
  Serial.println("Colour Match Running ");
  GearMotor.moveTo(GearMotor.currentPosition() + 5095); //90 degrees from colour sense (will need CHanging)
  GearMotor.run();
  while (GearMotor.distanceToGo() > 0)
  {
    GearMotor.run();
  }
  digitalWrite(SendPin, HIGH);
  delay (1000);
  digitalWrite(SendPin, LOW);
  delay (35000); //change to robot return signal???

  //robot now finished, start feeding
  GearMotor.moveTo(GearMotor.currentPosition() + 10190); //rotate plate 180 deg to initiate feeding
  GearMotor.run();
  while (GearMotor.distanceToGo() > 0)
  {
    GearMotor.run();
  }
  FeedMotor.moveTo(FeedMotor.currentPosition() - 4076); //begin feed
  FeedMotor.run();
  while (FeedMotor.distanceToGo() < 0)
  {
    FeedMotor.run();
  }
  SweetColour = 0;
  purpleSearch = false;
  redSearch = false;
  orangeSearch = false;
  greenSearch = false;
}


void loop() {

  GearMotor.run();
  digitalWrite(SendPin, LOW);

  if (digitalRead(RedIn) |  digitalRead(GreenIn) | digitalRead(OrangeIn) | digitalRead(PurpleIn)) {
    inputs = ((digitalRead(RedIn) << 3) |  (digitalRead(GreenIn) << 2) | (digitalRead(OrangeIn) << 1) | digitalRead(PurpleIn));

    Serial.println(inputs );

    if (!buttonPressed) {
      buttonPressed = true;
      Serial.println( "buttonPressed " + buttonPressed );
      Serial.println(GearMotor.distanceToGo());
    }
  }

  if (buttonPressed) {

    if (millis() > referenceTime + 1000 ) {
      ReadColour();

      switch (inputs) {
        case 0: //no button
          Serial.println("NO INPUT");
          break;

        case 1: //purple
          Serial.println("PURPLE button HIGH");
          purpleSearch = true;
          if (SweetColour == 1)
          {
            Serial.println("Colour Correct Purple ");
            correctColour = true;
          }
          else {
            Serial.println("Colour incorrect Purple ");
            correctColour = false;
          }
          break;
        case 2: //orange
          Serial.println("ORANGE button HIGH");
          orangeSearch = true;
          if (SweetColour == 2)
          {
            Serial.println("Colour Correct ORANGE ");
            correctColour = true;
          }
          else {
            Serial.println("Colour incorrect ORANGE ");
            correctColour = false;
          }
          break;
        case 4: //green
          Serial.println("GREEN button HIGH");
          greenSearch = true;
          if (SweetColour == 3)
          {
            Serial.println("Colour Correct GREEN ");
            correctColour = true;
          }
          else {
            Serial.println("Colour incorrect GREEN ");
            correctColour = false;
          }
          break;
        case 8: //red
          Serial.println("RED button HIGH");
          redSearch = true;
          if (SweetColour == 4)
          {
            Serial.println("Colour Correct RED ");
            correctColour = true;
          }
          else {
            Serial.println("Colour incorrect RED ");
            correctColour = false;
          }
          break;

        default:
          Serial.println("NO INPUT");
          break;
      }
      referenceTime = millis();
    }

    if (correctColour) {
      ColourMatch();
      correctColour = false;
      buttonPressed = false;
      inputs = 0;

    }
    else {
      if (GearMotor.distanceToGo() < 200) {
        GearMotor.moveTo(GearMotor.currentPosition() + 400); //Spin until colour matches
        correctColour = 0;
        Serial.println("correctcolour else ");
      }
    }
  }
  else {
    if (GearMotor.distanceToGo() < 200) {
      GearMotor.moveTo(GearMotor.currentPosition() + 400); //Spin until colour matches
      Serial.println("Final Else print");
    }
  }

}
