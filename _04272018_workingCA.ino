#include "LCD_Launchpad.h"

//Pin Definitions
const int buttonPin = PUSH2;
const int potential = 40;
const int sensorPin = A10; //Pin P9.2

//Global variables
int previousButtonState = 1;
int buttonState = 1;
LCD_LAUNCHPAD myLCD;

//Chronoamp settings
int measurementDuration = 2 * 1000;
int averageDuration = 150;

class Results {
  private:
    double iavg;

  public:
    void printiavg() {
      Serial.print("iavg: ");
      Serial.println(iavg);
    }
    void setiavg(double val) {
      iavg = val;
    }

    double getiavg() {
      return iavg;
    }

    void compareToBaseline(Results baseline){
      if(getiavg() > baseline.getiavg()){
        myLCD.displayText("CO2");
        delay(5000);
        myLCD.clear();
      } else {
        myLCD.displayText("No CO2");
        delay(5000);
        myLCD.clear();
      }
    }
};

Results baseline = Results();

void setup() {
  myLCD.init();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(potential, OUTPUT);
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
  delay(1000);

  // Get baseline
  baseline = chronoamp();
}

void loop() {
  digitalWrite(potential, LOW);
  previousButtonState = buttonState;
  buttonState = digitalRead(buttonPin);

  if ((previousButtonState == 0) && (buttonState == 1)) {
    Results result = chronoamp();
    result.printiavg();
    result.compareToBaseline(baseline);
  }
}

double fmap(double x, double input_min, double input_max, double output_min, double output_max) {
  return (x * (output_max - output_min) / (input_max - input_min)) + output_min;
}

Results chronoamp() {
  digitalWrite(potential, LOW);
  delay(500);
  double iavg = 0;
  long unsigned int startTime = millis();
  long unsigned int currentTime = 0;
  long unsigned int lastTime = 0;
  long unsigned int thisTime = 0;
  digitalWrite(potential, HIGH);
  while (startTime + measurementDuration > millis()) {
    double v = fmap(analogRead(sensorPin), 0, 4095, 0.0, 3.3);
    currentTime = millis();
    thisTime = currentTime - startTime;
    Serial.print(thisTime);
    Serial.print(", ");
    Serial.println(v);
    if ((startTime + averageDuration) > currentTime) {
      iavg += v*(thisTime - lastTime);
      lastTime = thisTime;
    }
    
  }
  iavg = iavg / lastTime;

  Results result = Results();
  result.setiavg(iavg);
  return result;
}

