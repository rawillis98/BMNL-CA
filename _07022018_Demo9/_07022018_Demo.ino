#include "LCD_Launchpad.h"

//Pin Definitions
const int buttonPin = PUSH2;
const int buttonPin1 = PUSH1;
const int potential = 40;
const int sensorPin = A10; //Pin P9.2

//Global variables
int previousButtonState = 1;
int buttonState = 1;
int previousButtonState1 = 1;
int buttonState1 = 1;
LCD_LAUNCHPAD myLCD;

class Results {
  private:
    double iavg;

  public:
    Results() {
      iavg = 0;
    }
    void printiavg() {
      Serial.print("iavg: ");
      Serial.println(iavg);
    }
    void setiavg(double val) {
      iavg = val;
    }

    float getConcentration() {
      return exp((iavg - 1114) / 89.208);
    }

    double getiavg() {
      return iavg;
    }

    void compareToBaseline(Results baseline) {
      if (getiavg() > baseline.getiavg()) {
        myLCD.displayText("CO2");
        delay(5000);
        myLCD.clear();
      } else {
        myLCD.displayText("No CO2");
        delay(5000);
        myLCD.clear();
      }
    }

    void displayiavg() {
      myLCD.clear();
      myLCD.displayText(String(int(iavg)));
      delay(1000);
    }
};

Results baseline = Results();

void setup() {
  myLCD.init();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(potential, OUTPUT);
  pinMode(sensorPin, INPUT);
  Serial.begin(115200);
  // Get baseline
  baseline = chronoamp();
  baseline.displayiavg();
  Serial.print("Baseline: ");
  Serial.println(baseline.getiavg());
}

void loop() {
  digitalWrite(potential, LOW);
  previousButtonState = buttonState;
  buttonState = digitalRead(buttonPin);

  previousButtonState1 = buttonState1;
  buttonState1 = digitalRead(buttonPin1);

  if ((previousButtonState1 == 0) && (buttonState1 == 1)) {//take baseline
    baseline = chronoamp();
    baseline.displayiavg();
    Serial.print("Baseline: ");
    Serial.println(baseline.getiavg());
  }


  if ((previousButtonState == 0) && (buttonState == 1)) {//take measurement
    Results result = chronoamp();
    double iavg = result.getiavg();
    double base = baseline.getiavg();



    if (iavg > base * 1.2) {
      myLCD.displayText("1.2");
      delay(1000);
      myLCD.clear();
    } else if (iavg > base * 1.15) {
      myLCD.displayText("1.15");
      delay(1000);
      myLCD.clear();
    } else if (iavg > base * 1.1) {
      myLCD.displayText("1.1");
      delay(1000);
      myLCD.clear();
    } else if (iavg > base * 1.05) {
      myLCD.displayText("1.05");
      delay(1000);
      myLCD.clear();
    }  else if (iavg > base) {
      myLCD.displayText("1");
      delay(1000);
      myLCD.clear();
    } else if (iavg < base) {
      myLCD.displayText("0");
      delay(1000);
      myLCD.clear();
    }
    Serial.println(iavg);
  }

}

double fmap(double x, double input_min, double input_max, double output_min, double output_max) {
  return (x * (output_max - output_min) / (input_max - input_min)) + output_min;
}

Results chronoamp() {
  //Chronoamp settings
  bool verbose = false;
  int measurementDuration = 200;
  int averageDuration = 150;
  digitalWrite(potential, LOW);
  delay(500);
  double iavg = 0;
  long unsigned int startTime = millis();
  long unsigned int currentTime = 0;
  long unsigned int lastTime = 0;
  long unsigned int thisTime = 0;
  digitalWrite(potential, HIGH);
  int i = 0;
  while (startTime + measurementDuration > millis()) {
    ++i;
    double v = analogRead(sensorPin);
    currentTime = millis();
    thisTime = currentTime - startTime;
    if (verbose) {
      Serial.print(thisTime);
      Serial.print(", ");
      Serial.println(v);
    }
    if ((startTime + averageDuration) > currentTime) {
      iavg += v * (thisTime - lastTime);
      lastTime = thisTime;
    }

  }
  digitalWrite(potential, LOW);
  iavg = iavg / lastTime;

  Results result = Results();
  result.setiavg(iavg);
  return result;
}

