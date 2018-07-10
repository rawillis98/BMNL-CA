#include "LCD_Launchpad.h"

//Pin Definitions
const int buttonPin = PUSH2;
const int potential = 40;
const int sensorPin = A10; //Pin P9.2

//Global variables
int previousButtonState = 1;
int buttonState = 1;
LCD_LAUNCHPAD myLCD;

class Results {
  private:
    double iavg;

  public:
  Results(){
    iavg = 0;
  }
    void printiavg() {
      Serial.print("iavg: ");
      Serial.println(iavg);
    }
    void setiavg(double val) {
      iavg = val;
    }

    float getConcentration(){
      return exp((iavg - 1114)/89.208);
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

    void displayiavg(){
      myLCD.clear();
      delay(1000);
      myLCD.displayText(String(int(iavg)));
    }
};

Results baseline = Results();
Results previousResult = Results();

void setup() {
  myLCD.init();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(potential, OUTPUT);
  pinMode(sensorPin, INPUT);
  Serial.begin(115200);
  // Get baseline
  //baseline = chronoamp();
}

void loop() {
  digitalWrite(potential, LOW);
  delay(3 * 1000);
  Results result = chronoamp();
  result.displayiavg();
  Serial.println(result.getiavg());
  //Serial.println((result.getiavg()+previousResult.getiavg())/2);

  previousResult = result;
}

double fmap(double x, double input_min, double input_max, double output_min, double output_max) {
  return (x * (output_max - output_min) / (input_max - input_min)) + output_min;
}

Results chronoamp() {
  //Chronoamp settings
  bool verbose = true;
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
  while (startTime + measurementDuration > millis()) {
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

