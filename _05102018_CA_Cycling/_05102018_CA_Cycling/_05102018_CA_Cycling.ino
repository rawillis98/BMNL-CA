#include "LCD_Launchpad.h"

//Pin Definitions
const int buttonPin = PUSH2;
const int potential = 40; //Pin P2.7, chronoamp square wave pulse
const int sensorPin = A10; //Pin P9.2, output from PCB to MSP

const int red = 39; //Pin P2.6
const int blue = 38;//Pin P3.3
const int green = 37;//Pin P3.6

//Settings
const bool verbose = false;
const bool N2 = true;
const bool CO2 = true;
const int measurementDuration = 20; //how many miliseconds to measure current for during the chronoamp measurement
const int averageDuration = 10; //how many miliseconds of data over to get the time-averaged current for a chronoamp measurement

const int BASELINE_LIST_LENGTH = 100; //The length of the moving median used to calculate the baseline

const int yellowPoint = 2000; //The concentration at which the LED (should) display completely yellow and start decreasing green and increasing red
const int redPoint = 4000;//The concentration after which the LED is entirely red

const int first_scroll_delay = 600; //when text longer than 6 characters is displayed, it waits <-- this much before starting to scroll
const int next_scroll_delay = 250; //when text longer than 6 characters is displayed, it waits <-- this long inbetween scrolls


//Global variables
int previousButtonState = 1;
int buttonState = 1;
LCD_LAUNCHPAD myLCD;

void displayText(String output) { //pass a string to make the LCD display say anything, will scroll if necessary
  int displayLength = 6;
  if (output.length() <= displayLength) {
    myLCD.displayText(output);
  } else {
    for (int i = 0; i < output.length() - displayLength + 1; ++i) {
      String tempOutput = "";
      for (int j = 0; j < displayLength; ++j) {
        if ((i + j) <= output.length()) {
          tempOutput += output.charAt(i + j);
        }
      }
      Serial.println(tempOutput);
      myLCD.displayText(tempOutput);
      if (i == 0) {
        delay(first_scroll_delay);
      } else {
        delay(next_scroll_delay);
      }
    }
  }
}


void setLED(float concentration) {
  int r, g, b;
  if (concentration == -1.0) {
    r = 0;
    g = 0;
    b = 0;
  } else if (concentration < yellowPoint) {
    r = int((concentration - 400) * 250 / (yellowPoint - 400));
    g = 250;
    b = 0;
  } else if (concentration < redPoint) {
    r = 250;
    g = int(250 - (concentration - 2000) * 200 / (redPoint - yellowPoint));
    b = 0;
  } else {
    r = 250;
    g = 50;
    b = 0;
  }
  analogWrite(red, r);
  analogWrite(green, g);
  analogWrite(blue, b);
  return;
}

typedef struct{
  String name;
  float concentration;
} Gas;

float getConcentration(float baseline, float iavg) {
  String CO2_output = "";
  String N2_output = "";
  float concentration;
  float pc = round((iavg - baseline) * 100 / baseline);
  if (iavg < baseline) {
    concentration = 20 * pc + 500;
  } else {
    concentration = 100 * pc + 500;
  }

  //concentration is the CO2 concentration
  //assign the following variables to the concentration of each gas
  Gas gasses[9];

  //name the gasses here
  gasses[0].name = "gas a";
  gasses[1].name = "gas b";
  gasses[2].name = "gas c";
  gasses[3].name = "gas d";
  gasses[4].name = "gas e";
  gasses[5].name = "gas f";
  gasses[6].name = "gas g";
  gasses[7].name = "gas h";
  gasses[8].name = "gas i";
  
  //put formulas for gas concentrations as a function of CO2 concentration here <-- Devang change here
  gasses[0].concentration = concentration;
  gasses[1].concentration = 1 * concentration;
  gasses[2].concentration = 2 * concentration;
  gasses[3].concentration = 3 * concentration;
  gasses[4].concentration = 4 * concentration;
  gasses[5].concentration = 5 * concentration;
  gasses[6].concentration = 6 * concentration;
  gasses[7].concentration = 7 * concentration;
  gasses[8].concentration = 8 * concentration;

  String output = "";
  for (int i = 0; i < 9; ++i) {
    output = gasses[i].name + " " + String(int(gasses[i].concentration));
    myLCD.clear();
    displayText(output);
    delay(500);
  }
  myLCD.clear();
  return concentration;
}

void bubbleSort(float myarr[], int n) {
  int i, j;
  for (i = 0; i < n - 1; i++) {

    // Last i elements are already in place
    for (j = 0; j < n - i - 1; j++) {
      if (myarr[j] > myarr[j + 1]) {
        swap(&myarr[j], &myarr[j + 1]);
      }
    }
  }
}
class CoolList {
  private:
    float myArray[BASELINE_LIST_LENGTH];
    float myMedianArray[BASELINE_LIST_LENGTH];
    int actualArrayLength;
  public:

    CoolList() {
      for (int i = 0; i < BASELINE_LIST_LENGTH; ++i) {
        myArray[i] = float();
        myMedianArray[i] = float();
        actualArrayLength = 0;
      }
    }

    void copyElements(int a, int b) {
      myArray[b] = myArray[a];
    }

    void printContents() {
      for (int i = 0; i < BASELINE_LIST_LENGTH; ++i) {
        Serial.print(i);
        Serial.print(" ");
        Serial.print(myArray[i]);
        Serial.print("    ");
      }
    }

    float operator[](int key) {
      float output;
      if (key >= 0) {
        output = myArray[key];
      } else {
        output = myArray[BASELINE_LIST_LENGTH - key];
        Serial.println("error");
      }
      return output;
    }

    void appendToFront(float something) {
      if (actualArrayLength < BASELINE_LIST_LENGTH) {
        actualArrayLength++;
      }
      for (int i = 0; i < BASELINE_LIST_LENGTH; ++i) {
        myArray[BASELINE_LIST_LENGTH - i] = myArray[BASELINE_LIST_LENGTH - i - 1];
      }
      if (myArray[0] != myArray[1]) {
        Serial.println("error");
      } else {
        myArray[0] = something;
      }
    }

    float getBaseline() {
      Serial.println("No arguments");
      for (int i = 0; i < BASELINE_LIST_LENGTH; ++i) {
        myMedianArray[i] = myArray[i];
      }
      float median;
      int medianPos;
      bubbleSort(myMedianArray, actualArrayLength);

      if (actualArrayLength % 2 == 0) {
        medianPos = int(actualArrayLength / 2) - 1;
        median = myMedianArray[medianPos] + myMedianArray[medianPos + 1];
        median = median / 2.0;
      } else {
        medianPos = int(actualArrayLength / 2);
        median = myMedianArray[medianPos];
      }
      return median;
    }

    float getBaseline(float currentBaseline) {
      for (int i = 0; i < BASELINE_LIST_LENGTH; ++i) {
        myMedianArray[i] = myArray[i];
      }
      float median;
      int medianPos;
      bubbleSort(myMedianArray, actualArrayLength);
      int maxElement = -1;

      float c = 1.1;
      /*Serial.print("Current Baseline*c: ");
        Serial.println(currentBaseline*c);
        Serial.print("actualArrayLength: ");
        Serial.println(actualArrayLength);*/
      for (int i = 0; i < actualArrayLength; ++i) {
        maxElement = i;
        if (myMedianArray[i] > currentBaseline * c) {
          break;
        }
        /*Serial.print(i);
          Serial.print(", ");
          Serial.print(myMedianArray[i]);
          Serial.print(", ");*/

      }
      if (maxElement == -1) {
        return currentBaseline;
      }

      if (maxElement % 2 == 0) {
        medianPos = int(maxElement / 2) - 1;
        median = myMedianArray[medianPos] + myMedianArray[medianPos + 1];
        median = median / 2.0;
      } else {
        medianPos = int(maxElement / 2);
        median = myMedianArray[medianPos];
      }
      /*Serial.println();
        Serial.print("maxElement: ");
        Serial.println(maxElement);
        Serial.print("medianPos: ");
        Serial.println(medianPos);
        Serial.print("Median: " );
        Serial.print(median);
        Serial.println();
        Serial.println();*/
      return median;
    }
};

double fmap(double x, double input_min, double input_max, double output_min, double output_max) {
  return (x * (output_max - output_min) / (input_max - input_min)) + output_min;
}

float chronoamp() {
  //Chronoamp settings



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
      //Serial.print(thisTime);
      //Serial.print(", ");
      Serial.println(v);
    }
    if ((startTime + averageDuration) > currentTime) {
      iavg += v * (thisTime - lastTime);
      lastTime = thisTime;
    }

  }
  digitalWrite(potential, LOW);
  iavg = iavg / lastTime;

  float result = iavg;
  return result;
}

void swap(float *xp, float *yp) {
  float temp = *xp;
  *xp = *yp;
  *yp = temp;
}

CoolList baselineData = CoolList();
float baseline;

void setup() {
  myLCD.init();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(potential, OUTPUT);
  pinMode(sensorPin, INPUT);
  Serial.begin(115200);
  float initial = chronoamp();
  /*for(int i = 0; i < int(BASELINE_LIST_LENGTH/4); ++i){
    baselineData.appendToFront(initial);
    }*/
  baselineData.appendToFront(initial);
  baseline = baselineData.getBaseline();
  setLED(400);
}

void loop() {
  digitalWrite(potential, LOW);
  baselineData.appendToFront(chronoamp());

  if (baselineData[2] != 0) {
    /*bool smallOneThree = abs(baselineData[2] - baselineData[0]) / baselineData[2] < 0.05; //the change between the most recent baselineData point and the one before the previous one should be relatively small
      bool largeOneTwo = abs(baselineData[1] - baselineData[0]) / baselineData[0] > 0.06; //the change between the most recent data point and the previous one should be relatively large*/

    bool largeOneTwo = abs(baselineData[2] - baselineData[1]) / baselineData[2] > 0.05;
    bool down = baselineData[1] > baselineData[0];
    bool largeDown = (abs(baselineData[1] - baselineData[0]) / baselineData[1]) > 0.05;
    if (largeOneTwo and largeDown and down or (not largeOneTwo and not down and not largeDown)) {
      //spike
      baselineData.copyElements(2, 1);
      //spike fixed
    }


    //the output data stream is data[1]
    baseline = baselineData.getBaseline(baseline);
    if (((baselineData[1] - baseline) / baseline) < -0.05) {
      for (int i = 1; i < 3; ++i) {
        baselineData.copyElements(2, BASELINE_LIST_LENGTH - i);
      }
    }
    Serial.print(baselineData[1]);
    Serial.print(",");
    Serial.print(baseline);
    Serial.print(",");
    Serial.println((baselineData[1] - baseline) * 100 / baseline);
    float concentration = getConcentration(baseline, baselineData[1]);
  }


}


