#include <ESP8266WiFi.h>
//////////////////////
// WiFi Definitions ////////////////////////

const char WiFiAPPSK[] = "password";

/////////////////////
// Pin Definitions //
/////////////////////
int sensorPin = A0; //input pin
int potential = 13; //pin to apply voltage to RTIL
int nitrogen = 2;
int carbondioxide = 0;

/////////////////////
// Chronoamp Settings//
////////////////////
int end_time = 2000; //How long to take data for in ms
int delay_time = 3;
int avg_end_time = 150;

///////////////////////
// Misc Variables //
//////////////////////
int sensorValue = 0; //variable to hold analog in of sensorPin
int its = 0; //variable to hold iterations
float r = 1000000; //1MOhm Resistor
float atmosphere = 1;
int ready_time = 0;
float baseline_iavg = -1;

///////////////////////
// Percent Changes //
//////////////////////
float pc_co2 = 1.0;
float pc_breath = 1.0;

WiFiServer server(80);

void setup() {
  initHardware();
  setupWiFi();
  server.begin();
}

void loop() {
  // Check if a client has connected
  digitalWrite(potential, LOW);

  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  ///////////////////////////////////////////
  ///// HTML GOES HERE //////////////////
  ///////////////////////////////////////////

  // Prepare the response. Start with the common header:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<head><style> .div1 { width: 200px; height 100px; padding: 50px; border: 3px solid green; font-size: 125%;} .results { font-size: 135%; }</style></head>";

  s += "<h1>UTD BMNL CO2 Sensor</h1>";
  s += "<h3>Based on ESP8266</h3>";
  s += "<div class = \"div1\">";

  //readBasline
  //readCO2
  //readBreath

  if (req.indexOf("/read") != -1) { //if "/read" is found in the request URL, return the appropriate response
    digitalWrite(carbondioxide, LOW);
    digitalWrite(nitrogen, LOW);

    float iavg = chronoamp();

    if (req.indexOf("Basline") != -1) {
      baseline_iavg = iavg; //set the baseline

    } else if (req.indexOf("CO2") != - 1) { //if we're taking a CO2 measurement,
      if (baseline_iavg = -1) { //if a baseline hasn't been taken yet
        s += "<p class =\"results\">A baseline measurement has not been taken yet.</p><br>";
        Serial.println("A basline measurement has not been taken yet.");
      } else if (iavg > baseline_iavg * pc_co2) { //if a baseline has been taken, and iavg is bigger than the percent change * the baseline
        s += "<p class =\"results\">[CO<sub>2</sub>]: 1000ppm</p><br>";
        Serial.println("1000ppm");
        digitalWrite(carbondioxide, HIGH);
      } else { //if iavg is not bigger, write 0 ppm
        s += "<p class =\"results\">[CO<sub>2</sub>]: 0ppm</p><br>";
        digitalWrite(nitrogen, HIGH);
        digitalWrite(carbondioxide, LOW);
      }

    } else if (req.indexOf("Breath") != -1) {
      if (baseline_iavg = -1) { //if a baseline hasn't been taken yet
        s += "<p class =\"results\">A baseline measurement has not been taken yet.</p><br>";
        Serial.println("A basline measurement has not been taken yet.");
      } else if (iavg > baseline_iavg * pc_breath) { //if a baseline has been taken, and iavg is bigger than the percent change * the baseline
        s += "<p class =\"results\">[CO<sub>2</sub>]: 10,000 - 20,000ppm</p><br>";
        Serial.println("10,000 - 20,000ppm");
        digitalWrite(carbondioxide, HIGH);
        digitalWrite(nitrogen, LOW);
      } else { //if iavg is not bigger, write 0 ppm
        s += "<p class =\"results\">[CO<sub>2</sub>]: 0ppm</p><br>";
        digitalWrite(nitrogen, HIGH);
        digitalWrite(carbondioxide, LOW);
      }

    }
  }


  //add links to next step
  s += "</div><a href=\"http://192.168.4.1/readBaseline\">Take Baseline Measurement</a><br>";
  s += "<a href=\"http://192.168.4.1/readCO2\">Take 1000ppm Measurement</a><br>";
  s += "<a href=\"http://192.168.4.1/readCO2\">Take Breath Measurement</a>";
  s += "</html>\n";

  client.print(s);
  delay(1);
  Serial.println("Client disonnected");
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESP8266 Thing " + macID;
  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);
  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);
  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

void initHardware() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(carbondioxide, OUTPUT);
  pinMode(nitrogen, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

float fmap(float x, float input_min, float input_max, float output_min, float output_max) {
  return (x * (output_max - output_min) / (input_max - input_min)) + output_min;
}

float chronoamp() {
  Serial.println("Stabilizing..");
  pinMode(potential, OUTPUT);
  digitalWrite(potential, LOW); //stabilize at 0V
  delay(500); //wait 500ms

  int start_time = millis();
  int current_time = millis() - start_time;
  float k = 0;
  float avg = 0;
  its += 1;
  Serial.println("Intialized");
  while (current_time < end_time) {
    digitalWrite(potential, HIGH);
    sensorValue = analogRead(sensorPin);
    float v = fmap(sensorValue, 0, 1023, 0.0, 3.3); //voltage across 1MOhm resistor
    float i = v / r; //Solve Ohm's law for current across 1MOhm resistor
    current_time = millis() - start_time;
    Serial.print(current_time);
    Serial.print(", ");
    Serial.println(v);
    if (current_time < avg_end_time) {
      avg += delay_time * v;
    }
    delay(delay_time);
  }
  avg = avg / avg_end_time;
  Serial.println("Done");
  digitalWrite(potential, LOW);
  pinMode(potential, INPUT);
  Serial.print("iavg: ");
  Serial.println(avg);
  return avg;
}



