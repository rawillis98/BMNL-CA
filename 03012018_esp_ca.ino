#include <ESP8266WiFi.h>
//////////////////////
// WiFi Definitions ////////////////////////

const char WiFiAPPSK[] = "password";

/////////////////////
// Pin Definitions //
/////////////////////
int sensorPin = A0; //input pin
int potential = 13; //pin to apply voltage to RTIL
int ready_led = 4;

/////////////////////
// Chronoamp Settings//
////////////////////
int end_time = 2000; //How long to take data for in ms
int sample_period = 3; //Delay between samples in ms

///////////////////////
// Misc Variables //
//////////////////////

int sensorValue = 0; //variable to hold analog in of sensorPin
int its = 0; //iterations
float r = 1000000; //1MOhm Resistor
float atmosphere = 1;
int ready_time = 0;
int breathed_time = 0;

///////////////////////
// Sensor Settings //
/////////////////////

unsigned long relaxation_time = 60 * 90 * 1000; // 1000ms/s * 60s/min * 10min
float differential = 0.02;
float percentChange1000 = 1.075;
float percentChange400 = 1.04;

WiFiServer server(80);

void setup()
{
  initHardware();
  setupWiFi();
  server.begin();
}



void loop()
{
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    digitalWrite(potential, LOW); //just to be sure
    if (its > 0) {
      if ((millis() - ready_time) > relaxation_time ) {
        digitalWrite(ready_led, HIGH);
      } else {
        digitalWrite(ready_led, LOW);
      }
    }

    if (millis() - breathed_time > relaxation_time) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
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
  if (req.indexOf("/read") != -1) { //if "/read" is found in the request URL, return the appropriate response
    String avg_current = chronoamp(sample_period);
    Serial.print("Avg current is ");
    Serial.println(avg_current);

    //Headers
    s += "<h1>UTD BMNL CO2 Sensor</h1>";
    s += "<h3>Based on ESP8266</h3>";
    
    //div and current results
    s += "<div class = \"div1\">";
    s += "<p class = \"results\">i<sub>avg</sub>: ";
    s += avg_current;
    s += " nA</p><br>";

    //Reset the Ready LED
    digitalWrite(ready_led, LOW);
    ready_time = millis();

    //Breathing Stuff
    if (req.indexOf("Baseline") != -1) { //if this is the baseline measurement
      atmosphere = avg_current.toFloat();
    } else {
      Serial.print("Atmosphere: ");
      Serial.println(atmosphere);
      s += "<p class = \"results\">i<sub>atm</sub>: ";
      s += atmosphere;
      s += " nA</p><br>";
      Serial.println(avg_current.toFloat());
      Serial.println(atmosphere*percentChange400);
      Serial.println(atmosphere*percentChange1000);
      if(avg_current.toFloat() > atmosphere*percentChange400){
        Serial.println("400ppm");
        s += "<p class =\"results\">[CO<sub>2</sub>]: 400ppm</p><br>";
      } else if (avg_current.toFloat() > atmosphere*percentChange1000){
        Serial.println("1000ppm");
        s += "<p class =\"results\">[CO<sub>2</sub>]: 1000ppm</p><br>";
      } else {
        Serial.println("0ppm");
        s += "<p class =\"results\">[CO<sub>2</sub>]: 0ppm</p><br>";
      }

      if(avg_current.toFloat() > atmosphere){
        Serial.println("CO2 was present");
      }
    }

    //Links
    s += "</div><a href=\"http://192.168.4.1/read\">Take Measurement</a><br>";
    s += "<a href=\"http://192.168.4.1/readBaseline\">Retake Baseline Measurement</a>";

  } if(req.indexOf("test") != - 1) {
    s += "<p> Test <p>";
  } else { //otherwise, return something basic
    s += "<title>CO2 Sensor BMNL UTD</title>";
    s += "<h1>Welcome</h1>";
    s += "<a href=\"http://192.168.4.1/readBaseline\">Take Baseline Measurement</a>";
  }
  s += "</html>\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");
}

void setupWiFi(){
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

void initHardware(){
  Serial.begin(115200);
  pinMode(ready_led, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(ready_led, HIGH);
}



float fmap(float x, float input_min, float input_max, float output_min, float output_max) {
  return (x * (output_max - output_min) / (input_max - input_min)) + output_min;
}



String chronoamp(int delay_time) {
  Serial.println("Stabilizing..");
  digitalWrite(ready_led, LOW);
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
    avg += delay_time * v;
    delay(delay_time);
  }
  avg = avg / end_time;
  Serial.println("Done");
  digitalWrite(potential, LOW);
  pinMode(potential, INPUT);
  return String(avg);
}



