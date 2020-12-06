/*
 * Project leaf_dash
 * Description:
 * Author: sfeidersullivan
 * Date: 7/21/2020
 */

// #include "Grove_Temperature_And_Humidity_Sensor.h"
// #include "Adafruit_DHT.h"
#include "Adafruit_DHT_Particle.h"
// #include "DHT.h"
#include <deque>

#define DHTPIN D2
#define DHTTYPE DHT11 // DHT 11, DHT 22 (AM2302), DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE); // instantiate temp/humidity over dht
STARTUP(WiFi.selectAntenna(ANT_AUTO)); // continually switches at high speed between antennas

// Particle variables
double tempF, humidity, dewPoint, heatIndex;
bool lightsOn;

// Particle pins
int ledPin = D6;

// 18/6 lighting cycle
// 12/12 flowering on 7/31/20, on at night to keep temps down
int startHour = 22; // 10pm
int endHour = 10; // 10am

// temp moving average
std::deque<double> tempsHistory;
double tempsHistorySum = 0;
double tempsHistoryAverage = 0; // average of last 10 readings
int tempsHistoryLength = 20; // monitor the last 10 readings
double tempTooHotThreshold = 105; // F
unsigned long lastValidTempTime;
unsigned long lastValidTempTimeBuffer = 300000; // 5 min

void setup() {
  Time.zone(-7); // UTC --> PST

  // define pins
  pinMode(ledPin, OUTPUT);

  // init serial logging
  Serial.begin(9600);
  Serial.printlnf("setup...");
  Particle.publish("state", "setup...");

  // track variables
  Particle.variable("temp_F", tempF);
  Particle.variable("humidity", humidity);
  Particle.variable("dewPoint", dewPoint);
  Particle.variable("heatIndex", heatIndex);
  Particle.variable("lights_On", lightsOn);

  // Wire.begin();
  dht.begin(); // begin reading temp/humidity via dht
}

void loop() {
  // LIGHTS
  int hour = Time.hour();
  bool isPastEnd = hour >= endHour;//10
  bool isBeforeStart = hour < startHour;//22
  bool shouldBeOn = !(isPastEnd && isBeforeStart);

  // if it has been too long, turn off lights
  unsigned long nowMillis = millis();
  bool hasRecentTempReadings = ((lastValidTempTime + lastValidTempTimeBuffer) >= nowMillis);
  bool tempAverageBelowMax = tempsHistoryAverage < tempTooHotThreshold;

  if (true) {
    // germination cycle - use light as heater
    double germinationTempMin = 75; // F
    bool tempAverageBelowMin = tempsHistoryAverage < germinationTempMin;
    if (hasRecentTempReadings && tempAverageBelowMax && tempAverageBelowMin) {
      if (digitalRead(ledPin) == LOW) {
        digitalWrite(ledPin, HIGH); // turn on
        lightsOn = true;
        Particle.publish("lights", "On");
      };
    } else {
      if (digitalRead(ledPin) == HIGH) {
        digitalWrite(ledPin, LOW); // turn off
        lightsOn = false;
        Particle.publish("lights", "Off");
        String boolString = String(String::format("hasRecentTempReadings: %d, tempAverageBelowMax: %d", hasRecentTempReadings, tempAverageBelowMax));
        Particle.publish("light bools", boolString);
      };
    };
  } else {
    // light cycle
    if (shouldBeOn && hasRecentTempReadings && tempAverageBelowMax) {
      if (digitalRead(ledPin) == LOW) {
        digitalWrite(ledPin, HIGH); // turn on
        lightsOn = true;
        Particle.publish("lights", "On");
      };
    } else {
      if (digitalRead(ledPin) == HIGH) {
        digitalWrite(ledPin, LOW); // turn off
        lightsOn = false;
        Particle.publish("lights", "Off");
        String boolString = String(String::format("shouldBeOn: %d, hasRecentTempReadings: %d, tempAverageBelowMax: %d", shouldBeOn, hasRecentTempReadings, tempAverageBelowMax));
        Particle.publish("light bools", boolString);
      };
    };
  };

  // LOGGING
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humidity = dht.getHumidity();
  // float tempC = dht.getTempCelcius(); // read temp (C)
  tempF = dht.getTempFarenheit(); // read temp (F)

  if (isnan(tempF)) {
    String tempErrorMessage = "Failed to read from DHT Temperature sensor!";
    Serial.println(tempErrorMessage);
    Particle.publish("Sensor Error", tempErrorMessage);
  } else {
    // calculate temp moving average
    tempsHistory.push_back(tempF);
    tempsHistorySum += tempF;
    if (tempsHistory.size() > tempsHistoryLength) {
      double front = tempsHistory.front();
      tempsHistorySum -= front;
      tempsHistory.pop_front();
    };
    tempsHistoryAverage = (tempsHistorySum / tempsHistoryLength);

    // log valid temp timestamp
    lastValidTempTime = millis();
  };

  if (isnan(humidity)) {
    String humErrorMessage = "Failed to read from DHT Humidity sensor!";
		Serial.println(humErrorMessage);
    Particle.publish("Sensor Error", humErrorMessage);
	};

  // Must send in temp in Fahrenheit
  heatIndex = dht.getHeatIndex(); // (C)
  dewPoint = dht.getDewPoint(); // (C)
  float k = dht.getTempKelvin(); // (K)

  String readings = String(String::format("{\"Hum(\%)\": %4.2f, \"Temp(°F)\": %4.2f, \"DP(°C)\": %4.2f, \"HI(°C)\": %4.2f, \"TempAvg(°F)\": %4.2f}", humidity, tempF, dewPoint, heatIndex, tempsHistoryAverage));
  Serial.println(readings);
  Serial.println(Time.timeStr());
  Particle.publish("readings", readings);

  delay(5000);
};
