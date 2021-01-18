/*
 * Project leaf_dash
 * Description:
 * Author: sfeidersullivan
 * Date: 7/21/2020
 */

// #include "Grove_Temperature_And_Humidity_Sensor.h"
// #include "Adafruit_DHT.h"
// #include "Adafruit_DHT_Particle.h"
// #include "DHT.h"
#include "Wire.h"
#include "PietteTech_DHT.h"
#include "SparkFun_Qwiic_Humidity_AHT20.h"

#include <deque>

#define DHTPIN D2
#define DHTTYPE DHT11 // DHT 11, DHT 22 (AM2302), DHT 21 (AM2301)

// DHT dht(DHTPIN, DHTTYPE); // instantiate temp/humidity over dht
PietteTech_DHT DHT(DHTPIN, DHTTYPE);
AHT20 ahtSensor;

STARTUP(WiFi.selectAntenna(ANT_AUTO)); // continually switches at high speed between antennas

// Particle variables
double tempF, ahtTempF, ahtHumidity, humidity, dewPoint, heatIndex;
bool lightsOn, heaterOn;

// Particle pins
int ledPin = D6;
int heaterPin = D4;

// 12/12 flowering on 7/31/20, on at night to keep temps down (10pm-10am)
// 18/6 veg on 12/9/20 (6pm-10am)
int startHour = 18; // 6pm
int endHour = 10; // 10am

// temp moving average
std::deque<double> tempsHistory;
double tempsHistorySum = 0;
double tempsHistoryAverage = 0; // average of last 10 readings
int tempsHistoryLength = 10; // monitor the last 10 readings
double tempTooHotThreshold = 105; // F
unsigned long lastValidTempTime;
unsigned long lastValidTempTimeBuffer = 300000; // 5 min

class HeaterTimer {
private:
  Timer t = Timer(240000, &HeaterTimer::callback, *this);
public:
  void start() { t.reset(); };
  bool active() { return t.isActive(); };
  void callback() {
    t.stop();
    digitalWrite(heaterPin, LOW); // turn off
    heaterOn = false;
    Particle.publish("heater", "Off");
  };
};
HeaterTimer heaterTimer = HeaterTimer();

void checkDhtStatus() {
  int result = DHT.acquireAndWait(1000); // wait up to 1 sec (default indefinitely)

  switch (result) {
  case DHTLIB_OK:
    // Particle.publish("state","OK");
    break;
  case DHTLIB_ERROR_CHECKSUM:
    Particle.publish("DHT Error","Checksum error");
    break;
  case DHTLIB_ERROR_ISR_TIMEOUT:
    Particle.publish("DHT Error","ISR time out error");
    break;
  case DHTLIB_ERROR_RESPONSE_TIMEOUT:
    Particle.publish("DHT Error","Response time out error");
    break;
  case DHTLIB_ERROR_DATA_TIMEOUT:
    Particle.publish("DHT Error","Data time out error");
    break;
  case DHTLIB_ERROR_ACQUIRING:
    Particle.publish("DHT Error","Acquiring");
    break;
  case DHTLIB_ERROR_DELTA:
    Particle.publish("DHT Error","Delta time to small");
    break;
  case DHTLIB_ERROR_NOTSTARTED:
    Particle.publish("DHT Error","Not started");
    break;
  default:
    Particle.publish("DHT Error","Unknown error");
    break;
  };
};

void setup() {
  Time.zone(-7); // UTC --> PST

  // define pins
  pinMode(ledPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);

  // init serial logging
  Serial.begin(9600);
  Serial.printlnf("setup...");
  Particle.publish("board state", "setup...");

  // track variables
  Particle.publish("setup:", "Particle variables");
  Particle.variable("temp_F", tempF);
  Particle.variable("humidity", humidity);
  Particle.variable("dewPoint", dewPoint);
  Particle.variable("heatIndex", heatIndex);
  Particle.variable("lights_On", lightsOn);
  Particle.variable("heater_On", heaterOn);

  Particle.publish("setup:", "I2C protocol");
  Wire.begin(); //Join I2C bus

  Particle.publish("setup:", "DHT communication");
  DHT.begin(); // begin reading temp/humidity via dht

  // begin temp/humidity via AHT20
  Particle.publish("setup:", "AHT20 communication");
  if (ahtSensor.begin() == false) Particle.publish("AHT20 setup", "failed");
  else Particle.publish("AHT20 setup", "success");
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

  checkDhtStatus();
  // germination cycle - use heater
  double germinationTempMin = 70; // F
  bool tempAverageBelowMin = tempsHistoryAverage < germinationTempMin;
  if (hasRecentTempReadings && tempAverageBelowMax && tempAverageBelowMin && !heaterTimer.active()) {
    digitalWrite(heaterPin, HIGH); // turn on
    heaterOn = true;
    Particle.publish("heater", "On");

    heaterTimer.start();
  }

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

  // LOGGING
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humidity = DHT.getHumidity();
  // float tempC = dht.getTempCelcius(); // read temp (C)
  // tempF = dht.getTempFarenheit(); // read temp (F)
  tempF = DHT.getFahrenheit();
  dewPoint = DHT.getDewPoint(); // (C)

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

  // read AHT20 sensor
  if (ahtSensor.available() == true)
  {
    double ahtTempC = ahtSensor.getTemperature();
    ahtHumidity = ahtSensor.getHumidity();
    ahtTempF = (ahtTempC * 9/5) + 32;
  } else {
    Particle.publish("AHT readings", "not available");
  }

  String readings = String(String::format("{\"Temp(°F)\": %4.2f, \"ahtTemp(°F)\": %4.2f, \"Hum(\%)\": %4.2f, \"ahtHum(\%)\": %4.2f, \"TempAvg(°F)\": %4.2f}", tempF, ahtTempF, humidity, ahtHumidity, tempsHistoryAverage));
  Particle.publish("readings", readings);

  delay(10000);
};
