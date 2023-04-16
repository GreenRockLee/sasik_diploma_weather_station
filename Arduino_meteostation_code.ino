//Libraries
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Firebase_Arduino_WiFiNINA.h>
#include <NTPClient.h>
#include <time.h> 

//Values for connection
//DAtabase
#define FIREBASE_HOST "your_host.firebaseio.com"
#define FIREBASE_AUTH "your_auth"
//Wifi
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
//Wind
#define windDir A0 // pin A0 for wind direction
#define ANEMOMETER_PIN 3 // pin D3 for wind speed
#define DEBOUNCE_TIME 15
#define CALC_INTERVAL 1000
//Precipitation
#define RAIN_PIN 2           // pin D2 fow rain gauge
#define CALC_INTERVAL 1000  // increment of measurements
#define DEBOUNCE_TIME 150   // referenced in ISR; multiply by 1000 if using a reed switch
//Pressure
#define SEALEVELPRESSURE_HPA (1013.25)

//Firebase
FirebaseData firebaseData;

//BME280 inicialization
Adafruit_BME280 bme; // I2C

//wind 
int sensorExp[] = {66,84,93,126,184,244,287,406,461,599,630,702,785,827,886,945};
float dirDeg[] = {112.5,67.5,90,157.5,135,202.5,180,22.5,45,247.5,225,337.5,0,292.5,315,270};
char* dirCard[] = {"ESE","ENE","E","SSE","SE","SSW","S","NNE","NE","WSW","SW","NNW","N","WNW","NW","W"};
int sensorMin[] = {63,80,89,120,175,232,273,385,438,569,613,667,746,812,869,931};
int sensorMax[] = {69,88,98,133,194,257,301,426,484,612,661,737,811,868,930,993};
int incoming = 0;
float angle = 0;
char* dir = "temp";
unsigned long _nextCalc;
unsigned long wind_timer;
volatile int _anemometerCounter;
volatile unsigned long last_micros_an;
int _windSpd;

//Precipitation
unsigned long nextCalc;
unsigned long rain_timer;
volatile unsigned int rainTrigger = 0;
volatile unsigned long last_micros_rg;

//Variables
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 7200, 60000);
String jsonStr;
String actual_year;
String actual_time;
unsigned long delayTime;
String jsonTemperature, jsonHumidity, jsonAir_pressure, jsonAltitude, jsonWind_direction, jsonWind_speed, jsonPrecipitation;


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);

//Wifi connection
  Serial.print("Connecting to WiFi...");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(".");
    Serial.println();
    delay(300);
  }
  Serial.print("Successfully connected, here is your IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

//Firebase inicialization
Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
Firebase.reconnectWiFi(true);
Serial.println("Firebase is working.");

//Time start
timeClient.begin();
Serial.println("Time is working.");

//Wind
pinMode(ANEMOMETER_PIN, INPUT);
attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countAnemometer, FALLING); 
Serial.println("Wind sensor is working.");

//Precipitation
attachInterrupt(digitalPinToInterrupt(RAIN_PIN), countingRain, RISING); 
pinMode(RAIN_PIN, INPUT);
nextCalc = millis() + CALC_INTERVAL;
Serial.println("Precipitation sensor is working.");

//BME280 setup
unsigned bme_status;
bme_status = bme.begin(0x76);  
    if (!bme_status) {
        Serial.println("There is the problem with BME280 sensor.");
    }else{
        Serial.println("BME280 sensor is working.");
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Time update
  timeClient.update();

//Recalculation of current time and date
  unsigned long epochTime = timeClient.getEpochTime();
  epochTime -= 63072000; // Subtract 2 years in seconds (60 * 60 * 24 * 365 * 2)
  struct tm *ptm = gmtime((time_t *)&epochTime);

  int day = ptm->tm_mday;
  int month = ptm->tm_mon + 1;
  int year = ptm->tm_year + 1872;
  actual_year = String(day) + "." + String(month) + "." + String(year);
  actual_time = actual_year + " " + timeClient.getFormattedTime();
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Wind direction
  incoming = analogRead(windDir);
  for(int i=0; i<=15; i++) {
   if(incoming >= sensorMin[i] && incoming <= sensorMax[i]) {
    dir = dirCard[i];
    break;
   } 
  }

//Wind speed
 wind_timer = millis();
  
 if(wind_timer > _nextCalc) {
  _nextCalc = wind_timer + CALC_INTERVAL;
  //UPDATE ALL VALUES
  _windSpd = readWindSpd();
 }

 //Precipitation
   rain_timer = millis();
  if(rain_timer > nextCalc) {
    nextCalc = rain_timer + CALC_INTERVAL;   
  }
 /////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.println("Setting values into JSON format");
  jsonTemperature = "{\"time\":\"" + actual_time + "\",\"value\":" + String(bme.readTemperature()) + "}";
  delay(1000);
  jsonHumidity = "{\"time\":\"" + actual_time + "\",\"value\":" + String(bme.readHumidity()) + "}";
  delay(1000);
  jsonAir_pressure = "{\"time\":\"" + actual_time + "\",\"value\":" + String(bme.readPressure() /100) + "}";
  delay(1000);
  jsonAltitude = "{\"time\":\"" + actual_time + "\",\"value\":" + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + "}";
  delay(1000);
  jsonWind_direction = "{\"time\":\"" + actual_time + "\",\"value\":" + String(dir) + "}";
  delay(1000);
  jsonWind_speed = "{\"time\":\"" + actual_time + "\",\"value\":" + String(_windSpd) + "}";
  delay(1000);
  jsonPrecipitation = "{\"time\":\"" + actual_time + "\",\"value\":" + String(rainTrigger) + "}";
  delay(5000);


    //Send data to Firebase
    Serial.println("Trying to send data to Firebase");
    //TEMPERATURE
    if (Firebase.pushJSON(firebaseData, "/Temperature", jsonTemperature)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    //Humidity
    if (Firebase.pushJSON(firebaseData, "/Humidity", jsonHumidity)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    //Air_pressure
    if (Firebase.pushJSON(firebaseData, "/Air_pressure", jsonAir_pressure)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    //Altitude
    if (Firebase.pushJSON(firebaseData, "/Altitude", jsonAltitude)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    //Wind_direction
    if (Firebase.pushJSON(firebaseData, "/Wind_Direction", jsonWind_direction)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    //Wind_speed
    if (Firebase.pushJSON(firebaseData, "/Wind_Speed", jsonWind_speed)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    //Precipitation
    if (Firebase.pushJSON(firebaseData, "/Precipitation", jsonPrecipitation)) {
      delay(6000);
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }
    Serial.println("Data has been sent successfully. I'm gonna take a nap for 30 seconds.");

}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


//returns the wind speed since the last calcInterval.
int readWindSpd() {
 unsigned char i;  
 long spd = 24000; // one turn = 2.4 kilometers per hour
 spd *= _anemometerCounter;
 spd /= 10000;
 _anemometerCounter = 0;
 return (int) spd;
}

void countAnemometer() {
 if((long)(micros() - last_micros_an) >= DEBOUNCE_TIME * 1000) {
    _anemometerCounter++;
    last_micros_an = micros();
 }
}

void countingRain() {
  if((long)(micros() - last_micros_rg) >= DEBOUNCE_TIME) { 
   rainTrigger += 1;
   last_micros_rg = micros();
  }  
}