
// libraries for measurin temperature
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "DHT_U.h"

#define DHTPIN 2  // what digital pin the DHT is connected to

#define DHTTYPE DHT22  //type of our DHT

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// define reading function
int temperature(float *tempptr, float *humidptr) {

  // read temperature as Celsius
  *tempptr = dht.readTemperature();

  // read Humidity
  *humidptr = dht.readHumidity();

  if (isnan(*tempptr) || isnan(*humidptr) ) {
    return 0;
  }
  else {
    return 1;
  }

}

void setup() {
  Serial.begin(9600);

  dht.begin();  //to initialize DHT sensor
}

void loop() {
  // wait a few seconds between measurements
  delay(2000);
  
  float temp, humid;

  // read temperature and humidity and give error if not successful
  if (!temperature(&temp, &humid)) {
  Serial.println("Failed to read from DHT sensor");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humid);
    Serial.println(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" *C\t");
    Serial.println("*******");
  }
}
