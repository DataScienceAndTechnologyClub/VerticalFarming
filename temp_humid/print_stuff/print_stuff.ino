
#include "Adafruit_Sensor.h"

#include "DHT.h"
#include "DHT_U.h"

#define DHTPIN 2  // what digital pin we're connected to

#define DHTTYPE DHT22

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHTxx test!");

  dht.begin();
}

void loop() {
  // wait a few seconds between measurements
  delay(2000);

  // reading temperature or humidity takes about 250 milliseconds!
  // sensor readings may also be up to 2 seconds old (its a very slow sensor)
  float h = dht.readHumidity();

  // read temperature as Celsius
  float t = dht.readTemperature();  //for farenheit dht.readTemperature(true):

  // Check if any reads failed and exit early (to try again)
  if (isnan(h) || isnan(t) ) {
    Serial.println("Failed to read from DHT sensor");
    return;
  }

  // compute heat index in Celsius
  float hic = dht.computeHeatIndex(t,h,false);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.println(" *C");
  Serial.println("*******");
}
