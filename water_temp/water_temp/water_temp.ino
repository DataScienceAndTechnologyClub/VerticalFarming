#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

 float Celcius=0;
 float Fahrenheit=0;


// define reading function
int temperature(float *tempptr) {

  // read temperature as Celsius
  sensors.requestTemperatures();
  *tempptr = sensors.getTempCByIndex(0);


  if (isnan(*tempptr) || (*tempptr == -127)) {
    return 0;
  }
  else {
    return 1;
  }

}


void setup(void)
{
  
  Serial.begin(9600);
  sensors.begin();
}

void loop(void) {
  // wait a few seconds between measurements
  delay(2000);
  
  float temp, humid;

  // read temperature and humidity and give error if not successful
  if (!temperature(&temp)) {
  Serial.println("Failed to read from water_temperature sensor");
  } else {
    Serial.print("Water Temperature: ");
    Serial.print(temp);
    Serial.println(" *C\t");
    Serial.println("*******");
  }

}
