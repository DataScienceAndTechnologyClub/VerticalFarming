#include <DFRobot_EC.h> // Download from https://github.com/DFRobot/DFRobot_EC
#include <OneWire.h> // Download from http://www.dfrobot.com/image/data/DFR0198/DFRobot%20DFR0198.zip
#include <math.h>
#include <EEPROM.h>


/* Peristaltic Pumps */

#define PHDOWN_PUMP_PIN 8
#define MICRONUT_PUMP_PIN 7
#define MACRONUT_PUMP_PIN 4
#define PUMP_SPEED 1.13636

void initPumps() {
  pinMode(PHDOWN_PUMP_PIN, OUTPUT);
  pinMode(MICRONUT_PUMP_PIN, OUTPUT);
  pinMode(MACRONUT_PUMP_PIN, OUTPUT);
}

void pump(int pump_pin, double ml) {
  digitalWrite(pump_pin, HIGH);
  delay(round(ml/PUMP_SPEED*1000.0));
  digitalWrite(pump_pin, LOW);
}


/* EC Sensor */

#define EC_PIN A1
#define EC_SAMPLE_N 10

DFRobot_EC ec_sensor;

void initEC() {
  ec_sensor.begin();
  pinMode(EC_PIN, INPUT);  
}

double readEC() {
  return ec_sensor.readEC(readECVoltage(),readTemperature());
}

double readECVoltage() {
  double volts = 0;
  for (int i=0;i<EC_SAMPLE_N;i++) {
    volts+=analogRead(EC_PIN)/1024.0*5000;
    delay(1);
  }
  volts/=EC_SAMPLE_N;
  return volts;
}

double calibrateEC(char cmd[]) {
  ec_sensor.calibration(readECVoltage(),readTemperature(),cmd);
}


/* Temperature Sensor */

#define TEMPERATURE_PIN 9

OneWire temp_sensor(TEMPERATURE_PIN);

double readTemperature() {
  byte data[12];
  byte addr[8];

  if (!temp_sensor.search(addr)) {
    temp_sensor.reset_search();
    return NAN;
  }

  if (OneWire::crc8(addr,7)!=addr[7]) {
    Serial.println("Temperature: CRC is not valid.");
    return NAN;
  }

  if (addr[0]!=0x10 && addr[0]!=0x28) {
    Serial.println("Temperature: Device is not recognized.");
    return NAN;
  }

  temp_sensor.reset();
  temp_sensor.select(addr);
  temp_sensor.write(0x44,1);

  temp_sensor.reset();
  temp_sensor.select(addr);
  temp_sensor.write(0xBE);

  for (int i=0;i<9;i++) {
    data[i] = temp_sensor.read();
  }
  temp_sensor.reset_search();

  return ((data[1] << 8) | data[0]) / 16;
}


/* Main Program */

#define BAUD_RATE 115200

void setup() {
  Serial.begin(BAUD_RATE);
  initEC();
}

void loop() {

  // Process serial commands.
  char cmd[64];
  if (readLine(cmd)) {
    if (strstr(cmd,"EC")) calibrateEC(cmd);
    else if (strstr(cmd,"PHDOWN")) pump(PHDOWN_PUMP_PIN,5);
    else if (strstr(cmd,"MACRO")) pump(MACRONUT_PUMP_PIN,5);
    else if (strstr(cmd,"MICRO")) pump(MICRONUT_PUMP_PIN,5);
    else Serial.println("ERROR: Invalid command.");
  }

  Serial.print("EC: ");
  Serial.print(readEC());
  Serial.println(" ms/cm");

  Serial.print("Temp.: ");
  Serial.print(readTemperature());
  Serial.println(" °C");

  Serial.println(" ");
  delay(500);
}

int cmdPos = 0;
bool readLine(char cmd[]) {
  while (Serial.available()>0) {
    char in = Serial.read();
    if (in=='\n') {
      cmd[cmdPos] = '\0';
      Serial.flush();
      cmdPos=0;
      strupr(cmd);
      return true;
    }
    if (in!='\r') {
      cmd[cmdPos++] = in;
    }
    delay(1);
  }
  return false;
}
