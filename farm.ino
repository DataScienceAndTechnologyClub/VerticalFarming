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


/* PH Sensor */

#define PH_PIN A2
#define PH_SAMPLE_N 10
#define PH_INIT_OFFSET 2.8 //0
#define PH_INIT_SLOPE 2.09 //3.5

double ph_offset;
double ph_slope;
double ph_cal7;
double ph_cal4;
bool ph_cal;

void initPH() {
  ph_offset = PH_INIT_OFFSET;
  ph_slope = PH_INIT_SLOPE;
  ph_cal7 = NAN;
  ph_cal4 = NAN;
  ph_cal = false;
  pinMode(PH_PIN, INPUT);
}

double readPH() {
  return readPHVoltage()*ph_slope + ph_offset;
}

double readPHVoltage() {
  double volts = 0;
  for (int i=0;i<PH_SAMPLE_N;i++) {
    volts += analogRead(PH_PIN)/1024.0*5;
    delay(1);
  }
  volts/=PH_SAMPLE_N;
  return volts;
}

void calibratePH(char cmd[]) {
  if (strstr(cmd,"ENTERPH")) {
    if (!ph_cal) Serial.println("SUCCESS: Begin calibration of pH probe. Put it into pH 7 buffer and type 'CALPH'. Type 'EXITPH' to abort.");
    else Serial.println("SUCCESS: pH calibration reset. Put it into pH 7 buffer and type 'CALPH'. Type 'EXITPH' to abort.");
    ph_cal = true;
    ph_cal4 = NAN;
    ph_cal7 = NAN;
  }
  else if (strstr(cmd,"CALPH")) {
    if (!ph_cal) {
      Serial.println("ERROR: pH probe not in calibration mode. Type 'ENTERPH' first.");
      return;
    }
    double volts = readPHVoltage();
    Serial.print("Volts: ");
    Serial.println(volts);
    Serial.print("Buffer solution: ");
    if (isnan(ph_cal7)) {
      Serial.println(7);
      ph_cal7 = volts;
      Serial.println("SUCCESS: Put pH probe into pH 4 buffer and type 'CALPH'. Type 'ENTERPH' to reset, or 'EXITPH' to abort.");
    }
    else if (isnan(ph_cal4)) {
      Serial.println(4);
      ph_cal4 = volts;
      Serial.println("SUCCESS: Type 'EXITPH' to finalize calibration. Type 'ENTERPH' to reset.");
    }
    else {
      Serial.println("ERROR: Calibration measures already taken. Type 'EXITPH' to finalize calibration, or 'ENTERPH' to reset.");
    }
  }
  else if (strstr(cmd,"EXITPH")) {
    if (!ph_cal) {
      Serial.println("ERROR: pH probe not in calibration mode. Type 'ENTERPH' first.");
      return;
    }
    if (isnan(ph_cal7) || isnan(ph_cal4)) {
      Serial.println("SUCCESS: pH calibration incomplete. Aborting.");
    }
    else {
      Serial.println("SUCCESS: pH calibration complete.");
      ph_slope = 3.0/(ph_cal7 - ph_cal4);
      ph_offset = 4.0 - ph_slope * ph_cal4;
      Serial.print("Offset: ");
      Serial.println(ph_offset);
      Serial.print("Slope: ");
      Serial.println(ph_slope);
    }
    ph_cal = false;
  }
  else {
    Serial.println("ERROR: Command unknown. Available commands: 'ENTERPH', 'CALPH', 'EXITPH'.");  
  }
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
#define TEMPERATURE_TIMEOUT 1000

OneWire temp_sensor(TEMPERATURE_PIN);

double last_temperature;
unsigned long last_temperature_time;

void initTemperature() {
  last_temperature = NAN;
  last_temperature_time = 0;
}

double readTemperature() {

  if (!isnan(last_temperature) && millis()<last_temperature_time+TEMPERATURE_TIMEOUT)
    return last_temperature;
    
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

  last_temperature = ((data[1] << 8) | data[0]) / 16;
  last_temperature_time = millis();
  return last_temperature;
}


/* Main Program */

#define BAUD_RATE 115200

void setup() {
  Serial.begin(BAUD_RATE);
  initTemperature();
  initEC();
  initPH();
}

void loop() {

  // Process serial commands.
  char cmd[64];
  if (readLine(cmd)) {
    if (strstr(cmd,"EC")) calibrateEC(cmd);
    else if (strstr(cmd,"PH")) calibratePH(cmd);
    else if (strstr(cmd,"DOWN")) pump(PHDOWN_PUMP_PIN,5);
    else if (strstr(cmd,"MACRO")) pump(MACRONUT_PUMP_PIN,5);
    else if (strstr(cmd,"MICRO")) pump(MICRONUT_PUMP_PIN,5);
    else Serial.println("ERROR: Invalid command.");
  }

  Serial.print("PH: ");
  Serial.println(readPH());

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
