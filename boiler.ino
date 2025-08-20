#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusSlave.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <NextionX2.h>

//#define PIN_FLOW_SENSOR 2

#define PIN_REMOTE_CONTROL A0
#define PIN_TEMPERATURE A1
#define PIN_PUMP A2
#define PIN_FAN 2

#define PIN_COIL1 A3
#define PIN_COIL2 A4
#define PIN_COIL3 A5

#define TEMPERATURE_EEPROM_ADDRESS 0

#define MAX_TEMPERATURE 85
#define MIN_TEMPERATURE 20

#define HYSTERESIS 5

#define HEATING_COIL2_THRESHOLD 4
#define HEATING_COIL3_THRESHOLD 2
#define PUMP_TEMP_THRESHOLD 40

#define TEMP_SENSOR_FAILURE 0
#define FLOW_SENSOR_START_DELAY 10000
#define W1_POLL 10000
#define TIME_POLL 60000
#define TEMP_CHANGE_POLL 2000


// Screen
#define COLOR_OK 2016
#define COLOR_FAIL 64073
#define COLOR_NORMAL 50712
#define NEXTION_FAIL 0xFFFFFFFF

SoftwareSerial screenSerial(4, 5);

NextionComPort nextion;
NextionComponent tCoil1(nextion, 0, 4);
NextionComponent tCoil2(nextion, 0, 5);
NextionComponent tCoil3(nextion, 0, 6);

NextionComponent tPump(nextion, 0, 9);
NextionComponent tRemote(nextion, 0, 10);
NextionComponent tSensor(nextion, 0, 11);

NextionComponent nTSetpoint(nextion, 0, 2);
NextionComponent nTCurrent(nextion, 0, 3);

NextionComponent bPlus(nextion, 0, 7);
NextionComponent bMinus(nextion, 0, 8);

// Sensor
OneWire oneWire(PIN_TEMPERATURE);
DallasTemperature sensors(&oneWire);

// Modbus
#define PIN_RS484_RO 0 //tx
#define PIN_RS484_DI 1 //rx
#define PIN_RS484_SIG 3 // signal
#define MODBUS_ADDRESS 237

Modbus modbus(MODBUS_ADDRESS, PIN_RS484_SIG);

typedef struct {
  bool boiler;
  bool coil2;
  bool coil3;
  bool pump;
  bool remote;
//  bool flow;
  bool tempok;
  String time;
  String date;
  int temperature;
  int currentTemperature;
  unsigned long tempLastPoll;
  unsigned long tempChangeLastPoll;
  unsigned long pumpStartTime;
} State;

State state;
int forceRedraw = false;
int displayInit = false;

void bPlusCallback() {
  state.temperature = changeTemperature(state.temperature, state.temperature + 1);
  forceRedraw = true;
}

void bMinusCallback() {
  state.temperature = changeTemperature(state.temperature, state.temperature - 1);
  forceRedraw = true;
}

void setup() {
  pinMode(PIN_RS484_SIG, OUTPUT);

  Timer1.initialize(500);
  Timer1.attachInterrupt(modbusPoll, 500);

  modbus.cbVector[CB_READ_REGISTERS] = modbusOut;
  modbus.cbVector[CB_WRITE_MULTIPLE_REGISTERS] = modbusIn;

  Serial.begin(9600, SERIAL_8N2);
  modbus.begin(9600);

  nextion.begin(screenSerial);

  pinMode(PIN_TEMPERATURE, INPUT);
//  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  pinMode(PIN_REMOTE_CONTROL, INPUT_PULLUP);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);

  pinMode(PIN_COIL1, OUTPUT);
  pinMode(PIN_COIL2, OUTPUT);
  pinMode(PIN_COIL3, OUTPUT);

  state.boiler = false;
  state.coil2 = false;
  state.coil3 = false;
  state.pump = false;
  state.tempok = false;
  state.remote = getState(PIN_REMOTE_CONTROL);
//  state.flow = getState(PIN_FLOW_SENSOR);
  state.currentTemperature = 0;
  state.temperature = 75;
  state.tempLastPoll = 0;
  state.tempChangeLastPoll = 0;
  state.pumpStartTime = 0;

  int savedTemp = EEPROM.read(TEMPERATURE_EEPROM_ADDRESS);
  if (savedTemp) {
    state.temperature = changeTemperature(state.temperature, savedTemp);
  }

  sensors.begin();

  bPlus.release(bPlusCallback);
  bMinus.release(bMinusCallback);
}

void loop() {
  State oldState = state;

  manageBoiler();
  checkCoils();
  checkPump();

  updateScreen(oldState, state);

  if (state.tempChangeLastPoll > 0 && millis() - state.tempChangeLastPoll > TEMP_CHANGE_POLL) {
    EEPROM.update(TEMPERATURE_EEPROM_ADDRESS, state.temperature);
    state.tempChangeLastPoll = 0;
  }

  nextion.update();
}

void updateScreen(State oldState, State state)
{
  if (!displayInit) {
    if (nTSetpoint.value() == NEXTION_FAIL) {
      return;
    }

    displayInit = true;
    forceRedraw = true;
  }

  if (oldState.boiler != state.boiler || forceRedraw) {
    tCoil1.attribute("bco", state.boiler ? COLOR_OK : COLOR_NORMAL);
  }

  if (oldState.coil2 != state.coil2 || forceRedraw) {
    tCoil2.attribute("bco", state.coil2 ? COLOR_OK : COLOR_NORMAL);
  }

  if (oldState.coil3 != state.coil3 || forceRedraw) {
    tCoil3.attribute("bco", state.coil3 ? COLOR_OK : COLOR_NORMAL);
  }

  if (oldState.pump != state.pump|| forceRedraw) {
    tPump.attribute("bco", state.pump ? COLOR_OK : COLOR_NORMAL);
  }

  if (oldState.remote != state.remote || forceRedraw) {
    tRemote.attribute("bco", state.remote ? COLOR_OK : COLOR_NORMAL);
  }

  if (oldState.tempok != state.tempok || forceRedraw) {  
    tSensor.attribute("bco", state.tempok ? COLOR_OK : COLOR_FAIL);
  }

  if (oldState.currentTemperature != state.currentTemperature || forceRedraw) {
    nTCurrent.value(state.currentTemperature);
  }

  if (oldState.temperature != state.temperature || forceRedraw) {
    nTSetpoint.value(state.temperature);
  }

  forceRedraw = false;
}

void modbusPoll()
{
    modbus.poll();
}

uint8_t modbusOut(uint8_t fc, uint16_t address, uint16_t length)
{
    int readAddress = 0;
    for (int i = 0; i < length; i++) {
        readAddress = address + i;
        if (readAddress == 0) {
            modbus.writeRegisterToBuffer(i, state.currentTemperature);
        } else if (readAddress == 1) {
            modbus.writeRegisterToBuffer(i, state.temperature);
        } else if (readAddress == 2) {
            modbus.writeRegisterToBuffer(i, state.boiler ? 1 : 0);
        } else if (readAddress == 3) {
            modbus.writeRegisterToBuffer(i, state.coil2 ? 1 : 0);
        } else if (readAddress == 4) {
            modbus.writeRegisterToBuffer(i, state.coil3 ? 1 : 0);
        } else if (readAddress == 5) {
            modbus.writeRegisterToBuffer(i, state.pump ? 1 : 0);
        } else if (readAddress == 6) {
            modbus.writeRegisterToBuffer(i, state.remote ? 1 : 0);
        } else if (readAddress == 7) {
            modbus.writeRegisterToBuffer(i, state.tempok ? 1 : 0);
        }
    }

    return STATUS_OK;
}

uint8_t modbusIn(uint8_t fc, uint16_t address, uint16_t length) 
{
    int value;
    int readAddress = 0;
    
    for (int i = 0; i < length; i++) {
        readAddress = address + i;

        if (readAddress == 1) {
          value = modbus.readRegisterFromBuffer(i);
          state.temperature = changeTemperature(state.temperature, value);
          forceRedraw = true;
        }
    }

    return STATUS_OK;
}

void manageBoiler()
{
  /*if (millis() < state.pumpStartTime) { // overflow, update pumpStartTime
    state.pumpStartTime = millis();
  }

  // boiler set to failed state - always off
  // need reset to restart
  if (failedState == true) {
    switchBoiler(false, true);
    return;
  }

  // set off: no flow
  if (getState(PIN_FLOW_SENSOR) == false && state.pump == true && (millis() - state.pumpStartTime) >= FLOW_SENSOR_START_DELAY) {
    failedState = true;
    switchBoiler(false, true);
    return;
  }*/

  // set off: temperature sensor failure
  if (getTemperature() == false) {
    switchBoiler(false, true);
    return;
  }

  // set off: remote control
  if (getState(PIN_REMOTE_CONTROL) == false) {
    switchBoiler(false, false);
    return ;
  }

  // set off: is on and current temperature is equal or higher than target temperature
  if (state.boiler == true && state.currentTemperature >= state.temperature) {
    switchBoiler(false, false);
    return;
  }

  // set on: is off and current temperature less than target temperature minus hysteresis
  if (state.boiler == false && state.currentTemperature <= state.temperature - HYSTERESIS) {
    switchBoiler(true, false);
    return;
  }

  return;
}

void checkPump()
{
  if (state.boiler == false && state.currentTemperature <= PUMP_TEMP_THRESHOLD) {
    writePin(PIN_PUMP, LOW);
    writePin(PIN_FAN, LOW);
  }
}

void checkCoils()
{
    if (state.boiler == false) {
      return;
    }

    bool newState;

    newState = state.currentTemperature < state.temperature - HEATING_COIL2_THRESHOLD;
    if (state.coil2!= newState) {
      delay(1000);
      writePin(PIN_COIL2, newState ? HIGH : LOW);
    }

    newState = state.currentTemperature < state.temperature - HEATING_COIL3_THRESHOLD;
    if (state.coil3 != newState) {
      delay(1000);
      writePin(PIN_COIL3, newState ? HIGH : LOW);
    }
}

bool switchBoiler(bool on, bool forceoff)
{
  if (on == state.boiler) {
    return false;
  }

  int value = on ? HIGH : LOW;

  if (on == true) {
    writePin(PIN_PUMP, HIGH);
    writePin(PIN_FAN, HIGH);
  }

  if (on == false && forceoff == true) {
    writePin(PIN_PUMP, LOW);
  }

  writePin(PIN_COIL1, value);
  delay(1000);
  writePin(PIN_COIL2, value);
  delay(1000);
  writePin(PIN_COIL3, value);

  return true;
}


/*bool handleKeys()
{
  // reset failed state
  if (digitalRead(PIN_KEY3) == LOW) {
    failedState = false;
    return true;
  }

  return false;
}*/

int changeTemperature(int oldTemperature, int newTemperature)
{
  if (oldTemperature == newTemperature) {
    return oldTemperature;
  }

  if (newTemperature > MAX_TEMPERATURE || newTemperature < MIN_TEMPERATURE) {
    return oldTemperature;
  }

  state.tempChangeLastPoll = millis();
  return newTemperature;
}

bool writePin(unsigned char pin, int value)
{
  switch(pin)
  {
    case PIN_PUMP:
      if (state.pump == (value == HIGH)) {
        return false;
      }
      state.pump = (value == HIGH);
      if (state.pump == true) {
        state.pumpStartTime = millis();
      }
      digitalWrite(pin, value);
      break;

    case PIN_FAN:
      digitalWrite(pin, value);
      break;

    case PIN_COIL1:
      if (state.boiler == (value == HIGH)) {
        return false;
      }
      state.boiler = (value == HIGH);
      digitalWrite(pin, value);
      break;

    case PIN_COIL2:
      if (state.coil2 == (value == HIGH)) {
        return false;
      }
      state.coil2 = (value == HIGH);
      digitalWrite(pin, value);
      break;

    case PIN_COIL3:
      if (state.coil3 == (value == HIGH)) {
        return false;
      }
      state.coil3 = (value == HIGH);
      digitalWrite(pin, value);
      break;
  }
}

bool getTemperature()
{
    int i = 0;
    int value = 0;
    int max_tries = 3;
    int sleep = 5000;

    if (millis() < state.tempLastPoll) { // overflow
      state.tempLastPoll = millis();
    }

    if ((millis() - state.tempLastPoll) > W1_POLL || state.tempLastPoll == 0) {
      state.tempLastPoll = millis();
      for (i = 0; i <= max_tries; i++) {
        sensors.requestTemperatures();
        value = round(sensors.getTempCByIndex(0));
        if (value > TEMP_SENSOR_FAILURE) {
          break;
        }
        delay(i * sleep);
      }

      state.currentTemperature = value;
      state.tempok = value > TEMP_SENSOR_FAILURE;
    }


    return state.tempok;
}

bool getState(unsigned char pin)
{
  int value = 0;

  value = digitalRead(pin);

  switch(pin)
  {
    /*case PIN_FLOW_SENSOR:
      state.flow = (value == HIGH);
      return state.flow;*/
    case PIN_REMOTE_CONTROL:
      state.remote = (value == LOW);
      return state.remote;
  }
}
