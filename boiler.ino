#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusSlave.h>
#include <TimerOne.h>
#include <Nextion.h>
#include <EEPROM.h>

#define COLORED     0
#define UNCOLORED   1

//#define PIN_FLOW_SENSOR 2

#define PIN_RS484_RO 0 //tx
#define PIN_RS484_DI 1 //rx
#define PIN_RS484_SIG 3 // signal


#define PIN_REMOTE_CONTROL A0
#define PIN_TEMPERATURE A1
#define PIN_PUMP A2
#define PIN_FAN 2

#define PIN_COIL1 A3
#define PIN_COIL2 A4
#define PIN_COIL3 A5


#define TEMPERATURE_EEPROM_ADDRESS 0

#define MAX_TEMPERATURE 100
#define MIN_TEMPERATURE 30

#define INCREASE_TEMP '+'
#define DECREASE_TEMP '-'

#define HYSTERESIS 5

#define HEATING_COIL2_THRESHOLD 4
#define HEATING_COIL3_THRESHOLD 2
#define PUMP_TEMP_THRESHOLD 40

#define TEMP_SENSOR_FAILURE 0
#define FLOW_SENSOR_START_DELAY 10000
#define W1_POLL 10000
#define TIME_POLL 60000
#define TEMP_CHANGE_POLL 2000

#define COLOR_OK 2016
#define COLOR_FAIL 64073
#define COLOR_NORMAL 50712

NexText tCoil1 = NexText(0, 1, "tCoil1");
NexText tCoil2 = NexText(0, 2, "tCoil2");
NexText tCoil3 = NexText(0, 3, "tCoil3");

NexText tPump = NexText(0, 9, "tPump");
NexText tRemote = NexText(0, 10, "tRemote");
NexText tSensor = NexText(0, 11, "tSensor");

NexNumber nTSetpoint = NexNumber(0, 6, "nTSetpoint");
NexNumber nTCurrent = NexNumber(0, 7, "nTCurrent");

NexButton bPlus = NexButton(0, 4, "bPlus");
NexButton bMinus = NexButton(0, 5, "bMinus");

NexTouch *nex_listen_list[] = {
  &bPlus,
  &bMinus,
  NULL
};

OneWire oneWire(PIN_TEMPERATURE);
DallasTemperature sensors(&oneWire);

Modbus slave(237, PIN_RS484_SIG); // [stream = Serial,] slave id = 237, rs485 control-pin = 3

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

void bPlusCallback(void *ptr) {
  state.temperature = changeTemperature(state.temperature, INCREASE_TEMP);
}

void bMinusCallback(void *ptr) {
  state.temperature = changeTemperature(state.temperature, DECREASE_TEMP);
}

void setup() {
  pinMode(PIN_RS484_SIG, OUTPUT);

  Timer1.initialize(500);
  Timer1.attachInterrupt(modbusPoll, 500);

  slave.cbVector[CB_READ_REGISTERS] = modbusOut;
  
  Serial.begin(9600, SERIAL_8N2);
  slave.begin(9600);

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
  state.remote = getState(PIN_REMOTE_CONTROL);
//  state.flow = getState(PIN_FLOW_SENSOR);
  state.temperature = 75;
  state.tempLastPoll = 0;
  state.tempChangeLastPoll = 0;
  getTemperature();
  state.pumpStartTime = 0;

  int savedTemp = EEPROM.read(TEMPERATURE_EEPROM_ADDRESS);
  if (savedTemp) {
    state.temperature = savedTemp;
  }

  sensors.begin();

  nexInit();

  bPlus.attachPop(bPlusCallback, &bPlus);
  bMinus.attachPop(bMinusCallback, &bMinus);
}

void loop() {
  State oldState = state;

  manageBoiler();
  checkCoils();
  checkPump();

  if (compareStates(oldState, state)) {
    updateScreen();
  }

  nexLoop(nex_listen_list);

  if (state.tempChangeLastPoll > 0 && millis() - state.tempChangeLastPoll > TEMP_CHANGE_POLL) {
    EEPROM.write(TEMPERATURE_EEPROM_ADDRESS, state.temperature);
    state.tempChangeLastPoll = 0;
  }

  delay(1000);
}

void updateScreen()
{
  tCoil1.Set_background_color_bco(state.boiler ? COLOR_OK : COLOR_NORMAL);
  tCoil2.Set_background_color_bco(state.coil2 ? COLOR_OK : COLOR_NORMAL);
  tCoil3.Set_background_color_bco(state.coil3 ? COLOR_OK : COLOR_NORMAL);

  tPump.Set_background_color_bco(state.pump ? COLOR_OK : COLOR_NORMAL);
  tRemote.Set_background_color_bco(state.remote ? COLOR_OK : COLOR_NORMAL);
  tSensor.Set_background_color_bco(state.tempok ? COLOR_OK : COLOR_FAIL);

  nTCurrent.setValue(state.currentTemperature);
  nTSetpoint.setValue(state.temperature);
}

void modbusPoll()
{
    slave.poll();
}

uint8_t modbusOut(uint8_t fc, uint16_t address, uint16_t length)
{
    int readAddress = 0;
    for (int i = 0; i < length; i++) {
        readAddress = address + i;
        if (readAddress == 0) {
            slave.writeRegisterToBuffer(i, state.currentTemperature);
        } else if (readAddress == 1) {
            slave.writeRegisterToBuffer(i, state.temperature);
        } else if (readAddress == 2) {
            slave.writeRegisterToBuffer(i, state.boiler ? 1 : 0);
        } else if (readAddress == 3) {
            slave.writeRegisterToBuffer(i, state.coil2 ? 1 : 0);
        } else if (readAddress == 4) {
            slave.writeRegisterToBuffer(i, state.coil3 ? 1 : 0);
        } else if (readAddress == 5) {
            slave.writeRegisterToBuffer(i, state.pump ? 1 : 0);
        } else if (readAddress == 6) {
            slave.writeRegisterToBuffer(i, state.remote ? 1 : 0);
        } else if (readAddress == 7) {
            slave.writeRegisterToBuffer(i, state.tempok ? 1 : 0);
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

int changeTemperature(int temperature, char sign)
{
  if (sign == INCREASE_TEMP && temperature < MAX_TEMPERATURE) {
    temperature++;
    state.tempChangeLastPoll = millis();
  }

  if (sign == DECREASE_TEMP && temperature > MIN_TEMPERATURE) {
    temperature--;
    state.tempChangeLastPoll = millis();
  }

  return temperature;
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

bool compareStates(State old, State current)
{
  if (old.boiler != current.boiler ||
      old.coil2 != current.coil2 ||
      old.coil3 != current.coil3 ||
      old.pump != current.pump ||
      old.remote != current.remote ||
//      old.flow != current.flow ||
      old.temperature != current.temperature ||
      old.currentTemperature != current.currentTemperature ||
      old.tempok != current.tempok) {
        return true;
  }

  return false;
}
