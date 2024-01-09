#include <AccelStepper.h>
#include <ctype.h>
#include <string.h>

// Pin and stepper definitions
// Tube joints
const int STEP_PIN_TOP = 3;
const int DIRECTION_PIN_TOP = 2;
const int STEP_PIN_BOT = 6;
const int DIRECTION_PIN_BOT = 5;
const int SLEEP_PIN_TOP = 4;
const int SLEEP_PIN_BOT = 7;
AccelStepper stepperTop(AccelStepper::DRIVER, STEP_PIN_TOP, DIRECTION_PIN_TOP); // (Driver default has step_pin = 2, direction_pin = 3)
AccelStepper stepperBot(AccelStepper::DRIVER, STEP_PIN_BOT, DIRECTION_PIN_BOT); // (Driver default has step_pin = 2, direction_pin = 3)
// Valve
const int STEP_PIN_VALVE = 9;
const int DIRECTION_PIN_VALVE = 8;
const int SLEEP_PIN_VALVE = 10;
AccelStepper stepperValve(AccelStepper::DRIVER, STEP_PIN_VALVE, DIRECTION_PIN_VALVE); // (Driver default has step_pin = 2, direction_pin = 3)


// Kinematic definitions
const int accel = 500;
// Tube joints
const int max_speed_top = 100;
const int max_speed_bot = 200;
const int dir_top = -1; // Switch between 1 and -1 to reverse stepper directions
const int dir_bot = 1;
// Valve
const int max_speed_valve = 260;
const int steps_valve = 792; // 3.9*812/4

// Serial signal definitions
// Tube joints
const int stepper_number = 2;
const int number_size = 5;
const int signal_size = stepper_number*number_size;
String signal = "";  // Signal consists of 10 chars, 5 for each stepper, with format sddddsdddd where s is sign (0 or -) and d are the digits. Positive means CCW from top view.
String signal_empty = "";
// Valve
const int valve_signal_size = 2;
String valve_signal_list = "v01"; // v0 closes valve, v1 opens valve

// Others
boolean running_top = false;
boolean running_bot = false;
boolean running_valve = false;

void setup() {
// put your setup code here, to run once:
  Serial.begin(9600);

  stepperTop.setMaxSpeed(max_speed_top);
  stepperTop.setAcceleration(accel);
  stepperBot.setMaxSpeed(max_speed_bot);
  stepperBot.setAcceleration(accel);

  stepperValve.setMaxSpeed(max_speed_valve);
  stepperValve.setAcceleration(accel);

  for (int i = 0; i < signal_size; i++) {
    signal.concat(0);
  }
  signal_empty = signal;

  pinMode(SLEEP_PIN_TOP, OUTPUT);
  pinMode(SLEEP_PIN_BOT, OUTPUT);
  pinMode(SLEEP_PIN_VALVE, OUTPUT);

  DriverSleep(SLEEP_PIN_TOP);
  DriverSleep(SLEEP_PIN_BOT);
  DriverSleep(SLEEP_PIN_VALVE);
}


void loop() {
// put your main code here, to run repeatedly:
  stepperTop.run();
  stepperBot.run();
  stepperValve.run();

  if (Serial.available() > 0) {
    signal = Serial.readString();
    signal.trim();
    if (isValidSignal(signal)){
      signalToTarget(signal);
      running_top = true;
      running_bot = true;
    }
    else if (isValidValveSignal(signal)){
      signalToValve(signal);
      running_valve = true;
    }
    signal = signal_empty;
  }

//  if (running_top && !stepperTop.isRunning()) {
//    running_top = false;
//    DriverSleep(SLEEP_PIN_TOP);
//    Serial.println("top finished");
//  }
//
//  if (running_bot && !stepperBot.isRunning()) {
//    running_bot = false;
//    DriverSleep(SLEEP_PIN_BOT);
//    Serial.println("bot finished");
//  }

  if (running_top && running_bot && !stepperTop.isRunning() && !stepperBot.isRunning()) {
    running_top = false;
    running_bot = false;
    DriverSleep(SLEEP_PIN_TOP);
    DriverSleep(SLEEP_PIN_BOT);
    Serial.println("pos finished");
  }

  if (running_valve && !stepperValve.isRunning()) {
    running_valve = false;
    DriverSleep(SLEEP_PIN_VALVE);
    Serial.println("val finished");
  }
}


void newTarget(int steps_top, int steps_bot){
// Sets target position to specified number of steps
  DriverWakeUp(SLEEP_PIN_TOP);
  DriverWakeUp(SLEEP_PIN_BOT);
  // Serial.println("Updated target");
  stepperTop.move(dir_top*steps_top);
  stepperBot.move(dir_bot*steps_bot);
}

void signalToTarget(String signal){
  String steps_top = signal.substring(0, number_size);
  String steps_bot = signal.substring(number_size, 2*number_size);
  newTarget(steps_top.toInt(), steps_bot.toInt());
}

boolean isValidNumber(String str){
  // if(str.length() != number_size) return false;
  if(str.charAt(0) != '0' && str.charAt(0) != '-') return false;
  for(byte i=1; i<str.length(); i++){
    if(!isDigit(str.charAt(i))) return false;
  }
  return true;
}

boolean isValidSignal(String str){
  if(str.length() != signal_size) return false;
  for (int i = 0; i < stepper_number; i++) {
    if(!isValidNumber(str.substring(i*number_size, (i+1)*number_size))) return false;
  }
  return true;
}

void DriverWakeUp(int SLEEP_PIN){
  digitalWrite(SLEEP_PIN, HIGH);
  delay(100); // Driver Datasheet: "When emerging from Sleep mode, in order to allow the charge pump to stabilize, provide a delay of 1 ms before issuing a Step command."
}

void DriverSleep(int SLEEP_PIN){
  digitalWrite(SLEEP_PIN, LOW);
}


boolean isValidValveSignal(String str){
  if(str.length() != valve_signal_size) return false;
  if(str.charAt(0) != valve_signal_list.charAt(0)) return false;
//  for (int i = 1; i < valve_signal_list.length(); i++) {
//    if(str.charAt(1) != valve_signal_list.charAt(i)) return false;
//  }
  return true;
}

void runValve(char valve_mode){
  DriverWakeUp(SLEEP_PIN_VALVE);
  if (valve_mode == valve_signal_list.charAt(1)){
    Serial.println("Closing valve");
    stepperValve.move(steps_valve);
  }
  else if(valve_mode == valve_signal_list.charAt(2)){
    Serial.println("Opening valve");
    stepperValve.move(-steps_valve);
  }
}

void signalToValve(String signal){
  runValve(signal.charAt(1));
}