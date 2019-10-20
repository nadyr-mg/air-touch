/*
  Wiring:

  Left laser:
  VIN, GND: on breadboard (3.3V)
  SDA, SCL: SDA, SCL on arduino
  XShut: pin #9

  Right laser:
  VIN, GND: on breadboard (3.3V)
  SDA, SCL: pin #2, pin #3
  XShut: pin #10

  Reset pin: #12
*/

#include <Wire.h>
#include <VL53L0X.h>
#include "Mouse.h"

bool DEBUG = true;

const double PRESSING_BOUNDARY = 0.9; //how low distance should drop for mouse press to be performed

int LEFT_SENSOR = 1;
int RIGHT_SENSOR = 2;

int INSTANT_CLICK_DELAY = 500;
unsigned long TRIGGER_PRESS_TIME = 600;

// by default sensor measures every ~30 ms
uint32_t TAKE_MEASURE_EVERY = 80;
uint16_t TIMEOUT = 500;
uint32_t TIME_BUDGET = 60000;

const int ledPin = LED_BUILTIN;

const int RESET_PIN = 12;

// LEFT
VL53L0X l_sensor;
uint16_t l_max_dist = 0;
uint8_t l_address = 0x30;
int l_xshut_pin = 9; // ~9

// RIGHT
VL53L0X r_sensor;
uint16_t r_max_dist = 0;
uint8_t r_address = 0x32;
int r_xshut_pin = 10; // ~10

int l_prev_state = LOW;
int r_prev_state = LOW;

unsigned long enter_high_state_time = 0;

uint16_t get_dist(VL53L0X sensor) {
  uint16_t dist = sensor.readRangeContinuousMillimeters();
  if (dist == -1 || dist == 65535) {
    full_reset();
  }
  return dist;
}

void full_reset() {
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(ledPin, LOW);
  digitalWrite(RESET_PIN, LOW);
}

void init_sensors() {
  pinMode(l_xshut_pin, OUTPUT);
  pinMode(r_xshut_pin, OUTPUT);
  digitalWrite(r_xshut_pin, LOW);
  digitalWrite(l_xshut_pin, LOW);
  delay(500);

  // left sensor setup
  pinMode(l_xshut_pin, INPUT);
  delay(150);
  bool success = l_sensor.init(true);
  if (success == false) {
    full_reset();
  }

  delay(100);
  l_sensor.setAddress((uint8_t)l_address);

  // right sensor setup
  pinMode(r_xshut_pin, INPUT);
  delay(150);
  success = r_sensor.init(true);
  if (success == false) {
    full_reset();
  }

  delay(100);
  r_sensor.setAddress((uint8_t)r_address);

  // LEFT SENSOR
  l_sensor.setTimeout(TIMEOUT);
  success = l_sensor.setMeasurementTimingBudget(TIME_BUDGET);
  if (success == false) {
    Serial.println("Can't establish time budget");
  }
  else {
    Serial.println("Time budget established");
  }
  l_sensor.startContinuous(TAKE_MEASURE_EVERY);

  // RIGHT SENSOR
  r_sensor.setTimeout(TIMEOUT);
  success = r_sensor.setMeasurementTimingBudget(TIME_BUDGET);
  if (success == false) {
    Serial.println("Can't establish time budget");
  }
  else {
    Serial.println("Time budget established");
  }
  r_sensor.startContinuous(TAKE_MEASURE_EVERY);
}

void setup()
{
  digitalWrite(RESET_PIN, HIGH);
  delay(200);
  pinMode(RESET_PIN, OUTPUT);
  
  pinMode(ledPin, OUTPUT);
  //digitalWrite(ledPin, HIGH);

  // mouse setup
  Mouse.begin();
  
  Wire.begin();

  // serial port setup
  Serial.begin(9600);

  // setup sensors addresses
  init_sensors();

  l_max_dist = get_dist(l_sensor);
  Serial.print("Setup l_max_dist: ");
  Serial.println(l_max_dist);

  Serial.print("Left sensor address: ");
  Serial.println(l_sensor.getAddress());

  r_max_dist = get_dist(r_sensor);
  Serial.print("Setup r_max_dist: ");
  Serial.println(r_max_dist);

  Serial.print("Right sensor address: ");
  Serial.println(r_sensor.getAddress());

  //digitalWrite(ledPin, LOW);
}

void print_dist(int sensor_choice, uint16_t dist) {
  if (sensor_choice == LEFT_SENSOR) {
    Serial.print("LEFT: ");
  } else {
    Serial.print("RIGT: ");
  }

  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.print("mm");
  if (l_sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();
}

int signal_state(uint16_t dist, uint16_t max_dist) {
  int state = LOW;

  double boundary_dist = max_dist * PRESSING_BOUNDARY;
  if ((double)dist < boundary_dist) {
    state = HIGH;
  }

  return state;
}

void mouse_press_action(int state, int mouse) {
  unsigned long cur_time = millis();
  bool is_pressed = Mouse.isPressed(mouse);

  if (state == HIGH) {
    if (enter_high_state_time == 0) {
      // first entrance
      enter_high_state_time = cur_time;
      if (is_pressed) {
        Mouse.release(mouse);
      }
      Mouse.click(mouse);
    }

    if (cur_time - enter_high_state_time >= TRIGGER_PRESS_TIME) {
      if (!is_pressed) {
        Mouse.press(mouse);
      }
    }

  } else { // state == LOW
    if (is_pressed) {
      Mouse.release(mouse);
    }
    enter_high_state_time = 0;
  }
}

void mouse_click_action(int prev_state, int state, int mouse) {
  if (state == HIGH) {
    if (prev_state == LOW) {
      if (Mouse.isPressed(mouse)) {
        Mouse.release(mouse);
      }

      Mouse.click(mouse);
      delay(INSTANT_CLICK_DELAY);
    }
  }
}

int handle_sensor(int prev_state, int sensor_choice) {
  VL53L0X sensor;
  uint16_t max_dist;
  int mouse;
  if (sensor_choice == LEFT_SENSOR) {
    sensor = l_sensor;
    max_dist = l_max_dist;
    mouse = MOUSE_LEFT;
  } else {
    sensor = r_sensor;
    max_dist = r_max_dist;
    mouse = MOUSE_RIGHT;
  }

  uint16_t dist = get_dist(sensor);
  
  int state = signal_state(dist, max_dist);

  if (sensor_choice == RIGHT_SENSOR) {
    mouse_click_action(prev_state, state, mouse);
  } else {
    mouse_press_action(state, mouse);
  }
  if (DEBUG) {
    print_dist(sensor_choice, dist);
  }

  return state;
}


unsigned long loop_count = 0;
unsigned long LOOP_COUNT_RESET = 9000;  // ~15 mins

void loop()
{
  l_prev_state = handle_sensor(l_prev_state, LEFT_SENSOR);
  r_prev_state = handle_sensor(r_prev_state, RIGHT_SENSOR);

  // if (loop_count > LOOP_COUNT_RESET) {
  //   loop_count = 0;
  // }

  if (DEBUG) {
    Serial.println(loop_count);
  }

  loop_count += 1;
}
