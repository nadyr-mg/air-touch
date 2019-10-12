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

Sensor button:
VCC, GND: 5V, GND
SIG: pin #6
*/

#include <Wire.h>
#include <VL53L0X.h>
#include "Mouse.h"

//const
double PRESSING_BOUNDARY = 0.8;  // how low distance should drop for mouse press to be performed

int LEFT_SENSOR = 1;
int RIGHT_SENSOR = 2;

int CLICK_MODE = 0;
int PRESS_MODE = 1;

int INSTANT_CLICK_DELAY = 200;
int PRESS_DELAY = 300;

int SWITCH_BTN_PIN = 7; // ~7

// LEFT
VL53L0X l_sensor;
int l_max_dist = 0;
uint8_t l_address = 0x30;
int l_xshut_pin = 9; // ~9

// RIGHT
VL53L0X r_sensor;
int r_max_dist = 0;
uint8_t r_address = 0x32;
int r_xshut_pin = 10; // ~10

int cur_mouse_mode = CLICK_MODE;  // by default

int l_prev_state, r_prev_state;

int get_dist(VL53L0X sensor) {
  return sensor.readRangeContinuousMillimeters();
}

void init_sensors() {
  pinMode(l_xshut_pin, OUTPUT);
  pinMode(r_xshut_pin, OUTPUT);
  digitalWrite(r_xshut_pin, LOW);
  digitalWrite(l_xshut_pin, LOW);
  delay(500);
  
  Wire.begin();

  // left sensor setup
  pinMode(l_xshut_pin, INPUT);
  delay(150);
  l_sensor.init(true);

  Serial.println("01");
  delay(100);
  l_sensor.setAddress((uint8_t)l_address);
  Serial.println("02");

  // right sensor setup
  pinMode(r_xshut_pin, INPUT);
  delay(150);
  r_sensor.init(true);
  Serial.println("03");
  delay(100);
  r_sensor.setAddress((uint8_t)r_address);
  Serial.println("04");

  Serial.println("addresses set");
}

void setup()
{
  // mouse setup
  pinMode(12,INPUT_PULLUP);
  digitalWrite(12,HIGH);
  Mouse.begin();
  
  // serial port setup
  Serial.begin(9600);
  
  // sensor button setup
  pinMode(SWITCH_BTN_PIN, INPUT);
  
  Wire.begin();
  // setup sensors addresses
  init_sensors();
  // LEFT SENSOR
  l_sensor.setTimeout(500);
  l_sensor.startContinuous();

  l_max_dist = get_dist(l_sensor);
  Serial.print("Setup l_max_dist: ");
  Serial.println(l_max_dist);
  
  Serial.print("Left sensor address: ");
  Serial.println(l_sensor.getAddress());
  
  // RIGHT SENSOR
  r_sensor.setTimeout(500);
  r_sensor.startContinuous();

  r_max_dist = get_dist(r_sensor);
  Serial.print("Setup r_max_dist: ");
  Serial.println(r_max_dist);
  
  Serial.print("Right sensor address: ");
  Serial.println(r_sensor.getAddress());
}

void print_dist(int sensor_choice, int dist) {
  if (sensor_choice == LEFT_SENSOR) {
    Serial.print("LEFT: ");
  } else {
    Serial.print("RIGHT: ");
  }
  
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.print("mm");
  if (l_sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}

int signal_state(int dist, int max_dist) {
  int state = LOW;
    
  double boundary_dist = max_dist * PRESSING_BOUNDARY;
  if (dist < boundary_dist) {
    state = HIGH;
  }
  
  return state;
}

void mouse_press_action(int state, int mouse) {
  if (state == HIGH) {  
    if (!Mouse.isPressed(mouse)) {
      Mouse.click(mouse);
      Mouse.press(mouse);
      delay(PRESS_DELAY);
    }
  } else {
    if (Mouse.isPressed(mouse)) {
      Mouse.release(mouse);
      // delay(100);
    }
  }
}

void mouse_click_action(int prev_state, int state, int mouse) {
  if (state == HIGH) {
    if (prev_state == LOW) {
      Mouse.click(mouse);
      delay(INSTANT_CLICK_DELAY);
    }
  }
}

int handle_sensor(int prev_state, int sensor_choice) {
  VL53L0X sensor;
  int max_dist;
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
  
  int dist = get_dist(sensor);
  int state = signal_state(dist, max_dist);
  
  if (cur_mouse_mode == CLICK_MODE) {
    mouse_click_action(prev_state, state, mouse);
  } else {
    mouse_press_action(state, mouse);
  }
  
  print_dist(sensor_choice, dist);
  
  return state;
}

void switch_modes() {
  int btn_state = digitalRead(SWITCH_BTN_PIN);
  
  if (btn_state == HIGH) {
    cur_mouse_mode = (cur_mouse_mode + 1) % 2; // switch between 1 and 2
    
    if (cur_mouse_mode == CLICK_MODE) {
      Serial.println("Switched to CLICKING MODE");
    } else {
      Serial.println("Switched to PRESSING MODE");
    }
    delay(1500);
  }
}

void loop()
{
  //switch_modes();
  l_prev_state = handle_sensor(l_prev_state, LEFT_SENSOR);
  r_prev_state = handle_sensor(r_prev_state, RIGHT_SENSOR);

  //delay(100);
}
