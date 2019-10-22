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
uint8_t r_address = 0x38;
int r_xshut_pin = 10; // ~10

int l_prev_state = LOW;
int r_prev_state = LOW;

unsigned long enter_high_state_time = 0;

uint16_t get_dist(int sensor_choice) {
    VL53L0X *sensor;

    if (sensor_choice == LEFT_SENSOR) {
        sensor = &l_sensor;
    } else {
        sensor = &r_sensor;
    }

    uint16_t dist = sensor->readRangeContinuousMillimeters();
    if (dist == -1 || dist == 65535) {
        full_reset();
    }
    return dist;
}

void full_reset() {
    //digitalWrite(ledPin, HIGH);
    // delay(2000);
    // digitalWrite(ledPin, LOW);
    // digitalWrite(RESET_PIN, LOW);
}

void init_sensor(int sensor_choice) {
    bool success;
    uint8_t address;
    int xshut_pin;
    VL53L0X *sensor;

    if (sensor_choice == LEFT_SENSOR) {
        sensor = &l_sensor;
        address = l_address;
        xshut_pin = l_xshut_pin;
    } else {
        sensor = &r_sensor;
        address = r_address;
        xshut_pin = r_xshut_pin;
    }

    pinMode(xshut_pin, OUTPUT);
    digitalWrite(xshut_pin, LOW);
    delay(500);

    // left sensor setup
    pinMode(xshut_pin, INPUT);
    delay(150);
    sensor->setAddress(address);

    success = sensor->init(true);
    if (success == false) {
        full_reset();
    }
    delay(100);

    sensor->setTimeout(TIMEOUT);
    success = sensor->setMeasurementTimingBudget(TIME_BUDGET);
    if (success == false) {
        Serial.println("Can't establish time budget");
    } else {
        Serial.println("Time budget established");
    }
    sensor->startContinuous(TAKE_MEASURE_EVERY);

    if (sensor_choice == LEFT_SENSOR) {
        l_max_dist = get_dist(sensor_choice);
    } else {
        r_max_dist = get_dist(sensor_choice);
    }
}


void setup() {
    digitalWrite(RESET_PIN, HIGH);
    delay(200);
    pinMode(RESET_PIN, OUTPUT);

    pinMode(ledPin, OUTPUT);

    // mouse setup
    Mouse.begin();

    Wire.begin();

    // serial port setup
    Serial.begin(9600);

    // setup sensors addresses
    init_sensor(LEFT_SENSOR);
    init_sensor(RIGHT_SENSOR);
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
    if ((double) dist < boundary_dist) {
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
    uint16_t max_dist;
    int mouse;
    if (sensor_choice == LEFT_SENSOR) {
        max_dist = l_max_dist;
        mouse = MOUSE_LEFT;
    } else {
        max_dist = r_max_dist;
        mouse = MOUSE_RIGHT;
    }

    uint16_t dist = get_dist(sensor_choice);

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

void loop() {
    l_prev_state = handle_sensor(l_prev_state, LEFT_SENSOR);
    r_prev_state = handle_sensor(r_prev_state, RIGHT_SENSOR);

    if (DEBUG) {
        Serial.println(loop_count);
    }

    loop_count += 1;
}
