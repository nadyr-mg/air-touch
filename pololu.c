/*
  Wiring:

  Left laser:
  VIN, GND: on breadboard (5V)
  SDA, SCL: SDA, SCL on arduino
  XShut: pin #9

  Right laser:
  VIN, GND: on breadboard (5V)
  SDA, SCL: pin #2, pin #3
  XShut: pin #10
  Reset pin: #12
*/

#include <Wire.h>
#include <VL53L0X.h>
#include "Mouse.h"

bool DEBUG = true;

const double PRESSING_BOUNDARY = 0.94; //how low distance should drop for mouse press to be performed
const uint16_t L_LEAST_MAX_DIST = 280;
const uint16_t R_LEAST_MAX_DIST = 190;

const int INIT_MEASURES = 10;

int LEFT_SENSOR = 1;
int RIGHT_SENSOR = 2;

int INSTANT_CLICK_DELAY = 500;
unsigned long TRIGGER_PRESS_TIME = 600;

// by default sensor measures every ~30 ms
uint32_t TAKE_MEASURE_EVERY = 80;
uint16_t TIMEOUT = 150;
uint32_t TIME_BUDGET = 60000;

const int ledPin = LED_BUILTIN;

const int RESET_PIN = 12;

// LEFT
VL53L0X l_sensor;
uint16_t l_max_dist = 0;
uint8_t l_address = 0x30;
int l_xshut_pin = 10;

// RIGHT
VL53L0X r_sensor;
uint16_t r_max_dist = 0;
uint8_t r_address = 0x38;
int r_xshut_pin = 9;

int l_prev_state = LOW;
int r_prev_state = LOW;

unsigned long l_entered_high_time = 0;
unsigned long r_entered_high_time = 0;

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

uint16_t get_max_dist(int sensor_choice) {
    uint16_t least_max_dist;
    if (sensor_choice == LEFT_SENSOR) {
        least_max_dist = L_LEAST_MAX_DIST;
    } else {
        least_max_dist = R_LEAST_MAX_DIST;
    }

    unsigned long sum = 0;
    uint16_t max_dist;
    for (int measure = 0; measure < INIT_MEASURES; measure++) {
        max_dist = get_dist(sensor_choice);
        if (max_dist < least_max_dist) {
            full_reset();
        }

        sum += max_dist;
    }

    max_dist = (uint16_t)(sum / INIT_MEASURES);
    return max_dist;
}

void full_reset() {
    //digitalWrite(ledPin, HIGH);
    delay(1000);
    // digitalWrite(ledPin, LOW);
    digitalWrite(RESET_PIN, LOW);
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

    digitalWrite(xshut_pin, LOW);
    delay(100);

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
        l_max_dist = get_max_dist(sensor_choice);
    } else {
        r_max_dist = get_max_dist(sensor_choice);
    }
}

void pin_left() {
    pinMode(l_xshut_pin, OUTPUT);
}

void pin_right() {
    pinMode(r_xshut_pin, OUTPUT);
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

    pin_left();
    pin_right();
    delay(500);

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

unsigned long mouse_press_action(unsigned long entered_high_time, int state, int mouse) {
    unsigned long cur_time = millis();
    bool is_pressed = Mouse.isPressed(mouse);

    if (state == HIGH) {
        if (entered_high_time == 0) {
            // first entrance
            entered_high_time = cur_time;
            if (is_pressed) {
                Mouse.release(mouse);
            }
            Mouse.click(mouse);
        }
        if (cur_time - entered_high_time >= TRIGGER_PRESS_TIME) {
            if (!is_pressed) {
                Mouse.press(mouse);
            }
        }
    } else { // state == LOW
        if (is_pressed) {
            Mouse.release(mouse);
        }
        entered_high_time = 0;
    }

    return entered_high_time;
}

int mouse_click_action(int prev_state, int state, int mouse) {
    if (state == HIGH) {
        if (prev_state == LOW) {
            if (Mouse.isPressed(mouse)) {
                Mouse.release(mouse);
            }
            Mouse.click(mouse);
            delay(INSTANT_CLICK_DELAY);
        }
    }

    return state;
}

void handle_sensor(int sensor_choice) {
    uint16_t max_dist;
    int mouse;
    int prev_state;
    unsigned long entered_high_time;

    if (sensor_choice == LEFT_SENSOR) {
        max_dist = l_max_dist;
        mouse = MOUSE_LEFT;
        entered_high_time = l_entered_high_time;
        prev_state = l_prev_state;
    } else {
        max_dist = r_max_dist;
        mouse = MOUSE_RIGHT;
        entered_high_time = r_entered_high_time;
        prev_state = r_prev_state;
    }

    uint16_t dist = get_dist(sensor_choice);

    int state = signal_state(dist, max_dist);

    if (sensor_choice == RIGHT_SENSOR) {
        prev_state = mouse_click_action(prev_state, state, mouse);
    } else {
        entered_high_time = mouse_press_action(entered_high_time, state, mouse);
    }
    if (DEBUG) {
        print_dist(sensor_choice, dist);
    }

    if (sensor_choice == LEFT_SENSOR) {
        l_entered_high_time = entered_high_time;
        l_prev_state = prev_state;
    } else {
        r_entered_high_time = entered_high_time;
        r_prev_state = prev_state;
    }
}


unsigned long loop_count = 0;

void loop() {
    handle_sensor(LEFT_SENSOR);
    handle_sensor(RIGHT_SENSOR);

    if (DEBUG) {
        Serial.println(loop_count);
    }

    loop_count += 1;
}
