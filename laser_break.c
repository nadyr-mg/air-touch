/*

*/
#include "Mouse.h"

bool DEBUG = true;

int LEFT_SENSOR = 1;
int RIGHT_SENSOR = 2;

int INSTANT_CLICK_DELAY = 500;
unsigned long TRIGGER_PRESS_TIME = 800;

const int LEFT_SENSOR_PIN = 10;
const int RIGHT_SENSOR_PIN = 11;

int l_prev_state = LOW;
int r_prev_state = LOW;

unsigned long l_entered_high_time = 0;
unsigned long r_entered_high_time = 0;

void setup() {
    Mouse.begin();
    Serial.begin(9600);

    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
}

void handle_sensor(int sensor_choice) {
    int mouse;
    int prev_state;
    unsigned long entered_high_time;
    if (sensor_choice == LEFT_SENSOR) {
        mouse = MOUSE_LEFT;
        entered_high_time = l_entered_high_time;
        prev_state = l_prev_state;
    } else {
        mouse = MOUSE_RIGHT;
        entered_high_time = r_entered_high_time;
        prev_state = r_prev_state;
    }

    int state = signal_state(sensor_choice);
    /*
    if (sensor_choice == RIGHT_SENSOR) {
        prev_state = mouse_click_action(prev_state, state, mouse);
    } else {
        entered_high_time = mouse_press_action(entered_high_time, state, mouse);
    }*/
    if (DEBUG) {
        print_state(sensor_choice, state);
    }

    if (sensor_choice == LEFT_SENSOR) {
        l_entered_high_time = entered_high_time;
        l_prev_state = prev_state;
    } else {
        r_entered_high_time = entered_high_time;
        r_prev_state = prev_state;
    }
}

void print_state(int sensor_choice, int state) {
    if (sensor_choice == LEFT_SENSOR) {
        Serial.print("LEFT: ");
    } else {
        Serial.print("RIGT: ");
    }

    Serial.print("Signal: ");
    if (state == HIGH) {
        Serial.print("HIGH");
    } else {
        Serial.print("LOW");
    }
    Serial.println();
}

int signal_state(int sensor_choice) {
    int state;
    if (sensor_choice == LEFT_SENSOR) {
        state = digitalRead(LEFT_SENSOR_PIN);
    } else {
        state = digitalRead(RIGHT_SENSOR_PIN);
    }

    // invert the value
    return state == LOW ? HIGH: LOW;
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

unsigned long loop_count = 0;

void loop() {
    handle_sensor(LEFT_SENSOR);
//    handle_sensor(RIGHT_SENSOR);

    if (DEBUG) {
        Serial.println(loop_count);
    }

    loop_count += 1;
    delay(100);
}