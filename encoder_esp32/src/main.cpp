#include <Arduino.h>
#include <RotaryEncoder.h>
#include <ESP32Encoder.h>

#define ENCODER0PINA 12               // white A
#define ENCODER0PINB 14               // green B
#define CPR 600                       // encoder cycles per revolution
#define WHEEL_DIAMETER_CM 0.77        // centimeters

RotaryEncoder *rotary_encoder = nullptr;
ESP32Encoder *esp32_encoder = nullptr;
volatile int last_rotary_encoder_pos = 0;
volatile int last_esp32_encoder_pos = 0;

void checkPosition() {
    rotary_encoder->tick();
}

void setup() {
    rotary_encoder = new RotaryEncoder(ENCODER0PINA, ENCODER0PINB, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINA), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINB), checkPosition, CHANGE);
    rotary_encoder->setCPR(CPR);
    rotary_encoder->setWheelDiameter(WHEEL_DIAMETER_CM);

    esp32_encoder = new ESP32Encoder(CPR, 50, false);
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    esp32_encoder->attachSingleEdge(ENCODER0PINA, ENCODER0PINB);
    esp32_encoder->clearCount();
    esp32_encoder->setCPR(CPR);
    esp32_encoder->setWheelDiameter(WHEEL_DIAMETER_CM);

    last_rotary_encoder_pos = rotary_encoder->getPosition();
    last_esp32_encoder_pos = esp32_encoder->getCount();

    Serial.begin(115200);
}

void loop() {
    int rotary_encoder_pos = rotary_encoder->getPosition();
    int esp32_encoder_pos = esp32_encoder->getCount();

    if (rotary_encoder_pos != last_rotary_encoder_pos || esp32_encoder_pos != last_esp32_encoder_pos) {
        double rotary_encoder_rpm = rotary_encoder->getRPM();
        double rotary_encoder_odom_cm = rotary_encoder->getOdometry();
        float rotary_encoder_linearspeed = rotary_encoder->getLinearSpeed();
        Serial.println("RotaryEncoder\todom(cm):" + String((double)rotary_encoder_odom_cm) + "\tlin(m/s):" + String(rotary_encoder_linearspeed) + "\trpm:" + String(rotary_encoder_rpm) + "\tpos:" + String(rotary_encoder_pos));
    
        double esp32_encoder_rpm = esp32_encoder->getRPM();
        long double esp32_encoder_odom = esp32_encoder->getOdometry();
        float esp32_encoder_linearspeed = esp32_encoder->getLinearSpeed();
        Serial.println("ESP32Encoder\todom(cm):" + String((double)esp32_encoder_odom) + "\tlin(m/s):" + String(esp32_encoder_linearspeed) + "\trpm:" + String(esp32_encoder_rpm) + "\tpos:" + String(esp32_encoder_pos));

        last_rotary_encoder_pos = rotary_encoder_pos;
        last_esp32_encoder_pos = esp32_encoder_pos;

        Serial.println("---------------------------------------------------");
    }

    delay(100);
}