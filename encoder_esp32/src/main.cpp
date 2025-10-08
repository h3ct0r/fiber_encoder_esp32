#include <Arduino.h>
#include <RotaryEncoder.h>
#include <ESP32Encoder.h>

#define ENCODER0PINA 12               // white A
#define ENCODER0PINB 14               // green B
#define CPR 600                       // encoder cycles per revolution
#define WHEEL_DIAMETER_CM 0.77        // centimeters

RotaryEncoder *rotary_enc = nullptr;
volatile int last_rotary_enc_pos = 0;

ESP32Encoder *esp32_enc_sing = nullptr;
volatile int last_esp32_enc_sing_pos = 0;

ESP32Encoder *esp32_enc_half = nullptr;
volatile int last_esp32_enc_half_pos = 0;

ESP32Encoder *esp32_enc_full = nullptr;
volatile int last_esp32_enc_full_pos = 0;


void checkPosition() {
    rotary_enc->tick();
}

void setup() {
    rotary_enc = new RotaryEncoder(ENCODER0PINA, ENCODER0PINB, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINA), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINB), checkPosition, CHANGE);
    rotary_enc->setCPR(CPR);
    rotary_enc->setWheelDiameter(WHEEL_DIAMETER_CM);
    last_rotary_enc_pos = rotary_enc->getPosition();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    esp32_enc_sing = new ESP32Encoder(CPR, 50, false);
    esp32_enc_sing->attachSingleEdge(ENCODER0PINA, ENCODER0PINB);
    esp32_enc_sing->clearCount();
    esp32_enc_sing->setCPR(CPR);
    esp32_enc_sing->setWheelDiameter(WHEEL_DIAMETER_CM);
    last_esp32_enc_sing_pos = esp32_enc_sing->getCount();

    esp32_enc_half = new ESP32Encoder(CPR, 50, false);
    esp32_enc_half->attachHalfQuad(ENCODER0PINA, ENCODER0PINB);
    esp32_enc_half->clearCount();
    esp32_enc_half->setCPR(CPR);
    esp32_enc_half->setWheelDiameter(WHEEL_DIAMETER_CM);
    last_esp32_enc_half_pos = esp32_enc_half->getCount();

    esp32_enc_full = new ESP32Encoder(CPR, 50, false);
    esp32_enc_full->attachFullQuad(ENCODER0PINA, ENCODER0PINB);
    esp32_enc_full->clearCount();
    esp32_enc_full->setCPR(CPR);
    esp32_enc_full->setWheelDiameter(WHEEL_DIAMETER_CM);
    last_esp32_enc_full_pos = esp32_enc_full->getCount();

    Serial.begin(115200);

    Serial.println("Starting...");
}

void loop() {
    int rotary_enc_pos = rotary_enc->getPosition();
    int esp32_enc_sing_pos = esp32_enc_sing->getCount();
    int esp32_enc_half_pos = esp32_enc_half->getCount();
    int esp32_enc_full_pos = esp32_enc_full->getCount();

    if (
        rotary_enc_pos != last_rotary_enc_pos
        || esp32_enc_sing_pos != last_esp32_enc_sing_pos
        || esp32_enc_half_pos != last_esp32_enc_half_pos
        || esp32_enc_full_pos != last_esp32_enc_full_pos
    ) {
        double rotary_enc_rpm = rotary_enc->getRPM();
        double rotary_enc_odom_cm = rotary_enc->getOdometry();
        float rotary_enc_linearspeed = rotary_enc->getLinearSpeed();
        Serial.println("rotary_enc\todom(cm):" + String((double)rotary_enc_odom_cm) + "\tlin(cm/s):" + String(rotary_enc_linearspeed) + "\trpm:" + String(rotary_enc_rpm) + "\tpos:" + String(rotary_enc_pos));
    
        double esp32_enc_sing_rpm = esp32_enc_sing->getRPM();
        long double esp32_enc_sing_odom = esp32_enc_sing->getOdometry();
        float esp32_enc_sing_linearspeed = esp32_enc_sing->getLinearSpeed();
        Serial.println("esp32_enc_sing\todom(cm):" + String((double)esp32_enc_sing_odom) + "\tlin(cm/s):" + String(esp32_enc_sing_linearspeed) + "\trpm:" + String(esp32_enc_sing_rpm) + "\tpos:" + String(esp32_enc_sing_pos));

        double esp32_enc_half_rpm = esp32_enc_half->getRPM();
        long double esp32_enc_half_odom = esp32_enc_half->getOdometry();
        float esp32_enc_half_linearspeed = esp32_enc_half->getLinearSpeed();
        Serial.println("esp32_enc_half\todom(cm):" + String((double)esp32_enc_half_odom/2) + "\tlin(cm/s):" + String(esp32_enc_half_linearspeed) + "\trpm:" + String(esp32_enc_half_rpm) + "\tpos:" + String(esp32_enc_half_pos));

        double esp32_enc_full_rpm = esp32_enc_full->getRPM();
        long double esp32_enc_full_odom = esp32_enc_full->getOdometry();
        float esp32_enc_full_linearspeed = esp32_enc_full->getLinearSpeed();
        Serial.println("esp32_enc_full\todom(cm):" + String((double)esp32_enc_full_odom/4) + "\tlin(cm/s):" + String(esp32_enc_full_linearspeed) + "\trpm:" + String(esp32_enc_full_rpm) + "\tpos:" + String(esp32_enc_full_pos));

        last_rotary_enc_pos = rotary_enc_pos;
        last_esp32_enc_sing_pos = esp32_enc_sing_pos;
        last_esp32_enc_half_pos = esp32_enc_half_pos;
        last_esp32_enc_full_pos = esp32_enc_full_pos;

        Serial.println("---------------------------------------------------");
    }

    delay(100);
}