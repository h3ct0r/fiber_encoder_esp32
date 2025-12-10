#include <Arduino.h>
#include <RotaryEncoder.h>
#include <ESP32Encoder.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define ENCODER0PINA 12               // white A
#define ENCODER0PINB 14               // green B
#define CPR 600                       // encoder cycles per revolution
#define WHEEL_DIAMETER_CM 0.755        // centimeters
#define ENVIRONMENT "production" // Change to "production" to enable ROS publishing
#define ROS_PUBLISH_INTERVAL 1000 // Publish every 1 second


RotaryEncoder *rotary_enc = nullptr;
volatile int last_rotary_enc_pos = 0;

ESP32Encoder *esp32_enc_sing = nullptr;
volatile int last_esp32_enc_sing_pos = 0;

ESP32Encoder *esp32_enc_half = nullptr;
volatile int last_esp32_enc_half_pos = 0;

ESP32Encoder *esp32_enc_full = nullptr;
volatile int last_esp32_enc_full_pos = 0;

volatile unsigned long last_change_time = 0;

ros::NodeHandle nh;
std_msgs::Float32 odometry_msg;
ros::Publisher pub_metric("theter_odometry", &odometry_msg);
long last_publish_time = 0;

void checkPosition() {
    rotary_enc->tick();
}

void setup() {
    /**
     * During development, several methods of reading the encoder are compared:
     * 1. Using the RotaryEncoder library with interrupts
     * 2. Using the ESP32Encoder library in single edge mode
     * 3. Using the ESP32Encoder library in half quadrature mode
     * 4. Using the ESP32Encoder library in full quadrature mode
     * 
     * No ROS topics are published in this mode; data is printed to Serial for
     * analysis.
     */
    if (ENVIRONMENT == "dev") {
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
    } else {
        /**
         * In production mode, only the ESP32Encoder library in single edge mode
         * is used, and encoder data is published to a ROS topic.
         */
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        esp32_enc_sing = new ESP32Encoder(CPR, 50, false);
        esp32_enc_sing->attachSingleEdge(ENCODER0PINA, ENCODER0PINB);
        esp32_enc_sing->clearCount();
        esp32_enc_sing->setCPR(CPR);
        esp32_enc_sing->setWheelDiameter(WHEEL_DIAMETER_CM);

        // Initialize the ROS node and advertise the topic
        nh.initNode();
        nh.advertise(pub_metric);
    }

    Serial.begin(115200);
    Serial.println("Starting...");
}

void loop() {
    if (ENVIRONMENT == "dev") {

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
            Serial.println("rotary_enc\todom(cm):" + String((double)rotary_enc_odom_cm) + "\tlin(cm/s):" + String(rotary_enc_linearspeed) + "\trpm:" + String(rotary_enc_rpm) + "\tpos:" + String(rotary_enc_pos) + "\tdiff:" + String(rotary_enc_pos-last_rotary_enc_pos));
        
            double esp32_enc_sing_rpm = esp32_enc_sing->getRPM();
            long double esp32_enc_sing_odom = esp32_enc_sing->getOdometry();
            float esp32_enc_sing_linearspeed = esp32_enc_sing->getLinearSpeed();
            Serial.println("esp32_enc_sing\todom(cm):" + String((double)esp32_enc_sing_odom) + "\tlin(cm/s):" + String(esp32_enc_sing_linearspeed) + "\trpm:" + String(esp32_enc_sing_rpm) + "\tpos:" + String(esp32_enc_sing_pos) + "\tdiff:" + String(esp32_enc_sing_pos-last_esp32_enc_sing_pos));
    
            double esp32_enc_half_rpm = esp32_enc_half->getRPM();
            long double esp32_enc_half_odom = esp32_enc_half->getOdometry();
            float esp32_enc_half_linearspeed = esp32_enc_half->getLinearSpeed();
            Serial.println("esp32_enc_half\todom(cm):" + String((double)esp32_enc_half_odom/2) + "\tlin(cm/s):" + String(esp32_enc_half_linearspeed) + "\trpm:" + String(esp32_enc_half_rpm) + "\tpos:" + String(esp32_enc_half_pos) + "\tdiff:" + String(esp32_enc_half_pos-last_esp32_enc_half_pos));
    
            double esp32_enc_full_rpm = esp32_enc_full->getRPM();
            long double esp32_enc_full_odom = esp32_enc_full->getOdometry();
            float esp32_enc_full_linearspeed = esp32_enc_full->getLinearSpeed();
            Serial.println("esp32_enc_full\todom(cm):" + String((double)esp32_enc_full_odom/4) + "\tlin(cm/s):" + String(esp32_enc_full_linearspeed) + "\trpm:" + String(esp32_enc_full_rpm) + "\tpos:" + String(esp32_enc_full_pos) + "\tdiff:" + String(esp32_enc_full_pos-last_esp32_enc_full_pos));
    
            last_rotary_enc_pos = rotary_enc_pos;
            last_esp32_enc_sing_pos = esp32_enc_sing_pos;
            last_esp32_enc_half_pos = esp32_enc_half_pos;
            last_esp32_enc_full_pos = esp32_enc_full_pos;
            last_change_time = millis();
    
            Serial.println(
                String(last_change_time) + ","
                + String((double)rotary_enc_odom_cm) + ","
                + String((double)esp32_enc_sing_odom) + ","
                + String((double)esp32_enc_half_odom/2) + ","
                + String((double)esp32_enc_full_odom/4)
            );
    
            Serial.println(String(last_change_time) + " ---------------------------------------------------");
        }
    
        // If the encoders positions didn't change for 2 seconds, reset them
        if (
            millis() - last_change_time > 2000
            && (rotary_enc_pos != 0 || esp32_enc_sing_pos != 0 || esp32_enc_half_pos != 0 || esp32_enc_full_pos != 0)
        ) {
            rotary_enc->reset();
            esp32_enc_sing->reset();
            esp32_enc_half->reset();
            esp32_enc_full->reset();
            last_rotary_enc_pos = 0;
            last_esp32_enc_sing_pos = 0;
            last_esp32_enc_half_pos = 0;
            last_esp32_enc_full_pos = 0;
            last_change_time = millis();
            Serial.println("Encoders reset due to inactivity.");
        }

        delay(100);
    } else {
        // Production mode: publish odometry data to ROS topic

        // Check if it's time to publish
        if (millis() - last_publish_time > ROS_PUBLISH_INTERVAL) {
            long double esp32_enc_sing_odom = esp32_enc_sing->getOdometry();
            odometry_msg.data = (float)esp32_enc_sing_odom;
            pub_metric.publish(&odometry_msg);
            // Log for debugging on the serial monitor
            Serial.print("Theter odometry: ");
            Serial.print(odometry_msg.data);
            Serial.println(" cm");
            last_publish_time = millis();
        }
        nh.spinOnce();
        delay(10);
    }
}