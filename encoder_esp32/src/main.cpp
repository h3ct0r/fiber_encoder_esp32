#include <Arduino.h>
#include <RotaryEncoder.h>

#define ENCODER0PINA 12                // white A
#define ENCODER0PINB 14               // green B
#define CPR 600                       // encoder cycles per revolution
#define CLOCKWISE 1                   // direction constant
#define COUNTER_CLOCKWISE -1          // direction constant
#define POSITION_COMPUTE_INTERVAL 10  // milliseconds
#define SHAFT_RADIUS_CM 0.385         // centimeters

// variables modified by interrupt handler must be declared as volatile
volatile long encoder0Position = 0;
volatile unsigned long encoder0intReceived = 0;
volatile long long absoluteEncoder0intReceived = 0;

// track direction: -1 = counter-clockwise; 1 = clockwise
short currentDirection = CLOCKWISE;

// track last position so we know whether it's worth printing new output
unsigned long previous0Position = 0;
unsigned long previous0intReceived = 0;
volatile long long previous0AbsoluteEncoder0intReceived = 0;

unsigned long prevPositionComputeTime = 0;
unsigned long prevWheelComputeTime = 0;
unsigned long prevServoComputeTime = 0;
// unsigned long prevOledComputeTime = 0;
unsigned long dt_omega = 0;

// Latest RPM
double rpm0 = 0;
double odom_cm = 0;
volatile unsigned char prev_reading = 0;
volatile unsigned char curr_reading = 0;

int motor_state = 0;

// Quadrature Encoder Matrix from OddBot
const int QEM[16] = {0, -1, 1, 2,
                     1, 0, 2, -1,
                     -1, 2, 0, 1,
                     2, 1, -1, 0};

int c = 0;

RotaryEncoder *encoder = nullptr;

void checkPosition() {
    encoder->tick();
}

void setup() {
    encoder = new RotaryEncoder(ENCODER0PINA, ENCODER0PINB, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINA), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINB), checkPosition, CHANGE);

    Serial.begin(115200);
}

void loop() {
    int newPos = encoder->getPosition();
    if (encoder0Position != newPos) {
        Serial.print("pos:");
        Serial.print(newPos);
        Serial.print(" dir:");
        Serial.print((int)(encoder->getDirection()));
        Serial.print(" rpm:");
        Serial.print((int)(encoder->getRPM()));
        Serial.print(" odom:");
        odom_cm = ((2 * SHAFT_RADIUS_CM * PI) * (newPos / (CPR * 1.0)));
        Serial.println(odom_cm);

        encoder0Position = newPos;
    }
}