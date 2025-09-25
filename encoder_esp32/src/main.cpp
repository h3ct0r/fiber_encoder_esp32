#include <Arduino.h>
#include <RotaryEncoder.h>
#include <U8g2lib.h>

#define OLED_RESET U8X8_PIN_NONE  // Reset pin
#define OLED_SDA 5
#define OLED_SCL 6
#define LED_BUILTIN 8

#define ENCODER0PINA 9                // white A
#define ENCODER0PINB 10               // green B
#define CPR 600                       // encoder cycles per revolution
#define CLOCKWISE 1                   // direction constant
#define COUNTER_CLOCKWISE -1          // direction constant
#define POSITION_COMPUTE_INTERVAL 10  // milliseconds
#define OLED_COMPUTE_INTERVAL 500     // milliseconds
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
unsigned long prevOledComputeTime = 0;
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

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, OLED_RESET, OLED_SCL, OLED_SDA);
int c = 0;

RotaryEncoder *encoder = nullptr;

void checkPosition() {
    encoder->tick();
}

void handle_oled() {
    // 18 chars per line, 10px of height per line
    u8g2.clearBuffer();
    // u8g2.clearDisplay();

    // u8g2.setFont(u8g2_font_8x13_tr);
    // u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.setFont(u8g2_font_5x7_mf);

    char buffer[20];
    snprintf(buffer, sizeof(buffer), "RPM: %.2lf", rpm0);
    u8g2.drawStr(0, 10, buffer);

    snprintf(buffer, sizeof(buffer), "RPS: %.2lf", rpm0 / 60.0);
    u8g2.drawStr(0, 20, buffer);

    snprintf(buffer, sizeof(buffer), "ODOM: %.3lfm", odom_cm / 100.0);
    u8g2.drawStr(0, 30, buffer);

    u8g2.sendBuffer();

    prevOledComputeTime = millis();
}

void setup() {
    encoder = new RotaryEncoder(ENCODER0PINA, ENCODER0PINB, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINA), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER0PINB), checkPosition, CHANGE);

    Serial.begin(115200);

    u8g2.begin();
    u8g2.setContrast(255);     // set contrast to maximum
    u8g2.setBusClock(400000);  // 400kHz I2C

    pinMode(LED_BUILTIN, OUTPUT);

    // while (!Serial);
    Serial.println("START OLED MODULE");
}

void loop() {
    if (millis() - prevOledComputeTime > OLED_COMPUTE_INTERVAL)
        handle_oled();

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

    // if (millis() - prevPositionComputeTime < POSITION_COMPUTE_INTERVAL)
    //     return;

    // dt_omega = millis() - prevWheelComputeTime;
    // double changeTicks0 = encoder0intReceived - previous0intReceived;
    // rpm0 = (changeTicks0 * (60000.0 / dt_omega)) / CPR;
    // odom_cm = ((2 * SHAFT_RADIUS_MM * PI) * (absoluteEncoder0intReceived / (CPR * 1.0))) / 10;

    // if (absoluteEncoder0intReceived < previous0AbsoluteEncoder0intReceived) {
    //     currentDirection = COUNTER_CLOCKWISE;
    // } else if (absoluteEncoder0intReceived > previous0AbsoluteEncoder0intReceived) {
    //     currentDirection = CLOCKWISE;
    // }

    // if (encoder0Position != previous0Position) {
    //     Serial.print("pos:");
    //     // Serial.print(encoder0Position, 4);
    //     char temp[6];
    //     sprintf(temp, "%04d", encoder0Position);
    //     Serial.print(temp);

    //     // Serial.print("\t dir:");
    //     // Serial.print(currentDirection); //== CLOCKWISE ? "CW" : "CCW"
    //     Serial.print("\tcounter:");
    //     Serial.print(encoder0intReceived, DEC);
    //     Serial.print("\t abscounter:");
    //     Serial.print(absoluteEncoder0intReceived, DEC);
    //     Serial.print("\t prevabscounter:");
    //     Serial.print(previous0AbsoluteEncoder0intReceived, DEC);
    //     // Serial.print("\t diff ticks:");
    //     // Serial.print(changeTicks0, DEC);
    //     // Serial.print("\t lrpm:");
    //     // Serial.print(lrpm);
    //     // Serial.print("\t lrps:");
    //     // Serial.println(lrpm / 60.0);
    //     // Serial.print("pos0:");
    //     // Serial.print(encoder0Position, DEC);
    //     // Serial.print("\t\tpos1:");
    //     // Serial.print(encoder1Position, DEC);
    //     Serial.print("\tdir:");
    //     Serial.print(currentDirection);  //== CLOCKWISE ? "CW" : "CCW"
    //     Serial.print("\trpm0:");
    //     Serial.print(rpm0, 2);
    //     // Serial.print("\t rps0:");
    //     // Serial.print(rpm0 / 60, 2);
    //     Serial.print("\todom_cm:");
    //     Serial.print(odom_cm, 2);
    //     Serial.println();
    // }

    // previous0Position = encoder0Position;
    // previous0intReceived = encoder0intReceived;
    // previous0AbsoluteEncoder0intReceived = absoluteEncoder0intReceived;

    // prevWheelComputeTime = millis();
    // prevPositionComputeTime = millis();
}