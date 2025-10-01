#ifndef RotaryEncoder_h
#define RotaryEncoder_h

#include "Arduino.h"

/**
 * @class RotaryEncoder
 * @brief A class to interface with a rotary encoder using two digital input pins.
 *
 * This class provides methods to read the position, direction, and speed (RPM) of a rotary encoder.
 * It supports different latch modes for compatibility with various encoder types and wiring configurations.
 *
 * Usage:
 * - Instantiate with the two encoder pin numbers and an optional latch mode.
 * - Call tick() periodically (e.g., in a timer interrupt or main loop) to update the encoder state.
 * - Use getPosition(), getDirection(), getMillisBetweenRotations(), and getRPM() to retrieve encoder data.
 */
class RotaryEncoder {
   public:
    enum class Direction {
        NOROTATION = 0,
        CLOCKWISE = 1,
        COUNTERCLOCKWISE = -1
    };

    enum class LatchMode {
        FOUR3 = 1,  // 4 steps, Latch at position 3 only (compatible to older versions)
        FOUR0 = 2,  // 4 steps, Latch at position 0 (reverse wirings)
        TWO03 = 3   // 2 steps, Latch at position 0 and 3
    };

    RotaryEncoder(int pin1, int pin2, LatchMode mode = LatchMode::TWO03);

    // retrieve the current position
    long getPosition();

    // simple retrieve of the direction the knob was rotated last time. 0 = No rotation, 1 = Clockwise, -1 = Counter Clockwise
    Direction getDirection();

    // adjust the current position
    void setPosition(long newPosition);

    // call this function every some milliseconds or by using an interrupt for handling state changes of the rotary encoder
    void tick(void);

    // Returns the time in milliseconds between the current observed
    unsigned long getMillisBetweenRotations() const;

    // Returns the RPM
    unsigned long getRPM();

    // Returns the linear speed in meters per second
    double getLinearSpeed();

    // Returns the odometry in centimeters
    long double getOdometry();

    // Sets the counts per revolution
    void setCPR(int64_t value);

    // Sets the wheel diameter in centimeters
    void setWheelDiameter(float wheelDiameter);

   private:
    int _pin1, _pin2;  // Arduino pins used for the encoder.

    LatchMode _mode;  // Latch mode from initialization

    volatile int8_t _oldState;

    volatile long _position;         // Internal position (4 times _positionExt)
    volatile long _positionExt;      // External position
    volatile long _positionExtPrev;  // External position (used only for direction checking)

    unsigned long _positionExtTime;      // The time the last position change was detected.
    unsigned long _positionExtTimePrev;  // The time the previous position change was detected.

    int64_t CPR;         // Counts per revolution
    float wheelDiameter; // The wheel diameter in centimeters
};

#endif