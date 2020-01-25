typedef struct
{
    unsigned long timestamp;

    // Chassis
    float front_left_motor_current;
    float front_right_motor_current;
    float back_left_motor_current;
    float back_right_motor_current;

    float gyroscope;

    // Arm
    float joint_one_encoder;
    float joint_two_encoder;
    float joint_three_encoder;

    // Science
    float science_temperature;
    float science_humidity;

} OutboundData;