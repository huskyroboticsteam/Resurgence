#ifndef STATUS_DATA_H
#define STATUS_DATA_H

typedef struct
{
    // Current list of things to keep track
    // Lots more to add

    // Chassis
    int front_left_motor_current_ma;
    int front_right_motor_current_ma;
    int back_left_motor_current_ma;
    int back_right_motor_current_ma;

    int front_left_motor_voltage_mv;
    int front_right_motor_voltage_mv;
    int back_left_motor_voltage_mv;
    int back_right_motor_voltage_mv;

    int front_left_motor_temp_mc;
    int front_right_motor_temp_mc;
    int back_left_motor_temp_mc;
    int back_right_motor_temp_mc;

    unsigned int gps_lat;
    unsigned int gps_long;

    int magnetometer_dir_md;

    int accel_x_mmss;
    int accel_y_mmss;
    int accel_z_mmss;

    int gyro_x_mdps;
    int gyro_y_mdps;
    int gyro_z_mdps;

    // Arm
    int base_current_ma;
    int shoulder_current_ma;
    int elbow_current_ma;
    int wrist_current_ma;
    int diffleft_current_ma;
    int diffright_current_ma;
    int hand_current_ma;

    int base_voltage_mv;
    int shoulder_voltage_mv;
    int elbow_voltage_mv;
    int wrist_voltage_mv;
    int diffleft_voltage_mv;
    int diffright_voltage_mv;
    int hand_voltage_mv;

    int base_temp_mc;
    int shoulder_temp_mc;
    int elbow_temp_mc;
    int wrist_temp_mc;
    int diffleft_temp_mc;
    int diffright_temp_mc;
    int hand_temp_mc;

    bool shoulder_limit_switch;
    bool elbow_limit_switch;

    // Science
    // TODO: Add in science readings

} StatusData;

#endif
