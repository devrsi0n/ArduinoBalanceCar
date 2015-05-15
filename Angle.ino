#include <Angle.h>

void initAnglePID(void)
{
    angle_pid.SetOutputLimits(PWM_MIN, PWM_MAX);
    angle_pid.SetSampleTime(10); // 2ms X 5 = 10ms
    angle_pid.SetMode(AUTOMATIC);
    angle_pid.SetTunings(CarArgs.angleCtrlP, CarArgs.angleCtrlI, CarArgs.angleCtrlD);
}

void readSampleMPU6050(void)
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

/*
* caculate original angle and angle velocity.
*/
void getOriginalAngleGyro(void)
{
#define GRY_OFFSET  296
#define GYR_GAIN    0.00763
#define ACC_GAIN    0.000061
#define PI          3.14159326

    double y_accel = ay * ACC_GAIN;
    double z_accel = az * ACC_GAIN;
    original_angle = (float)atan(y_accel / z_accel) * 180.0 / PI;
    float gx_revise = gx + GRY_OFFSET;
    original_gyro = GYR_GAIN * gx_revise;
}

void angleFilter(void)
{
    static unsigned long pre_time = 0;
    unsigned long now = micros();
    float delta_time = (now - pre_time) / 1000000.0;
    pre_time = now;

    board_gyro = complementaryFilter(original_gyro, delta_time);
    klm_angle = kalmanFilter(original_angle, board_gyro, delta_time);
    board_angle = complementary2Filter(klm_angle, board_gyro, delta_time);

    board_angle = constrain(board_angle, MIN_ANGLE, MAX_ANGLE); // board angle limit
}

void angleCtrlPID(void)
{
    angle_input = board_angle;
    angle_pid.Compute();
    angle_ctrl_output = angle_output;
}

/*
* kalman filter for angle
*/
float kalmanFilter(float angle, float gyro, float delta_time)
{
#define Q_ANGLE     0.001   // test show: 0.001 is better than 0.01
#define Q_OMEGA     0.005   // test show: 0.005 is better than 0.0003
#define R_ANGLE     0.1     // test show: 0.1 is better than 0.01

    static float klm_angle = 0; // kalman fiter output
    static float bias = 0;
    static float P[2][2] = {0};

    klm_angle += (gyro - bias) * delta_time;
    P[0][0] += -(P[1][0] + P[0][1]) * delta_time + Q_ANGLE * delta_time;
    P[0][1] -= P[1][1] * delta_time;
    P[1][0] -= P[1][1] * delta_time;
    P[1][1] += Q_OMEGA * delta_time;
    float K_0 = P[0][0] / (P[0][0] + R_ANGLE);
    float K_1 = P[1][0] / (P[0][0] + R_ANGLE);
    bias += K_1 * (angle - klm_angle);
    klm_angle += K_0 * (angle - klm_angle);
    P[0][0] -= K_0 * P[0][0];
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];

    return klm_angle;
}

/*
* the second order complementary filter for angle
*/
float complementary2Filter(float angle, float gyro, float delta_time)
{
#define K       3.5

    static float com2_angle = 0;
    static float y1 = 0;

    float x1 = (angle - com2_angle) * K * K;
    y1 += x1 * delta_time;
    float x2 = y1 + 2 * K * (angle - com2_angle) + gyro;
    com2_angle += x2 * delta_time;

    return com2_angle;
}

/*
* complementary filter for gyro
*/
float complementaryFilter(float gyro, float delta_time)
{
#define P       0.03    // 0.04

    static float com_gyro = 0;
    float proporation =  P / (P + delta_time);
    com_gyro = proporation * com_gyro + (1 - proporation) * gyro;

    return com_gyro;
}
