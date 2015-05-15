#include <Angle.h>

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

    // board_gyro = complementaryFilter(original_gyro, delta_time);
    klm_angle = kalmanFilter(original_angle, original_gyro, delta_time);
    // board_angle = complementary2Filter(klm_angle, original_gyro, delta_time);
    board_angle = klm_angle;

    board_angle = constrain(board_angle, MIN_ANGLE, MAX_ANGLE); // board angle limit
}

void angleCtrlPID(void)
{
    angle_ctrl_output = anglePIDCompute(board_angle);
}

/*
* kalman filter for angle
*/
float kalmanFilter(float angle_m, float gyro_m, float delta_time)
{
// #define Q_ANGLE     0.01     // test show: 0.001 is better than 0.01
// #define Q_OMEGA     0.0003   // test show: 0.005 is better than 0.0003
// #define R_ANGLE     0.01     // test show: 0.1 is better than 0.01

//     static float klm_angle = 0; // kalman fiter output
//     static float bias = 0;
//     static float P[2][2] = {0};

//     klm_angle += (gyro - bias) * delta_time;
//     P[0][0] += -(P[1][0] + P[0][1]) * delta_time + Q_ANGLE * delta_time;
//     P[0][1] -= P[1][1] * delta_time;
//     P[1][0] -= P[1][1] * delta_time;
//     P[1][1] += Q_OMEGA * delta_time;
//     float K_0 = P[0][0] / (P[0][0] + R_ANGLE);
//     float K_1 = P[1][0] / (P[0][0] + R_ANGLE);
//     bias += K_1 * (angle - klm_angle);
//     klm_angle += K_0 * (angle - klm_angle);
//     P[0][0] -= K_0 * P[0][0];
//     P[0][1] -= K_0 * P[0][1];
//     P[1][0] -= K_1 * P[0][0];
//     P[1][1] -= K_1 * P[0][1];


    static float klm_angle = 0; // kalman fiter output
    static float R_anglex = 0.5, R_gyrox = 0.5;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E1, E2;
    static float K_0, K_1, t_0, t_1;
    static float Angle, Angle_dot;   //外部需要引用的变量,倾角值和倾角加速度值

    const float Q_angle = 0.002, Q_gyro = 0.002;
    const float C_0 = 1;

    /* 当前状态的最优值 */
    static float P[2][2] = {{ 1, 0 },{ 0, 1 }};
    static float Pdot[4] = {0,0,0,0}; //先验估计协方差

    float dt = delta_time ;

    klm_angle += (gyro_m-Q_bias) * dt;          //先验估计

    /* 上一时刻，先验误差协方差矩阵：Pdot = A*P + P*A' + Q
     * （过程噪声协方差）Pk-先验估计误差协方差的微分
     */
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = - P[1][1];
    Pdot[2] = - P[1][1];
    Pdot[3] = Q_gyro;                 //过程噪声协方差

    //当前先验误差协方差矩阵更新
    P[0][0] += Pdot[0] * dt;        // Pk-先验估计误差协方差微分的积分
    P[0][1] += Pdot[1] * dt;        // 先验估计误差协方差
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    Angle_err = angle_m - klm_angle;    // zk-先验估计，测量值 - 上一时刻的估计值

    PCt_0 = C_0 * P[0][0];          // +h_1*P[0][1] = 0， PCt_0 = H*P(k|k-1)H'(卡尔曼增益的分母组成部分)
    PCt_1 = C_0 * P[1][0];          //  + h_1*P[1][1] = 0,PCt_1 = P(k|k-1)H' (卡尔曼增益的分子部分)

    E1 = R_anglex + C_0 * PCt_0;    // (H P(k|k-1) H’ + R)
    E2 = R_gyrox + C_0 * PCt_0;     // (H P(k|k-1) H’ + R)

    K_0 = PCt_0 / E1;               //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
    K_1 = PCt_1 / E2;

    /* P(k|k)=(I-Kg(k) H)P(k|k-1) <---> P(k|k) = P(k|k-1) - K C P(k|k-1)
     * t[0,0] = C[0,0] * P[0,0] + C[0,1] * P[1,0]
     */
    t_0 = PCt_0;                    /* C_0 * P[0][0] + C_1 * P[1][0] */
    t_1 = C_0 * P[0][1];            /* + C_1 * P[1][1] = 0 */

    P[0][0] -= K_0 * t_0;           //后验估计误差协方差
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    klm_angle   += K_0 * Angle_err;     //后验估计,当前时刻最优值=上一时刻估计值+Kg*（当前测量值-上一时刻的估计值）
    Q_bias  += K_1 * Angle_err;     //后验估计
    Angle_dot = gyro_m - Q_bias;    //输出值(后验估计)的微分=角速度

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
