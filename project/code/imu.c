
#include "zf_common_headfile.h"

ANGLE my_angle;

#define dt 0.003 //卡尔曼滤波所在定时器时间
#define MY_PI 3.14159265358979323846 // 圆周率π的定义
typedef float Matrix3f[3][3];
typedef float Vector3f[3];
float angle=0;
float alpha=0.1;

ANGLE my_angle;
/***************************************************************
* 函数名称：uint8 Err_Reduce(void)
* 函数输入：无
* 函数输出：0
* 功能说明：陀螺仪去零漂
***************************************************************/
uint8 Err_Reduce(void)
{
    system_delay_ms(500);
    float Err[3]={0.0f};
    my_angle.Gyro.err[0]=0.0f;
    my_angle.Gyro.err[1]=0.0f;
    my_angle.Gyro.err[2]=0.0f;
    for(int i=0;i<50;i++)
    {
        imu660ra_get_gyro();
        system_delay_ms(3);
    }
    for(int i=0;i<100;i++)
    {
        imu660ra_get_gyro();
        Err[0]+=(float)imu660ra_gyro_x;
        Err[1]+=(float)imu660ra_gyro_y;
        Err[2]+=(float)imu660ra_gyro_z;
        system_delay_ms(3);
    }
    my_angle.Gyro.err[0]=Err[0]/100.0f;
    my_angle.Gyro.err[1]=Err[1]/100.0f;
    my_angle.Gyro.err[2]=Err[2]/100.0f;

    return 0;
}
/***************************************************************
* 函数名称：void imu660ra_value_get(void)
* 函数输入：无
* 函数输出：无
* 功能说明：陀螺仪数据读取
***************************************************************/
void imu660ra_value_get(void)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();


    my_angle.Gyro.x=(imu660ra_gyro_transition(imu660ra_gyro_x-my_angle.Gyro.err[0]));//真实角速度
    my_angle.Gyro.y=(imu660ra_gyro_transition(imu660ra_gyro_y-my_angle.Gyro.err[1]));
    my_angle.Gyro.z=(imu660ra_gyro_transition(imu660ra_gyro_z-my_angle.Gyro.err[2]));
    
    

    my_angle.Acc.x=imu660ra_acc_transition(imu660ra_acc_x);
    my_angle.Acc.y=imu660ra_acc_transition(imu660ra_acc_y);
    my_angle.Acc.z=imu660ra_acc_transition(imu660ra_acc_z);
    
    my_angle.Gyro.x_filtered= alpha * my_angle.Gyro.x + (1-alpha) * my_angle.Gyro.x_filtered;
    my_angle.Gyro.y_filtered= alpha * my_angle.Gyro.y + (1-alpha) * my_angle.Gyro.y_filtered;
    my_angle.Gyro.z_filtered= alpha * my_angle.Gyro.z + (1-alpha) * my_angle.Gyro.z_filtered;

//一阶低通滤波
    my_angle.Acc.ax_filtered = alpha * my_angle.Acc.x + (1-alpha) * my_angle.Acc.ax_filtered;//滤波后的加速度计值
    my_angle.Acc.ay_filtered = alpha * my_angle.Acc.y + (1-alpha) * my_angle.Acc.ay_filtered;
    my_angle.Acc.az_filtered = alpha * my_angle.Acc.z + (1-alpha) * my_angle.Acc.az_filtered;

    my_angle.Gyro.x_PI=my_angle.Gyro.x*(MY_PI/180.0);//弧度制角速度
    my_angle.Gyro.y_PI=my_angle.Gyro.y*(MY_PI/180.0);
    my_angle.Gyro.z_PI=my_angle.Gyro.z*(MY_PI/180.0);

}

/************************ 基础矩阵/向量运算函数 ************************/
// 3x3矩阵求逆（卡尔曼增益计算必需）
void matrix3f_inverse(const Matrix3f A, Matrix3f A_inv) {
    // 计算行列式
    float det = A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
              - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
              + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    // 行列式接近0时返回单位矩阵，避免除零
    if (fabs(det) < 1e-8) {
        memcpy(A_inv, (float[3][3]){{1,0,0},{0,1,0},{0,0,1}}, sizeof(Matrix3f));
        return;
    }

    float inv_det = 1.0f / det;
    // 伴随矩阵转置 / 行列式 = 逆矩阵
    A_inv[0][0] = (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * inv_det;
    A_inv[0][1] = (A[0][2]*A[2][1] - A[0][1]*A[2][2]) * inv_det;
    A_inv[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * inv_det;
    A_inv[1][0] = (A[1][2]*A[2][0] - A[1][0]*A[2][2]) * inv_det;
    A_inv[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * inv_det;
    A_inv[1][2] = (A[0][2]*A[1][0] - A[0][0]*A[1][2]) * inv_det;
    A_inv[2][0] = (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * inv_det;
    A_inv[2][1] = (A[0][1]*A[2][0] - A[0][0]*A[2][1]) * inv_det;
    A_inv[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * inv_det;
}

// 3x3矩阵乘法：C = A * B
void matrix3f_mult(const Matrix3f A, const Matrix3f B, Matrix3f C) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 3x3矩阵转置：B = A^T
void matrix3f_transpose(const Matrix3f A, Matrix3f B) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = A[j][i];
        }
    }
}

// 3x3矩阵加法：C = A + B
void matrix3f_add(const Matrix3f A, const Matrix3f B, Matrix3f C) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

// 3x3矩阵减法：C = A - B
void matrix3f_sub(const Matrix3f A, const Matrix3f B, Matrix3f C) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

// 3x3矩阵数乘：B = A * k
void matrix3f_scale(const Matrix3f A, float k, Matrix3f B) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = A[i][j] * k;
        }
    }
}

// 3x3矩阵乘3维向量：b = A * a
void matrix3f_mult_vec3f(const Matrix3f A, const Vector3f a, Vector3f b) {
    for (int i = 0; i < 3; i++) {
        b[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            b[i] += A[i][j] * a[j];
        }
    }
}

// 3维向量加法：c = a + b
void vec3f_add(const Vector3f a, const Vector3f b, Vector3f c) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

// 3 维向量减法：c = a - b
void vec3f_sub(const Vector3f a, const Vector3f b, Vector3f c) {
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

//Q 矩阵：反映陀螺仪的信任度，飞行速度越快，Q 应调小（减少陀螺仪漂移的影响）；
//R 矩阵：反映加速度计的信任度，机动越剧烈，R 应调大（加速度计受运动加速度干扰，可信度降低）。
//可通过 “固定机身摆动”“低速悬停” 两个场景，逐步微调参数，记录欧拉角的稳定性。

/***************************************************************
* 函数名称：void KF_6_Axis(float acc_x, float acc_y, float acc_z,
               float gyro_x, float gyro_y, float gyro_z)
* 函数输入：加速度计数值（m/s^2）
*       陀螺仪角速度值（rad/s）
* 函数输出：无
* 功能说明：陀螺仪数据卡尔曼滤波
***************************************************************/
void KF_6_Axis(float acc_x,float acc_y,float acc_z,
                float gyro_x,float gyro_y,float gyro_z)
{
    static Matrix3f Q = {{0.002f,0,0},{0,0.002f,0},{0,0,0.002f}};
    static Matrix3f R = {{0.2f,0,0},{0,0.2f,0},{0,0,0.2f}};

    static Matrix3f A = {{1,0,0},{0,1,0},{0,0,1}};
    static Matrix3f B = {{1,0,0},{0,1,0},{0,0,1}};
    static Matrix3f H = {{1,0,0},{0,1,0},{0,0,0}};

    static Vector3f X_bar_k_1 = {0, 0, 0}; 
    static Vector3f u_k = {0, 0, 0};       
    static Vector3f X_bar_k__ = {0,0,0};   
    static Vector3f X_bar_k = {0,0,0};     
    static Vector3f Z_k = {0,0,0};         

    static Matrix3f P_k_1 = {{0,0,0}, {0,0,0}, {0,0,0}}; // 
    static Matrix3f P_k__ = {{0,0,0}, {0,0,0}, {0,0,0}}; // 
    static Matrix3f P_k = {{1,0,0}, {0,1,0}, {0,0,1}};   // 
    static Matrix3f K_k = {{0,0,0}, {0,0,0}, {0,0,0}};   // 
    static Matrix3f I = {{1,0,0}, {0,1,0}, {0,0,1}};     // 

    matrix3f_scale(B, 0.003f, B);

    float roll = X_bar_k[0];    // 
    float pitch = X_bar_k[1];   // 
    float cos_pitch = cos(pitch);
    //        
    if (fabs(cos_pitch) < 1e-8)
    {
        cos_pitch = 1e-8;
    }

    //         
    float droll_dt = gyro_x +
                    (sin(pitch)*sin(roll)/cos_pitch)*gyro_y +
                    (sin(pitch)*cos(roll)/cos_pitch)*gyro_z;
    // 
    float dpitch_dt = cos(roll)*gyro_y - sin(roll)*gyro_z;
            // y  轴角速度计算
    float dyaw_dt = (sin(roll)/cos_pitch)*gyro_y + (cos(roll)/cos_pitch)*gyro_z;

    u_k[0] = droll_dt;
    u_k[1] = dpitch_dt;
    u_k[2] = dyaw_dt;

    // -------------------------- 3. 卡尔曼滤波核心步骤 --------------------------
    // 步骤1：状态预测 X_k' = A*X_{k-1} + B*u_k
    Vector3f A_X, B_u;
    matrix3f_mult_vec3f(A, X_bar_k_1, A_X);       // A * X_bar_k_1
    matrix3f_mult_vec3f(B, u_k, B_u);             // B * u_k
    vec3f_add(A_X, B_u, X_bar_k__);               // 预测状态,A_X+B_u

    // 步骤2：协方差预测 P_k' = A*P_{k-1}*A^T + Q
    Matrix3f A_P, A_P_AT, A_T;
    matrix3f_mult(A, P_k_1, A_P);                 // A * P_k_1
    matrix3f_transpose(A, A_T);                   // A的转置
    matrix3f_mult(A_P, A_T, A_P_AT);              // (A*P_k_1)*A^T
    matrix3f_add(A_P_AT, Q, P_k__);               // 加过程噪声Q,A_P_AT+Q

    // 步骤3：计算卡尔曼增益 K_k = P_k'*H^T*(H*P_k'*H^T + R)^-1
    Matrix3f H_T, P_Ht, H_P_Ht, H_P_Ht_R, H_P_Ht_R_inv;
    matrix3f_transpose(H, H_T);                   // H的转置
    matrix3f_mult(P_k__, H_T, P_Ht);              // P_k' * H^T

    matrix3f_mult(H, P_k__, H_P_Ht);              // H * P_k'
    matrix3f_mult(H_P_Ht, H_T, H_P_Ht);           // (H*P_k')*H^T
    matrix3f_add(H_P_Ht, R, H_P_Ht_R);            // 加观测噪声R,H_P_Ht + R
    matrix3f_inverse(H_P_Ht_R, H_P_Ht_R_inv);     // 求逆矩阵
    matrix3f_mult(P_Ht, H_P_Ht_R_inv, K_k);       // 最终卡尔曼增益，H_P_Ht_R_inv * P_Ht

    // 步骤4：更新协方差矩阵 P_k = (I - K_k*H) * P_k'
    Matrix3f K_H, I_K_H;
    matrix3f_mult(K_k, H, K_H);                   // K_k * H
    matrix3f_sub(I, K_H, I_K_H);                  // I - K_k*H
    matrix3f_mult(I_K_H, P_k__, P_k);             // 最终协方差，I_K_H * P_k__

    // -------------------------- 4. 计算观测值Z_k（加速度计解算角度） --------------------------
    // Roll角：a tan(acc_y / acc_z)
    float acc_roll = atan2(acc_y, acc_z);
    // Pitch角：-a tan(acc_x / s q r t(acc_y *acc_y + acc_z*acc_z))
    float acc_pitch = -atan2(acc_x, sqrt(acc_y*acc_y + acc_z*acc_z));


    Z_k[0] = acc_roll;
    Z_k[1] = acc_pitch;
    Z_k[2] = 0.0f; // Y a w轴无观测值

    // -------------------------- 5. 更新状态量 X_k = X_k' + K_k*(Z_k - H*X_k') --------------------------
    Vector3f H_X, Z_HX, K_ZHX;
    matrix3f_mult_vec3f(H, X_bar_k__, H_X);        // H * X_bar_k__
    vec3f_sub(Z_k, H_X, Z_HX);                    // Z_k - H*X_bar_k__
    matrix3f_mult_vec3f(K_k, Z_HX, K_ZHX);        // K_k * (Z_k - H*X_bar_k__)
    vec3f_add(X_bar_k__, K_ZHX, X_bar_k);         // 最终估计状态

    // -------------------------- 6. 保存状态供下次循环使用 --------------------------
    memcpy(P_k_1, P_k, sizeof(Matrix3f));
    memcpy(X_bar_k_1, X_bar_k, sizeof(Vector3f));

    // -------------------------- 7. 输出解算后的欧拉角 --------------------------
    my_angle.Roll = X_bar_k[0] * 57.3; // roll
    my_angle.Pitch = X_bar_k[1] * 57.3; // pitch
//    my_angle.Yaw = X_bar_k[2] * 57.3; // y a w

}
//——————————————————————————kf滤波——————————————————————————————————


//——————————————————————————mahony滤波——————————————————————————————————
#define Kp      3.5f                        // 比例增益控制着加速度计/磁力计的收敛速度//1.42f
#define Ki      0.005f                       //积分增益控制陀螺仪偏差的收敛速度
#define halfT   0.0015f                   // 采样周期的一半
#define TIME    0.003f
#define INTEGRAL_LIMIT 0.1f  // 积分项最大绝对值限制，建议初始值0.2~1.0
static float q01 = 1, q11 = 0, q21 = 0, q31 = 0;    // 表示估计方向的四元数元素
static float exInt = 0, eyInt = 0, ezInt = 0;    // 比例积分误差
///***************************************************************
//* 函数名称：void IMU_Update(float gx, float gy, float gz, float ax, float ay, float az)
//* 函数输入：  gx:角速度计X轴数值(必须是弧度制)             ax:加速度计X轴值
//*           gy:角速度计y轴数值(必须是弧度制)             ay:加速度计y轴值
//*           gz:角速度计z轴数值(必须是弧度制)             az:加速度计z轴值
//* 函数输出：无
//* 功能说明：四元数解算
//* //解算出来pitch和roll相反，-180<=roll<=180,-90<=pitch<=90,-180<=yaw<=180
//* //所以直接将两者换位，以后注意坐标系
//***************************************************************/
////四元数解算
void IMU_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float temp0, temp1, temp2, temp3;

    float q0q0 = q01 * q01;
    float q0q1 = q01 * q11;
    float q0q2 = q01 * q21;
//    float q0q3 = q0 * q3;
    float q1q1 = q11 * q11;
//    float q1q2 = q1 * q2;
    float q1q3 = q11 * q31;
    float q2q2 = q21 * q21;
    float q2q3 = q21 * q31;
    float q3q3 = q31 * q31;

    if (ax * ay * az == 0)
    {
        return;
    }
    

    norm = sqrt(ax * ax + ay * ay + az * az);       //
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    // 重力和通量的估计方向（v和w）
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    // 误差是场的参考方向与传感器测量的方向之间的叉积之和
    ex = (ay * vz - az * vy) ;
    ey = (az * vx - ax * vz) ;
    ez = (ax * vy - ay * vx) ;

    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
    exInt = (exInt > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT :
            ((exInt < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : exInt);
    eyInt = (eyInt > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT :
            ((eyInt < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : eyInt);
    ezInt = (ezInt > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT :
            ((ezInt < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : ezInt);

    //调整后的陀螺仪测量
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    // 四元数率的积分与归一化
    temp0 = q01;
    temp1 = q11;
    temp2 = q21;
    temp3 = q31;
    q01 += (-temp1 * gx - temp2 * gy - temp3 * gz) * halfT;
    q11 += (temp0 * gx + temp2 * gz - temp3 * gy) * halfT;
    q21 += (temp0 * gy - temp1 * gz + temp3 * gx) * halfT;
    q31 += (temp0 * gz + temp1 * gy - temp2 * gx) * halfT;

    // 归一化四元数
    norm = sqrt(q01*q01 + q11*q11 + q21*q21 + q31*q31);
    if (norm < 1e-6) // 防止除零
    {
        q01 = 1.0; q11 = q21 = q31 = 0.0;
    }
    else
    {
        q01 /= norm;
        q11 /= norm;
        q21 /= norm;
        q31 /= norm;
    }
if(flagqingling==1)//两秒后数据稳定再输出
{    my_angle.Yaw = (atan2(2 * q11 * q21 + 2 * q01 * q31, -2 * q21 * q21 - 2 * q31 * q31 + 1)) * 57.3f; // 单位：度
//    my_angle.Pitch= asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;

//    my_angle.Roll = atan2(2 * q21 * q31 + 2 * q01 * q11, -2 * q11 * q11- 2 * q21 * q21 + 1) * 57.3f;
}
}

//——————————————————————————mahony滤波——————————————————————————————————
ANGLE angle_ekf;
#define  halfT 0.0015f
#define GRAVITY         9.7969f
//

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float w1 = 0.0f, w2 = 0.0f, w3 = 0.0f;             // 陀螺仪零偏


// 3x7观测矩阵H（仅加速度维度，移除磁力计后3行）
static float H[21] = {0};

// 7x7协方差矩阵P（初始值保留原逻辑）
static float P[49] = {
    0.0001, 0, 0, 0, 0, 0, 0,
    0, 0.0001, 0, 0, 0, 0, 0,
    0, 0, 0.0001, 0, 0, 0, 0,
    0, 0, 0, 0.0001, 0, 0, 0,
    0, 0, 0, 0, 0.0002, 0, 0,
    0, 0, 0, 0, 0, 0.0002, 0,
    0, 0, 0, 0, 0, 0, 0.0002
};

// 7x7过程噪声矩阵Q（陀螺仪零偏噪声略大，适配6轴）
static float Q[49] = {
    0.0006f, 0, 0, 0, 0, 0, 0,
    0, 0.0006f, 0, 0, 0, 0, 0,
    0, 0, 0.0006f, 0, 0, 0, 0,
    0, 0, 0, 0.0006f, 0, 0, 0,
    0, 0, 0, 0, 0.0001f, 0, 0,
    0, 0, 0, 0, 0, 0.0001f, 0,
    0, 0, 0, 0, 0, 0, 0.0001f
};

// 3x3观测噪声矩阵R（仅加速度维度，移除磁力计后3行）
static float R[9] = {
    0.00005f, 0, 0,
    0, 0.00005f, 0,
    0, 0, 0.00005f
};

// 7x7单位矩阵I
static float I[49] = {
    1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1
};
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


/**
 * @brief  矩阵乘法（简化版，仅适配本代码维度）
 * @param  A: 输入矩阵1, row1: A的行数, col1: A的列数
 * @param  B: 输入矩阵2, row2: B的行数, col2: B的列数
 * @param  C: 输出矩阵（A*B）
 */
static void MatrixMultiply(float *A, int row1, int col1, float *B, int row2, int col2, float *C)
{
    for (int i = 0; i < row1; i++) {
        for (int j = 0; j < col2; j++) {
            C[i*col2 + j] = 0.0f;
            for (int k = 0; k < col1; k++) {
                C[i*col2 + j] += A[i*col1 + k] * B[k*col2 + j];
            }
        }
    }
}

/**
 * @brief  矩阵转置
 * @param  A: 输入矩阵, row: 行数, col: 列数
 * @param  AT: 输出转置矩阵
 */
static void MatrixTranspose(float *A, int row, int col, float *AT)
{
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            AT[j*row + i] = A[i*col + j];
        }
    }
}

/**
 * @brief  矩阵加法
 */
static void MatrixAdd(float *A, float *B, float *C, int row, int col)
{
    for (int i = 0; i < row*col; i++) {
        C[i] = A[i] + B[i];
    }
}

/**
 * @brief  矩阵减法
 */
static void MatrixSub(float *A, float *B, float *C, int row, int col)
{
    for (int i = 0; i < row*col; i++) {
        C[i] = A[i] - B[i];
    }
}

/**
 * @brief  矩阵求逆（仅适配3x3矩阵）
 */
static void MatrixInverse(float *A, int n, int flag)
{
    if (n != 3) return; // 仅支持3x3（加速度观测维度）
    float det = A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
    if (fabsf(det) < 1e-6f) det = 1e-6f;
    float inv_det = 1.0f / det;
    
    float temp[9];
    temp[0] = (A[4]*A[8] - A[5]*A[7]) * inv_det;
    temp[1] = (A[2]*A[7] - A[1]*A[8]) * inv_det;
    temp[2] = (A[1]*A[5] - A[2]*A[4]) * inv_det;
    temp[3] = (A[5]*A[6] - A[3]*A[8]) * inv_det;
    temp[4] = (A[0]*A[8] - A[2]*A[6]) * inv_det;
    temp[5] = (A[2]*A[3] - A[0]*A[5]) * inv_det;
    temp[6] = (A[3]*A[7] - A[4]*A[6]) * inv_det;
    temp[7] = (A[1]*A[6] - A[0]*A[7]) * inv_det;
    temp[8] = (A[0]*A[4] - A[1]*A[3]) * inv_det;
    
    for (int i = 0; i < 9; i++) A[i] = temp[i];
}



uint8_t ekf_6dof_update(float acc_x, float acc_y, float acc_z,
                        float gyro_x, float gyro_y, float gyro_z)
{
    float norm;                   // 模值
    float vx, vy, vz;             // 加速度观测预测值（仅保留加速度）
    float g = GRAVITY;            // 当地重力加速度（适配m/s2）
    static float A[49], B[49], E[21], F[49], F1[9], Ft[49], Ht[21], K[21];
    static float O[49], P1[49], T[3], X[9], Y[7], Z[49];
//    static  float D1[9], U1[9], U1t[9], X1[9], X2[9];
    // 加速度数据有效性校验（模长过小则返回错误）
   
    norm = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z); 
    if (norm < 0.1f * g) { // 小于0.1g则认为数据无效
        return 1;
    }

    // 加速度归一化（适配m/s2单位）
    acc_x = acc_x / norm * g;
    acc_y = acc_y / norm * g;
    acc_z = acc_z / norm * g;

    // 陀螺仪补偿零偏（w1-w3是陀螺仪零偏）
    gyro_x = gyro_x - w1;
    gyro_y = gyro_y - w2;
    gyro_z = gyro_z - w3;

    // 采样周期计算（修正原代码halfT错误，默认200Hz采样→T=0.005s）
    

    // 预计算四元数乘积（减少重复计算）
    

    /************************* 步骤1：状态预测（四元数更新） *************************/
    // 一阶龙格库塔法更新四元数（仅保留陀螺仪积分）
    float q0o=q0, q1o=q1, q2o=q2, q3o=q3;
    q0 = q0o + (-q1o * gyro_x - q2o * gyro_y - q3o * gyro_z) * halfT;
    q1 = q1o + (q0o * gyro_x + q2o * gyro_z - q3o * gyro_y) * halfT;
    q2 = q2o + (q0o * gyro_y - q1o * gyro_z + q3o * gyro_x) * halfT;
    q3 = q3o + (q0o * gyro_z + q1o * gyro_y - q2o * gyro_x) * halfT;

    // 四元数归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    /************************* 步骤2：观测预测（加速度） *************************/
    // 仅保留加速度预测值（移除磁力计wx/wy/wz）
    vx = 2 * (q1q3 - q0q2) * g;  // ax预测值
    vy = 2 * (q0q1 + q2q3) * g;  // ay预测值
    vz = (q0q0 - q1q1 - q2q2 + q3q3) * g; // az预测值

    /************************* 步骤3：状态转移矩阵F赋值 *************************/
    F[0] = 1;                F[1] = -gyro_x * halfT;    F[2] = -gyro_y * halfT;   F[3] = -gyro_z * halfT;    F[4] = 0;    F[5] = 0;   F[6] = 0;
    F[7] = gyro_x * halfT;   F[8] = 1;                  F[9] = gyro_z * halfT;    F[10] = -gyro_y * halfT;   F[11] = 0;   F[12] = 0;  F[13] = 0;
    F[14] = gyro_y * halfT;  F[15] = -gyro_z * halfT;   F[16] = 1;                F[17] = gyro_x * halfT;    F[18] = 0;   F[19] = 0;  F[20] = 0;
    F[21] = gyro_z * halfT;  F[22] = gyro_y * halfT;    F[23] = -gyro_x * halfT;  F[24] = 1;                 F[25] = 0;   F[26] = 0;  F[27] = 0;
    F[28] = 0;               F[29] = 0;                 F[30] = 0;                F[31] = 0;                 F[32] = 1;   F[33] = 0;  F[34] = 0;
    F[35] = 0;               F[36] = 0;                 F[37] = 0;                F[38] = 0;                 F[39] = 0;   F[40] = 1;  F[41] = 0;
    F[42] = 0;               F[43] = 0;                 F[44] = 0;                F[45] = 0;                 F[46] = 0;   F[47] = 0;  F[48] = 1;

    /************************* 步骤4：观测矩阵H赋值（仅加速度） *************************/
    // 仅保留加速度对应的3行（移除磁力计后3行）
    float Ha1 = 2 * -q2  * g;
    float Ha2 = 2 * q3   * g;
    float Ha3 = 2 * -q0  * g;
    float Ha4 = 2 * q1   * g;

    H[0] = Ha1;   H[1] = Ha2;   H[2] = Ha3;  H[3] = Ha4;   H[4] = 0;  H[5] = 0;  H[6] = 0;
    H[7] = Ha4;   H[8] = -Ha3;  H[9] = Ha2;  H[10] = -Ha1; H[11] = 0; H[12] = 0; H[13] = 0;
    H[14] = -Ha3; H[15] = -Ha4; H[16] = Ha1; H[17] = Ha2;  H[18] = 0; H[19] = 0; H[20] = 0;
//    float acc_norm = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
//float trust = 1.0f - fminf(1.0f, fabsf(acc_norm - g) / g);
//float R_diag = 0.003f / (trust*trust + 0.01f);
//R[0] = R[4] = R[8] = R_diag;

    /************************* 步骤5：协方差预测 P(k|k-1) = F*P*F' + Q *************************/
    MatrixMultiply(F, 7, 7, P, 7, 7, A);  // A = F*P
    MatrixTranspose(F, 7, 7, Ft);         // Ft = F'
    MatrixMultiply(A, 7, 7, Ft, 7, 7, B); // B = F*P*F'
    MatrixAdd(B, Q, P1, 7, 7);            // P1 = F*P*F' + Q

    /************************* 步骤6：计算卡尔曼增益 K = P*H'*(H*P*H' + R)^-1 *************************/
    // -------- 自适应R：用原始模长偏离g来放大R --------
    MatrixTranspose(H, 3, 7, Ht);         // Ht = H'
    MatrixMultiply(P1, 7, 7, Ht, 7, 3, E); // E = P1*H'
    MatrixMultiply(H, 3, 7, E, 7, 3, F1);  // F1 = H*P1*H'
    MatrixAdd(F1, R, X, 3, 3);            // X = H*P1*H' + R

    // UD分解+求逆（防止矩阵奇异）
//    UD(X, 3, U1, D1);
//    MatrixTranspose(U1, 3, 3, U1t);
//    MatrixMultiply(U1, 3, 3, D1, 3, 3, X1);
//    MatrixMultiply(X1, 3, 3, U1t, 3, 3, X2);
//    MatrixInverse(X2, 3, 0);
//
//    // 计算卡尔曼增益K（7x3）
//    MatrixMultiply(E, 7, 3, X2, 3, 3, K); 
MatrixInverse(X, 3, 0); // 直接求逆
MatrixMultiply(E, 7, 3, X, 3, 3, K);

    /************************* 步骤7：状态更新 X(k) = X(k|k-1) + K*(Y-Y_pred) *************************/
    // 仅保留加速度误差（移除磁力计e4/e5/e6）
    float e1 = acc_x - vx;
    float e2 = acc_y - vy;
    float e3 = acc_z - vz;
    
    // 误差矩阵赋值（3x1）
    T[0]=e1;
    T[1]=e2;
    T[2]=e3;
  


    // 计算状态更新量Y（7x1）
    MatrixMultiply(K, 7, 3, T, 3, 1, Y);

    // 更新四元数和陀螺仪零偏
    q0 += Y[0];
    q1 += Y[1];
    q2 += Y[2];
    q3 += Y[3];
    w1 += Y[4];
    w2 += Y[5];
    w3 += Y[6];

    // 四元数重新归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    /************************* 步骤8：协方差更新 P(k) = (I-K*H)*P1 *************************/
    MatrixMultiply(K, 7, 3, H, 3, 7, Z);  // Z = K*H
    MatrixSub(I, Z, O, 7, 7);             // O = I - K*H
    MatrixMultiply(O, 7, 7, P1, 7, 7, P); // P = (I-K*H)*P1

    /************************* 步骤9：四元数转欧拉角 *************************/
;
    angle_ekf.Roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f/MY_PI;
    angle_ekf.Pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f/MY_PI;  
    angle_ekf.Yaw  = atan2f(2.0f*(q0*q3 +q1*q2), 
                          1.0f - 2.0f*(q2*q2 +q3*q3)) * 180.0f/MY_PI;
    if (angle_ekf.Yaw < 0.0f) angle_ekf.Yaw += 360.0f;
    return 0; // 更新成功
}
//------------------------------------------------EKF---------------------------------




