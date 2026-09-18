#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>

// ============================================================
// MPU6500 CONFIGURATION
// ============================================================

#define MPU_ADDR 0x68

#define SDA_PIN 8
#define SCL_PIN 9
#define INT_PIN 7

#define MPU_WHO_AM_I      0x75
#define MPU_CONFIG        0x1A
#define MPU_SMPLRT_DIV    0x19
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_CONFIG2 0x1D
#define MPU_INT_PIN_CFG   0x37
#define MPU_INT_ENABLE    0x38
#define MPU_PWR_MGMT_1    0x6B
#define MPU_ACCEL_XOUT_H  0x3B

#define MPU_WHO_AM_I_VALUE 0x70

// ============================================================
// SENSOR SETTINGS
// ============================================================

constexpr float ACCEL_LSB_PER_G = 16384.0f;   // ±2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;    // ±250 deg/s

constexpr float G_TO_MS2 = 9.80665f;

// Avoid collision with Arduino.h's DEG_TO_RAD macro
constexpr float D2R = 0.017453292519943295f;
constexpr float R2D = 57.29577951308232f;

constexpr float TARGET_DT = 0.005f;
constexpr float TARGET_HZ = 200.0f;

// ============================================================
// SERVO HARDWARE
// ============================================================

constexpr int SERVO_PIN       = 18;
constexpr int SERVO_MIN_DEG   = 0;
constexpr int SERVO_MAX_DEG   = 180;
constexpr int SERVO_START_DEG = 90;
constexpr int SERVO_CENTER_DEG = 90;

Servo testServo;

// ============================================================
// CONTROL MODE
// ============================================================

enum ControlMode
{
    MODE_MANUAL,
    MODE_AUTO
};

ControlMode currentMode = MODE_MANUAL;

// ============================================================
// PID CONTROLLER (attitude-hold demo)
// ============================================================

constexpr float PID_KP = 0.5f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.05f;

constexpr float PID_INTEGRAL_CLAMP = 10.0f;
constexpr float PID_OUTPUT_CLAMP   = 15.0f;

float pidIntegral  = 0.0f;
float pidPrevError = 0.0f;

// ============================================================
// ACCELEROMETER CALIBRATION
// ============================================================

constexpr float ACCEL_BIAS_X = +0.018273f;
constexpr float ACCEL_BIAS_Y = -0.023688f;
constexpr float ACCEL_BIAS_Z = +0.222157f;

constexpr float ACCEL_SCALE_X = 1.007620f;
constexpr float ACCEL_SCALE_Y = 1.028348f;
constexpr float ACCEL_SCALE_Z = 0.991540f;

// ============================================================
// GYROSCOPE CALIBRATION
// ============================================================

constexpr float GYRO_BIAS_X = +1.396359f;
constexpr float GYRO_BIAS_Y = -3.878987f;
constexpr float GYRO_BIAS_Z = +2.094733f;

// ============================================================
// MAHONY AHRS
// ============================================================

constexpr float MAHONY_KP = 2.0f;
constexpr float MAHONY_KI = 0.0f;

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float mahonyIntegralX = 0.0f;
float mahonyIntegralY = 0.0f;
float mahonyIntegralZ = 0.0f;

// ============================================================
// 15-STATE ERROR-STATE EKF
// ============================================================

constexpr int EKF_N = 15;

float P[EKF_N][EKF_N];

float positionE = 0.0f;
float positionN = 0.0f;
float positionU = 0.0f;

float velocityE = 0.0f;
float velocityN = 0.0f;
float velocityU = 0.0f;

float gyroBiasEstX = 0.0f;
float gyroBiasEstY = 0.0f;
float gyroBiasEstZ = 0.0f;

float accelBiasEstX = 0.0f;
float accelBiasEstY = 0.0f;
float accelBiasEstZ = 0.0f;

// ============================================================
// NOISE / EKF TUNING
// ============================================================

constexpr float ACCEL_NOISE_STD_MPS2 =
    0.00336f * G_TO_MS2;

constexpr float GYRO_NOISE_STD_RADPS =
    0.207446f * D2R;

constexpr float GYRO_BIAS_RW =
    0.0001f * D2R;

constexpr float ACCEL_BIAS_RW = 0.001f;

// ============================================================
// ZUPT SETTINGS
// ============================================================

constexpr float ZUPT_GYRO_THRESHOLD_DPS = 5.0f;
constexpr float ZUPT_ACCEL_ERROR_G = 0.08f;

constexpr int ZUPT_ON_COUNT  = 10;
constexpr int ZUPT_OFF_COUNT = 5;

constexpr float ZUPT_VELOCITY_STD = 0.05f;

bool zuptActive = false;

int stationaryCount = 0;
int movingCount     = 0;

// ============================================================
// BIAS CLAMP LIMITS
// ============================================================

constexpr float GYRO_BIAS_CLAMP  = 0.1f;
constexpr float ACCEL_BIAS_CLAMP = 1.0f;

// ============================================================
// IMU DATA
// ============================================================

struct IMUData
{
    float ax;
    float ay;
    float az;

    float gx;   // deg/s
    float gy;   // deg/s
    float gz;   // deg/s
};

IMUData imu;

// ============================================================
// DATA READY INTERRUPT
// ============================================================

volatile bool imuDataReady = false;

void IRAM_ATTR imuDataReadyISR()
{
    imuDataReady = true;
}

// ============================================================
// BASIC VECTOR HELPERS
// ============================================================

float vectorMagnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

// ============================================================
// 3x3 MATRIX
// ============================================================

struct Matrix3
{
    float m[3][3];
};

Matrix3 skew(float x, float y, float z)
{
    Matrix3 S = {};

    S.m[0][0] = 0.0f;
    S.m[0][1] = -z;
    S.m[0][2] = y;

    S.m[1][0] = z;
    S.m[1][1] = 0.0f;
    S.m[1][2] = -x;

    S.m[2][0] = -y;
    S.m[2][1] = x;
    S.m[2][2] = 0.0f;

    return S;
}

// ============================================================
// QUATERNION NORMALIZATION
// ============================================================

void normalizeQuaternion()
{
    float norm =
        sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (norm < 1e-12f)
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        return;
    }

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

// ============================================================
// QUATERNION MULTIPLICATION
// ============================================================

void quaternionMultiply(
    float a0, float a1, float a2, float a3,
    float b0, float b1, float b2, float b3,
    float &r0, float &r1, float &r2, float &r3)
{
    r0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
    r1 = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2;
    r2 = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1;
    r3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
}

// ============================================================
// QUATERNION → ROLL / PITCH (degrees)
// ============================================================

float getRollDeg()
{
    float sinr = 2.0f * (q0 * q1 + q2 * q3);
    float cosr = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    return atan2f(sinr, cosr) * R2D;
}

float getPitchDeg()
{
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = constrain(sinp, -1.0f, 1.0f);
    return asinf(sinp) * R2D;
}

// ============================================================
// MAHONY AHRS
// ============================================================

void updateMahony(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float dt)
{
    gx *= D2R;
    gy *= D2R;
    gz *= D2R;

    float accelNorm = sqrtf(ax * ax + ay * ay + az * az);

    if (accelNorm < 1e-8f)
        return;

    ax /= accelNorm;
    ay /= accelNorm;
    az /= accelNorm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    if (MAHONY_KI > 0.0f)
    {
        mahonyIntegralX += MAHONY_KI * ex * dt;
        mahonyIntegralY += MAHONY_KI * ey * dt;
        mahonyIntegralZ += MAHONY_KI * ez * dt;

        gx += mahonyIntegralX;
        gy += mahonyIntegralY;
        gz += mahonyIntegralZ;
    }
    else
    {
        mahonyIntegralX = 0.0f;
        mahonyIntegralY = 0.0f;
        mahonyIntegralZ = 0.0f;
    }

    gx += MAHONY_KP * ex;
    gy += MAHONY_KP * ey;
    gz += MAHONY_KP * ez;

    float halfDt = 0.5f * dt;

    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
    float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
    float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
    float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * halfDt;

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    normalizeQuaternion();
}

// ============================================================
// BODY -> ENU ROTATION
// ============================================================

Matrix3 bodyToENU()
{
    Matrix3 R;

    R.m[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    R.m[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    R.m[0][2] = 2.0f * (q1 * q3 + q0 * q2);

    R.m[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    R.m[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    R.m[1][2] = 2.0f * (q2 * q3 - q0 * q1);

    R.m[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    R.m[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    R.m[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    return R;
}

// ============================================================
// INITIALIZE EKF COVARIANCE
// ============================================================

void initializeCovariance()
{
    memset(P, 0, sizeof(P));

    float positionStd = 1.0f;
    P[0][0] = positionStd * positionStd;
    P[1][1] = positionStd * positionStd;
    P[2][2] = positionStd * positionStd;

    float velocityStd = 0.5f;
    P[3][3] = velocityStd * velocityStd;
    P[4][4] = velocityStd * velocityStd;
    P[5][5] = velocityStd * velocityStd;

    float attitudeStd = 5.0f * D2R;
    P[6][6] = attitudeStd * attitudeStd;
    P[7][7] = attitudeStd * attitudeStd;
    P[8][8] = attitudeStd * attitudeStd;

    float gyroBiasStd = 1.0f * D2R;
    P[9][9]   = gyroBiasStd * gyroBiasStd;
    P[10][10] = gyroBiasStd * gyroBiasStd;
    P[11][11] = gyroBiasStd * gyroBiasStd;

    float accelBiasStd = 0.2f;
    P[12][12] = accelBiasStd * accelBiasStd;
    P[13][13] = accelBiasStd * accelBiasStd;
    P[14][14] = accelBiasStd * accelBiasStd;
}

// ============================================================
// EKF PREDICTION
// ============================================================

void ekfPredict(
    const Matrix3 &R,
    float fx, float fy, float fz,
    float gx, float gy, float gz,
    float dt)
{
    float F[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
        F[i][i] = 1.0f;

    for (int i = 0; i < 3; i++)
        F[i][i + 3] = dt;

    Matrix3 Sf = skew(fx, fy, fz);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float value = 0.0f;
            for (int k = 0; k < 3; k++)
                value += R.m[i][k] * Sf.m[k][j];

            F[i + 3][j + 6] = -value * dt;
        }
    }

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F[i + 3][j + 12] = -R.m[i][j] * dt;

    Matrix3 Sw = skew(gx, gy, gz);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            F[i + 6][j + 6] =
                (i == j ? 1.0f : 0.0f) - Sw.m[i][j] * dt;
        }
    }

    F[6][9]  = -dt;
    F[7][10] = -dt;
    F[8][11] = -dt;

    float Q[EKF_N][EKF_N] = {};

    float sigmaA2 = ACCEL_NOISE_STD_MPS2 * ACCEL_NOISE_STD_MPS2;
    float sigmaG2 = GYRO_NOISE_STD_RADPS  * GYRO_NOISE_STD_RADPS;
    float sigmaBg2 = GYRO_BIAS_RW * GYRO_BIAS_RW;
    float sigmaBa2 = ACCEL_BIAS_RW * ACCEL_BIAS_RW;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    float qPos    = sigmaA2 * dt3 / 3.0f;
    float qPosVel = sigmaA2 * dt2 / 2.0f;
    float qVel    = sigmaA2 * dt;

    for (int i = 0; i < 3; i++)
    {
        Q[i][i]         = qPos;
        Q[i][i + 3]     = qPosVel;
        Q[i + 3][i]     = qPosVel;
        Q[i + 3][i + 3] = qVel;
    }

    for (int i = 0; i < 3; i++)
    {
        Q[i + 6][i + 6]   = sigmaG2 * dt;
        Q[i + 9][i + 9]   = sigmaBg2 * dt;
        Q[i + 12][i + 12] = sigmaBa2 * dt;
    }

    float FP[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            for (int k = 0; k < EKF_N; k++)
                FP[i][j] += F[i][k] * P[k][j];

    float newP[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
    {
        for (int j = 0; j < EKF_N; j++)
        {
            for (int k = 0; k < EKF_N; k++)
                newP[i][j] += FP[i][k] * F[j][k];

            newP[i][j] += Q[i][j];
        }
    }

    memcpy(P, newP, sizeof(P));

    for (int i = 0; i < EKF_N; i++)
    {
        for (int j = i + 1; j < EKF_N; j++)
        {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = avg;
            P[j][i] = avg;
        }
    }
}

// ============================================================
// 3x3 MATRIX INVERSE
// ============================================================

bool inverse3x3(const float A[3][3], float invA[3][3])
{
    float det =
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
      - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
      + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < 1e-12f)
        return false;

    float invDet = 1.0f / det;

    invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    invA[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

    invA[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    invA[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;

    invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    invA[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

    return true;
}

// ============================================================
// ERROR STATE INJECTION
// ============================================================

void injectErrorState(const float dx[EKF_N])
{
    positionE += dx[0];
    positionN += dx[1];
    positionU += dx[2];

    velocityE += dx[3];
    velocityN += dx[4];
    velocityU += dx[5];

    float dq0 = 1.0f;
    float dq1 = 0.5f * dx[6];
    float dq2 = 0.5f * dx[7];
    float dq3 = 0.5f * dx[8];

    float nq0, nq1, nq2, nq3;

    quaternionMultiply(
        q0, q1, q2, q3,
        dq0, dq1, dq2, dq3,
        nq0, nq1, nq2, nq3
    );

    q0 = nq0;
    q1 = nq1;
    q2 = nq2;
    q3 = nq3;

    normalizeQuaternion();

    gyroBiasEstX += dx[9];
    gyroBiasEstY += dx[10];
    gyroBiasEstZ += dx[11];

    accelBiasEstX += dx[12];
    accelBiasEstY += dx[13];
    accelBiasEstZ += dx[14];

    gyroBiasEstX  = constrain(gyroBiasEstX,  -GYRO_BIAS_CLAMP, GYRO_BIAS_CLAMP);
    gyroBiasEstY  = constrain(gyroBiasEstY,  -GYRO_BIAS_CLAMP, GYRO_BIAS_CLAMP);
    gyroBiasEstZ  = constrain(gyroBiasEstZ,  -GYRO_BIAS_CLAMP, GYRO_BIAS_CLAMP);

    accelBiasEstX = constrain(accelBiasEstX, -ACCEL_BIAS_CLAMP, ACCEL_BIAS_CLAMP);
    accelBiasEstY = constrain(accelBiasEstY, -ACCEL_BIAS_CLAMP, ACCEL_BIAS_CLAMP);
    accelBiasEstZ = constrain(accelBiasEstZ, -ACCEL_BIAS_CLAMP, ACCEL_BIAS_CLAMP);
}

// ============================================================
// ZUPT UPDATE
// ============================================================

bool ekfZUPTUpdate()
{
    float innovation[3];
    innovation[0] = -velocityE;
    innovation[1] = -velocityN;
    innovation[2] = -velocityU;

    float S[3][3] = {};

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P[3 + i][3 + j];

    float measurementVariance =
        ZUPT_VELOCITY_STD * ZUPT_VELOCITY_STD;

    for (int i = 0; i < 3; i++)
        S[i][i] += measurementVariance;

    float Sinv[3][3];

    if (!inverse3x3(S, Sinv))
        return false;

    float K[EKF_N][3] = {};

    for (int i = 0; i < EKF_N; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
                K[i][j] += P[i][3 + k] * Sinv[k][j];
        }
    }

    float dx[EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
    {
        for (int j = 0; j < 3; j++)
            dx[i] += K[i][j] * innovation[j];
    }

    float A[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
        A[i][i] = 1.0f;

    for (int i = 0; i < EKF_N; i++)
    {
        A[i][3] -= K[i][0];
        A[i][4] -= K[i][1];
        A[i][5] -= K[i][2];
    }

    float AP[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            for (int k = 0; k < EKF_N; k++)
                AP[i][j] += A[i][k] * P[k][j];

    float APA[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            for (int k = 0; k < EKF_N; k++)
                APA[i][j] += AP[i][k] * A[j][k];

    float newP[EKF_N][EKF_N] = {};

    for (int i = 0; i < EKF_N; i++)
    {
        for (int j = 0; j < EKF_N; j++)
        {
            newP[i][j] = APA[i][j];

            for (int k = 0; k < 3; k++)
                newP[i][j] +=
                    K[i][k] * measurementVariance * K[j][k];
        }
    }

    memcpy(P, newP, sizeof(P));

    for (int i = 0; i < EKF_N; i++)
    {
        for (int j = i + 1; j < EKF_N; j++)
        {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = avg;
            P[j][i] = avg;
        }
    }

    injectErrorState(dx);

    return true;
}

// ============================================================
// COVARIANCE HEALTH CHECK
// ============================================================

bool covarianceIsHealthy()
{
    for (int i = 0; i < EKF_N; i++)
    {
        float value = P[i][i];

        if (!isfinite(value))  return false;
        if (value < 0.0f)      return false;
        if (value > 1.0e8f)    return false;
    }

    return true;
}

void checkCovarianceHealth()
{
    if (!covarianceIsHealthy())
    {
        Serial.println();
        Serial.println("WARNING: EKF COVARIANCE RESET");
        initializeCovariance();
    }
}

// ============================================================
// ZUPT DETECTOR
// ============================================================

bool detectStationary(
    float gx, float gy, float gz,
    float ax, float ay, float az)
{
    float gyroMagnitude = vectorMagnitude(gx, gy, gz);
    float accelMagnitude = vectorMagnitude(ax, ay, az);
    float accelError = fabsf(accelMagnitude - 1.0f);

    bool candidate =
        (gyroMagnitude < ZUPT_GYRO_THRESHOLD_DPS)
        && (accelError < ZUPT_ACCEL_ERROR_G);

    if (!zuptActive)
    {
        if (candidate) stationaryCount++;
        else           stationaryCount = 0;

        if (stationaryCount >= ZUPT_ON_COUNT)
        {
            zuptActive = true;
            stationaryCount = 0;
            movingCount = 0;
        }
    }
    else
    {
        if (!candidate) movingCount++;
        else            movingCount = 0;

        if (movingCount >= ZUPT_OFF_COUNT)
        {
            zuptActive = false;
            movingCount = 0;
            stationaryCount = 0;
        }
    }

    return zuptActive;
}

// ============================================================
// I2C REGISTER WRITE
// ============================================================

bool writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

// ============================================================
// I2C REGISTER READ
// ============================================================

bool readRegisters(
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return false;

    uint8_t received =
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)length);

    if (received != length)
        return false;

    for (uint8_t i = 0; i < length; i++)
        buffer[i] = Wire.read();

    return true;
}

// ============================================================
// READ MPU6500 RAW DATA
// ============================================================

bool readIMU(IMUData &data)
{
    uint8_t buffer[14];

    if (!readRegisters(MPU_ACCEL_XOUT_H, buffer, 14))
        return false;

    int16_t rawAx = ((int16_t)buffer[0] << 8) | buffer[1];
    int16_t rawAy = ((int16_t)buffer[2] << 8) | buffer[3];
    int16_t rawAz = ((int16_t)buffer[4] << 8) | buffer[5];

    int16_t rawGx = ((int16_t)buffer[8]  << 8) | buffer[9];
    int16_t rawGy = ((int16_t)buffer[10] << 8) | buffer[11];
    int16_t rawGz = ((int16_t)buffer[12] << 8) | buffer[13];

    float ax = (float)rawAx / ACCEL_LSB_PER_G;
    float ay = (float)rawAy / ACCEL_LSB_PER_G;
    float az = (float)rawAz / ACCEL_LSB_PER_G;

    ax = (ax - ACCEL_BIAS_X) * ACCEL_SCALE_X;
    ay = (ay - ACCEL_BIAS_Y) * ACCEL_SCALE_Y;
    az = (az - ACCEL_BIAS_Z) * ACCEL_SCALE_Z;

    float gx = (float)rawGx / GYRO_LSB_PER_DPS;
    float gy = (float)rawGy / GYRO_LSB_PER_DPS;
    float gz = (float)rawGz / GYRO_LSB_PER_DPS;

    gx -= GYRO_BIAS_X;
    gy -= GYRO_BIAS_Y;
    gz -= GYRO_BIAS_Z;

    gx -= gyroBiasEstX * R2D;
    gy -= gyroBiasEstY * R2D;
    gz -= gyroBiasEstZ * R2D;

    data.ax = ax;
    data.ay = ay;
    data.az = az;

    data.gx = gx;
    data.gy = gy;
    data.gz = gz;

    return true;
}

// ============================================================
// MPU6500 INITIALIZATION
// ============================================================

bool initializeMPU()
{
    if (!writeRegister(MPU_PWR_MGMT_1, 0x80)) return false;
    delay(100);

    if (!writeRegister(MPU_PWR_MGMT_1, 0x01)) return false;
    delay(10);

    if (!writeRegister(MPU_CONFIG, 0x03))        return false;
    if (!writeRegister(MPU_SMPLRT_DIV, 4))       return false;
    if (!writeRegister(MPU_GYRO_CONFIG, 0x00))   return false;
    if (!writeRegister(MPU_ACCEL_CONFIG, 0x00))  return false;
    if (!writeRegister(MPU_ACCEL_CONFIG2, 0x03)) return false;
    if (!writeRegister(MPU_INT_PIN_CFG, 0x00))   return false;
    if (!writeRegister(MPU_INT_ENABLE, 0x01))    return false;

    return true;
}

// ============================================================
// WHO AM I CHECK
// ============================================================

bool checkWHOAMI()
{
    uint8_t whoAmI = 0;

    if (!readRegisters(MPU_WHO_AM_I, &whoAmI, 1))
        return false;

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    return whoAmI == MPU_WHO_AM_I_VALUE;
}

// ============================================================
// NAVIGATION UPDATE
// ============================================================

void updateNavigation(const IMUData &data, float dt)
{
    updateMahony(
        data.ax, data.ay, data.az,
        data.gx, data.gy, data.gz,
        dt
    );

    float fx = data.ax * G_TO_MS2 - accelBiasEstX;
    float fy = data.ay * G_TO_MS2 - accelBiasEstY;
    float fz = data.az * G_TO_MS2 - accelBiasEstZ;

    Matrix3 R = bodyToENU();

    float accelE = R.m[0][0]*fx + R.m[0][1]*fy + R.m[0][2]*fz;
    float accelN = R.m[1][0]*fx + R.m[1][1]*fy + R.m[1][2]*fz;
    float accelU = R.m[2][0]*fx + R.m[2][1]*fy + R.m[2][2]*fz;

    float linearAccelE = accelE;
    float linearAccelN = accelN;
    float linearAccelU = accelU - G_TO_MS2;

    float oldVelocityE = velocityE;
    float oldVelocityN = velocityN;
    float oldVelocityU = velocityU;

    velocityE += linearAccelE * dt;
    velocityN += linearAccelN * dt;
    velocityU += linearAccelU * dt;

    positionE += 0.5f * (oldVelocityE + velocityE) * dt;
    positionN += 0.5f * (oldVelocityN + velocityN) * dt;
    positionU += 0.5f * (oldVelocityU + velocityU) * dt;

    ekfPredict(
        R,
        fx, fy, fz,
        data.gx * D2R,
        data.gy * D2R,
        data.gz * D2R,
        dt
    );

    bool stationary = detectStationary(
        data.gx, data.gy, data.gz,
        data.ax, data.ay, data.az
    );

    if (stationary)
        ekfZUPTUpdate();

    checkCovarianceHealth();
}

// ============================================================
// SERVO CONTROL UPDATE (Auto mode)
// ============================================================

void updateServoControl(float dt)
{
    if (currentMode == MODE_MANUAL)
        return;

    float rollDeg = getRollDeg();
    float error   = 0.0f - rollDeg;

    pidIntegral += error * dt;
    pidIntegral = constrain(
        pidIntegral,
        -PID_INTEGRAL_CLAMP,
        PID_INTEGRAL_CLAMP
    );

    float derivative = (error - pidPrevError) / dt;
    pidPrevError = error;

    float command =
          PID_KP * error
        + PID_KI * pidIntegral
        + PID_KD * derivative;

    command = constrain(command, -PID_OUTPUT_CLAMP, PID_OUTPUT_CLAMP);

    int servoAngle = SERVO_CENTER_DEG + (int)command;
    servoAngle = constrain(servoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);

    testServo.write(servoAngle);

    static int debugCounter = 0;
    debugCounter++;

    if (debugCounter >= 20)
    {
        debugCounter = 0;

        Serial.print("[AUTO] Roll=");
        Serial.print(rollDeg, 2);
        Serial.print("  err=");
        Serial.print(error, 2);
        Serial.print("  cmd=");
        Serial.print(command, 2);
        Serial.print("  servo=");
        Serial.println(servoAngle);
    }
}

// ============================================================
// SERIAL COMMAND INTERFACE (non-blocking)
// ============================================================

void handleSerialInput()
{
    static String buffer = "";

    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == '\n' || c == '\r')
        {
            buffer.trim();

            if (buffer.length() > 0)
            {
                if (buffer.equalsIgnoreCase("AUTO"))
                {
                    currentMode = MODE_AUTO;
                    pidIntegral  = 0.0f;
                    pidPrevError = 0.0f;
                    Serial.println(">>> MODE: AUTO (attitude hold)");
                }
                else if (buffer.equalsIgnoreCase("MAN"))
                {
                    currentMode = MODE_MANUAL;
                    Serial.println(">>> MODE: MANUAL");
                }
                else if (buffer.equalsIgnoreCase("LEVEL"))
                {
                    pidIntegral  = 0.0f;
                    pidPrevError = 0.0f;
                    Serial.println(">>> PID state reset");
                }
                else
                {
                    int angle = buffer.toInt();

                    if (currentMode == MODE_MANUAL &&
                        angle >= SERVO_MIN_DEG &&
                        angle <= SERVO_MAX_DEG)
                    {
                        testServo.write(angle);
                        Serial.print("SERVO (manual) = ");
                        Serial.print(angle);
                        Serial.println(" deg");
                    }
                    else if (currentMode == MODE_AUTO)
                    {
                        Serial.println("In AUTO mode. Send MAN to switch.");
                    }
                    else
                    {
                        Serial.println("Unknown. Use AUTO, MAN, LEVEL, or 0-180.");
                    }
                }
            }

            buffer = "";
        }
        else
        {
            buffer += c;

            if (buffer.length() > 12)
                buffer = "";
        }
    }
}

// ============================================================
// PRINT NAVIGATION STATE
// ============================================================

void printNavigation(const IMUData &data, float dt)
{
    float accelMagnitude =
        vectorMagnitude(data.ax, data.ay, data.az);

    Matrix3 R = bodyToENU();

    float fx = data.ax * G_TO_MS2 - accelBiasEstX;
    float fy = data.ay * G_TO_MS2 - accelBiasEstY;
    float fz = data.az * G_TO_MS2 - accelBiasEstZ;

    float accelE = R.m[0][0]*fx + R.m[0][1]*fy + R.m[0][2]*fz;
    float accelN = R.m[1][0]*fx + R.m[1][1]*fy + R.m[1][2]*fz;
    float accelU = R.m[2][0]*fx + R.m[2][1]*fy + R.m[2][2]*fz;

    float linearAccelU = accelU - G_TO_MS2;

    Serial.println();
    Serial.println("================ NAVIGATION ================");

    Serial.print("Q = ");
    Serial.print(q0, 6); Serial.print(", ");
    Serial.print(q1, 6); Serial.print(", ");
    Serial.print(q2, 6); Serial.print(", ");
    Serial.println(q3, 6);

    Serial.print("Roll / Pitch (deg) = ");
    Serial.print(getRollDeg(), 3);
    Serial.print(" / ");
    Serial.println(getPitchDeg(), 3);

    Serial.print("ACC MAG = ");
    Serial.print(accelMagnitude, 6);
    Serial.println(" g");

    Serial.print("Linear Accel ENU = ");
    Serial.print(accelE, 4); Serial.print(", ");
    Serial.print(accelN, 4); Serial.print(", ");
    Serial.print(linearAccelU, 4);
    Serial.println(" m/s2");

    Serial.print("Velocity ENU = ");
    Serial.print(velocityE, 4); Serial.print(", ");
    Serial.print(velocityN, 4); Serial.print(", ");
    Serial.print(velocityU, 4);
    Serial.println(" m/s");

    Serial.print("Position ENU = ");
    Serial.print(positionE, 4); Serial.print(", ");
    Serial.print(positionN, 4); Serial.print(", ");
    Serial.print(positionU, 4);
    Serial.println(" m");

    Serial.print("EKF Pos Sigma = ");
    Serial.print(sqrtf(fmaxf(P[0][0], 0.0f)), 4); Serial.print(", ");
    Serial.print(sqrtf(fmaxf(P[1][1], 0.0f)), 4); Serial.print(", ");
    Serial.println(sqrtf(fmaxf(P[2][2], 0.0f)), 4);

    Serial.print("EKF Vel Sigma = ");
    Serial.print(sqrtf(fmaxf(P[3][3], 0.0f)), 4); Serial.print(", ");
    Serial.print(sqrtf(fmaxf(P[4][4], 0.0f)), 4); Serial.print(", ");
    Serial.println(sqrtf(fmaxf(P[5][5], 0.0f)), 4);

    Serial.print("Gyro Bias Est (deg/s) = ");
    Serial.print(gyroBiasEstX * R2D, 6); Serial.print(", ");
    Serial.print(gyroBiasEstY * R2D, 6); Serial.print(", ");
    Serial.println(gyroBiasEstZ * R2D, 6);

    Serial.print("Accel Bias Est (m/s2) = ");
    Serial.print(accelBiasEstX, 6); Serial.print(", ");
    Serial.print(accelBiasEstY, 6); Serial.print(", ");
    Serial.println(accelBiasEstZ, 6);

    Serial.print("ZUPT = ");
    Serial.println(zuptActive ? "ON" : "OFF");

    Serial.print("Control Mode = ");
    Serial.println(currentMode == MODE_AUTO ? "AUTO" : "MANUAL");

    Serial.print("dt = ");
    Serial.print(dt * 1000.0f, 3);
    Serial.println(" ms");

    Serial.println("============================================");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(921600);
    delay(1000);

    Serial.println();
    Serial.println("============================================");
    Serial.println("ESP32-S3 + MPU6500 NAVIGATION + SERVO");
    Serial.println("6-AXIS IMU / MAHONY / 15-STATE ES-EKF");
    Serial.println("============================================");

    Wire.begin(SDA_PIN, SCL_PIN, 400000);

    if (!initializeMPU())
    {
        Serial.println("ERROR: MPU6500 initialization failed.");
        while (true) delay(1000);
    }

    if (!checkWHOAMI())
    {
        Serial.println("ERROR: WHO_AM_I mismatch.");
        while (true) delay(1000);
    }

    Serial.println("MPU6500 detected.");

    initializeCovariance();

    pinMode(INT_PIN, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(INT_PIN),
        imuDataReadyISR,
        RISING
    );

    Serial.println("DATA READY interrupt enabled.");
    Serial.println("Target rate: 200 Hz");
    Serial.println("Expected dt: 5.000 ms");
    Serial.println();

    // --------------------------------------------------------
    // SERVO INITIALIZATION
    // --------------------------------------------------------

    testServo.setPeriodHertz(50);
    testServo.attach(SERVO_PIN, 500, 2400);
    testServo.write(SERVO_START_DEG);

    Serial.println("Servo initialized at 90 degrees.");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  AUTO   -> attitude hold (PID controls servo)");
    Serial.println("  MAN    -> manual mode (numbers control servo)");
    Serial.println("  LEVEL  -> reset PID state");
    Serial.println("  0..180 -> servo angle (MANUAL mode only)");
    Serial.println();
    Serial.println("Keep the IMU stationary for startup.");
    Serial.println();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // ALWAYS process serial first
    // --------------------------------------------------------

    handleSerialInput();

    // --------------------------------------------------------
    // Atomically consume DATA READY flag
    // --------------------------------------------------------

    noInterrupts();
    bool dataReady = imuDataReady;
    imuDataReady = false;
    interrupts();

    if (!dataReady)
        return;

    // --------------------------------------------------------
    // Timing
    // --------------------------------------------------------

    static uint32_t previousMicros = 0;

    uint32_t currentMicros = micros();

    float dt;

    if (previousMicros == 0)
    {
        dt = TARGET_DT;
    }
    else
    {
        dt = (currentMicros - previousMicros) * 1.0e-6f;
    }

    previousMicros = currentMicros;

    if (dt <= 0.0f || dt > 0.02f)
        dt = TARGET_DT;

    // --------------------------------------------------------
    // Read IMU
    // --------------------------------------------------------

    if (!readIMU(imu))
    {
        Serial.println("ERROR: IMU read failed.");
        return;
    }

    // --------------------------------------------------------
    // Navigation + EKF
    // --------------------------------------------------------

    updateNavigation(imu, dt);

    // --------------------------------------------------------
    // Servo control (auto mode only)
    // --------------------------------------------------------

    updateServoControl(dt);

    // --------------------------------------------------------
    // Print navigation at ~10 Hz
    // --------------------------------------------------------

    static int printCounter = 0;
    printCounter++;

    if (printCounter >= 20)
    {
        printCounter = 0;
        printNavigation(imu, dt);
    }
}