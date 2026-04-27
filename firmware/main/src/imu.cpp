#include "imu.h"
#include "compat/i2c_bus.h"
#include <math.h>

// ─── MPU-6050 Register Map ───────────────────────────────────────────────────
#define MPU_ADDR         0x68  // I2C address — AD0 pin tied LOW on GY-521 module
#define REG_PWR_MGMT_1   0x6B  // Power management: sleep bit + clock source
#define REG_SMPLRT_DIV   0x19  // Sample rate divider
#define REG_CONFIG       0x1A  // DLPF configuration
#define REG_GYRO_CONFIG  0x1B  // Gyro full-scale range
#define REG_ACCEL_CONFIG 0x1C  // Accel full-scale range
#define REG_ACCEL_XOUT_H 0x3B  // Burst read start: accel[6] + temp[2] + gyro[6]

// ─── Sensor Scaling ──────────────────────────────────────────────────────────
// Must match the hardware range configured in initIMU() below.
// ±8g    → LSB/g    = 4096   (MPU-6050 datasheet Table 1)
// ±500°/s → LSB/dps = 65.5   (MPU-6050 datasheet Table 2)
#define ACCEL_SCALE  4096.0f
#define GYRO_SCALE   65.5f

// ─── Safety Thresholds ───────────────────────────────────────────────────────
// dt guard: if the loop stalls longer than this, the frame is invalid.
// 50ms = 20Hz minimum — far below our 500Hz target but catches real stalls.
#define MAX_SAFE_DT  0.05f

// ─── I2C Pins (ESP32-S3 Zero) ────────────────────────────────────────────────
#define I2C_SDA_PIN  3
#define I2C_SCL_PIN  4
#define I2C_FREQ_HZ  400000  // Fast Mode — MPU-6050 supports up to 400kHz

// ─── Module-Private Calibration State ────────────────────────────────────────
// Computed once in calibrateIMU(), applied every readIMU() call.
static float gyroXoffset  = 0.0f;
static float gyroYoffset  = 0.0f;
static float gyroZoffset  = 0.0f;
static float accelXoffset = 0.0f;
static float accelYoffset = 0.0f;
static float accelZoffset = 0.0f;

// Timestamp of the last successful readIMU() call — used for dt validation only
static unsigned long lastTimeUs = 0;

// ─── initIMU ─────────────────────────────────────────────────────────────────
// Initializes the I2C bus via i2c_bus::init(), configures all MPU-6050
// registers, then runs calibration. Call once in main setup().
void initIMU() {
    // Initialize I2C through the central bus layer — NOT Wire directly.
    // All I2C in this firmware goes through i2c_bus so errors and retries
    // are handled in one place.
    i2c_bus::init(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    delay(150); // MPU-6050 needs up to 100ms after power-on before accepting I2C

    // ── Debug: confirm MPU-6050 is visible on the bus ────────────────────────
    // Remove this scan call in production to save ~50ms of boot time
    i2c_bus::scan();

    // ── Wake MPU-6050 + select stable clock source ────────────────────────────
    // Default state after power-on: sleep mode, internal 8MHz RC oscillator
    // 0x01 = wake up + use PLL locked to X-axis gyro oscillator (more stable)
    if (!i2c_bus::writeByte(MPU_ADDR, REG_PWR_MGMT_1, 0x01)) {
        Serial.println("[IMU] FATAL: Cannot reach MPU-6050 — check wiring");
        while (true) delay(100); // Halt — cannot fly without IMU
    }
    delay(10); // Allow PLL to lock

    // ── DLPF: Digital Low Pass Filter ────────────────────────────────────────
    // Level 3 → Accel: 44Hz BW, Gyro: 42Hz BW, delay ~4.9ms
    // Filters motor vibration noise (typically 80–200Hz on brushed quads)
    // without introducing lag that would destabilize the 500Hz PID loop
    i2c_bus::writeByte(MPU_ADDR, REG_CONFIG, 0x03);

    // ── Sample Rate Divider ───────────────────────────────────────────────────
    // Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
    // With DLPF enabled, Gyro Output Rate = 1kHz
    // SMPLRT_DIV = 1 → 1000 / 2 = 500Hz sample rate
    i2c_bus::writeByte(MPU_ADDR, REG_SMPLRT_DIV, 0x01);

    // ── Accelerometer Range ───────────────────────────────────────────────────
    // 0x10 = ±8g → 4096 LSB/g
    // Chosen for: enough range to survive motor start impulse,
    // enough resolution to detect small tilt angles accurately
    i2c_bus::writeByte(MPU_ADDR, REG_ACCEL_CONFIG, 0x10);

    // ── Gyroscope Range ───────────────────────────────────────────────────────
    // 0x08 = ±500°/s → 65.5 LSB/dps
    // 8520 brushed motors can spin the frame fast but angular rates
    // during a stable hover stay well within ±100°/s — ±500 gives headroom
    i2c_bus::writeByte(MPU_ADDR, REG_GYRO_CONFIG, 0x08);

    Serial.println("[IMU] MPU-6050 configured.");

    calibrateIMU();

    lastTimeUs = micros(); // Seed the dt timer after calibration completes
}

// ─── calibrateIMU ────────────────────────────────────────────────────────────
// Averages 500 samples (~1 second) to measure sensor bias offsets.
// DRONE MUST BE FLAT AND MOTIONLESS during this entire function.
void calibrateIMU() {
    Serial.println("[IMU] Calibrating — keep drone flat and still...");

    const int SAMPLES = 500;
    long sumAX = 0, sumAY = 0, sumAZ = 0;
    long sumGX = 0, sumGY = 0, sumGZ = 0;

    uint8_t buf[14];

    for (int i = 0; i < SAMPLES; i++) {
        // Use i2c_bus for consistent error handling — retry logic is inside
        if (!i2c_bus::readBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) {
            Serial.printf("[IMU] Calibration read failed at sample %d\n", i);
            i--; // Retry this sample
            delay(5);
            continue;
        }

        // Parse big-endian signed 16-bit values from the 14-byte burst
        sumAX += (int16_t)(buf[0]  << 8 | buf[1]);
        sumAY += (int16_t)(buf[2]  << 8 | buf[3]);
        sumAZ += (int16_t)(buf[4]  << 8 | buf[5]);
        // buf[6], buf[7] = temperature — not needed for calibration
        sumGX += (int16_t)(buf[8]  << 8 | buf[9]);
        sumGY += (int16_t)(buf[10] << 8 | buf[11]);
        sumGZ += (int16_t)(buf[12] << 8 | buf[13]);

        delay(2); // 2ms × 500 = ~1 second total — matches 500Hz sample rate
    }

    // ── Gyro offsets ─────────────────────────────────────────────────────────
    // When perfectly still, gyro should output 0 — the average IS the bias
    gyroXoffset = (float)sumGX / SAMPLES;
    gyroYoffset = (float)sumGY / SAMPLES;
    gyroZoffset = (float)sumGZ / SAMPLES;

    // ── Accel offsets ─────────────────────────────────────────────────────────
    // When flat: X and Y should be 0g, Z should be +1g (= +ACCEL_SCALE counts)
    // Subtract expected value to get the offset
    accelXoffset = (float)sumAX / SAMPLES;           // Expected 0 → offset = average
    accelYoffset = (float)sumAY / SAMPLES;           // Expected 0 → offset = average
    accelZoffset = ((float)sumAZ / SAMPLES) - ACCEL_SCALE; // Expected 4096 → subtract 1g

    Serial.println("[IMU] Calibration complete.");
    Serial.printf("[IMU] Gyro offsets  → X:%.2f  Y:%.2f  Z:%.2f (LSB)\n",
                  gyroXoffset, gyroYoffset, gyroZoffset);
    Serial.printf("[IMU] Accel offsets → X:%.2f  Y:%.2f  Z:%.2f (LSB)\n",
                  accelXoffset, accelYoffset, accelZoffset);
}

// ─── readIMU ─────────────────────────────────────────────────────────────────
// Burst-reads 14 bytes from MPU-6050 via i2c_bus, applies calibration offsets,
// scales to physical units, validates dt, and returns an IMUData struct.
//
// Returns isHealthy = false if:
//   - I2C read fails (after retry)
//   - dt is outside the valid range (stall or first-frame artifact)
//   - Accel magnitude is implausible (sensor fault or free-fall)
IMUData readIMU() {
    // Default all fields to 0 and mark unhealthy — must be proven good below
    IMUData data = {};
    data.isHealthy = false;

    // ── Burst Read via i2c_bus ────────────────────────────────────────────────
    // 14 bytes: AX_H AX_L  AY_H AY_L  AZ_H AZ_L  TMP_H TMP_L
    //           GX_H GX_L  GY_H GY_L  GZ_H GZ_L
    uint8_t buf[14];
    if (!i2c_bus::readBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) {
        // i2c_bus already retried once — this is a real failure
        Serial.println("[IMU] ERROR: I2C read failed — flagging unhealthy");
        return data; // isHealthy = false
    }

    // ── Parse Raw Signed 16-bit Values ────────────────────────────────────────
    // Big-endian: high byte first, low byte second
    // (int16_t) cast is critical — without it negative values corrupt silently
    int16_t rAX = (int16_t)(buf[0]  << 8 | buf[1]);
    int16_t rAY = (int16_t)(buf[2]  << 8 | buf[3]);
    int16_t rAZ = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6], buf[7] = temperature register — skipped, not needed
    int16_t rGX = (int16_t)(buf[8]  << 8 | buf[9]);
    int16_t rGY = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t rGZ = (int16_t)(buf[12] << 8 | buf[13]);

    // ── dt Validation ─────────────────────────────────────────────────────────
    // dt is used only to validate timing here — actual integration dt is
    // computed fresh in Stabilizer::update() from its own micros() call
    unsigned long nowUs = micros();
    float dt = (nowUs - lastTimeUs) / 1000000.0f;
    lastTimeUs = nowUs;

    if (dt <= 0.0f || dt > MAX_SAFE_DT) {
        // First frame after calibration or CPU stall — skip integration this cycle
        Serial.printf("[IMU] WARNING: Bad dt=%.4fs — skipping frame\n", dt);
        return data; // isHealthy = false
    }

    // ── Apply Calibration Offsets + Scale to Physical Units ──────────────────
    // Subtract the bias measured during calibration, then divide by scale factor
    data.ax = (rAX - accelXoffset) / ACCEL_SCALE; // G
    data.ay = (rAY - accelYoffset) / ACCEL_SCALE; // G
    data.az = (rAZ - accelZoffset) / ACCEL_SCALE; // G
    data.gx = (rGX - gyroXoffset)  / GYRO_SCALE;  // deg/s
    data.gy = (rGY - gyroYoffset)  / GYRO_SCALE;  // deg/s
    data.gz = (rGZ - gyroZoffset)  / GYRO_SCALE;  // deg/s

    // ── Accel Magnitude Sanity Check ─────────────────────────────────────────
    // When the drone is in normal flight, total accel magnitude should be
    // close to 1G (gravity). Values far outside this range indicate:
    //   < 0.3G → free-fall or sensor disconnected
    //   > 3.0G → severe impact or sensor fault
    // In both cases, the data is untrustworthy — flag unhealthy.
    float mag = sqrtf(data.ax*data.ax + data.ay*data.ay + data.az*data.az);
    if (mag < 0.3f || mag > 3.0f) {
        Serial.printf("[IMU] WARNING: Accel magnitude implausible: %.2fG\n", mag);
        return data; // isHealthy = false
    }

    // ── All checks passed ─────────────────────────────────────────────────────
    data.isHealthy = true;
    return data;
}