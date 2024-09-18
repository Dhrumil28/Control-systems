#include <stdio.h>
#include <math.h>
#include <unistd.h>  // for usleep

// Complementary filter parameters
#define ALPHA 0.98  // Complementary filter coefficient (between 0 and 1)
#define DT 0.01     // Time step (10 ms or 100 Hz sensor)

// Function to calculate the pitch angle from the accelerometer data
double get_accelerometer_angle(double acc_x, double acc_y, double acc_z) {
    return atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * (180.0 / M_PI);
}

// Function to update the angle using gyroscope data
double get_gyroscope_angle(double gyro_rate, double prev_angle, double dt) {
    return prev_angle + gyro_rate * dt;
}

// Function to fuse accelerometer and gyroscope angles using a complementary filter
double complementary_filter(double acc_angle, double gyro_angle, double alpha) {
    return alpha * gyro_angle + (1 - alpha) * acc_angle;
}

int main() {
    // Initialize angles
    double angle_acc = 0.0;    // Angle from accelerometer
    double angle_gyro = 0.0;   // Angle from gyroscope
    double angle_fused = 0.0;  // Fused angle (final output)

    // Simulated sensor readings (in real cases, you'd get these from actual sensors)
    // Example accelerometer data over time: (acc_x, acc_y, acc_z)
    double accelerometer_data[4][3] = {
        {0.0, 0.707, 0.707},
        {0.1, 0.703, 0.707},
        {0.2, 0.690, 0.707},
        {0.3, 0.680, 0.707}
    };

    // Example gyroscope data: angular velocity in degrees per second
    double gyroscope_data[4] = {0.1, 0.15, 0.05, 0.0};

    // Loop through sensor data
    for (int i = 0; i < 4; i++) {
        double acc_x = accelerometer_data[i][0];
        double acc_y = accelerometer_data[i][1];
        double acc_z = accelerometer_data[i][2];
        double gyro_rate = gyroscope_data[i];

        // Get angles from individual sensors
        angle_acc = get_accelerometer_angle(acc_x, acc_y, acc_z);
        angle_gyro = get_gyroscope_angle(gyro_rate, angle_gyro, DT);

        // Fuse the data
        angle_fused = complementary_filter(angle_acc, angle_gyro, ALPHA);

        // Output the result
        printf("Step %d: Acc Angle = %.2f, Gyro Angle = %.2f, Fused Angle = %.2f\n", i + 1, angle_acc, angle_gyro, angle_fused);

        // Simulate time passing (10ms delay)
        usleep(DT * 1000000);  // usleep takes microseconds
    }

    return 0;
}
