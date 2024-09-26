#include <iostream>
#include <vector>
#include <random>
#include <cmath>

class KalmanFilter {
private:
    double x; // State estimate
    double P; // Estimate error covariance
    double Q; // Process noise covariance
    double R; // Measurement noise covariance
    double K; // Kalman gain

public:
    KalmanFilter(double initial_x, double initial_P, double Q, double R)
        : x(initial_x), P(initial_P), Q(Q), R(R) {}

    void predict() {
        // State prediction (assuming constant state)
        // x = x;
        
        // Error covariance prediction
        P = P + Q;
    }

    void update(double measurement) {
        // Compute Kalman gain
        K = P / (P + R);

        // Update estimate
        x = x + K * (measurement - x);

        // Update error covariance
        P = (1 - K) * P;
    }

    double getStateEstimate() const {
        return x;
    }
};

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, 1);

    // Initialize Kalman filter
    double initial_x = 0.0;
    double initial_P = 1.0;
    double Q = 0.1;
    double R = 1.0;
    KalmanFilter kf(initial_x, initial_P, Q, R);

    // True state
    double true_value = 5.0;

    // Simulate measurements and filtering
    int n_iterations = 10000;
    std::vector<double> measurements;
    std::vector<double> estimates;

    for (int i = 0; i < n_iterations; ++i) {
        // Generate noisy measurement
        double measurement = true_value + d(gen);
        measurements.push_back(measurement);

        // Kalman filter steps
        kf.predict();
        kf.update(measurement);

        // Store estimate
        estimates.push_back(kf.getStateEstimate());
    }

    // Print results
    std::cout << "True value: " << true_value << std::endl;
    std::cout << "Final estimate: " << estimates.back() << std::endl;
    std::cout << "Error: " << std::abs(true_value - estimates.back()) << std::endl;

    return 0;
}