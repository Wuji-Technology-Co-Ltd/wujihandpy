#pragma once

#include <cmath>

#include <numbers>

class LowPassFilter {
public:
    static constexpr double calculate_alpha(double cutoff_freq, double sampling_freq) {
        double dt = 1.0 / sampling_freq;
        double rc = 1.0 / (2 * std::numbers::pi * cutoff_freq);
        return dt / (dt + rc);
    }

    explicit LowPassFilter() = default;

    void reset() { value_ = nan_; }

    const double& update(const double& input, const double& alpha) {
        double output = alpha * input + (1.0 - alpha) * value_;
        output = std::isnan(output) ? input : output;
        value_ = output;
        return value_;
    }

    const double& value() const { return value_; }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double value_ = nan_;
};
