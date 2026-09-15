#pragma once
#include <cmath>
#include <iostream>
#include <string>

inline int failures = 0;
inline int checks = 0;

inline void check(bool condition, const std::string& name) {
    ++checks;
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << name << '\n';
    }
}

inline bool near(double actual, double expected, double tolerance = 0.000001) {
    return std::abs(actual - expected) <= tolerance;
}

inline int finish_tests() {
    std::cout << checks << " checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}
