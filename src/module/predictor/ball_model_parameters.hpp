#pragma once

namespace pingpong_tracker::predictor {

struct BallModelParameters {
    double gravity          = 9.81;
    double drag_coefficient = 0.47;
    double mass             = 0.0027;
    double radius           = 0.02;
    double air_density      = 1.225;
    double lift_coefficient = 0.15;
};

}  // namespace pingpong_tracker::predictor
