#pragma once

#include <cstddef>
#include <cmath>

// NOTE(sasha): Everything in here should be marked constexpr.
namespace Constants
{
    constexpr size_t PACKET_PAYLOAD_SIZE = 8;
    constexpr double SHOULDER_LENGTH = 0.6; //placeholder(m)
    constexpr double ELBOW_LENGTH = 0.7; //placeholder(m)

    // Joint limits
    constexpr double ARM_BASE_MIN = -M_PI/2;
    constexpr double ARM_BASE_MAX = M_PI/2;
    constexpr double SHOULDER_MIN = 0.0;
    constexpr double SHOULDER_MAX = M_PI * 5./6.; // Can't quite lie flat
    constexpr double ELBOW_MIN = 0.0;
    constexpr double ELBOW_MAX = M_PI;
}
