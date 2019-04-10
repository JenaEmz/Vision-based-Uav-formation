#pragma once

#include "math.hpp"

namespace matrix
{

template<typename Type>
bool is_finite(Type x) {
    return std::isfinite(x);
}

template<typename Type>
Type wrap_pi(Type x)
{
    if (!is_finite(x)) {
        return x;
    }

    int c = 0;

    while (x >= Type(M_PI)) {
        x -= Type(2 * M_PI);

        if (c++ > 100) {
            return INFINITY;
        }
    }

    c = 0;

    while (x < Type(-M_PI)) {
        x += Type(2 * M_PI);

        if (c++ > 100) {
            return INFINITY;
        }
    }

    return x;
}

template<typename Type>
Type wrap_2pi(Type x)
{
    if (!is_finite(x)) {
        return x;
    }

    int c = 0;

    while (x >= Type(2 * M_PI)) {
        x -= Type(2 * M_PI);

        if (c++ > 100) {
            return INFINITY;
        }
    }

    c = 0;

    while (x < Type(0)) {
        x += Type(2 * M_PI);

        if (c++ > 100) {
            return INFINITY;
        }
    }

    return x;
}

}
