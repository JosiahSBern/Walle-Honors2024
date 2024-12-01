#ifndef POINT_H
#define POINT_H

#include <cmath>

struct Point {
    double x;
    double y;

    // Operator Overload -
    Point operator-(const Point& other) const {
        return {x - other.x, y - other.y};
    }
};

#endif