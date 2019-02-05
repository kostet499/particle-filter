//
// Created by robocup on 12.11.18.
//

#ifndef ALGOS_JSONFIELD_H
#define ALGOS_JSONFIELD_H

#include "vector"
#include <cmath>
#include <json/json.h>
#include <iostream>
#include <string>

struct dot {
    double x;
    double y;

    dot(double, double);
    dot();
    double norm() const {
        return std::sqrt(pow(x, 2) + pow(y, 2));
    }
    dot operator+(const dot& a) const {
        return {this->x + a.x, this->y + a.y};
    }
    dot operator-(const dot& a) const {
        return {this->x - a.x, this->y - a.y};
    }
    dot operator*(double number) const {
        return {this->x * number, this->y * number};
    }
    double operator()(const dot& a) const {
        return this->x * a.x + this->y * a.y;
    }
};

using line = std::pair<dot, dot>;

class JsonField {
public:
    explicit JsonField(const char *filename);
    const std::pair<dot, dot> &operator[](size_t);
    double width() const;
    double height() const;
private:
    double field_width;
    double field_height;
    std::vector<line> field_lines;
};


#endif //ALGOS_JSONFIELD_H
