//
// Created by robocup on 11.11.18.
//

#ifndef PARTICLEFILTER_PARTICLEFILTER_H
#define PARTICLEFILTER_PARTICLEFILTER_H

#include "vector"
#include <cmath>
#include <json/json.h>
#include <iostream>
#include <string>
#include <chrono>
#include <fstream>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/taus88.hpp>
#include <boost/random/discrete_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <assert.h>

struct odometry {
    double old_x, new_x;
    double old_y, new_y;
    double old_angle, new_angle;
    odometry(double a1, double a2, double b1, double b2, double c1, double c2) {
        old_x = a1;
        new_x = a2;
        old_y = b1;
        new_y = b2;
        old_angle = c1;
        new_angle = c2;
    };
};


bool zero_compare(double a);

struct dot {
    double x;
    double y;

    dot(double value1, double value2) :
            x(value1),
            y(value2)
    {}

    dot() :
            x(-1),
            y(-1)
    {}

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

    dot rotate(double rotate_angle) {
        return {
            x * cos(rotate_angle) - y * sin(rotate_angle),
            x * sin(rotate_angle) + y * cos(rotate_angle)
        };
    }
};

// ax + by + c = 0 - уравнение прямой
// при задании уравнения из двух точек (т.е. фактически системы двух уравнений)
// получаем одну степень свободы, что как раз выражается в (a = -1, b = -1 в двух случаях соотвественно, для удобства)
struct line {
    double a, b, c;

    line(double val1, double val2, double val3) :
        a(val1), b(val2), c(val3)
    {};

    line(const dot &a1, const dot &b2) {
        if(zero_compare(a1.x - b2.x)) {
            a = -1;
            b = 0;
            c = a1.x;
        }
        else {
            double t = (b2.y - a1.y) / (a1.x - b2.x);
            b = -1;
            a = b * t;
            c = t * a1.x + a1.y;
        }
    }
};

struct state {
    dot position;
    // угол оси OY робота с глобальной осью OY, робот "смотрит" без поворота головы вдоль своей OY
    double angle;

    state(dot pos, double k) : position(pos), angle(k)
    {}

    state() : state(dot(0.0, 0.0), 0.0)
    {}
};

class ParticleFilter {
public:
    explicit ParticleFilter(const char*, state initial_robot_state, int field_half);
    void PassNewOdometry(const odometry &od);
    void PassNewVision(const std::vector<line> &vision_lines, const odometry &control_data);
    void MistakesToProbability(std::vector<double> &mistakes);
    double ScoreLines(const dot &pos, const line &x, const line &y) const;
    double ScoreMultyLines(const dot &pos, const std::vector<line> &lines) const;
    std::vector<line> TranslateVisionLines(const state &particle, const state &system, const std::vector<line> &lines) const;
    void WriteState(const char *filename) const;
public:
    boost::taus88 generator;
    state robot;
    state global_system;
    std::vector<line> baselines;
    double limit_height, limit_width;
private:
    void SetToNewSystem(const state &particle, const state &system, dot &object) const;
    void LowVarianceResample(size_t particles_count);
    double GaussFunction(double x, double m, double s);
private:
    std::vector<state> particles;

    std::vector<double> weights;
    std::vector<double> odometry_noise;

    size_t particles_amount;
    double gauss_mistake;

    double score_angle, score_distance;
};


#endif //PARTICLEFILTER_PARTICLEFILTER_H
