//
// Created by robocup on 11.11.18.
//

#include "ParticleFilter.h"


bool zero_compare(double a, double b) {
    return fabs(a - b) < 1e-6;
}

// config_filename предполагается, что в нём есть данные отклонения одометрии и линии
ParticleFilter::ParticleFilter(const char* config_filename, state initial_robot_state, size_t amount, int field_half)
: robot(initial_robot_state), particles_amount(amount), generator(42) {
    // работа с конфигом
    std::freopen(config_filename, "r", stdin);
    Json::Value root;
    std::cin >> root;
    for(auto &val : root["odometry"]) {
        odometry_noise.emplace_back(val.asDouble());
    }

    for(auto &val : root["field"]) {
        baselines.emplace_back(line(val["x1"].asDouble(), val["y1"].asDouble(), val["x2"].asDouble(), val["y2"].asDouble()));
        limit_width = std::max(limit_width, std::max(val["x1"].asDouble(), val["x2"].asDouble()));
        limit_height = std::max(limit_height, std::max(val["x1"].asDouble(), val["x2"].asDouble()));
    }

    score_angle = root["score_angle"].asDouble();
    score_distance = root["score_distance"].asDouble();

    // раскидываем частички по полю (половине поля)
    weights.resize(particles_amount, 1.0 / particles_amount);
    particles.resize(particles_amount);

    boost::random::uniform_real_distribution<double> x_coord(0.0, limit_width);
    boost::random::uniform_real_distribution<double> y_coord(field_half * limit_height / 2, (field_half + 1.0) * limit_height / 2);
    boost::random::uniform_real_distribution<double> angle(0.0, 2 * M_PI);

    for(auto &particle : particles) {
        particle.position.x = x_coord(generator);
        particle.position.y = y_coord(generator);
        particle.angle = angle(generator);
    }
}

// vision lines - в системе координат робота
void ParticleFilter::PassNewVision(const std::vector<line> &vision_lines, const odometry &control_data) {

    PassNewOdometry(control_data);
    for(size_t i = 0; i < particles.size(); ++i) {
        weights[i] = ScoreMultyLines(TranslateVisionLines(vision_lines, particles[i]));
    }

    MistakesToProbability(weights);

    // получаем новую позицию - просто берем среднее по координатам
    robot.position = dot(0.0, 0.0);
    for(size_t i = 0; i < particles.size(); ++i) {
        robot.position = robot.position + particles[i].position * weights[i];
        robot.angle += particles[i].angle * weights[i];
    }

    LowVarianceResample(particles_amount);
}

std::vector<line> ParticleFilter::TranslateVisionLines(const std::vector<line> &lines, const state &particle) const {
    std::vector<line> result;

    for(const auto &liny : lines) {
        dot a, b;
        if(zero_compare(liny.b, 0.0)) {
            a.x = -liny.c / liny.a;
            b.x = a.x;
            a.y = 1.0;
            b.y = 2.0;
        }
        else {
            a.x = 0.0;
            a.y = -liny.c / liny.b;
            b.x = 1.0;
            b.y = -(liny.a + liny.c) / liny.b;
        }
        SetToNewSystem(particle, a);
        SetToNewSystem(particle, b);
        result.emplace_back(a.x, a.y, b.x, b.y);
    }

    return result;
}

// пытаемся совместить типы линии, а потом оцениваем ошибку
// разность по 'c' - ошибка расстояния
// ошибка по углу - векторное произведение нормированных направляющих векторов

double ParticleFilter::ScoreLines(const line &x, const line &y) const {
    double score = 0.0;
    dot x_vec(-x.b, x.a), y_vec(-y.b, y.a);
    x_vec = dot(x_vec.x / x_vec.norm(), x_vec.y / x_vec.norm());
    y_vec = dot(y_vec.x / y_vec.norm(), y_vec.y / y_vec.norm());
    double square = fabs(x_vec.x * y_vec.y - x_vec.y * y_vec.x);

    score += square * score_angle;
    score += fabs(y.c - x.c) * score_distance;

    return score;
}

double ParticleFilter::ScoreMultyLines(const std::vector<line> &lines) const {
    double score = 0.0;
    for(const auto &liny : lines) {
        double min_score = 1000000000;
        for(const auto &base : baselines) {
            double cur_score = ScoreLines(liny, base);
            min_score = std::min(min_score, cur_score);
        }
        score += min_score;
    }
    return score;
}

void ParticleFilter::LowVarianceResample(size_t particles_count) {
    std::vector<state> new_particles;

    double weight = weights[0];
    boost::random::uniform_real_distribution<double> one_number_generator(0.0, 1.0 / particles_count);
    double number = one_number_generator(generator);
    size_t i = 0;

    for(size_t m = 0; m < particles_count; ++m) {
        double uber = number + static_cast<double>(m) / particles_count;
        while(uber > weight) {
            ++i;
            weight += weights[i];
        }
        new_particles.emplace_back(particles[i]);
    }

    particles = new_particles;
}

void ParticleFilter::MistakesToProbability(std::vector<double> &mistakes) {
    // мой первый вариант
    // введем четыре условия
    // p - функция вероятности от ошибки
    // p - линейная функция от ошибки
    // p(average(mistakes)) = 1 / particles_amount
    // p(max(mistakes)) = 0
    // sum p(mistake) by all mistakes = 1
    // фактически только что я задал прямую по двум точкам, чтобы сумма вероятностей была 1 ))

    double average = 0.0;
    double maximum = 0.0;
    for(double &mistake : mistakes) {
        average += mistake;
        maximum = std::max(maximum, mistake);
    }
    average /= mistakes.size(); // mistakes.size() совпадает с числом частиц
    double coef = (0.0 - 1.0 / mistakes.size()) / (maximum - average);
    double intercept = -maximum * coef;
    for(double &mistake : mistakes) {
        mistake = mistake * coef + intercept;
    }
}

// координаты точки приводятся к системе координат поля
// угол обзора поля от обычно горизонтального положения = pi / 2
void ParticleFilter::SetToNewSystem(const state &particle, dot &object) {
    // поворот - тут матрицу нужно проверить
    double rotate_angle = M_PI / 2  - particle.angle;
    dot rotated(
            object.x * cos(rotate_angle) + object.y * sin(rotate_angle),
            object.x * (-sin(rotate_angle)) + object.y * cos(rotate_angle)
    );
    // параллельный перенос
    object = rotated + particle.position;
}

// углы [-pi, pi] от оси абсцисс, система координат с началом в левой нижней точке поля
void ParticleFilter::PassNewOdometry(const odometry &od) {
    std::vector<state> new_particles(particles.size());

    double rot1 = atan2(od.new_y - od.old_y, od.new_x - od.old_x) - od.old_angle;
    double shift = dot(od.new_x - od.old_x, od.new_y - od.old_y).norm();
    double rot2 = od.new_angle - od.old_angle - rot1;

    double sigma_rot1 = std::sqrt(odometry_noise[0] * pow(rot1, 2) + odometry_noise[1] * pow(shift, 2));
    double sigma_shift = std::sqrt(odometry_noise[2] * pow(shift, 2) + odometry_noise[3] * (pow(rot1, 2) + pow(rot2, 2)));
    double sigma_rot2 = std::sqrt(odometry_noise[0] * pow(rot2, 2) + odometry_noise[1] * pow(shift, 2));

    // гауссовский шум
    boost::normal_distribution<double> shift_noise(0.0, sigma_shift);
    boost::normal_distribution<double> rot1_noise(0.0, sigma_rot1);
    boost::normal_distribution<double> rot2_noise(0.0, sigma_rot2);

    for(auto &particle : particles) {
        double rot1_n = rot1 - rot1_noise(generator);
        double rot2_n = rot2 - rot2_noise(generator);
        double shift_n = shift - shift_noise(generator);

        particle.position.x = particle.position.x + shift_n * cos(particle.angle + rot1_n);
        particle.position.y = particle.position.y + shift_n * sin(particle.angle + rot1_n);
        particle.angle += rot1_n + rot2_n;
    }
}