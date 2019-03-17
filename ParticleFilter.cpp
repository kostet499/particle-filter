//
// Created by robocup on 11.11.18.
//

#include "ParticleFilter.h"

state ParticleFilter::global_system(dot(0, 0), M_PI / 2);

bool zero_compare(double a) {
    return fabs(a) < 1e-10;
}

void ParticleFilter::WriteState(const char *filename) const {
    std::ofstream out(filename);
    out << "x;y" << std::endl;
    out << robot.position.x << ";" << robot.position.y << std::endl;
    for(const auto &particle : particles) {
        out << particle.position.x << ";" << particle.position.y << std::endl;
    }
    out.close();
}

double ParticleFilter::GaussFunction(double x, double m, double s) {
    return pow(M_E, -pow((x - m) / (sqrt(2) * s), 2));
}

// config_filename предполагается, что в нём есть данные отклонения одометрии и линии
ParticleFilter::ParticleFilter(const char* config_filename, int field_half)
: generator(42) {
    // работа с конфигом
    std::freopen(config_filename, "r", stdin);
    Json::Value root;
    std::cin >> root;
    for(auto &val : root["odometry"]) {
        odometry_noise.emplace_back(val.asDouble());
    }

    particles_amount = root["particle_amount"].asInt();
    gauss_mistake = root["gauss_mistake"].asDouble();

    for(auto &val : root["field"]) {
        baselines.emplace_back(dot(val["x1"].asDouble(), val["y1"].asDouble()), dot(val["x2"].asDouble(), val["y2"].asDouble()));
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
    std::vector<int> bad_particles;
    std::vector<double> good_weights;

    for(size_t i = 0; i < particles.size(); ++i) {
        if(particles[i].position.x < 0 || particles[i].position.y < 0 || particles[i].position.x > limit_width
        || particles[i].position.y > limit_height) {
            weights[i] = 0;
            bad_particles.emplace_back(i);
            continue;
        }
        good_weights.emplace_back(ScoreMultyLines(particles[i].position, TranslateVisionLines(particles[i], global_system, vision_lines)));
    }

    MistakesToProbability(good_weights);
    size_t bad_ind = 0;
    for(size_t i = 0; i < particles.size(); ++i) {
        if(bad_ind < bad_particles.size() && bad_particles[bad_ind] == i) {
            ++bad_ind;
        }
        else {
            weights[i] = good_weights[i - bad_ind];
        }
    }

    // получаем новую позицию - просто берем среднее по координатам
    robot.position = dot(0, 0);
    robot.angle = 0;
    for(size_t i = 0; i < particles.size(); ++i) {
        robot.position = robot.position + particles[i].position * weights[i];
        robot.angle += particles[i].angle * weights[i];
    }

    LowVarianceResample(particles_amount);
    WriteState("log.txt");
}

std::vector<line> ParticleFilter::TranslateVisionLines(const state &particle, const state &system, const std::vector<line> &lines) const {
    std::vector<line> result;

    for(const auto &liny : lines) {
        dot a, b;
        if(zero_compare(liny.b)) {
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
        SetToNewSystem(particle, system, a);
        SetToNewSystem(particle, system, b);

        result.emplace_back(a, b);
    }

    return result;
}

// пытаемся совместить типы линии, а потом оцениваем ошибку
// разность по 'c' - ошибка расстояния
// ошибка по углу - векторное произведение нормированных направляющих векторов

double ParticleFilter::ScoreLines(const dot &pos, const line &x, const line &y) const {
    double score = 0.0;
    dot x_vec(-x.b, x.a), y_vec(-y.b, y.a);

    x_vec = dot(x_vec.x / x_vec.norm(), x_vec.y / x_vec.norm());
    y_vec = dot(y_vec.x / y_vec.norm(), y_vec.y / y_vec.norm());

    assert(std::isfinite(x_vec.x));
    assert(std::isfinite(y_vec.x));

    double square = fabs(x_vec.x * y_vec.y - x_vec.y * y_vec.x);

    dot rox(-pos.x, (zero_compare(x.b) ? 0 : -x.c / x.b) - pos.y);
    dot roy(-pos.x, (zero_compare(y.b) ? 0 : -y.c / y.b) - pos.y);

    double to_x = fabs(rox.x * x_vec.y - rox.y * x_vec.x);
    double to_y = fabs(roy.x * y_vec.y - roy.y * y_vec.x);

    score += square * score_angle;
    score += fabs(to_x - to_y) * score_distance;

    return score;
}

double ParticleFilter::ScoreMultyLines(const dot &pos, const std::vector<line> &lines) const {
    double score = 0;
    for(size_t i = 0; i < lines.size(); ++i) {
        score += ScoreLines(pos, baselines[i], lines[i]);
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
    double sum = 0;
    for(auto &x : mistakes) {
        x = GaussFunction(x, 0, gauss_mistake);
        sum += x;
    }

    assert(!zero_compare(sum));

    for(auto &x : mistakes) {
        x /= sum;
    }
}

// координаты точки приводятся к системе координат поля
// переводим координаты из системы particle в system (object - объект для перевода в системе координат particle)
// (все position даны относительно глобальной системы координат)
void ParticleFilter::SetToNewSystem(const state &particle, const state &system, dot &object) {
    object = object.rotate(particle.angle - global_system.angle) + particle.position - system.position;
    object = object.rotate(global_system.angle - system.angle);
}

// углы [-pi, pi] от оси абсцисс, система координат с началом в левой нижней точке поля
void ParticleFilter::PassNewOdometry(const odometry &od) {

    double rot1 = atan2(od.new_y - od.old_y, od.new_x - od.old_x) - od.old_angle;
    double shift = dot(od.new_x - od.old_x, od.new_y - od.old_y).norm();
    double rot2 = od.new_angle - od.old_angle - rot1;

    double sigma_rot1 = std::sqrt(odometry_noise[0] * pow(rot1, 2) + odometry_noise[1] * pow(shift, 2));
    double sigma_shift = std::sqrt(odometry_noise[2] * pow(shift, 2) + odometry_noise[3] * (pow(rot1, 2) + pow(rot2, 2)));
    double sigma_rot2 = std::sqrt(odometry_noise[0] * pow(rot2, 2) + odometry_noise[1] * pow(shift, 2));

    // гауссовский шум
    boost::normal_distribution<double> shift_noise(0, sigma_shift);
    boost::normal_distribution<double> rot1_noise(0, sigma_rot1);
    boost::normal_distribution<double> rot2_noise(0, sigma_rot2);

    for(auto &particle : particles) {
        double rot1_n = rot1 - rot1_noise(generator);
        double rot2_n = rot2 - rot2_noise(generator);
        double shift_n = shift - shift_noise(generator);

        particle.position.x = particle.position.x + shift_n * cos(particle.angle + rot1_n);
        particle.position.y = particle.position.y + shift_n * sin(particle.angle + rot1_n);
        particle.angle += rot1_n + rot2_n;
        if(particle.angle > M_PI) {
            particle.angle -= 2 * M_PI;
        }
        else if(particle.angle < -M_PI) {
            particle.angle += 2 * M_PI;
        }
    }
}