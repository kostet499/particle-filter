//
// Created by coolhatsker on 28.01.19.
//
#include "ParticleFilter.h"

int main() {
    state robot(dot(3.0, 3.0), 0);
    ParticleFilter filter("../field.json", robot, 100, 0);
    boost::random::uniform_real_distribution<double> dist(0.25, 0.5);
    boost::random::uniform_real_distribution<double> rot(-1.0, 1.0);
    for(int i = 0; i < 1000; ++i) {
        double distance = dist(filter.generator);
        double angle = rot(filter.generator);
        odometry control_data(
                robot.position.x, robot.position.x + distance * cos(robot.angle),
                robot.position.y, robot.position.y + distance * sin(robot.angle),
                robot.angle, robot.angle + angle);
        if(control_data.new_angle > M_PI) {
            control_data.new_angle -= 2 * M_PI;
        }
        else if(control_data.new_angle < -M_PI) {
            control_data.new_angle += 2 * M_PI;
        }

        robot = state(dot(control_data.new_x, control_data.new_y), control_data.new_angle);

        // добавить видимых линий

        if(i % 50 == 0) {
            std::cout << filter.robot.position.x - robot.position.x << " " <<
            filter.robot.position.y - robot.position.y <<
            filter.robot.angle - robot.angle << std::endl;
        }
    }

    return 0;
}

