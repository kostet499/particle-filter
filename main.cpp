//
// Created by coolhatsker on 28.01.19.
//
#include "ParticleFilter.h"

int main() {
    state robot(dot(1, 1), 0);
    ParticleFilter filter("../field.json", robot, 0);
    boost::random::uniform_real_distribution<double> dist(0.2, 0.4);
    boost::random::uniform_real_distribution<double> rot(-0.5, 1.0);
    for(int i = 0; i < 10000; ++i) {
        double distance = dist(filter.generator);
        double angle = rot(filter.generator);
        odometry control_data(
                robot.position.x, std::min(fabs(robot.position.x + distance * cos(robot.angle)), filter.limit_width),
                robot.position.y, std::min(fabs(robot.position.y + distance * sin(robot.angle)), filter.limit_height),
                robot.angle, robot.angle + angle);

        if(control_data.new_angle > M_PI) {
            control_data.new_angle -= 2 * M_PI;
        }
        else if(control_data.new_angle < -M_PI) {
            control_data.new_angle += 2 * M_PI;
        }

        robot = state(dot(control_data.new_x, control_data.new_y), control_data.new_angle);

        std::vector<line> vision_lines = filter.TranslateVisionLines(filter.global_system, robot, filter.baselines);
        vision_lines.pop_back();
        vision_lines.pop_back();
        filter.PassNewVision(vision_lines, control_data);
        // добавить видимых линий

        if(i % 100 == 0) {
            printf("%0.1f %0.1f %0.1f \n", fabs(filter.robot.position.x - robot.position.x) / filter.limit_width * 100,
                    fabs(filter.robot.position.y - robot.position.y) / filter.limit_height * 100,
                    fabs(filter.robot.angle - robot.angle) * 50 / M_PI );
        }
    }

    return 0;
}

