#include "localisation.h"

#include <iostream>

Localisation::Localisation(boost::shared_ptr<AL::ALBroker> broker,
                   const std::string& name)
  : AL::ALModule(broker, name), memProxy(broker, "ALMemory")
{
  setModuleDescription("Localisation module");

  /*
  functionName("LocalisationMethod", getName(), "some method");
  BIND_METHOD(MyModule::LocalisationMethod);
  */

  functionName("SetLines", getName(), "Provide new vision information to Localisation module and renew position prediction");
  BIND_METHOD(Localisation::SetLines);

  functionName("SetMovement", getName(), "Provide new movement information");
  BIND_METHOD(Localisation::SetMovement);


  /**
   * addParam(<attribut_name>, <attribut_descrption>);
   * This enables to document the parameters of the method.
   * It is not compulsory to write this line.
   */

  /**
   * setReturn(<return_name>, <return_description>);
   * This enables to document the return of the method.
   * It is not compulsory to write this line.
   */
}

Localisation::~Localisation()
{
  delete filter;
}

void Localisation::init()
{
  filter = new ParticleFilter("field.json", 0);
  control_data.old_x = filter->robot.position.x;
  control_data.new_x = control_data.old_x;
  control_data.old_y = filter->robot.position.y;
  control_data.new_y = control_data.old_y;
  control_data.old_angle = filter->robot.angle;
  control_data.new_angle = control_data.old_angle;
}

void Localisation::SetLines() {
  // we get here 4 vectors (x1-s, y1-s, x2-s, y2-s) of points
  // block to test
  std::vector<float> linepoints_x1 =  memProxy.getData("linepoints_x1");
  std::vector<float> linepoints_y1 =  memProxy.getData("linepoints_y1");
  std::vector<float> linepoints_x2 =  memProxy.getData("linepoints_x2");
  std::vector<float> linepoints_y2 =  memProxy.getData("linepoints_y2");
  // end of block
  std::vector<line> vision_lines_robot;
  for(size_t i = 0; i < linepoints_x1.size(); ++i) {
    vision_lines.emplace_back(line(dot(static_cast<double>(linepoints_x1[i]), static_cast<double>(linepoints_y1[i])),
            dot(static_cast<double >(linepoints_x2[i]), static_cast<double>(linepoints_y2[i]))));
  }

  // somehow transfer lines via memory... and get std::vector of them
  filter->PassNewVision(vision_lines_robot, control_data);
  control_data.old_x = control_data.new_x;
  control_data.old_y = control_data.new_y;
  control_data.old_angle = control_data.new_angle;
}

void Localisation::SetMovement() {
  float fl_dist = memProxy.getData("move_distance");
  float fl_angle = memProxy.getData("move_angle");
  float fl_an_change = memProxy.getdata("move_angle_change");

  double distance = static_cast<double>(fl_dist);
  double angle = static_cast<double>(fl_angle);
  double angle_change = static_cast<double>(fl_an_change);
  // угол в системе отсчёта робота
  dot radius_vector(distance * std::cos(angle), distance * std::sin(angle));
  filter->SetToNewSystem(filter->robot, filter->global_system, radius_vector);
  control_data.new_x = min(filter->limit_width, max(0, control_data.new_x + radius_vector.x));
  control_data.new_y = min(filter->limit_height, max(0, control_data.new_y + radius_vector.y));
  control_data.new_angle += angle_change; // maybe here we should limit it with [-pi, pi]
}