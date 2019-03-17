#include "localisation.h"

#include <iostream>

Localisation::Localisation(boost::shared_ptr<AL::ALBroker> broker,
                   const std::string& name)
  : AL::ALModule(broker, name)
{
  setModuleDescription("Localisation module");

  /*
  functionName("LocalisationMethod", getName(), "some method");
  BIND_METHOD(MyModule::LocalisationMethod);
  */

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
  // somehow transfer lines via memory... and get std::vector of them
  std::vector<line> vision_lines_robot;
  filter->PassNewVision(vision_lines_robot, control_data);
  control_data.old_x = control_data.new_x;
  control_data.old_y = control_data.new_y;
  control_data.old_angle = control_data.new_angle;
}

void Localisation::SetMovement(double distance, double angle, double angle_change) {
  // угол в системе отсчёта робота
  dot radius_vector(distance * std::cos(angle), distance * std::sin(angle));
  filter->SetToNewSystem(filter->robot, filter->global_system, radius_vector);
  control_data.new_x = min(filter->limit_width, max(0, control_data.new_x + radius_vector.x));
  control_data.new_y = min(filter->limit_height, max(0, control_data.new_y + radius_vector.y));
  control_data.new_angle += angle_change; // maybe here we should limit it with [-pi, pi]
}