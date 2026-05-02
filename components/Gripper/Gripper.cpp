#include "Gripper.hpp"

Gripper::Gripper(Servo* servo) : servo(servo) {}

void Gripper::increase_angle() {
  this->current_angle += this->angle_step;
  this->servo->set_angle(this->current_angle);
}

void Gripper::decrease_angle() {
  this->current_angle -= this->angle_step;
  this->servo->set_angle(this->current_angle);
}

float Gripper::get_angle() {
  return this->current_angle;
}