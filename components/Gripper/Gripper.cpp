#include "Gripper.hpp"

Gripper::Gripper(Servo* servo) : servo(servo), max_angle(90), min_angle(0) {}

void Gripper::close() {
  this->current_angle += this->angle_step;
  this->servo->set_angle(this->current_angle);
}

void Gripper::open() {
  this->current_angle -= this->angle_step;
  this->servo->set_angle(this->current_angle);
}

void Gripper::lift() {
  this->current_angle = this->max_angle;
  this->servo->set_angle(this->current_angle);
}

void Gripper::drop() {
  this->current_angle = this->min_angle;
  this->servo->set_angle(this->current_angle);
}