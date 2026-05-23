#include "Gripper.hpp"
#include <cstdio>

Gripper::Gripper(Servo *claw, Servo *lifter, float claw_step,
                 float lifted_angle, float drop_angle)
    : claw(claw), lifter(lifter), claw_step(claw_step),
      lifted_angle(lifted_angle), drop_angle(drop_angle) {}

void Gripper::close() {
  if (this->current_angle > 0) {
    this->current_angle -= this->claw_step;
    this->claw->set_angle(current_angle);
  }
}

void Gripper::open() {
  if (this->current_angle < this->claw->max_angle) {
    this->current_angle += this->claw_step;
    this->claw->set_angle(current_angle);
  }
}

void Gripper::lift() { this->lifter->set_angle(this->lifted_angle); }

void Gripper::drop() { this->lifter->set_angle(this->drop_angle); }
