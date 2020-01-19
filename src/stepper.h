#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);
struct stepper *stepper_oid_lookup(uint8_t oid);
void stepper_stop(struct stepper *s);


void command_position_adjust(uint8_t oid, int8_t steps);

#endif // stepper.h
