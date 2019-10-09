#ifndef TRAPQ_H
#define TRAPQ_H

#include <stdint.h> // uint32_t
#include "list.h" // list_node

struct stepper_kinematics;
struct move;
void trapq_add_move(struct stepper_kinematics *sk, struct move *m);
void trapq_clear(struct stepper_kinematics *sk);
struct move *trapq_find_move(struct stepper_kinematics *sk, struct move *m
                             , double *ptime);
double trapq_integrate(struct stepper_kinematics *sk, struct move *m, int axis
                       , double start, double end);
int trapq_flush(struct stepper_kinematics *sk, double flush_time
                , double prev_scan, double next_scan);

#endif // trapq.h
