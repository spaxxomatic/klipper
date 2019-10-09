#ifndef ITERSOLVE_H
#define ITERSOLVE_H

#include <stdint.h> // uint32_t
#include "list.h" // list_node

struct coord {
    union {
        struct {
            double x, y, z;
        };
        double axis[3];
    };
};

struct move_accel {
    double c1, c2;
};

struct move {
    struct list_node node;

    double print_time, move_t;
    double accel_t, cruise_t;
    double cruise_start_d, decel_start_d;
    double cruise_v;
    struct move_accel accel, decel;
    struct coord start_pos, axes_r;
};

struct move *move_alloc(void);
void move_fill(struct move *m, double print_time
               , double accel_t, double cruise_t, double decel_t
               , double start_pos_x, double start_pos_y, double start_pos_z
               , double axes_d_x, double axes_d_y, double axes_d_z
               , double start_v, double cruise_v, double accel);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);

struct stepper_kinematics;
typedef double (*sk_callback)(struct stepper_kinematics *sk, struct move *m
                              , double move_time);
typedef int (*sk_flush_callback)(struct stepper_kinematics *sk
                                 , double step_gen_time, double print_time);
struct stepper_kinematics {
    double step_dist, commanded_pos;
    struct stepcompress *sc;
    sk_callback calc_position;
    struct list_head moves;
    double last_print_time; // XXX
    sk_flush_callback flush;
};

int32_t itersolve_gen_steps_range(struct stepper_kinematics *sk, struct move *m
                                  , double start, double end);
int32_t itersolve_gen_steps(struct stepper_kinematics *sk, struct move *m);
void itersolve_set_stepcompress(struct stepper_kinematics *sk
                                , struct stepcompress *sc, double step_dist);
double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
                                          , double x, double y, double z);
void itersolve_set_commanded_pos(struct stepper_kinematics *sk, double pos);
double itersolve_get_commanded_pos(struct stepper_kinematics *sk);
int itersolve_flush(struct stepper_kinematics *sk, double step_gen_time
                    , double print_time);

#endif // itersolve.h
