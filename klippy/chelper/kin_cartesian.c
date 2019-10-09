// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // move_get_coord
#include "pyhelper.h" // errorf
#include "trapq.h" // trapq_find_move

static double
cart_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
cart_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
cart_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).z;
}

struct stepper_kinematics * __visible
cartesian_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x')
        sk->calc_position = cart_stepper_x_calc_position;
    else if (axis == 'y')
        sk->calc_position = cart_stepper_y_calc_position;
    else if (axis == 'z')
        sk->calc_position = cart_stepper_z_calc_position;
    return sk;
}

// XXX

struct cart_res_stepper {
    struct stepper_kinematics sk;
    double half_smooth_time, inv_smooth_time;
};

static double
res_x_calc_position(struct stepper_kinematics *sk, struct move *m
                    , double move_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    double hst = cs->half_smooth_time;
    if (! hst)
        // Calculate nominal stepper position
        return move_get_coord(m, move_time).x;
    // Calculate average position over smooth_time window
    double area = trapq_integrate(sk, m, 0, move_time - hst, move_time + hst);
    return area * cs->inv_smooth_time;
}

static double
res_y_calc_position(struct stepper_kinematics *sk, struct move *m
                    , double move_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    double hst = cs->half_smooth_time;
    if (! hst)
        return move_get_coord(m, move_time).y;
    double area = trapq_integrate(sk, m, 1, move_time - hst, move_time + hst);
    return area * cs->inv_smooth_time;
}

void __visible
cart_set_smooth_time(struct stepper_kinematics *sk, double smooth_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    if (! smooth_time) {
        cs->half_smooth_time = cs->inv_smooth_time = 0.;
        return;
    }
    cs->half_smooth_time = .5 * smooth_time;
    cs->inv_smooth_time = 1. / smooth_time;
}

static int
cart_res_flush(struct stepper_kinematics *sk
               , double step_gen_time, double print_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    double hst = cs->half_smooth_time, flush_time = print_time - hst;
    if (flush_time < step_gen_time)
        flush_time = step_gen_time;
    return trapq_flush(&cs->sk, flush_time, hst, hst);
}

struct stepper_kinematics * __visible
cartesian_res_stepper_alloc(char axis)
{
    struct cart_res_stepper *cs = malloc(sizeof(*cs));
    memset(cs, 0, sizeof(*cs));
    list_init(&cs->sk.moves);
    if (axis == 'x')
        cs->sk.calc_position = res_x_calc_position;
    else if (axis == 'y')
        cs->sk.calc_position = res_y_calc_position;
    else if (axis == 'z')
        cs->sk.calc_position = cart_stepper_z_calc_position;
    cs->sk.flush = cart_res_flush;
    return &cs->sk;
}
