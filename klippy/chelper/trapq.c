// Trapezoidal velocity movement queue
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // free
#include <string.h> // memset
#include "compiler.h" // unlikely
#include "itersolve.h" // itersolve_gen_steps_range
#include "list.h" // list_empty
#include "pyhelper.h" // errorf
#include "trapq.h" // trapq_find_move

void __visible
trapq_add_move(struct stepper_kinematics *sk, struct move *m)
{
    struct move *nm = move_alloc();
    memcpy(nm, m, sizeof(*nm));
    list_add_tail(&nm->node, &sk->moves);
}

void
trapq_clear(struct stepper_kinematics *sk)
{
    if (!sk->moves.root.prev)
        return; // XXX
    while (!list_empty(&sk->moves)) {
        struct move *m = list_first_entry(&sk->moves, struct move, node);
        list_del(&m->node);
        free(m);
    }
}

struct move *
trapq_find_move(struct stepper_kinematics *sk, struct move *m, double *ptime)
{
    double end_time = *ptime;
    for (;;) {
        if (unlikely(end_time < 0.)) {
            // Check previous move in list
            if (list_is_first(&m->node, &sk->moves)) {
                end_time = 0.;
                break;
            }
            struct move *prev = list_prev_entry(m, node);
            end_time += m->print_time - prev->print_time;
            if (end_time >= prev->move_t) {
                end_time = 0.;
                break;
            }
            m = prev;
        } else if (unlikely(end_time > m->move_t)) {
            // Check next move in list
            if (list_is_last(&m->node, &sk->moves)) {
                end_time = m->move_t;
                break;
            }
            struct move *next = list_next_entry(m, node);
            end_time -= next->print_time - m->print_time;
            if (end_time <= 0.) {
                end_time = m->move_t;
                break;
            }
            m = next;
        } else {
            break;
        }
    }
    *ptime = end_time;
    return m;
}

static double
move_integrate_accel(struct move *m, struct move_accel *ma
                     , int axis, double start, double end)
{
    double start_pos = m->start_pos.axis[axis];
    if (ma == &m->decel) // XXX
        start_pos += m->decel_start_d * m->axes_r.axis[axis];
    double half_c1 = .5 * ma->c1, third_c2 = (1. / 3.) * ma->c2;
    double si = start * start * (half_c1 + third_c2 * start);
    double ei = end * end * (half_c1 + third_c2 * end);
    double avg_pos = (ei - si) * m->axes_r.axis[axis];
    return start_pos * (end - start) + avg_pos;
}

static double
move_integrate_cruise(struct move *m, int axis, double start, double end)
{
    double avg_d = m->cruise_start_d + m->cruise_v * .5 * (start + end);
    double avg_pos = m->start_pos.axis[axis] + avg_d * m->axes_r.axis[axis];
    return avg_pos * (end - start);
}

// Calculate the definitive integral for a cartesian axis
double
trapq_integrate(struct stepper_kinematics *sk, struct move *m, int axis
                , double start, double end)
{
    double res = 0.;
    if (start < 0.) {
        // Integrate over previous moves
        if (list_is_first(&m->node, &sk->moves)) {
            res += m->start_pos.axis[axis] * -start;
        } else {
            struct move *prev = list_prev_entry(m, node);
            double delta = m->print_time - prev->print_time;
            double new_start = start + delta;
            if (new_start >= prev->move_t) {
                res += m->start_pos.axis[axis] * -start;
            } else {
                res += m->start_pos.axis[axis] * (delta - prev->move_t);
                res += trapq_integrate(sk, prev, axis, new_start, prev->move_t);
            }
        }
        start = 0.;
    }
    // Integrate over this move
    if (start < m->accel_t) {
        if (end <= m->accel_t)
            return res + move_integrate_accel(m, &m->accel, axis, start, end);
        res += move_integrate_accel(m, &m->accel, axis, start, m->accel_t);
        start = m->accel_t;
    }
    double cruise_end_t = m->accel_t + m->cruise_t;
    if (start < cruise_end_t) {
        if (end <= cruise_end_t)
            return res + move_integrate_cruise(m, axis, start - m->accel_t
                                               , end - m->accel_t);
        res += move_integrate_cruise(m, axis, start - m->accel_t, m->cruise_t);
        start = cruise_end_t;
    }
    if (end <= m->move_t)
        return res + move_integrate_accel(
            m, &m->decel, axis, start - cruise_end_t, end - cruise_end_t);
    // XXX - only call if there is decel
    res += move_integrate_accel(
        m, &m->decel, axis, start - cruise_end_t, m->move_t - cruise_end_t);
    // Integrate over future moves
    double end_pos = move_get_coord(m, m->move_t).axis[axis];
    double extra_t = end - m->move_t;
    if (list_is_last(&m->node, &sk->moves))
        return res + end_pos * extra_t;
    struct move *next = list_next_entry(m, node);
    double delta = next->print_time - (m->print_time + m->move_t);
    if (delta >= extra_t)
        return res + end_pos * extra_t;
    res += end_pos * delta;
    return res + trapq_integrate(sk, next, axis, 0., extra_t - delta);
}

int
trapq_flush(struct stepper_kinematics *sk, double flush_time
            , double prev_scan, double next_scan)
{
    if (list_empty(&sk->moves))
        return 0;
    struct move *cur = list_first_entry(&sk->moves, struct move, node);
    struct move null_move;
    double last_print_time = sk->last_print_time;
    for (;;) {
        struct move *m = cur;
        double move_print_time = m->print_time;
        double move_end_time = move_print_time + m->move_t;
        if (last_print_time >= move_end_time) {
            double scan_to_time = move_end_time + prev_scan;
            if (!list_is_last(&m->node, &sk->moves)) {
                struct move *next = list_next_entry(m, node);
                if (scan_to_time > next->print_time)
                    scan_to_time = next->print_time;
            }
            if (last_print_time + .000000001 < scan_to_time) {
                // Insert null move
                memset(&null_move, 0, sizeof(null_move));
                null_move.node.prev = &m->node;
                null_move.node.next = m->node.next;
                null_move.print_time = move_end_time;
                null_move.move_t = scan_to_time - move_end_time;
                null_move.cruise_t = null_move.move_t;
                null_move.start_pos = move_get_coord(m, m->move_t);
                m = &null_move;
                move_print_time = move_end_time;
            } else {
                if (list_is_last(&m->node, &sk->moves))
                    break;
                cur = list_next_entry(m, node);
                continue;
            }
        } else if (next_scan
                   && last_print_time + .000000001 < move_print_time) {
            // Insert null move
            double null_print_time = move_print_time - next_scan;
            if (last_print_time > null_print_time)
                null_print_time = last_print_time;
            memset(&null_move, 0, sizeof(null_move));
            null_move.node.prev = m->node.prev;
            null_move.node.next = &m->node;
            null_move.print_time = null_print_time;
            null_move.move_t = move_print_time - null_print_time;
            null_move.cruise_t = null_move.move_t;
            null_move.start_pos = m->start_pos;
            m = &null_move;
            move_print_time = null_print_time;
        }

        double start = 0., end = m->move_t;
        if (last_print_time > move_print_time)
            start = last_print_time - move_print_time;
        if (move_print_time + start >= flush_time)
            break;
        if (move_print_time + end > flush_time)
            end = flush_time - move_print_time;
        int32_t ret = itersolve_gen_steps_range(sk, m, start, end);
        if (ret)
            return ret;
        last_print_time = move_print_time + end;
    }
    sk->last_print_time = last_print_time;

    // Free moves that are no longer needed
    while (!list_empty(&sk->moves)) {
        struct move *m = list_first_entry(&sk->moves, struct move, node);
        if (last_print_time < m->print_time + m->move_t + prev_scan)
            break;
        list_del(&m->node);
        free(m);
    }

    return 0;
}
