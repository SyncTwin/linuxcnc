/* The jog planner of motion, src/emc/motion/simple_tp.c, with its S-curve
 * (max_jerk > 0, planner type 1), replayed period by period on the inputs
 * that cmpj.py recorded: stdin "target enable maxvel maxaccel maxjerk" per
 * period, stdout "position velocity" per period.  simple_tp.comp has no jerk
 * input, so this is the simple_tp that plans with a jerk limit. */
#include <stdio.h>
#include <string.h>
#include "motion.h"
#include "simple_tp.h"

static emcmot_status_t status;
emcmot_status_t *emcmotStatus = &status;

int main(int argc, char **argv)
{
    double dt = 0.001, target, maxvel, maxaccel, maxjerk;
    int enable;
    simple_tp_t tp;
    memset(&tp, 0, sizeof(tp));
    emcmotStatus->planner_type = 1;   /* S-curve; 0 would ignore max_jerk */
    while (scanf("%lf %d %lf %lf %lf", &target, &enable, &maxvel, &maxaccel, &maxjerk) == 5) {
        tp.pos_cmd = target;
        tp.enable = enable;
        tp.max_vel = maxvel;
        tp.max_acc = maxaccel;
        tp.max_jerk = maxjerk;
        simple_tp_update(&tp, dt);
        printf("%.9f %.9f\n", tp.curr_pos, tp.curr_vel);
    }
    return 0;
}
