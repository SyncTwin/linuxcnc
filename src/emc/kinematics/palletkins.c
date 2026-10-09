/* palletkins.c — AICNC 4-axis palletizer kinematics (4R, flange kept level).
 *
 * joints (deg): J0 base turn q1, J1 shoulder q2, J2 elbow q3, J3 flange turn q4.
 * world: X Y Z = flange point (mm, base frame), C = flange turn (deg) = q1 + q4.
 * Two parallelograms keep the flange level, so only q1..q3 place the point:
 *   r = SX - U*sin(q2) + F*cos(q2+q3) + FX
 *   h = SZ + U*cos(q2) + F*sin(q2+q3) + FZ
 *   X = r*cos(q1) - FY*sin(q1),  Y = r*sin(q1) + FY*cos(q1),  Z = h
 * Inverse is closed form (planar two-link) with four branches; the branch is
 * chosen by the inverse flags, as pumakins (PUMA_SHOULDER_RIGHT, PUMA_ELBOW_DOWN)
 * and scarakins do: kinematicsForward reports the configuration of the joints
 * in *iflags, kinematicsInverse solves in the configuration *iflags names
 * (motion keeps one iflags: control.c sets it by the forward kinematics and
 * hands it to the inverse).  Picking among branches (joint limits, nearest) is
 * the caller's.  Geometry is NOT baked in: HAL pins palletkins.* are set by the
 * generated .hal from the machine's URDF.
 *
 * Built in-tree (src/Makefile, obj-m like scarakins).
 */
#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "rtapi_math.h"
#include "kinematics.h"

static int comp_id;

KINS_NOT_SWITCHABLE
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

/* hal_real_t is opaque in the 2.10 HAL: read via hal_get_real(), created with
 * hal_pin_new_real() (same as genhexkins.c). */
typedef hal_real_t geom_real_t;
#define GEOM_GET(p) hal_get_real(p)
#define GEOM_PIN_NEW(name, ref) hal_pin_new_real(comp_id, HAL_IN, (ref), 0.0, "%s", (name))

struct geom {
    geom_real_t sx, sz, u, f, fx, fy, fz;
};
static struct geom *g;

/* Inverse flags (iflags): 0 = the flange in front of the base axis, elbow up. */
#define PALLET_SHOULDER_BACK 0x01   /* r < 0: the flange behind the base axis (shoulder leaned back) */
#define PALLET_ELBOW_DOWN    0x02   /* the forearm turned past the upper arm: sin(q3 - 90 deg) > 0 */

#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

int kinematicsForward(const double *joint, EmcPose *pos,
        const KINEMATICS_FORWARD_FLAGS *fflags, KINEMATICS_INVERSE_FLAGS *iflags) {
    (void)fflags;
    double q1 = joint[0] * D2R, q2 = joint[1] * D2R, q3 = joint[2] * D2R;
    double sx = GEOM_GET(g->sx), sz = GEOM_GET(g->sz), u = GEOM_GET(g->u), f = GEOM_GET(g->f);
    double fx = GEOM_GET(g->fx), fy = GEOM_GET(g->fy), fz = GEOM_GET(g->fz);
    double r = sx - u * sin(q2) + f * cos(q2 + q3) + fx;
    double h = sz + u * cos(q2) + f * sin(q2 + q3) + fz;
    pos->tran.x = r * cos(q1) - fy * sin(q1);
    pos->tran.y = r * sin(q1) + fy * cos(q1);
    pos->tran.z = h;
    pos->a = 0.0; pos->b = 0.0;
    pos->c = joint[0] + joint[3];
    pos->u = pos->v = pos->w = 0.0;
    if (iflags) {   /* NULL from a caller that only wants the pose */
        *iflags = 0;
        if (r < 0.0) *iflags |= PALLET_SHOULDER_BACK;
        if (cos(q3) < 0.0) *iflags |= PALLET_ELBOW_DOWN;   /* sin(q3 - 90 deg) = -cos(q3) */
    }
    return 0;
}

int kinematicsInverse(const EmcPose *pos, double *joint,
        const KINEMATICS_INVERSE_FLAGS *iflags, KINEMATICS_FORWARD_FLAGS *fflags) {
    (void)fflags;
    KINEMATICS_INVERSE_FLAGS fl = iflags ? *iflags : 0;
    double x = pos->tran.x, y = pos->tran.y, fy = GEOM_GET(g->fy);
    double U = GEOM_GET(g->u), F = GEOM_GET(g->f);
    double rho2 = x * x + y * y;
    if (rho2 < fy * fy || U <= 0.0 || F <= 0.0) return -1;
    double r = ((fl & PALLET_SHOULDER_BACK) ? -1.0 : 1.0) * sqrt(rho2 - fy * fy);
    double q1 = (atan2(y, x) - atan2(fy, r)) * R2D;
    while (q1 > 180.0) q1 -= 360.0;
    while (q1 < -180.0) q1 += 360.0;
    double a = r - GEOM_GET(g->sx) - GEOM_GET(g->fx);
    double b = pos->tran.z - GEOM_GET(g->sz) - GEOM_GET(g->fz);
    double D = (a * a + b * b - U * U - F * F) / (2.0 * U * F);
    if (D > 1.0 || D < -1.0) return -1;
    double t2 = ((fl & PALLET_ELBOW_DOWN) ? 1.0 : -1.0) * acos(D);   /* forearm vs upper arm */
    double t1 = atan2(b, a) - atan2(F * sin(t2), U + F * cos(t2));
    double q2 = (t1 * R2D) - 90.0, q3 = (t2 * R2D) + 90.0;
    while (q2 > 180.0) q2 -= 360.0;
    while (q2 < -180.0) q2 += 360.0;
    joint[0] = q1;
    joint[1] = q2;
    joint[2] = q3;
    joint[3] = pos->c - joint[0];
    return 0;
}

KINEMATICS_TYPE kinematicsType(void) {
    return KINEMATICS_BOTH;
}

int rtapi_app_main(void) {
    int res = 0;
    comp_id = hal_init("palletkins");
    if (comp_id < 0) return comp_id;
    g = hal_malloc(sizeof(struct geom));
    if (!g) { hal_exit(comp_id); return -1; }
    if ((res = GEOM_PIN_NEW("palletkins.shoulder-x", &g->sx)) < 0) goto fail;
    if ((res = GEOM_PIN_NEW("palletkins.shoulder-z", &g->sz)) < 0) goto fail;
    if ((res = GEOM_PIN_NEW("palletkins.upper", &g->u)) < 0) goto fail;
    if ((res = GEOM_PIN_NEW("palletkins.fore", &g->f)) < 0) goto fail;
    if ((res = GEOM_PIN_NEW("palletkins.flange-x", &g->fx)) < 0) goto fail;
    if ((res = GEOM_PIN_NEW("palletkins.flange-y", &g->fy)) < 0) goto fail;
    if ((res = GEOM_PIN_NEW("palletkins.flange-z", &g->fz)) < 0) goto fail;
    hal_ready(comp_id);
    return 0;
fail:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
