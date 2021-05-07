#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8228530102676043774);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5997967712644757779);
void car_H_mod_fun(double *state, double *out_4822971942340996086);
void car_f_fun(double *state, double dt, double *out_3135785908568435086);
void car_F_fun(double *state, double dt, double *out_7437156320522608262);
void car_h_25(double *state, double *unused, double *out_8229301071472580126);
void car_H_25(double *state, double *unused, double *out_8814633963559888772);
void car_h_24(double *state, double *unused, double *out_1891611977411130115);
void car_H_24(double *state, double *unused, double *out_1882105312097276551);
void car_h_30(double *state, double *unused, double *out_4061995806396311361);
void car_H_30(double *state, double *unused, double *out_4380146183783888564);
void car_h_26(double *state, double *unused, double *out_4665311691072788878);
void car_H_26(double *state, double *unused, double *out_3892316593469647164);
void car_h_27(double *state, double *unused, double *out_6941595189489514901);
void car_H_27(double *state, double *unused, double *out_8315400077271002573);
void car_h_29(double *state, double *unused, double *out_6666401127205009012);
void car_H_29(double *state, double *unused, double *out_3805587569928352389);
void car_h_28(double *state, double *unused, double *out_397525187180519846);
void car_H_28(double *state, double *unused, double *out_6264987618935476476);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}