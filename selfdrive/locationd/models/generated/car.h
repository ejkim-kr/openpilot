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
void car_err_fun(double *nom_x, double *delta_x, double *out_5144003432639031458);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1019709690714833271);
void car_H_mod_fun(double *state, double *out_3160954694154719337);
void car_f_fun(double *state, double dt, double *out_7809129704521149226);
void car_F_fun(double *state, double dt, double *out_3511533356282032530);
void car_h_25(double *state, double *unused, double *out_6300983868460679792);
void car_H_25(double *state, double *unused, double *out_6936622825500736383);
void car_h_24(double *state, double *unused, double *out_2249814897033317498);
void car_H_24(double *state, double *unused, double *out_2468436691888653813);
void car_h_30(double *state, double *unused, double *out_4685201440999983499);
void car_H_30(double *state, double *unused, double *out_2677276085448446897);
void car_h_26(double *state, double *unused, double *out_316471826660862146);
void car_H_26(double *state, double *unused, double *out_5805528093269605960);
void car_h_27(double *state, double *unused, double *out_1299310600799834147);
void car_H_27(double *state, double *unused, double *out_3964858073285072209);
void car_h_29(double *state, double *unused, double *out_5288517325676461016);
void car_H_29(double *state, double *unused, double *out_544954434057577975);
void car_h_28(double *state, double *unused, double *out_2606527359166809232);
void car_H_28(double *state, double *unused, double *out_8960474903584402937);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}