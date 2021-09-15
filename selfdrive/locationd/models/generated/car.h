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
void car_err_fun(double *nom_x, double *delta_x, double *out_7530412064838234300);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1203665384329719755);
void car_H_mod_fun(double *state, double *out_2321076618871876345);
void car_f_fun(double *state, double dt, double *out_3121253294902994323);
void car_F_fun(double *state, double dt, double *out_4840371344081398094);
void car_h_25(double *state, double *unused, double *out_7726019570887069113);
void car_H_25(double *state, double *unused, double *out_167167667836240578);
void car_h_24(double *state, double *unused, double *out_7198177503220940015);
void car_H_24(double *state, double *unused, double *out_1429556732031980710);
void car_h_30(double *state, double *unused, double *out_3052468125618195096);
void car_H_30(double *state, double *unused, double *out_9000012830596608914);
void car_h_26(double *state, double *unused, double *out_448062804809497445);
void car_H_26(double *state, double *unused, double *out_5871760822775449851);
void car_h_27(double *state, double *unused, double *out_2500112355051744166);
void car_H_27(double *state, double *unused, double *out_7712430842759983602);
void car_h_29(double *state, double *unused, double *out_2449152240941717579);
void car_H_29(double *state, double *unused, double *out_1826143340622549702);
void car_h_28(double *state, double *unused, double *out_1845099704568526585);
void car_H_28(double *state, double *unused, double *out_2953325574806219215);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}