#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1999204832261003601);
void live_err_fun(double *nom_x, double *delta_x, double *out_767188084803886405);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4714925448013145752);
void live_H_mod_fun(double *state, double *out_9140883660282067940);
void live_f_fun(double *state, double dt, double *out_4678334467472980726);
void live_F_fun(double *state, double dt, double *out_4482153219716734158);
void live_h_3(double *state, double *unused, double *out_2019360639508187824);
void live_H_3(double *state, double *unused, double *out_6774267715067578989);
void live_h_4(double *state, double *unused, double *out_5169689526569343836);
void live_H_4(double *state, double *unused, double *out_613036226747160280);
void live_h_9(double *state, double *unused, double *out_6740477071867596717);
void live_H_9(double *state, double *unused, double *out_8465173014456678541);
void live_h_10(double *state, double *unused, double *out_7056491405653215776);
void live_H_10(double *state, double *unused, double *out_7707316827557393254);
void live_h_12(double *state, double *unused, double *out_705245763695429570);
void live_H_12(double *state, double *unused, double *out_2719591738486369849);
void live_h_31(double *state, double *unused, double *out_1557071753237636284);
void live_H_31(double *state, double *unused, double *out_44501963342950028);
void live_h_32(double *state, double *unused, double *out_450063243709963498);
void live_H_32(double *state, double *unused, double *out_1905999758304801619);
void live_h_13(double *state, double *unused, double *out_4768374638462110812);
void live_H_13(double *state, double *unused, double *out_1256405034010384152);
void live_h_14(double *state, double *unused, double *out_6740477071867596717);
void live_H_14(double *state, double *unused, double *out_8465173014456678541);
void live_h_19(double *state, double *unused, double *out_4769154211671200418);
void live_H_19(double *state, double *unused, double *out_4032493163354247360);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}