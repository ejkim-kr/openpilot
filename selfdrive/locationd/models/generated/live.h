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
void live_H(double *in_vec, double *out_83290133610455021);
void live_err_fun(double *nom_x, double *delta_x, double *out_1295404048766981892);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7718846480313973371);
void live_H_mod_fun(double *state, double *out_1359201248254019031);
void live_f_fun(double *state, double dt, double *out_1465447188795773295);
void live_F_fun(double *state, double dt, double *out_3920041907016275812);
void live_h_3(double *state, double *unused, double *out_271482028885263482);
void live_H_3(double *state, double *unused, double *out_6517061992468698667);
void live_h_4(double *state, double *unused, double *out_4706127011587876512);
void live_H_4(double *state, double *unused, double *out_6594544378669371375);
void live_h_9(double *state, double *unused, double *out_3018609892509436051);
void live_H_9(double *state, double *unused, double *out_126318548185852723);
void live_h_10(double *state, double *unused, double *out_5194968214894559743);
void live_H_10(double *state, double *unused, double *out_8423363738041589047);
void live_h_12(double *state, double *unused, double *out_2336519524919619223);
void live_H_12(double *state, double *unused, double *out_89631483945793678);
void live_h_31(double *state, double *unused, double *out_3272129980774585730);
void live_H_31(double *state, double *unused, double *out_2764721259089213499);
void live_h_32(double *state, double *unused, double *out_2301249317401500166);
void live_H_32(double *state, double *unused, double *out_2409439959414559335);
void live_h_13(double *state, double *unused, double *out_484676118093603460);
void live_H_13(double *state, double *unused, double *out_7074796751230287230);
void live_h_14(double *state, double *unused, double *out_3018609892509436051);
void live_H_14(double *state, double *unused, double *out_126318548185852723);
void live_h_19(double *state, double *unused, double *out_2033301854628199061);
void live_H_19(double *state, double *unused, double *out_4558998399288283904);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}