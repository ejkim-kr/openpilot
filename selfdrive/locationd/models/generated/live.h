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
void live_H(double *in_vec, double *out_2171810797954634884);
void live_err_fun(double *nom_x, double *delta_x, double *out_8542546535441190970);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8399843449917516150);
void live_H_mod_fun(double *state, double *out_8891062055483122034);
void live_f_fun(double *state, double dt, double *out_2240038446206287964);
void live_F_fun(double *state, double dt, double *out_7703937601435102592);
void live_h_3(double *state, double *unused, double *out_7428854366516085539);
void live_H_3(double *state, double *unused, double *out_3256564296091412995);
void live_h_4(double *state, double *unused, double *out_8430018997242678260);
void live_H_4(double *state, double *unused, double *out_2082170513073981530);
void live_h_9(double *state, double *unused, double *out_3356817634526894403);
void live_H_9(double *state, double *unused, double *out_6996038728129857291);
void live_h_10(double *state, double *unused, double *out_4018377394709224055);
void live_H_10(double *state, double *unused, double *out_1027932559565730950);
void live_h_12(double *state, double *unused, double *out_7025509156379546761);
void live_H_12(double *state, double *unused, double *out_4188726024813191099);
void live_h_31(double *state, double *unused, double *out_6589023536365609729);
void live_H_31(double *state, double *unused, double *out_1513636249669771278);
void live_h_32(double *state, double *unused, double *out_5567556164815529359);
void live_H_32(double *state, double *unused, double *out_1769083286801434943);
void live_h_13(double *state, double *unused, double *out_1853642320553116850);
void live_H_13(double *state, double *unused, double *out_6729170183730499664);
void live_h_14(double *state, double *unused, double *out_3356817634526894403);
void live_H_14(double *state, double *unused, double *out_6996038728129857291);
void live_h_19(double *state, double *unused, double *out_2201584990905246391);
void live_H_19(double *state, double *unused, double *out_2563358877027426110);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}