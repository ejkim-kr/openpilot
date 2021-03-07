/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2864232012878443181);
void inv_err_fun(double *nom_x, double *true_x, double *out_4839076456681737827);
void H_mod_fun(double *state, double *out_5274645477792287522);
void f_fun(double *state, double dt, double *out_5144599170092554026);
void F_fun(double *state, double dt, double *out_3179693152305072353);
void h_3(double *state, double *unused, double *out_1987358052747696678);
void H_3(double *state, double *unused, double *out_8079375767292091058);
void h_4(double *state, double *unused, double *out_6001025428229906438);
void H_4(double *state, double *unused, double *out_3056999740309968474);
void h_9(double *state, double *unused, double *out_7306585433069788796);
void H_9(double *state, double *unused, double *out_6311535092195744321);
void h_10(double *state, double *unused, double *out_3002066118015959335);
void H_10(double *state, double *unused, double *out_7689362326216469170);
void h_12(double *state, double *unused, double *out_933956342195034247);
void H_12(double *state, double *unused, double *out_950444228570758905);
void h_31(double *state, double *unused, double *out_9084376364434547488);
void H_31(double *state, double *unused, double *out_3625534003714178726);
void h_32(double *state, double *unused, double *out_6716759229705702306);
void H_32(double *state, double *unused, double *out_4195324550166757210);
void h_13(double *state, double *unused, double *out_3093182505743757273);
void H_13(double *state, double *unused, double *out_3422858521270881615);
void h_14(double *state, double *unused, double *out_7306585433069788796);
void H_14(double *state, double *unused, double *out_6311535092195744321);
void h_19(double *state, double *unused, double *out_3488942378243753036);
void H_19(double *state, double *unused, double *out_7702529130411376114);
#define DIM 23
#define EDIM 22
#define MEDIM 22
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_3 = 3.841459;
void update_3(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814728;
void update_4(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_9 = 7.814728;
void update_9(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_10 = 7.814728;
void update_10(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_12 = 7.814728;
void update_12(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_31 = 7.814728;
void update_31(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_32 = 9.487729;
void update_32(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);