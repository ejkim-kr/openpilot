/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_879539959337219109);
void inv_err_fun(double *nom_x, double *true_x, double *out_8481518425670246390);
void H_mod_fun(double *state, double *out_8655776821554035084);
void f_fun(double *state, double dt, double *out_7014554748423705679);
void F_fun(double *state, double dt, double *out_6891519007627046374);
void h_25(double *state, double *unused, double *out_5318297026083439055);
void H_25(double *state, double *unused, double *out_8104691973345650739);
void h_24(double *state, double *unused, double *out_2511437235748253558);
void H_24(double *state, double *unused, double *out_6842302909149910607);
void h_30(double *state, double *unused, double *out_5247174396872063376);
void H_30(double *state, double *unused, double *out_728153189414717597);
void h_26(double *state, double *unused, double *out_6953118619377314772);
void H_26(double *state, double *unused, double *out_2400098818406441466);
void h_27(double *state, double *unused, double *out_8126773779965266916);
void H_27(double *state, double *unused, double *out_559428798421907715);
void h_29(double *state, double *unused, double *out_7851579717680761027);
void H_29(double *state, double *unused, double *out_1302711803270253772);
void h_28(double *state, double *unused, double *out_2194679860431949296);
void H_28(double *state, double *unused, double *out_175529569086584259);
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_25 = 3.841459;
void update_25(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_24 = 5.991465;
void update_24(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_30 = 3.841459;
void update_30(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_26 = 3.841459;
void update_26(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_27 = 3.841459;
void update_27(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_29 = 3.841459;
void update_29(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_28 = 5.991465;
void update_28(double *, double *, double *, double *, double *);
void set_mass(double x);

void set_rotational_inertia(double x);

void set_center_to_front(double x);

void set_center_to_rear(double x);

void set_stiffness_front(double x);

void set_stiffness_rear(double x);
