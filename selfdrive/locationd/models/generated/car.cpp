#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 5.991464547107981;

/******************************************************************************
 *                      Code generated with sympy 1.7.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7530412064838234300) {
   out_7530412064838234300[0] = delta_x[0] + nom_x[0];
   out_7530412064838234300[1] = delta_x[1] + nom_x[1];
   out_7530412064838234300[2] = delta_x[2] + nom_x[2];
   out_7530412064838234300[3] = delta_x[3] + nom_x[3];
   out_7530412064838234300[4] = delta_x[4] + nom_x[4];
   out_7530412064838234300[5] = delta_x[5] + nom_x[5];
   out_7530412064838234300[6] = delta_x[6] + nom_x[6];
   out_7530412064838234300[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1203665384329719755) {
   out_1203665384329719755[0] = -nom_x[0] + true_x[0];
   out_1203665384329719755[1] = -nom_x[1] + true_x[1];
   out_1203665384329719755[2] = -nom_x[2] + true_x[2];
   out_1203665384329719755[3] = -nom_x[3] + true_x[3];
   out_1203665384329719755[4] = -nom_x[4] + true_x[4];
   out_1203665384329719755[5] = -nom_x[5] + true_x[5];
   out_1203665384329719755[6] = -nom_x[6] + true_x[6];
   out_1203665384329719755[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_2321076618871876345) {
   out_2321076618871876345[0] = 1.0;
   out_2321076618871876345[1] = 0.0;
   out_2321076618871876345[2] = 0.0;
   out_2321076618871876345[3] = 0.0;
   out_2321076618871876345[4] = 0.0;
   out_2321076618871876345[5] = 0.0;
   out_2321076618871876345[6] = 0.0;
   out_2321076618871876345[7] = 0.0;
   out_2321076618871876345[8] = 0.0;
   out_2321076618871876345[9] = 1.0;
   out_2321076618871876345[10] = 0.0;
   out_2321076618871876345[11] = 0.0;
   out_2321076618871876345[12] = 0.0;
   out_2321076618871876345[13] = 0.0;
   out_2321076618871876345[14] = 0.0;
   out_2321076618871876345[15] = 0.0;
   out_2321076618871876345[16] = 0.0;
   out_2321076618871876345[17] = 0.0;
   out_2321076618871876345[18] = 1.0;
   out_2321076618871876345[19] = 0.0;
   out_2321076618871876345[20] = 0.0;
   out_2321076618871876345[21] = 0.0;
   out_2321076618871876345[22] = 0.0;
   out_2321076618871876345[23] = 0.0;
   out_2321076618871876345[24] = 0.0;
   out_2321076618871876345[25] = 0.0;
   out_2321076618871876345[26] = 0.0;
   out_2321076618871876345[27] = 1.0;
   out_2321076618871876345[28] = 0.0;
   out_2321076618871876345[29] = 0.0;
   out_2321076618871876345[30] = 0.0;
   out_2321076618871876345[31] = 0.0;
   out_2321076618871876345[32] = 0.0;
   out_2321076618871876345[33] = 0.0;
   out_2321076618871876345[34] = 0.0;
   out_2321076618871876345[35] = 0.0;
   out_2321076618871876345[36] = 1.0;
   out_2321076618871876345[37] = 0.0;
   out_2321076618871876345[38] = 0.0;
   out_2321076618871876345[39] = 0.0;
   out_2321076618871876345[40] = 0.0;
   out_2321076618871876345[41] = 0.0;
   out_2321076618871876345[42] = 0.0;
   out_2321076618871876345[43] = 0.0;
   out_2321076618871876345[44] = 0.0;
   out_2321076618871876345[45] = 1.0;
   out_2321076618871876345[46] = 0.0;
   out_2321076618871876345[47] = 0.0;
   out_2321076618871876345[48] = 0.0;
   out_2321076618871876345[49] = 0.0;
   out_2321076618871876345[50] = 0.0;
   out_2321076618871876345[51] = 0.0;
   out_2321076618871876345[52] = 0.0;
   out_2321076618871876345[53] = 0.0;
   out_2321076618871876345[54] = 1.0;
   out_2321076618871876345[55] = 0.0;
   out_2321076618871876345[56] = 0.0;
   out_2321076618871876345[57] = 0.0;
   out_2321076618871876345[58] = 0.0;
   out_2321076618871876345[59] = 0.0;
   out_2321076618871876345[60] = 0.0;
   out_2321076618871876345[61] = 0.0;
   out_2321076618871876345[62] = 0.0;
   out_2321076618871876345[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_3121253294902994323) {
   out_3121253294902994323[0] = state[0];
   out_3121253294902994323[1] = state[1];
   out_3121253294902994323[2] = state[2];
   out_3121253294902994323[3] = state[3];
   out_3121253294902994323[4] = state[4];
   out_3121253294902994323[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3121253294902994323[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3121253294902994323[7] = state[7];
}
void F_fun(double *state, double dt, double *out_4840371344081398094) {
   out_4840371344081398094[0] = 1;
   out_4840371344081398094[1] = 0;
   out_4840371344081398094[2] = 0;
   out_4840371344081398094[3] = 0;
   out_4840371344081398094[4] = 0;
   out_4840371344081398094[5] = 0;
   out_4840371344081398094[6] = 0;
   out_4840371344081398094[7] = 0;
   out_4840371344081398094[8] = 0;
   out_4840371344081398094[9] = 1;
   out_4840371344081398094[10] = 0;
   out_4840371344081398094[11] = 0;
   out_4840371344081398094[12] = 0;
   out_4840371344081398094[13] = 0;
   out_4840371344081398094[14] = 0;
   out_4840371344081398094[15] = 0;
   out_4840371344081398094[16] = 0;
   out_4840371344081398094[17] = 0;
   out_4840371344081398094[18] = 1;
   out_4840371344081398094[19] = 0;
   out_4840371344081398094[20] = 0;
   out_4840371344081398094[21] = 0;
   out_4840371344081398094[22] = 0;
   out_4840371344081398094[23] = 0;
   out_4840371344081398094[24] = 0;
   out_4840371344081398094[25] = 0;
   out_4840371344081398094[26] = 0;
   out_4840371344081398094[27] = 1;
   out_4840371344081398094[28] = 0;
   out_4840371344081398094[29] = 0;
   out_4840371344081398094[30] = 0;
   out_4840371344081398094[31] = 0;
   out_4840371344081398094[32] = 0;
   out_4840371344081398094[33] = 0;
   out_4840371344081398094[34] = 0;
   out_4840371344081398094[35] = 0;
   out_4840371344081398094[36] = 1;
   out_4840371344081398094[37] = 0;
   out_4840371344081398094[38] = 0;
   out_4840371344081398094[39] = 0;
   out_4840371344081398094[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4840371344081398094[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4840371344081398094[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4840371344081398094[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4840371344081398094[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4840371344081398094[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4840371344081398094[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4840371344081398094[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4840371344081398094[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4840371344081398094[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4840371344081398094[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4840371344081398094[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4840371344081398094[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4840371344081398094[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4840371344081398094[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4840371344081398094[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4840371344081398094[56] = 0;
   out_4840371344081398094[57] = 0;
   out_4840371344081398094[58] = 0;
   out_4840371344081398094[59] = 0;
   out_4840371344081398094[60] = 0;
   out_4840371344081398094[61] = 0;
   out_4840371344081398094[62] = 0;
   out_4840371344081398094[63] = 1;
}
void h_25(double *state, double *unused, double *out_7726019570887069113) {
   out_7726019570887069113[0] = state[6];
}
void H_25(double *state, double *unused, double *out_167167667836240578) {
   out_167167667836240578[0] = 0;
   out_167167667836240578[1] = 0;
   out_167167667836240578[2] = 0;
   out_167167667836240578[3] = 0;
   out_167167667836240578[4] = 0;
   out_167167667836240578[5] = 0;
   out_167167667836240578[6] = 1;
   out_167167667836240578[7] = 0;
}
void h_24(double *state, double *unused, double *out_7198177503220940015) {
   out_7198177503220940015[0] = state[4];
   out_7198177503220940015[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1429556732031980710) {
   out_1429556732031980710[0] = 0;
   out_1429556732031980710[1] = 0;
   out_1429556732031980710[2] = 0;
   out_1429556732031980710[3] = 0;
   out_1429556732031980710[4] = 1;
   out_1429556732031980710[5] = 0;
   out_1429556732031980710[6] = 0;
   out_1429556732031980710[7] = 0;
   out_1429556732031980710[8] = 0;
   out_1429556732031980710[9] = 0;
   out_1429556732031980710[10] = 0;
   out_1429556732031980710[11] = 0;
   out_1429556732031980710[12] = 0;
   out_1429556732031980710[13] = 1;
   out_1429556732031980710[14] = 0;
   out_1429556732031980710[15] = 0;
}
void h_30(double *state, double *unused, double *out_3052468125618195096) {
   out_3052468125618195096[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9000012830596608914) {
   out_9000012830596608914[0] = 0;
   out_9000012830596608914[1] = 0;
   out_9000012830596608914[2] = 0;
   out_9000012830596608914[3] = 0;
   out_9000012830596608914[4] = 1;
   out_9000012830596608914[5] = 0;
   out_9000012830596608914[6] = 0;
   out_9000012830596608914[7] = 0;
}
void h_26(double *state, double *unused, double *out_448062804809497445) {
   out_448062804809497445[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5871760822775449851) {
   out_5871760822775449851[0] = 0;
   out_5871760822775449851[1] = 0;
   out_5871760822775449851[2] = 0;
   out_5871760822775449851[3] = 0;
   out_5871760822775449851[4] = 0;
   out_5871760822775449851[5] = 0;
   out_5871760822775449851[6] = 0;
   out_5871760822775449851[7] = 1;
}
void h_27(double *state, double *unused, double *out_2500112355051744166) {
   out_2500112355051744166[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7712430842759983602) {
   out_7712430842759983602[0] = 0;
   out_7712430842759983602[1] = 0;
   out_7712430842759983602[2] = 0;
   out_7712430842759983602[3] = 1;
   out_7712430842759983602[4] = 0;
   out_7712430842759983602[5] = 0;
   out_7712430842759983602[6] = 0;
   out_7712430842759983602[7] = 0;
}
void h_29(double *state, double *unused, double *out_2449152240941717579) {
   out_2449152240941717579[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1826143340622549702) {
   out_1826143340622549702[0] = 0;
   out_1826143340622549702[1] = 1;
   out_1826143340622549702[2] = 0;
   out_1826143340622549702[3] = 0;
   out_1826143340622549702[4] = 0;
   out_1826143340622549702[5] = 0;
   out_1826143340622549702[6] = 0;
   out_1826143340622549702[7] = 0;
}
void h_28(double *state, double *unused, double *out_1845099704568526585) {
   out_1845099704568526585[0] = state[5];
   out_1845099704568526585[1] = state[6];
}
void H_28(double *state, double *unused, double *out_2953325574806219215) {
   out_2953325574806219215[0] = 0;
   out_2953325574806219215[1] = 0;
   out_2953325574806219215[2] = 0;
   out_2953325574806219215[3] = 0;
   out_2953325574806219215[4] = 0;
   out_2953325574806219215[5] = 1;
   out_2953325574806219215[6] = 0;
   out_2953325574806219215[7] = 0;
   out_2953325574806219215[8] = 0;
   out_2953325574806219215[9] = 0;
   out_2953325574806219215[10] = 0;
   out_2953325574806219215[11] = 0;
   out_2953325574806219215[12] = 0;
   out_2953325574806219215[13] = 0;
   out_2953325574806219215[14] = 1;
   out_2953325574806219215[15] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7530412064838234300) {
  err_fun(nom_x, delta_x, out_7530412064838234300);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1203665384329719755) {
  inv_err_fun(nom_x, true_x, out_1203665384329719755);
}
void car_H_mod_fun(double *state, double *out_2321076618871876345) {
  H_mod_fun(state, out_2321076618871876345);
}
void car_f_fun(double *state, double dt, double *out_3121253294902994323) {
  f_fun(state,  dt, out_3121253294902994323);
}
void car_F_fun(double *state, double dt, double *out_4840371344081398094) {
  F_fun(state,  dt, out_4840371344081398094);
}
void car_h_25(double *state, double *unused, double *out_7726019570887069113) {
  h_25(state, unused, out_7726019570887069113);
}
void car_H_25(double *state, double *unused, double *out_167167667836240578) {
  H_25(state, unused, out_167167667836240578);
}
void car_h_24(double *state, double *unused, double *out_7198177503220940015) {
  h_24(state, unused, out_7198177503220940015);
}
void car_H_24(double *state, double *unused, double *out_1429556732031980710) {
  H_24(state, unused, out_1429556732031980710);
}
void car_h_30(double *state, double *unused, double *out_3052468125618195096) {
  h_30(state, unused, out_3052468125618195096);
}
void car_H_30(double *state, double *unused, double *out_9000012830596608914) {
  H_30(state, unused, out_9000012830596608914);
}
void car_h_26(double *state, double *unused, double *out_448062804809497445) {
  h_26(state, unused, out_448062804809497445);
}
void car_H_26(double *state, double *unused, double *out_5871760822775449851) {
  H_26(state, unused, out_5871760822775449851);
}
void car_h_27(double *state, double *unused, double *out_2500112355051744166) {
  h_27(state, unused, out_2500112355051744166);
}
void car_H_27(double *state, double *unused, double *out_7712430842759983602) {
  H_27(state, unused, out_7712430842759983602);
}
void car_h_29(double *state, double *unused, double *out_2449152240941717579) {
  h_29(state, unused, out_2449152240941717579);
}
void car_H_29(double *state, double *unused, double *out_1826143340622549702) {
  H_29(state, unused, out_1826143340622549702);
}
void car_h_28(double *state, double *unused, double *out_1845099704568526585) {
  h_28(state, unused, out_1845099704568526585);
}
void car_H_28(double *state, double *unused, double *out_2953325574806219215) {
  H_28(state, unused, out_2953325574806219215);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
