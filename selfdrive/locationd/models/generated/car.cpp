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
void err_fun(double *nom_x, double *delta_x, double *out_5144003432639031458) {
   out_5144003432639031458[0] = delta_x[0] + nom_x[0];
   out_5144003432639031458[1] = delta_x[1] + nom_x[1];
   out_5144003432639031458[2] = delta_x[2] + nom_x[2];
   out_5144003432639031458[3] = delta_x[3] + nom_x[3];
   out_5144003432639031458[4] = delta_x[4] + nom_x[4];
   out_5144003432639031458[5] = delta_x[5] + nom_x[5];
   out_5144003432639031458[6] = delta_x[6] + nom_x[6];
   out_5144003432639031458[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1019709690714833271) {
   out_1019709690714833271[0] = -nom_x[0] + true_x[0];
   out_1019709690714833271[1] = -nom_x[1] + true_x[1];
   out_1019709690714833271[2] = -nom_x[2] + true_x[2];
   out_1019709690714833271[3] = -nom_x[3] + true_x[3];
   out_1019709690714833271[4] = -nom_x[4] + true_x[4];
   out_1019709690714833271[5] = -nom_x[5] + true_x[5];
   out_1019709690714833271[6] = -nom_x[6] + true_x[6];
   out_1019709690714833271[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_3160954694154719337) {
   out_3160954694154719337[0] = 1.0;
   out_3160954694154719337[1] = 0.0;
   out_3160954694154719337[2] = 0.0;
   out_3160954694154719337[3] = 0.0;
   out_3160954694154719337[4] = 0.0;
   out_3160954694154719337[5] = 0.0;
   out_3160954694154719337[6] = 0.0;
   out_3160954694154719337[7] = 0.0;
   out_3160954694154719337[8] = 0.0;
   out_3160954694154719337[9] = 1.0;
   out_3160954694154719337[10] = 0.0;
   out_3160954694154719337[11] = 0.0;
   out_3160954694154719337[12] = 0.0;
   out_3160954694154719337[13] = 0.0;
   out_3160954694154719337[14] = 0.0;
   out_3160954694154719337[15] = 0.0;
   out_3160954694154719337[16] = 0.0;
   out_3160954694154719337[17] = 0.0;
   out_3160954694154719337[18] = 1.0;
   out_3160954694154719337[19] = 0.0;
   out_3160954694154719337[20] = 0.0;
   out_3160954694154719337[21] = 0.0;
   out_3160954694154719337[22] = 0.0;
   out_3160954694154719337[23] = 0.0;
   out_3160954694154719337[24] = 0.0;
   out_3160954694154719337[25] = 0.0;
   out_3160954694154719337[26] = 0.0;
   out_3160954694154719337[27] = 1.0;
   out_3160954694154719337[28] = 0.0;
   out_3160954694154719337[29] = 0.0;
   out_3160954694154719337[30] = 0.0;
   out_3160954694154719337[31] = 0.0;
   out_3160954694154719337[32] = 0.0;
   out_3160954694154719337[33] = 0.0;
   out_3160954694154719337[34] = 0.0;
   out_3160954694154719337[35] = 0.0;
   out_3160954694154719337[36] = 1.0;
   out_3160954694154719337[37] = 0.0;
   out_3160954694154719337[38] = 0.0;
   out_3160954694154719337[39] = 0.0;
   out_3160954694154719337[40] = 0.0;
   out_3160954694154719337[41] = 0.0;
   out_3160954694154719337[42] = 0.0;
   out_3160954694154719337[43] = 0.0;
   out_3160954694154719337[44] = 0.0;
   out_3160954694154719337[45] = 1.0;
   out_3160954694154719337[46] = 0.0;
   out_3160954694154719337[47] = 0.0;
   out_3160954694154719337[48] = 0.0;
   out_3160954694154719337[49] = 0.0;
   out_3160954694154719337[50] = 0.0;
   out_3160954694154719337[51] = 0.0;
   out_3160954694154719337[52] = 0.0;
   out_3160954694154719337[53] = 0.0;
   out_3160954694154719337[54] = 1.0;
   out_3160954694154719337[55] = 0.0;
   out_3160954694154719337[56] = 0.0;
   out_3160954694154719337[57] = 0.0;
   out_3160954694154719337[58] = 0.0;
   out_3160954694154719337[59] = 0.0;
   out_3160954694154719337[60] = 0.0;
   out_3160954694154719337[61] = 0.0;
   out_3160954694154719337[62] = 0.0;
   out_3160954694154719337[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_7809129704521149226) {
   out_7809129704521149226[0] = state[0];
   out_7809129704521149226[1] = state[1];
   out_7809129704521149226[2] = state[2];
   out_7809129704521149226[3] = state[3];
   out_7809129704521149226[4] = state[4];
   out_7809129704521149226[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7809129704521149226[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7809129704521149226[7] = state[7];
}
void F_fun(double *state, double dt, double *out_3511533356282032530) {
   out_3511533356282032530[0] = 1;
   out_3511533356282032530[1] = 0;
   out_3511533356282032530[2] = 0;
   out_3511533356282032530[3] = 0;
   out_3511533356282032530[4] = 0;
   out_3511533356282032530[5] = 0;
   out_3511533356282032530[6] = 0;
   out_3511533356282032530[7] = 0;
   out_3511533356282032530[8] = 0;
   out_3511533356282032530[9] = 1;
   out_3511533356282032530[10] = 0;
   out_3511533356282032530[11] = 0;
   out_3511533356282032530[12] = 0;
   out_3511533356282032530[13] = 0;
   out_3511533356282032530[14] = 0;
   out_3511533356282032530[15] = 0;
   out_3511533356282032530[16] = 0;
   out_3511533356282032530[17] = 0;
   out_3511533356282032530[18] = 1;
   out_3511533356282032530[19] = 0;
   out_3511533356282032530[20] = 0;
   out_3511533356282032530[21] = 0;
   out_3511533356282032530[22] = 0;
   out_3511533356282032530[23] = 0;
   out_3511533356282032530[24] = 0;
   out_3511533356282032530[25] = 0;
   out_3511533356282032530[26] = 0;
   out_3511533356282032530[27] = 1;
   out_3511533356282032530[28] = 0;
   out_3511533356282032530[29] = 0;
   out_3511533356282032530[30] = 0;
   out_3511533356282032530[31] = 0;
   out_3511533356282032530[32] = 0;
   out_3511533356282032530[33] = 0;
   out_3511533356282032530[34] = 0;
   out_3511533356282032530[35] = 0;
   out_3511533356282032530[36] = 1;
   out_3511533356282032530[37] = 0;
   out_3511533356282032530[38] = 0;
   out_3511533356282032530[39] = 0;
   out_3511533356282032530[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3511533356282032530[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3511533356282032530[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3511533356282032530[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3511533356282032530[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3511533356282032530[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3511533356282032530[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3511533356282032530[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3511533356282032530[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3511533356282032530[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3511533356282032530[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3511533356282032530[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3511533356282032530[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3511533356282032530[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3511533356282032530[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3511533356282032530[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3511533356282032530[56] = 0;
   out_3511533356282032530[57] = 0;
   out_3511533356282032530[58] = 0;
   out_3511533356282032530[59] = 0;
   out_3511533356282032530[60] = 0;
   out_3511533356282032530[61] = 0;
   out_3511533356282032530[62] = 0;
   out_3511533356282032530[63] = 1;
}
void h_25(double *state, double *unused, double *out_6300983868460679792) {
   out_6300983868460679792[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6936622825500736383) {
   out_6936622825500736383[0] = 0;
   out_6936622825500736383[1] = 0;
   out_6936622825500736383[2] = 0;
   out_6936622825500736383[3] = 0;
   out_6936622825500736383[4] = 0;
   out_6936622825500736383[5] = 0;
   out_6936622825500736383[6] = 1;
   out_6936622825500736383[7] = 0;
}
void h_24(double *state, double *unused, double *out_2249814897033317498) {
   out_2249814897033317498[0] = state[4];
   out_2249814897033317498[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2468436691888653813) {
   out_2468436691888653813[0] = 0;
   out_2468436691888653813[1] = 0;
   out_2468436691888653813[2] = 0;
   out_2468436691888653813[3] = 0;
   out_2468436691888653813[4] = 1;
   out_2468436691888653813[5] = 0;
   out_2468436691888653813[6] = 0;
   out_2468436691888653813[7] = 0;
   out_2468436691888653813[8] = 0;
   out_2468436691888653813[9] = 0;
   out_2468436691888653813[10] = 0;
   out_2468436691888653813[11] = 0;
   out_2468436691888653813[12] = 0;
   out_2468436691888653813[13] = 1;
   out_2468436691888653813[14] = 0;
   out_2468436691888653813[15] = 0;
}
void h_30(double *state, double *unused, double *out_4685201440999983499) {
   out_4685201440999983499[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2677276085448446897) {
   out_2677276085448446897[0] = 0;
   out_2677276085448446897[1] = 0;
   out_2677276085448446897[2] = 0;
   out_2677276085448446897[3] = 0;
   out_2677276085448446897[4] = 1;
   out_2677276085448446897[5] = 0;
   out_2677276085448446897[6] = 0;
   out_2677276085448446897[7] = 0;
}
void h_26(double *state, double *unused, double *out_316471826660862146) {
   out_316471826660862146[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5805528093269605960) {
   out_5805528093269605960[0] = 0;
   out_5805528093269605960[1] = 0;
   out_5805528093269605960[2] = 0;
   out_5805528093269605960[3] = 0;
   out_5805528093269605960[4] = 0;
   out_5805528093269605960[5] = 0;
   out_5805528093269605960[6] = 0;
   out_5805528093269605960[7] = 1;
}
void h_27(double *state, double *unused, double *out_1299310600799834147) {
   out_1299310600799834147[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3964858073285072209) {
   out_3964858073285072209[0] = 0;
   out_3964858073285072209[1] = 0;
   out_3964858073285072209[2] = 0;
   out_3964858073285072209[3] = 1;
   out_3964858073285072209[4] = 0;
   out_3964858073285072209[5] = 0;
   out_3964858073285072209[6] = 0;
   out_3964858073285072209[7] = 0;
}
void h_29(double *state, double *unused, double *out_5288517325676461016) {
   out_5288517325676461016[0] = state[1];
}
void H_29(double *state, double *unused, double *out_544954434057577975) {
   out_544954434057577975[0] = 0;
   out_544954434057577975[1] = 1;
   out_544954434057577975[2] = 0;
   out_544954434057577975[3] = 0;
   out_544954434057577975[4] = 0;
   out_544954434057577975[5] = 0;
   out_544954434057577975[6] = 0;
   out_544954434057577975[7] = 0;
}
void h_28(double *state, double *unused, double *out_2606527359166809232) {
   out_2606527359166809232[0] = state[5];
   out_2606527359166809232[1] = state[6];
}
void H_28(double *state, double *unused, double *out_8960474903584402937) {
   out_8960474903584402937[0] = 0;
   out_8960474903584402937[1] = 0;
   out_8960474903584402937[2] = 0;
   out_8960474903584402937[3] = 0;
   out_8960474903584402937[4] = 0;
   out_8960474903584402937[5] = 1;
   out_8960474903584402937[6] = 0;
   out_8960474903584402937[7] = 0;
   out_8960474903584402937[8] = 0;
   out_8960474903584402937[9] = 0;
   out_8960474903584402937[10] = 0;
   out_8960474903584402937[11] = 0;
   out_8960474903584402937[12] = 0;
   out_8960474903584402937[13] = 0;
   out_8960474903584402937[14] = 1;
   out_8960474903584402937[15] = 0;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5144003432639031458) {
  err_fun(nom_x, delta_x, out_5144003432639031458);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1019709690714833271) {
  inv_err_fun(nom_x, true_x, out_1019709690714833271);
}
void car_H_mod_fun(double *state, double *out_3160954694154719337) {
  H_mod_fun(state, out_3160954694154719337);
}
void car_f_fun(double *state, double dt, double *out_7809129704521149226) {
  f_fun(state,  dt, out_7809129704521149226);
}
void car_F_fun(double *state, double dt, double *out_3511533356282032530) {
  F_fun(state,  dt, out_3511533356282032530);
}
void car_h_25(double *state, double *unused, double *out_6300983868460679792) {
  h_25(state, unused, out_6300983868460679792);
}
void car_H_25(double *state, double *unused, double *out_6936622825500736383) {
  H_25(state, unused, out_6936622825500736383);
}
void car_h_24(double *state, double *unused, double *out_2249814897033317498) {
  h_24(state, unused, out_2249814897033317498);
}
void car_H_24(double *state, double *unused, double *out_2468436691888653813) {
  H_24(state, unused, out_2468436691888653813);
}
void car_h_30(double *state, double *unused, double *out_4685201440999983499) {
  h_30(state, unused, out_4685201440999983499);
}
void car_H_30(double *state, double *unused, double *out_2677276085448446897) {
  H_30(state, unused, out_2677276085448446897);
}
void car_h_26(double *state, double *unused, double *out_316471826660862146) {
  h_26(state, unused, out_316471826660862146);
}
void car_H_26(double *state, double *unused, double *out_5805528093269605960) {
  H_26(state, unused, out_5805528093269605960);
}
void car_h_27(double *state, double *unused, double *out_1299310600799834147) {
  h_27(state, unused, out_1299310600799834147);
}
void car_H_27(double *state, double *unused, double *out_3964858073285072209) {
  H_27(state, unused, out_3964858073285072209);
}
void car_h_29(double *state, double *unused, double *out_5288517325676461016) {
  h_29(state, unused, out_5288517325676461016);
}
void car_H_29(double *state, double *unused, double *out_544954434057577975) {
  H_29(state, unused, out_544954434057577975);
}
void car_h_28(double *state, double *unused, double *out_2606527359166809232) {
  h_28(state, unused, out_2606527359166809232);
}
void car_H_28(double *state, double *unused, double *out_8960474903584402937) {
  H_28(state, unused, out_8960474903584402937);
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
