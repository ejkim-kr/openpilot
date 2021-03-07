
extern "C"{

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

}
extern "C" {
#include <math.h>
/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_879539959337219109) {
   out_879539959337219109[0] = delta_x[0] + nom_x[0];
   out_879539959337219109[1] = delta_x[1] + nom_x[1];
   out_879539959337219109[2] = delta_x[2] + nom_x[2];
   out_879539959337219109[3] = delta_x[3] + nom_x[3];
   out_879539959337219109[4] = delta_x[4] + nom_x[4];
   out_879539959337219109[5] = delta_x[5] + nom_x[5];
   out_879539959337219109[6] = delta_x[6] + nom_x[6];
   out_879539959337219109[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8481518425670246390) {
   out_8481518425670246390[0] = -nom_x[0] + true_x[0];
   out_8481518425670246390[1] = -nom_x[1] + true_x[1];
   out_8481518425670246390[2] = -nom_x[2] + true_x[2];
   out_8481518425670246390[3] = -nom_x[3] + true_x[3];
   out_8481518425670246390[4] = -nom_x[4] + true_x[4];
   out_8481518425670246390[5] = -nom_x[5] + true_x[5];
   out_8481518425670246390[6] = -nom_x[6] + true_x[6];
   out_8481518425670246390[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_8655776821554035084) {
   out_8655776821554035084[0] = 1.0;
   out_8655776821554035084[1] = 0.0;
   out_8655776821554035084[2] = 0.0;
   out_8655776821554035084[3] = 0.0;
   out_8655776821554035084[4] = 0.0;
   out_8655776821554035084[5] = 0.0;
   out_8655776821554035084[6] = 0.0;
   out_8655776821554035084[7] = 0.0;
   out_8655776821554035084[8] = 0.0;
   out_8655776821554035084[9] = 1.0;
   out_8655776821554035084[10] = 0.0;
   out_8655776821554035084[11] = 0.0;
   out_8655776821554035084[12] = 0.0;
   out_8655776821554035084[13] = 0.0;
   out_8655776821554035084[14] = 0.0;
   out_8655776821554035084[15] = 0.0;
   out_8655776821554035084[16] = 0.0;
   out_8655776821554035084[17] = 0.0;
   out_8655776821554035084[18] = 1.0;
   out_8655776821554035084[19] = 0.0;
   out_8655776821554035084[20] = 0.0;
   out_8655776821554035084[21] = 0.0;
   out_8655776821554035084[22] = 0.0;
   out_8655776821554035084[23] = 0.0;
   out_8655776821554035084[24] = 0.0;
   out_8655776821554035084[25] = 0.0;
   out_8655776821554035084[26] = 0.0;
   out_8655776821554035084[27] = 1.0;
   out_8655776821554035084[28] = 0.0;
   out_8655776821554035084[29] = 0.0;
   out_8655776821554035084[30] = 0.0;
   out_8655776821554035084[31] = 0.0;
   out_8655776821554035084[32] = 0.0;
   out_8655776821554035084[33] = 0.0;
   out_8655776821554035084[34] = 0.0;
   out_8655776821554035084[35] = 0.0;
   out_8655776821554035084[36] = 1.0;
   out_8655776821554035084[37] = 0.0;
   out_8655776821554035084[38] = 0.0;
   out_8655776821554035084[39] = 0.0;
   out_8655776821554035084[40] = 0.0;
   out_8655776821554035084[41] = 0.0;
   out_8655776821554035084[42] = 0.0;
   out_8655776821554035084[43] = 0.0;
   out_8655776821554035084[44] = 0.0;
   out_8655776821554035084[45] = 1.0;
   out_8655776821554035084[46] = 0.0;
   out_8655776821554035084[47] = 0.0;
   out_8655776821554035084[48] = 0.0;
   out_8655776821554035084[49] = 0.0;
   out_8655776821554035084[50] = 0.0;
   out_8655776821554035084[51] = 0.0;
   out_8655776821554035084[52] = 0.0;
   out_8655776821554035084[53] = 0.0;
   out_8655776821554035084[54] = 1.0;
   out_8655776821554035084[55] = 0.0;
   out_8655776821554035084[56] = 0.0;
   out_8655776821554035084[57] = 0.0;
   out_8655776821554035084[58] = 0.0;
   out_8655776821554035084[59] = 0.0;
   out_8655776821554035084[60] = 0.0;
   out_8655776821554035084[61] = 0.0;
   out_8655776821554035084[62] = 0.0;
   out_8655776821554035084[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_7014554748423705679) {
   out_7014554748423705679[0] = state[0];
   out_7014554748423705679[1] = state[1];
   out_7014554748423705679[2] = state[2];
   out_7014554748423705679[3] = state[3];
   out_7014554748423705679[4] = state[4];
   out_7014554748423705679[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7014554748423705679[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7014554748423705679[7] = state[7];
}
void F_fun(double *state, double dt, double *out_6891519007627046374) {
   out_6891519007627046374[0] = 1;
   out_6891519007627046374[1] = 0;
   out_6891519007627046374[2] = 0;
   out_6891519007627046374[3] = 0;
   out_6891519007627046374[4] = 0;
   out_6891519007627046374[5] = 0;
   out_6891519007627046374[6] = 0;
   out_6891519007627046374[7] = 0;
   out_6891519007627046374[8] = 0;
   out_6891519007627046374[9] = 1;
   out_6891519007627046374[10] = 0;
   out_6891519007627046374[11] = 0;
   out_6891519007627046374[12] = 0;
   out_6891519007627046374[13] = 0;
   out_6891519007627046374[14] = 0;
   out_6891519007627046374[15] = 0;
   out_6891519007627046374[16] = 0;
   out_6891519007627046374[17] = 0;
   out_6891519007627046374[18] = 1;
   out_6891519007627046374[19] = 0;
   out_6891519007627046374[20] = 0;
   out_6891519007627046374[21] = 0;
   out_6891519007627046374[22] = 0;
   out_6891519007627046374[23] = 0;
   out_6891519007627046374[24] = 0;
   out_6891519007627046374[25] = 0;
   out_6891519007627046374[26] = 0;
   out_6891519007627046374[27] = 1;
   out_6891519007627046374[28] = 0;
   out_6891519007627046374[29] = 0;
   out_6891519007627046374[30] = 0;
   out_6891519007627046374[31] = 0;
   out_6891519007627046374[32] = 0;
   out_6891519007627046374[33] = 0;
   out_6891519007627046374[34] = 0;
   out_6891519007627046374[35] = 0;
   out_6891519007627046374[36] = 1;
   out_6891519007627046374[37] = 0;
   out_6891519007627046374[38] = 0;
   out_6891519007627046374[39] = 0;
   out_6891519007627046374[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6891519007627046374[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6891519007627046374[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6891519007627046374[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6891519007627046374[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6891519007627046374[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6891519007627046374[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6891519007627046374[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6891519007627046374[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6891519007627046374[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6891519007627046374[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6891519007627046374[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6891519007627046374[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6891519007627046374[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6891519007627046374[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6891519007627046374[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6891519007627046374[56] = 0;
   out_6891519007627046374[57] = 0;
   out_6891519007627046374[58] = 0;
   out_6891519007627046374[59] = 0;
   out_6891519007627046374[60] = 0;
   out_6891519007627046374[61] = 0;
   out_6891519007627046374[62] = 0;
   out_6891519007627046374[63] = 1;
}
void h_25(double *state, double *unused, double *out_5318297026083439055) {
   out_5318297026083439055[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8104691973345650739) {
   out_8104691973345650739[0] = 0;
   out_8104691973345650739[1] = 0;
   out_8104691973345650739[2] = 0;
   out_8104691973345650739[3] = 0;
   out_8104691973345650739[4] = 0;
   out_8104691973345650739[5] = 0;
   out_8104691973345650739[6] = 1;
   out_8104691973345650739[7] = 0;
}
void h_24(double *state, double *unused, double *out_2511437235748253558) {
   out_2511437235748253558[0] = state[4];
   out_2511437235748253558[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6842302909149910607) {
   out_6842302909149910607[0] = 0;
   out_6842302909149910607[1] = 0;
   out_6842302909149910607[2] = 0;
   out_6842302909149910607[3] = 0;
   out_6842302909149910607[4] = 1;
   out_6842302909149910607[5] = 0;
   out_6842302909149910607[6] = 0;
   out_6842302909149910607[7] = 0;
   out_6842302909149910607[8] = 0;
   out_6842302909149910607[9] = 0;
   out_6842302909149910607[10] = 0;
   out_6842302909149910607[11] = 0;
   out_6842302909149910607[12] = 0;
   out_6842302909149910607[13] = 1;
   out_6842302909149910607[14] = 0;
   out_6842302909149910607[15] = 0;
}
void h_30(double *state, double *unused, double *out_5247174396872063376) {
   out_5247174396872063376[0] = state[4];
}
void H_30(double *state, double *unused, double *out_728153189414717597) {
   out_728153189414717597[0] = 0;
   out_728153189414717597[1] = 0;
   out_728153189414717597[2] = 0;
   out_728153189414717597[3] = 0;
   out_728153189414717597[4] = 1;
   out_728153189414717597[5] = 0;
   out_728153189414717597[6] = 0;
   out_728153189414717597[7] = 0;
}
void h_26(double *state, double *unused, double *out_6953118619377314772) {
   out_6953118619377314772[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2400098818406441466) {
   out_2400098818406441466[0] = 0;
   out_2400098818406441466[1] = 0;
   out_2400098818406441466[2] = 0;
   out_2400098818406441466[3] = 0;
   out_2400098818406441466[4] = 0;
   out_2400098818406441466[5] = 0;
   out_2400098818406441466[6] = 0;
   out_2400098818406441466[7] = 1;
}
void h_27(double *state, double *unused, double *out_8126773779965266916) {
   out_8126773779965266916[0] = state[3];
}
void H_27(double *state, double *unused, double *out_559428798421907715) {
   out_559428798421907715[0] = 0;
   out_559428798421907715[1] = 0;
   out_559428798421907715[2] = 0;
   out_559428798421907715[3] = 1;
   out_559428798421907715[4] = 0;
   out_559428798421907715[5] = 0;
   out_559428798421907715[6] = 0;
   out_559428798421907715[7] = 0;
}
void h_29(double *state, double *unused, double *out_7851579717680761027) {
   out_7851579717680761027[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1302711803270253772) {
   out_1302711803270253772[0] = 0;
   out_1302711803270253772[1] = 1;
   out_1302711803270253772[2] = 0;
   out_1302711803270253772[3] = 0;
   out_1302711803270253772[4] = 0;
   out_1302711803270253772[5] = 0;
   out_1302711803270253772[6] = 0;
   out_1302711803270253772[7] = 0;
}
void h_28(double *state, double *unused, double *out_2194679860431949296) {
   out_2194679860431949296[0] = state[5];
   out_2194679860431949296[1] = state[6];
}
void H_28(double *state, double *unused, double *out_175529569086584259) {
   out_175529569086584259[0] = 0;
   out_175529569086584259[1] = 0;
   out_175529569086584259[2] = 0;
   out_175529569086584259[3] = 0;
   out_175529569086584259[4] = 0;
   out_175529569086584259[5] = 1;
   out_175529569086584259[6] = 0;
   out_175529569086584259[7] = 0;
   out_175529569086584259[8] = 0;
   out_175529569086584259[9] = 0;
   out_175529569086584259[10] = 0;
   out_175529569086584259[11] = 0;
   out_175529569086584259[12] = 0;
   out_175529569086584259[13] = 0;
   out_175529569086584259[14] = 1;
   out_175529569086584259[15] = 0;
}
}

extern "C"{
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



extern "C"{

      void update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
      }
    
      void update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
      }
    
      void update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
      }
    
      void update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
      }
    
      void update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
      }
    
      void update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
      }
    
      void update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
      }
    
}
