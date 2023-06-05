#include "coordinate_utils.h"

namespace utils {

void ToFrenet(const Pose& pose_c, const Twist& twist_c, const Accel& accel_c,
              WayPoints wp, Pose* pose_f, Twist* twist_f, Accel* accel_f) {
  const double x_c = pose_c.x;
  const double y_c = pose_c.y;
  const double yaw_c = pose_c.yaw;

  CubicSpline2D* csp = new CubicSpline2D(wp[0], wp[1]);
  double s_f = csp->find_s(x_c, y_c);

  // nearest waypoint on the spline
  double x_nr = csp->calc_x(s_f);
  double y_nr = csp->calc_y(s_f);
  double yaw_nr = csp->calc_yaw(s_f);
  double kappa_nr = csp->calc_curvature(s_f);
  double dx_over_ds_nr = csp->calc_dx_over_ds(s_f);
  double dy_over_ds_nr = csp->calc_dy_over_ds(s_f);

  std::tuple<double, double> unit_vec_vx_nr(
      dx_over_ds_nr / norm(dx_over_ds_nr, dy_over_ds_nr),
      dy_over_ds_nr / norm(dx_over_ds_nr, dy_over_ds_nr));
  std::tuple<double, double> unit_vec_vy_nr(-get<1>(unit_vec_vx_nr),
                                            get<0>(unit_vec_vx_nr));
  std::tuple<double, double> vec_d(x_c - x_nr, y_c - y_nr);
  double d_f = utils::cross_product(unit_vec_vx_nr, vec_d);  // signed
  double yaw_f = yaw_c - yaw_nr;
  *pose_f = {s_f, d_f, yaw_f};  // Pose in Frenet frame

  const double vx_c = twist_c.vx;
  const double vy_c = twist_c.vy;
  const double yaw_rate_c = twist_c.yaw_rate;
  std::tuple<double, double> unit_vec_vx_c(cos(yaw_c), sin(yaw_c));
  std::tuple<double, double> unit_vec_vy_c(-sin(yaw_c), cos(yaw_c));
  double s_d_f = utils::dot(unit_vec_vx_c, unit_vec_vx_nr) * vx_c +
                 utils::dot(unit_vec_vy_c, unit_vec_vx_nr) * vy_c;
  double d_d_f = utils::dot(unit_vec_vx_c, unit_vec_vy_nr) * vx_c +
                 utils::dot(unit_vec_vy_c, unit_vec_vy_nr) * vy_c;
  double yaw_d_f = yaw_rate_c - kappa_nr * s_d_f;
  *twist_f = {s_d_f, d_d_f, yaw_d_f};  // Twist in Frenet frame

  const double ax_c = accel_c.ax;
  const double ay_c = accel_c.ay;
  const double yaw_accel_c = accel_c.yaw_accel;
  double s_dd_f = utils::dot(unit_vec_vx_c, unit_vec_vx_nr) * ax_c +
                  utils::dot(unit_vec_vy_c, unit_vec_vx_nr) * ay_c;
  double d_dd_f = utils::dot(unit_vec_vx_c, unit_vec_vy_nr) * ax_c +
                  utils::dot(unit_vec_vy_c, unit_vec_vy_nr) * ay_c;
  double yaw_dd_f = yaw_accel_c - kappa_nr * s_dd_f;  // ignore kappa derivative
  *accel_f = {s_dd_f, d_dd_f, yaw_dd_f};              // Accel in Frenet frame

  delete csp;
}

void ToCartesian(const Pose& pose_f, const Twist& twist_f, const Accel& accel_f,
                 WayPoints wp, Pose* pose_c, Twist* twist_c, Accel* accel_c) {
  const double s_f = pose_f.x;
  const double d_f = pose_f.y;
  const double yaw_f = pose_f.yaw;

  CubicSpline2D* csp = new CubicSpline2D(wp[0], wp[1]);
  double x_nr = csp->calc_x(s_f);
  double y_nr = csp->calc_y(s_f);
  double yaw_nr = csp->calc_yaw(s_f);
  double kappa_nr = csp->calc_curvature(s_f);
  double dx_over_ds_nr = csp->calc_dx_over_ds(s_f);
  double dy_over_ds_nr = csp->calc_dy_over_ds(s_f);
  std::tuple<double, double> unit_vec_vx_nr(
      dx_over_ds_nr / norm(dx_over_ds_nr, dy_over_ds_nr),
      dy_over_ds_nr / norm(dx_over_ds_nr, dy_over_ds_nr));
  std::tuple<double, double> unit_vec_vy_nr(-get<1>(unit_vec_vx_nr),
                                            get<0>(unit_vec_vx_nr));

  double x_c = x_nr + d_f * get<0>(unit_vec_vy_nr);
  double y_c = y_nr + d_f * get<1>(unit_vec_vy_nr);
  double yaw_c = yaw_f + yaw_nr;
  *pose_c = {x_c, y_c, yaw_c};  // Pose in Cartesian frame

  const double s_d_f = twist_f.vx;
  const double d_d_f = twist_f.vy;
  const double yaw_d_f = twist_f.yaw_rate;
  std::tuple<double, double> unit_vec_vx_c(cos(yaw_c), sin(yaw_c));
  std::tuple<double, double> unit_vec_vy_c(-sin(yaw_c), cos(yaw_c));
  double vx_c = utils::dot(unit_vec_vx_c, unit_vec_vx_nr) * s_d_f +
                utils::dot(unit_vec_vx_c, unit_vec_vy_nr) * d_d_f;
  double vy_c = utils::dot(unit_vec_vy_c, unit_vec_vx_nr) * s_d_f +
                utils::dot(unit_vec_vy_c, unit_vec_vy_nr) * d_d_f;
  double yaw_rate_c = yaw_d_f + kappa_nr * s_d_f;
  *twist_c = {vx_c, vy_c, yaw_rate_c};  // Twist in Cartesian frame

  const double s_dd_f = accel_f.ax;
  const double d_dd_f = accel_f.ay;
  const double yaw_dd_f = accel_f.yaw_accel;
  double ax_c = utils::dot(unit_vec_vx_c, unit_vec_vx_nr) * s_dd_f +
                utils::dot(unit_vec_vx_c, unit_vec_vy_nr) * d_dd_f;
  double ay_c = utils::dot(unit_vec_vy_c, unit_vec_vx_nr) * s_dd_f +
                utils::dot(unit_vec_vy_c, unit_vec_vy_nr) * d_dd_f;
  double yaw_accel_c = yaw_dd_f + kappa_nr * s_dd_f;  // ignore kappa derivative

  delete csp;
}

}  // namespace utils
