#include "coordinate_utils.h"
#include <utility>

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
  *accel_c = {ax_c, ay_c, yaw_accel_c};  // Accel in Cartesian frame

  delete csp;
}

void ToFrenet(const Car& car_c, const WayPoints& wp, Car* car_f) {
  Pose pose_f;
  Twist twist_f;
  Accel accel_f;
  utils::ToFrenet(car_c.getPose(), car_c.getTwist(), car_c.getAccel(), wp,
                  &pose_f, &twist_f, &accel_f);
  car_f->setPose(pose_f);
  car_f->setTwist(twist_f);
  car_f->setAccel(accel_f);
}

void ToCartesian(const Car& car_f, const WayPoints& wp, Car* car_c) {
  Pose pose_c;
  Twist twist_c;
  Accel accel_c;
  utils::ToCartesian(car_f.getPose(), car_f.getTwist(), car_f.getAccel(), wp,
                     &pose_c, &twist_c, &accel_c);
  car_c->setPose(pose_c);
  car_c->setTwist(twist_c);
  car_c->setAccel(accel_c);
}

void ToFrenet(const Obstacle& ob_c, const WayPoints& wp,
              std::unique_ptr<Obstacle>& ob_f) {
  Pose pose_f;
  Twist twist_f;
  Accel accel_f;
  ToFrenet(ob_c.getPose(), ob_c.getTwist(), {0, 0, 0}, wp, &pose_f, &twist_f,
           &accel_f);
  ob_f = std::make_unique<Obstacle>(pose_f, twist_f, ob_c.getLength(),
                                    ob_c.getWidth(), ob_c.getClearence());
  ob_f->setSpeedLookupTable(ob_c.getSpeedLookupTable());
  ob_f->setLaneIds(ob_c.getLaneIds());
  // note(sheng): conversion of predict poses is not implemented here
}

void ToCartesian(const Obstacle& ob_f, const WayPoints& wp,
                 std::unique_ptr<Obstacle>& ob_c) {
  Pose pose_c;
  Twist twist_c;
  Accel accel_c;
  ToCartesian(ob_f.getPose(), ob_f.getTwist(), {0, 0, 0}, wp, &pose_c, &twist_c,
              &accel_c);
  ob_c = std::make_unique<Obstacle>(pose_c, twist_c, ob_f.getLength(),
                                    ob_f.getWidth(), ob_f.getClearence());
  ob_c->setSpeedLookupTable(ob_f.getSpeedLookupTable());
  ob_c->setLaneIds(ob_f.getLaneIds());

  // conversion of predict poses to Cartesian
  const auto& predict_poses_f = ob_f.getPredictPoses();
  auto predict_poses_c = ob_c->mutablePredictPoses();
  predict_poses_c->clear();
  for (const auto& predict_pose_f : predict_poses_f) {
    Pose predict_pose_c;
    Twist dummy_twist;
    Accel dummy_accel;
    ToCartesian(predict_pose_f.second, {0, 0, 0}, {0, 0, 0}, wp,
                &predict_pose_c, &dummy_twist, &dummy_accel);
    (*predict_poses_c)[predict_pose_f.first] = predict_pose_c;
  }
  ob_c->UpdatePredictBoxes();
}

void ShiftWaypoints(const WayPoints& ref_wp, double offset, WayPoints* wp) {
  if (wp == nullptr) {
    throw std::invalid_argument("wp is nullptr");
  }

  (*wp)[0].clear();
  (*wp)[1].clear();

  const int wp_size = ref_wp[0].size();
  CubicSpline2D csp = CubicSpline2D(ref_wp[0], ref_wp[1]);
  for (int i = 0; i < wp_size; ++i) {
    double s = csp.find_s(ref_wp[0][i], ref_wp[1][i]);

    double x = csp.calc_x(s);
    double y = csp.calc_y(s);
    double yaw = csp.calc_yaw(s);

    double x_shift = x - offset * sin(yaw);
    double y_shift = y + offset * cos(yaw);
    (*wp)[0].push_back(x_shift);
    (*wp)[1].push_back(y_shift);
  }
}

}  // namespace utils
