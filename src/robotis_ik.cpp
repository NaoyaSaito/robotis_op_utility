#include <robotis_op_utility/robotis_ik.h>

namespace robotis {

bool RobotisIk::computeIK(double out[], Affine3& af) {
  const Vector3 tr = af.translation();
  const Matrix3 ro = af.rotation();
  const Vector3 eu = ro.eulerAngles(0, 1, 2);
  const double x = tr[0], y = tr[1], z = tr[2];
  const double a = eu[0], b = eu[1], c = eu[2];

  Affine3 Tad, Tda, Tcd, Tdc, Tac;
  Vector3 vec;
  double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
  const double LEG_LENGTH = 219.5;   // [mm]
  const double THIGH_LENGTH = 93.0;  // [mm]
  const double CALF_LENGTH = 93.0;   // [mm]
  const double ANKLE_LENGTH = 33.5;  // [mm]

  std::cout << x << ", " << y << ", " << z << ", " << a << ", " << b << ", " << c << std::endl;

  // Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH),
  //                  Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 /
  // PI));
  Tad = Translation3(x, y, z - LEG_LENGTH) * ro;

  vec.x() = x + Tad(2, 0) * ANKLE_LENGTH;
  vec.y() = y + Tad(3, 1) * ANKLE_LENGTH;
  vec.z() = (z - LEG_LENGTH) + Tad(2, 2) * ANKLE_LENGTH;

  // Get Knee
  _Rac = vec.norm();
  _Acos = acos(
      (_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH)
      /
      (2 * THIGH_LENGTH * CALF_LENGTH));
  std::cout << _Rac << std::endl;
  if (std::isnan(_Acos) == 1) return false;
  out[3] = _Acos;

  // Get Ankle Roll
  // Tda = Tad;
  // if (Tda.inverse() == false) return false;
  Tda = Tad.inverse();
  _k = sqrt(Tda(3, 1) * Tda(3, 1) + Tda(3, 2) * Tda(3, 2));
  _l = sqrt(Tda(3, 1) * Tda(3, 1) +
            (Tda(3, 2) - ANKLE_LENGTH) * (Tda(3, 2) - ANKLE_LENGTH));
  _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) /
       (2 * _l * ANKLE_LENGTH);
  if (_m > 1.0)
    _m = 1.0;
  else if (_m < -1.0)
    _m = -1.0;
  _Acos = acos(_m);
  if (std::isnan(_Acos) == 1) return false;
  if (Tda(3, 1) < 0.0)
    out[5] = -_Acos;
  else
    out[5] = _Acos;

  // Get Hip Yaw
  // Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH),
  //                  Vector3D(out[5] * 180.0 / PI, 0, 0));
  Tcd = Translation3(0, 0, -ANKLE_LENGTH) * rpy2mat(rad2deg(out[5]), 0, 0);
  // Tdc = Tcd;
  // if (Tdc.inverse() == false) return false;
  Tdc = Tcd.inverse();
  Tac = Tad * Tdc;
  _Atan = atan2(-Tac(1, 0), Tac(1, 1));
  std::cout << _Atan << std::endl;
  if (std::isinf(_Atan) == 1) return false;
  out[0] = _Atan;

  // Get Hip Roll
  _Atan = atan2(Tac(1, 2), -Tac(1, 0) * sin(out[0]) + Tac(1, 1) *
  cos(out[0]));
  if (std::isinf(_Atan) == 1) return false;
  out[1] = _Atan;

  // Get Hip Pitch and Ankle Pitch
  _Atan = atan2(Tac(2, 0) * cos(out[0]) + Tac(2, 1) * sin(out[0]),
                Tac(0, 0) * cos(out[0]) + Tac(0, 1) * sin(out[0]));
  if (std::isinf(_Atan) == 1) return false;
  _theta = _Atan;
  _k = sin(out[3]) * CALF_LENGTH;
  _l = -THIGH_LENGTH - cos(out[3]) * CALF_LENGTH;
  _m = cos(out[0]) * vec.x() + sin(out[0]) * vec.y();
  _n = cos(out[1]) * vec.z() + sin(out[0]) * sin(out[1]) * vec.x() -
       cos(out[0]) * sin(out[1]) * vec.y();
  _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
  _c = (_n - _k * _s) / _l;
  _Atan = atan2(_s, _c);
  if (std::isinf(_Atan) == 1) return false;
  out[2] = _Atan;
  out[4] = _theta - out[3] - out[2];

  return true;
}

bool RobotisIk::computeIKold(double out[], Affine3& af) {
  const Vector3 tr = af.translation()*1e3;
  const Matrix3 ro = af.rotation();
  const Vector3 eu = ro.eulerAngles(0, 1, 2);
  const double x = tr[0], y = tr[1], z = tr[2];
  // const double a = eu[0], b = eu[1], c = eu[2];
  const double a = 0.0, b = 0.0, c = 0.0;

  Matrix3D Tad, Tda, Tcd, Tdc, Tac;
  Vector3D vec;
  double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
  const double LEG_LENGTH = 219.5;   // [mm]
  const double THIGH_LENGTH = 93.0;  // [mm]
  const double CALF_LENGTH = 93.0;   // [mm]
  const double ANKLE_LENGTH = 33.5;  // [mm]

  Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH),
                   Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI));

  // std::cout << x << ", " << y << ", " << z << ", " << rad2deg(a) << ", " << rad2deg(b) << ", " << rad2deg(c) << std::endl;

  vec.X = x + Tad.m[2] * ANKLE_LENGTH;
  vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
  vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;
  // std::cout << vec.X << ", " << vec.Y << ", " << vec.Z << std::endl;

  // Get Knee
  _Rac = vec.Length();
  _Acos = acos(
      (_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) /
      (2 * THIGH_LENGTH * CALF_LENGTH));
  //  std::cout << _Acos << std::endl;
  if (std::isnan(_Acos) == 1) return false;
  *(out + 3) = _Acos;

  // Get Ankle Roll
  Tda = Tad;
  if (Tda.Inverse() == false) return false;
  _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
  _l = sqrt(Tda.m[7] * Tda.m[7] +
            (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
  _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) /
       (2 * _l * ANKLE_LENGTH);
  if (_m > 1.0)
    _m = 1.0;
  else if (_m < -1.0)
    _m = -1.0;
  _Acos = acos(_m);
  if (std::isnan(_Acos) == 1) return false;
  if (Tda.m[7] < 0.0)
    *(out + 5) = -_Acos;
  else
    *(out + 5) = _Acos;

  // Get Hip Yaw
  Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH),
                   Vector3D(*(out + 5) * 180.0 / PI, 0, 0));
  Tdc = Tcd;
  if (Tdc.Inverse() == false) return false;
  Tac = Tad * Tdc;
  _Atan = atan2(-Tac.m[1], Tac.m[5]);
  if (std::isinf(_Atan) == 1) return false;
  *(out) = _Atan;

  // Get Hip Roll
  _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
  if (std::isinf(_Atan) == 1) return false;
  *(out + 1) = _Atan;

  // Get Hip Pitch and Ankle Pitch
  _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)),
                Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
  if (std::isinf(_Atan) == 1) return false;
  _theta = _Atan;
  _k = sin(*(out + 3)) * CALF_LENGTH;
  _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
  _m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
  _n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X -
       cos(*(out)) * sin(*(out + 1)) * vec.Y;
  _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
  _c = (_n - _k * _s) / _l;
  _Atan = atan2(_s, _c);
  if (std::isinf(_Atan) == 1) return false;
  *(out + 2) = _Atan;
  *(out + 4) = _theta - *(out + 3) - *(out + 2);

  std::cout << x << ", " << y << ", " << z << ", " << rad2deg(a) << ", " << rad2deg(b) << ", " << rad2deg(c) << ", " << out[0] << ", " << out[1] << ", " << out[2] << ", " << out[3] << ", " << out[4] << ", " << out[5] << std::endl;

  return true;
}

}  // namespace robotis