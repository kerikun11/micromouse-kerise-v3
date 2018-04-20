#pragma once

#include <cmath>
#include <algorithm>

/** @class 加速曲線を生成するクラス
*/
class AccelCurve {
public:
  AccelCurve() {}
  AccelCurve(const float a_max, const float v_start, const float v_end) {
    reset(a_max, v_start, v_end);
  }
  void reset(const float a_max, const float v_start, const float v_end) {
    tv = calcTimeVariable(a_max); //< 速度が曲線である時間を決定
    am = (v_end - v_start > 0) ? a_max : -a_max; //< 最大加速度の符号を決定
    v0 = v_start;
    v3 = v_end;
    t0 = 0;
    x0 = 0;
    tc = (v3 - v0) / am - tv; //< 等加速度直線運動の時間を決定
    if (tc > 0) {
      // 速度: 曲線 . 直線 . 曲線
      t1 = t0 + tv;
      t2 = t1 + tc;
      t3 = t2 + tv;
      v1 = v0 + 0.5f * am * tv; //< 加速度を積分
      v2 = v1 + am * tc; //< 加速度を積分
    } else {
      // 速度: 曲線 . 曲線
      t1 = t0 + std::sqrt(tv / am * (v3 - v0));
      t2 = t1;
      t3 = t2 + t1 - t0;
      v1 = (v0 + v3) / 2;
      v2 = v1;
    }
    x1 = x(t1);
    x2 = x(t2);
    x3 = x0 + (v0 + v3) / 2 * (t3 - t0); //< 対称性
  }
  float a(const float t) const {
    if      (t <= t0) return 0;
    else if (t <= t1) return 1.0f / tv * am * (t - t0);
    else if (t <= t2) return am;
    else if (t <= t3) return -1.0f / tv * am * (t - t3);
    else              return 0;
  }
  float v(const float t) const {
    if      (t <= t0) return v0;
    else if (t <= t1) return v0 + 0.50f / tv * am * (t - t0) * (t - t0);
    else if (t <= t2) return v1 + am * (t - t1);
    else if (t <= t3) return v0 + am * (tv + tc) - 0.5f / tv * am * (t - t3) * (t - t3);
    else              return v3;
  }
  float x(const float t) const {
    if      (t <= t0) return x0 + v0 * (t - t0);
    else if (t <= t1) return x0 + v0 * (t - t0) + 1.0f / 6 / tv * am * std::pow(t - t0, 3);
    else if (t <= t2) return x1 + v1 * (t - t1) + 0.5f * am * (t - t1) * (t - t1);
    else if (t <= t3) return x2 + (v0 + am * (tv + tc)) * (t - t2) - 1.0f / 6 / tv * am * (std::pow(t - t3, 3) - std::pow(t2 - t3, 3));
    else              return x3 + v3 * (t - t3);
  }
  float t_end() const { return t3; }
  float v_end() const { return v3; }
  float x_end() const { return x3; }
  static float calcVelocityMax(const float am, const float vs, const float ve, const float d) {
    const float tv = AccelCurve::calcTimeVariable(am);
    return (-am * tv + std::sqrt(am * am * tv * tv - 2 * (vs + ve) * am * tv + 4 * am * d + 2 * (vs * vs + ve * ve))) / 2;
  }
  static float calcVelocityEndMax(const float am, const float vs, const float ve, const float d) {
    const float tv = AccelCurve::calcTimeVariable(am);
    const float tc = (ve - vs) / am - tv; //< 等加速度直線運動の時間を決定
    if (tc > 0) {
      const float c2 = (std::sqrt(4*vs*vs - 4*vs*am*tv + am*(tv*tv*am + 8*d)) - am*tv)/2;
      return c2;
    } else {
      const float a = vs;
      const float b = am*d*d/tv;
      const float t0 = 3*std::sqrt(96*a*a*a*b + 81*b*b) + 16*a*a*a + 27*b;
      const float t1 = std::cbrt(t0/2);
      const float t2 = 4*a*a*std::cbrt(2/t0);
      const float c1 = (t1+t2-a)/3;
      return c1;
    }
  }
  static float calcTimeVariable(const float am) {
    return am / 50000; //< 経験的
  }
private:
  float am; //< max acceleration [mm/s/s]
  float t0, t1, t2, t3; //< time point [s]
  float v0, v1, v2, v3; //< velocity point [mm/s]
  float x0, x1, x2, x3; //< position point [mm]
  float tv; //< variation time [s]
  float tc; //< constant time [s]
};

class AccelDesigner{
public:
  AccelDesigner() {}
  AccelDesigner(const float a_max, const float v_start, const float v_max, const float v_end, const float distance) {
    reset(a_max, v_start, v_max, v_end, distance);
  }
  void reset(const float a_max, const float v_start, float v_max, float v_end, const float distance) {
    d = distance;
    v_max = std::min(v_max, AccelCurve::calcVelocityMax(a_max, v_start, v_end, distance));
    if(v_max > v_end && v_max > v_start){
      ac.reset(a_max, v_start, v_max); //< 加速
      dc.reset(a_max, v_max, v_end);   //< 減速
    } else {
      const float v_sat = AccelCurve::calcVelocityEndMax(a_max, v_start, v_end, distance);
      if(v_end > v_start) v_end = std::min(v_end, v_sat);
      else                v_end = std::max(v_end, v_sat);
      ac.reset(a_max, v_start, v_end); //< 加速 or 減速
      dc.reset(a_max, v_end, v_end);   //< 何もしない
    }
    t1 = ac.t_end();
    t2 = ac.t_end() + (d - ac.x_end() - dc.x_end()) / v_max;
    t3 = ac.t_end() + (d - ac.x_end() - dc.x_end()) / v_max + dc.t_end();
  }
  float t_end() const {
    return t3;
  }
  float x_end() const {
    return d;
  }
  float v_end() const {
    return dc.v_end();
  }
  float a(const float t) const {
    if (t < t2) return ac.a(t);
    else        return dc.a(t - t2);
  }
  float v(const float t) const {
    if (t < t2) return ac.v(t);
    else        return dc.v(t - t2);
  }
  float x(const float t) const {
    if (t < t2) return ac.x(t);
    else        return d - dc.x_end() + dc.x(t - t2);
  }
  void printCsv(const float offset = 0.0f, const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      printf("%f,%f,%f\n", a(t), v(t), x(t) + offset);
    }
  }
private:
  float t1, t2, t3;
  float d;
  AccelCurve ac, dc;
};
