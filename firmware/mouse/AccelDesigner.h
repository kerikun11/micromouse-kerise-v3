#pragma once

#include <cmath>
#include <algorithm>

class AccelCurve {
public:
  AccelCurve(const float a_max, const float v_start, const float v_end) {
    tv = calcTimeVariable(a_max);
    am = (v_end - v_start > 0) ? a_max : -a_max; //< 加速度の符号を決定
    v0 = v_start;
    v3 = v_end;
    t0 = 0;
    x0 = 0;
    tc = (v3 - v0) / am - tv; //< 等加速度直線運動の時間を決定
    if (tc > 0) {
      // 速度: 曲線 -> 直線 -> 曲線
      t1 = t0 + tv;
      t2 = t1 + tc;
      t3 = t2 + tv;
      v1 = v0 + 0.5f * am * tv;
      v2 = v0 + 0.5f * am * tv + am * tc;
    } else {
      // 速度: 曲線 -> 曲線
      t1 = t0 + sqrt(tv / am * (v3 - v0));
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
    return 0;
  }
  float v(const float t) const {
    if      (t <= t0) return v0;
    else if (t <= t1) return v0 + 0.50f / tv * am * (t - t0) * (t - t0);
    else if (t <= t2) return v1 + am * (t - t1);
    else if (t <= t3) return v0 + am * (tv + tc) - 0.5f / tv * am * (t - t3) * (t - t3);
    else              return v3;
    return 0;
  }
  float x(const float t) const {
    if      (t <= t0) return x0 + v0 * (t - t0);
    else if (t <= t1) return x0 + v0 * (t - t0) + 1.0f / 6 / tv * am * (t - t0) * (t - t0) * (t - t0);
    else if (t <= t2) return x1 + v1 * (t - t1) + 0.5f * am * (t - t1) * (t - t1);
    else if (t <= t3) return x2 + (v0 + am * (tv + tc)) * (t - t2) - 1.0f / 6 / tv * am * ((t - t3) * (t - t3) * (t - t3) - (t2 - t3) * (t2 - t3) * (t2 - t3));
    else              return x3 + v3 * (t - t3);
  }
  float t_end() const {
    return t3;
  }
  float x_end() const {
    return x3;
  }
  static float calcVelocityMax(const float tv, const float am, const float vs, const float ve, const float x) {
    return (-am * tv + sqrt(am * am * tv * tv - 2 * (vs + ve) * am * tv + 4 * am * x + 2 * (vs * vs + ve * ve))) / 2;
  }
  static float calcTimeVariable(const float am) {
    return am / 100000; //< 経験的
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
  AccelDesigner(const float a_max, const float v_start, float v_max, const float v_end, const float distance) {
    float tv = AccelCurve::calcTimeVariable(a_max);
    d = distance;
    v_max = std::min(v_max, AccelCurve::calcVelocityMax(tv, a_max, v_start, v_end, distance));
    if(v_max > v_end && v_max > v_start){
      // 加速 -> 減速
      ac = new AccelCurve(a_max, v_start, v_max);
      dc = new AccelCurve(a_max, v_max, v_end);
    } else {
      // 加速のみ or 減速のみ
      ac = new AccelCurve(a_max, v_start, v_end);
      dc = new AccelCurve(a_max, v_end, v_end);
    }
    t1 = ac->t_end();
    t2 = ac->t_end() + (d - ac->x_end() - dc->x_end()) / v_max;
    t3 = ac->t_end() + (d - ac->x_end() - dc->x_end()) / v_max + dc->t_end();
  }
  ~AccelDesigner(){
    delete ac;
    delete dc;
  }
  float t_end() const {
    return t3;
  }
  float x_end() const {
    return d;
  }
  float a(const float t) const {
    if (t < t2) return ac->a(t);
    else        return dc->a(t - t2);
  }
  float v(const float t) const {
    if (t < t2) return ac->v(t);
    else        return dc->v(t - t2);
  }
  float x(const float t) const {
    if (t < t2) return ac->x(t);
    else        return d - dc->x_end() + dc->x(t - t2);
  }
  void printCsv(const float offset = 0.0f, const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      printf("%f,%f,%f\n", a(t), v(t), x(t) + offset);
    }
  }
private:
  float t1, t2, t3;
  float d;
  AccelCurve *ac, *dc;
};
