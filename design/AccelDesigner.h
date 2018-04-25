#pragma once

#include <cmath>
#include <algorithm>

#include <complex>
typedef std::complex<float> C;

/** @class 加速曲線を生成するクラス
@brief 引数に従って加速曲線を生成する
*/
class AccelCurve {
public:
  /** @constructor
  @param a_max 最大加速度
  @param v_start 始点速度
  @param v_end 終点速度
  */
  AccelCurve(const float a_max, const float v_start, const float v_end) {
    reset(a_max, v_start, v_end);
  }
  AccelCurve() {
    am = t0 = t1 = t2 = t3 = v0 = v1 = v2 = v3 = x0 = x1 = x2 = x3 = tc = tm = 0;
  }
  /** @function reset
  @brief 引数の拘束条件から曲線を生成する．
  この関数によって，すべての変数が初期化される．(漏れはない)
  */
  void reset(const float a_max, const float v_start, const float v_end) {
    tc = calcTimeCurve(a_max); //< 速度が曲線である時間を決定
    am = (v_end - v_start > 0) ? a_max : -a_max; //< 最大加速度の符号を決定
    v0 = v_start; v3 = v_end; //< 代入
    t0 = 0; x0 = 0; //< ここでは初期値はゼロとする
    tm = (v3 - v0) / am - tc; //< 等加速度直線運動の時間を決定
    if (tm > 0) {
      // 速度: 曲線 . 直線 . 曲線
      t1 = t0 + tc;
      t2 = t1 + tm;
      t3 = t2 + tc;
    } else {
      // 速度: 曲線 . 曲線
      t1 = t0 + std::sqrt(tc / am * (v3 - v0)); //< 速度から算出
      t2 = t1;
      t3 = t2 + (t1 - t0);
    }
    v1 = v(t1); v2 = v(t2); //< 式から求めることができる
    x1 = x(t1); x2 = x(t2); //< 式から求めることができる
    x3 = x0 + (v0 + v3) / 2 * (t3 - t0); //< 図中の面積により
  }
  /** @function a
  @brief 加速度
  */
  float a(const float t) const {
    if      (t <= t0) return 0;
    else if (t <= t1) return  1.0f / tc * am * (t - t0);
    else if (t <= t2) return am;
    else if (t <= t3) return -1.0f / tc * am * (t - t3);
    else              return 0;
  }
  /** @function v
  @brief 速度
  */
  float v(const float t) const {
    if      (t <= t0) return v0;
    else if (t <= t1) return v0 + 0.50f / tc * am * (t - t0) * (t - t0);
    else if (t <= t2) return v1 + am * (t - t1);
    else if (t <= t3) return v3 - 0.50f / tc * am * (t - t3) * (t - t3);
    else              return v3;
  }
  /** @function x
  @brief 位置
  */
  float x(const float t) const {
    if      (t <= t0) return x0 + v0 * (t - t0);
    else if (t <= t1) return x0 + v0 * (t - t0) + am / 6 / tc * (t - t0) * (t - t0) * (t - t0);
    else if (t <= t2) return x1 + v1 * (t - t1) + am / 2 * (t - t1) * (t - t1);
    else if (t <= t3) return x3 + v3 * (t - t3) - am / 6 / tc * (t - t3) * (t - t3) * (t - t3);
    else              return x3 + v3 * (t - t3);
  }
  float t_end() const { return t3; }
  float v_end() const { return v3; }
  float x_end() const { return x3; }
  static float calcTimeCurve(const float am) {
    return am / 50000; //< 経験的
    // return 0.0f;
    // return 0.3f;
  }
private:
  float am; //< 最大加速度 [m/s/s]
  float t0, t1, t2, t3; //< 境界点の時刻 [s]
  float v0, v1, v2, v3; //< 境界点の速度 [m/s]
  float x0, x1, x2, x3; //< 境界点の位置 [m]
  float tc; //< 曲線加速の時間 [s]
  float tm; //< 最大加速度の時間 [s]
};

class AccelDesigner{
public:
  AccelDesigner() {
    t1 = t2 = t3 = 0;
    x0 = x3 = 0;
  }
  AccelDesigner(const float a_max, const float v_start, const float v_max, const float v_end, const float x_start, const float x_end) {
    reset(a_max, v_start, v_max, v_end, x_start, x_end);
  }
  void reset(const float a_max, const float v_start, float v_max, float v_end, const float x_start, float x_end) {
    x_end = std::max(x_start, x_end); //< バックはしないように飽和
    const float distance = x_end - x_start; //< 進む距離を算出
    // 飽和処理
    std::cout << "a_max: " << a_max << " v_start: " << v_start << " v_max: " << v_max << " v_end: " << v_end << " distance: " << distance << std::endl;
    const float v_max_d = calcVelocityMax(a_max, v_start, v_end, distance);
    // std::cout << "v_max: " << v_max << " v_max_d: " << v_max_d << std::endl;
    v_max = std::min(v_max, v_max_d);
    if(v_max > v_end && v_max > v_start){
      // 加速 -> 等速 -> 減速 の場合
      ac.reset(a_max, v_start, v_max); //< 加速
      dc.reset(a_max,   v_max, v_end); //< 減速
    } else {
      // 加速のみ or 減速のみ の場合
      // 加速中にx_endに達することのないのような終端速度を計算
      const float v_sat = calcVelocityEndMax(a_max, v_start, v_end, distance);
      // 飽和処理
      if(v_end > v_start) v_end = std::min(v_end, v_sat); //< 加速
      else                v_end = std::max(v_end, v_sat); //< 減速
      // v_end = v_sat;
      v_max = v_end;
      // 曲線を生成
      ac.reset(a_max, v_start, v_end); //< 加速 or 減速
      dc.reset(a_max,   v_end, v_end); //< 何もしない
    }
    // 各定数の算出
    t1 = ac.t_end(); //< 曲線加速終了の時刻
    t2 = ac.t_end() + (distance - ac.x_end() - dc.x_end()) / v_max; //< 等速走行終了の時刻
    t3 = ac.t_end() + (distance - ac.x_end() - dc.x_end()) / v_max + dc.t_end(); //< 曲線減速終了の時刻
    x0 = x_start;
    x3 = x_end;
  }
  float t_end() const {
    return t3;
  }
  float x_end() const {
    return x3;
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
    if (t < t2) return x0 + ac.x(t);
    else        return x3 - dc.x_end() + dc.x(t - t2);
  }
  void printCsv(const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      printf("%f,%f,%f\n", a(t), v(t), x(t));
    }
  }
  void printCsv(std::ofstream& f, const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      f << a(t) << "," << v(t) << "," << x(t) << "\n";
    }
  }
private:
  float t1, t2, t3;
  float x0, x3;
  AccelCurve ac, dc;

  static float calcVelocityMax(const float am, const float vs, const float ve, const float d) {
    const float tc = AccelCurve::calcTimeCurve(am);
    const float D = am * am * tc * tc - 2 * (vs + ve) * am * tc + 4 * am * d + 2 * (vs * vs + ve * ve);
    return (-am * tc + std::sqrt(D)) / 2;
  }
  static float calcVelocityEndMax(float am, const float vs, const float ve, const float d) {
    const float tc = AccelCurve::calcTimeCurve(am);
    am = (ve - vs > 0) ? std::abs(am) : -std::abs(am); //< 最大加速度の符号を決定
    const float tm = (ve - vs) / am - tc; //< 等加速度直線運動の時間を決定
    if (tm > 0) {
      const float D = 4*vs*vs - 4*vs*am*tc + am*(tc*tc*am + 8*d);
      const float ans = (std::sqrt(D) - am*tc)/2;
      return ans;
    } else {
      // const float a = vs;
      // const float b = am*d*d/tc;
      // const float t0 = 3*std::sqrt(96*a*a*a*b + 81*b*b) + 16*a*a*a + 27*b;
      // const float t1 = std::cbrt(t0/2);
      // const float t2 = 4*a*a*std::cbrt(2/t0);
      // const float c1 = (t1+t2-a)/3;
      // return c1;
      // const float a = vs;
      // const float b = am*d*d/tc;
      // const C t0 = 27*(32*a*a*a*b + 27*b*b);
      // const C t1 = std::sqrt(t0) + 16*a*a*a + 27*b;
      // const C t2 = std::pow(t1/C(2,0), 1.0f/3.0f);
      // const C t3 = 4*a*a/t2;
      // const C t4 = t2 + t3 - a;
      // const float ans = t4.real()/3;
      // std::cout << "cvem a: " << a << std::endl;
      // std::cout << "cvem b: " << b << std::endl;
      // std::cout << "cvem ans.i: " << t4.imag()/3 << std::endl;
      // std::cout << "cvem ans: " << ans << std::endl;
      // return ans;
      const float a = vs;
      const float b = am*d*d/tc;
      const float aaa = a*a*a;
      const float t0 = 27*(32*aaa*b + 27*b*b);
      const float t1 = 16*aaa + 27*b;
      if(t0 >= 0) {
        const float t2 = std::cbrt(std::sqrt(t0) + t1) / 2;
        return (t2 + 4*a*a/t2 - a) / 3;
      } else {
        // const C t2(t1/2,  std::sqrt(-t0)/2);
        // const C t3(t1/2, -std::sqrt(-t0)/2);
        // return (std::pow(t2, 1.0f/3) + std::pow(t3, 1.0f/3) - a).real() / 3;
        const auto t2 = std::pow(std::complex<float>(t1/2,  std::sqrt(-t0)/2), 1.0f/3);
        return (t2.real()*2 - a) / 3;
      }
    }
  }
};
