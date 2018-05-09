/** @file AccelDesigner.h
*   @author KERI
*   @date 2018.04.29 created
*   @date 2018.04.29 modified
*   @url https://kerikeri.top
*/
#pragma once

#include <cmath> //< for std::sqrt, std::cbrt, std::pow
#include <algorithm> //< for std::max, std::min
#include <complex> //< for std::complex

/* このファイルに定義されているクラス一覧 */
class AccelCurve;
class AccelDesigner;

/** @class 加速曲線を生成するクラス
*   @brief 引数に従って加速曲線を生成する
*/
class AccelCurve {
public:
  /** @constructor
  *   @param a_max 最大加速度 [mm/s/s]
  *   @param v_start 始点速度 [mm/s]
  *   @param v_end 終点速度 [mm/s]
  */
  AccelCurve(const float a_max, const float v_start, const float v_end) {
    reset(a_max, v_start, v_end);
  }
  /** @constructor
  *   @brief 空のコンストラクタ．あとで reset() により初期化すること．
  */
  AccelCurve() {
    am = t0 = t1 = t2 = t3 = v0 = v1 = v2 = v3 = x0 = x1 = x2 = x3 = tc = tm = 0;
  }
  /** @function reset
  *   @brief 引数の拘束条件から曲線を生成する．
  *   この関数によって，すべての変数が初期化される．(漏れはない)
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
  *   @brief 時刻$t$における加速度$a$
  *   @param t 時刻[s]
  *   @return 加速度[mm/s/s]
  */
  float a(const float t) const {
    if      (t <= t0) return 0;
    else if (t <= t1) return  1.0f / tc * am * (t - t0);
    else if (t <= t2) return am;
    else if (t <= t3) return -1.0f / tc * am * (t - t3);
    else              return 0;
  }
  /** @function v
  *   @brief 時刻$t$における速度$v$
  *   @param t 時刻[s]
  *   @return 速度[mm/s]
  */
  float v(const float t) const {
    if      (t <= t0) return v0;
    else if (t <= t1) return v0 + 0.50f / tc * am * (t - t0) * (t - t0);
    else if (t <= t2) return v1 + am * (t - t1);
    else if (t <= t3) return v3 - 0.50f / tc * am * (t - t3) * (t - t3);
    else              return v3;
  }
  /** @function x
  *   @brief 時刻$t$における位置$x$
  *   @param t 時刻[s]
  *   @return 位置[mm]
  */
  float x(const float t) const {
    if      (t <= t0) return x0 + v0 * (t - t0);
    else if (t <= t1) return x0 + v0 * (t - t0) + am / 6 / tc * (t - t0) * (t - t0) * (t - t0);
    else if (t <= t2) return x1 + v1 * (t - t1) + am / 2 * (t - t1) * (t - t1);
    else if (t <= t3) return x3 + v3 * (t - t3) - am / 6 / tc * (t - t3) * (t - t3) * (t - t3);
    else              return x3 + v3 * (t - t3);
  }
  /** @function xx_end
  *   @brief 終端xx
  */
  float t_end() const { return t3; }
  float v_end() const { return v3; }
  float x_end() const { return x3; }
  /** @function calcTimeCurve
  *   @brief 曲線加速部分の時間を決定する関数
  *   @param am 最大加速度の大きさ
  */
  static float calcTimeCurve(const float am) {
    const float ad = 500000; //< 躍度 [mm/s/s/s]
    const float tc = std::abs(am) / ad; //< 時間を算出
    return tc;
  }
  /** @function calcVelocityEnd
  *   @brief 走行距離から達しうる終点速度を算出する関数
  *   @param am 最大加速度の大きさ [mm/s/s]
  *   @param vs 始点速度 [mm/s]
  *   @param vt 目標速度 [mm/s]
  *   @param d 走行距離 [mm]
  *   @return vm 最大速度 [mm/s]
  */
  static float calcVelocityEnd(float am, const float vs, const float vt, const float d) {
    const float tc = AccelCurve::calcTimeCurve(am);
    am = (vt - vs > 0) ? std::abs(am) : -std::abs(am); //< 最大加速度の符号を決定
    const float tm = (vt - vs) / am - tc; //< 等加速度直線運動の時間を決定
    std::cout << "tm: " << tm << std::endl;
    float ve;
    // if (tm > 0) {
    //   ve = ((am>0?1:-1)*std::sqrt(4*vs*vs - 4*vs*am*tc + am*(tc*tc*am + 8*d)) - am*tc)/2; //< 2次方程式の解
    // } else {
      const float a = vs;
      const float b = am*d*d/tc;
      const float aaa = a*a*a;
      const float c0 = 27*(32*aaa*b + 27*b*b);
      const float c1 = 16*aaa + 27*b;
      if(c0 >= 0) {
        // ルートの中が非負のとき
        const float c2 = std::cbrt(std::sqrt(c0) + c1) / 2;
        ve = (c2 + 4*a*a/c2 - a) / 3; //< 3次方程式の解
        std::cout << "c2-p: " << c2 << std::endl;
      } else {
        // ルートの中が負のとき
        const auto c2 = std::pow(std::complex<float>(c1/2,  std::sqrt(-c0)/2), 1.0f/3);
        ve = (c2.real()*2 - a) / 3; //< 3次方程式の解
        std::cout << "c2-n: " << c2 << std::endl;
      }
    // }
    return (vt > vs) ? std::min(vt, ve) : std::max(vt, ve); //< 加速と減速で飽和方向が異なる
  }
  /** @function calcVelocityMax
  *   @brief 走行距離から達しうる最大速度を算出する関数
  *   @param am 最大加速度の大きさ [mm/s/s]
  *   @param vs 始点速度 [mm/s]
  *   @param ve 終点速度 [mm/s]
  *   @param d 走行距離 [mm]
  *   @return vm 最大速度 [mm/s]
  */
  static float calcVelocityMax(const float am, const float vs, const float va, const float ve, const float d) {
    const float tc = AccelCurve::calcTimeCurve(am);
    const float D = am * am * tc * tc - 2 * (vs + ve) * am * tc + 4 * am * d + 2 * (vs * vs + ve * ve);
    const float vm = (-am * tc + std::sqrt(D)) / 2; //< 2次方程式の解
    return std::max({vm, vs, ve}); //< 無駄な減速を避ける
    return std::min(vm, va); //< 飽和速度で飽和
  }
private:
  float am; //< 最大加速度 [m/s/s]
  float t0, t1, t2, t3; //< 境界点の時刻 [s]
  float v0, v1, v2, v3; //< 境界点の速度 [m/s]
  float x0, x1, x2, x3; //< 境界点の位置 [m]
  float tc; //< 曲線加速の時間 [s]
  float tm; //< 最大加速度の時間 [s]
};

/** @class 加減速曲線を生成するクラス
*   @brief 引数に従って速度計画をし，加減速曲線を生成する
*/
class AccelDesigner{
public:
  /** @constructor
  *   @param a_max 最大加速度 [mm/s/s]
  *   @param v_start 始点速度 [mm/s]
  *   @param v_end 終点速度 [mm/s]
  */
  AccelDesigner(const float a_max, const float v_start, const float v_sat, const float v_target, const float distance, const float x_start = 0) {
    reset(a_max, v_start, v_sat, v_target, distance, x_start);
  }
  /** @constructor
  *   @brief 空のコンストラクタ．あとで reset() により初期化すること．
  */
  AccelDesigner() { t1 = t2 = t3 = x0 = x3 = 0; }
  /** @function reset
  *   @brief 引数の拘束条件から曲線を生成する．
  *   この関数によって，すべての変数が初期化される．(漏れはない)
  */
  void reset(const float a_max, const float v_start, const float v_sat, const float v_target, const float distance, const float x_start = 0) {
    // 走行距離から終点速度$v_e$を算出
    const float v_end = AccelCurve::calcVelocityEnd(a_max, v_start, v_target, distance);
    // 走行距離から最大速度$v_m$を算出
    const float v_max = AccelCurve::calcVelocityMax(a_max, v_start, v_sat, v_end, distance);
    // 曲線を生成
    ac.reset(a_max, v_start, v_max); //< 加速
    dc.reset(a_max,   v_max, v_end); //< 減速
    // 各定数の算出
    x0 = x_start;
    x3 = x_start + distance;
    t1 = ac.t_end(); //< 曲線加速終了の時刻
    t2 = ac.t_end() + (distance - ac.x_end() - dc.x_end()) / v_max; //< 等速走行終了の時刻
    t3 = ac.t_end() + (distance - ac.x_end() - dc.x_end()) / v_max + dc.t_end(); //< 曲線減速終了の時刻
    // 表示
    std::cout << "a_max: " << a_max << " v_start: " << v_start << " v_max: " << v_max << " v_end: " << v_end << " v_target: " << v_target << " distance: " << distance << std::endl;
    std::cout << "t1: " << t1 << " t2: " << t2 << " t3: " << t3 << std::endl;
    std::cout << "ac.x_end: " << ac.x_end() << " dc.x_end: " << dc.x_end() << std::endl;
  }
  /** @function a
  *   @brief 時刻$t$における加速度$a$
  *   @param t 時刻[s]
  *   @return 加速度[mm/s/s]
  */
  float a(const float t) const {
    if (t < t2) return ac.a(t);
    else        return dc.a(t - t2);
  }
  /** @function v
  *   @brief 時刻$t$における速度$v$
  *   @param t 時刻[s]
  *   @return 速度[mm/s]
  */
  float v(const float t) const {
    if (t < t2) return ac.v(t);
    else        return dc.v(t - t2);
  }
  /** @function x
  *   @brief 時刻$t$における位置$x$
  *   @param t 時刻[s]
  *   @return 位置[mm]
  */
  float x(const float t) const {
    if (t < t2) return x0 + ac.x(t);
    else        return x3 - dc.x_end() + dc.x(t - t2);
  }
  /** @function xx_end
  *   @brief 終端xx
  */
  float t_end() const { return t3; }
  float v_end() const { return dc.v_end(); }
  float x_end() const { return x3; }
  /** @function printCsv
  *   @brief stdoutに軌道のcsvを出力する関数．
  */
  void printCsv(const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      printf("%f,%f,%f\n", a(t), v(t), x(t));
    }
  }
  /** @function printCsv
  *   @brief std::ofstream に軌道のcsvを出力する関数．
  */
  void printCsv(std::ofstream& f, const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      f << a(t) << "," << v(t) << "," << x(t) << "\n";
    }
  }
private:
  float t1, t2, t3; //< 境界点の時刻 [s]
  float x0, x3; //< 境界点の位置 [mm]
  AccelCurve ac, dc; //< 曲線加速，曲線減速
};
