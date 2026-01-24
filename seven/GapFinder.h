/*
 * GapFinder.h
 *
 * 最遠+隣接センサー方式によるギャップ検出
 * 最も遠いセンサーとその隣接センサーの距離重み付けで目標角度を決定
 *
 * 状態保持:
 * - _lastFarthestIdx: 前回の最遠センサーインデックス
 *   ヒステリシス処理により、センサー切り替えの頻繁な振動を防止
 *   新しいセンサーが FARTHEST_HYSTERESIS (mm) 以上遠い場合のみ切り替え
 */

#ifndef GAP_FINDER_H
#define GAP_FINDER_H

#include <Arduino.h>

#include "Config.h"
#include "SensorReader.h"

// ギャップ検出結果構造体
struct GapResult {
    float target_angle;  // 目標方向角度（度）正=右、負=左
    bool boost_left;     // 左隣接センサーにブースト適用
    bool boost_right;    // 右隣接センサーにブースト適用
};

class GapFinder {
   private:
    int _lastFarthestIdx;  // 前回の最遠センサーインデックス（ヒステリシス用）

    // 最も遠いセンサーを見つける（ヒステリシス適用前）
    int _findFarthestSensor(const SensorData* data, float& outDistance) const;

    // ヒステリシス処理を適用し、最終的な最遠センサーを決定
    int _applyHysteresis(const SensorData* data, int candidateIdx,
                         float candidateDist, float& outDistance);

    // 最遠センサーと隣接センサーから目標角度を計算
    float _calculateTargetAngle(const SensorData* data, int farthestIdx,
                                float farthestDist,
                                bool& outBoostLeft, bool& outBoostRight) const;

   public:
    GapFinder();

    // メイン処理: センサーデータから目標角度を決定
    GapResult find(const SensorData* sensorData);
};

#endif  // GAP_FINDER_H
