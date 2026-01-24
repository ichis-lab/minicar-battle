/*
 * GapFinder.cpp
 *
 * 最遠+隣接センサー方式によるギャップ検出（実装）
 *
 * アルゴリズム概要:
 * 1. 有効な7センサーから距離が最も遠い1つを選択（ヒステリシス付き）
 * 2. その隣接センサー（左右）を取得（端の場合は片側のみ）
 * 3. 三角形面積按分で目標角度を計算（両側隣接あり）
 *    または距離重み付けで計算（片側のみ）
 *
 * 三角形面積按分:
 * - 左三角形面積 = d_left × d_farthest × sin(角度差)
 * - 右三角形面積 = d_farthest × d_right × sin(角度差)
 * - target = (左角度 × 左面積 + 右角度 × 右面積) / (左面積 + 右面積)
 *
 * 状態保持:
 * - _lastFarthestIdx: 前回の最遠センサーインデックス
 *   毎ループ更新され、次回のヒステリシス判定に使用
 *   センサーノイズや微小な距離変動による頻繁な切り替えを防止
 *
 * 注: Ld（ルックアヘッド距離）はSteeringControllerで正面センサー距離から計算
 */

#include "GapFinder.h"

#include <math.h>

GapFinder::GapFinder() : _lastFarthestIdx(FRONT_SENSOR_INDEX) {}

// ============================================================================
// プライベートメソッド
// ============================================================================

int GapFinder::_findFarthestSensor(const SensorData* data,
                                  float& outDistance) const {
    int farthest_idx = -1;
    float farthest_dist = 0.0f;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        if (data[i].valid && data[i].distance > farthest_dist) {
            farthest_dist = data[i].distance;
            farthest_idx = i;
        }
    }

    outDistance = farthest_dist;
    return farthest_idx;
}

int GapFinder::_applyHysteresis(const SensorData* data, int candidateIdx,
                               float candidateDist, float& outDistance) {
    int result_idx = candidateIdx;
    float result_dist = candidateDist;

    // 前回の最遠センサーが有効で、新しい最遠との差がヒステリシス未満なら切り替えない
    if (_lastFarthestIdx >= 0 && _lastFarthestIdx < NUM_SENSORS &&
        data[_lastFarthestIdx].valid && candidateIdx != _lastFarthestIdx) {
        float diff = candidateDist - data[_lastFarthestIdx].distance;
        if (diff < FARTHEST_HYSTERESIS) {
            // 切り替えない：前回の最遠センサーを維持
            result_idx = _lastFarthestIdx;
            result_dist = data[_lastFarthestIdx].distance;
        }
    }

    // 状態を更新
    _lastFarthestIdx = result_idx;
    outDistance = result_dist;
    return result_idx;
}

float GapFinder::_calculateTargetAngle(const SensorData* data, int farthestIdx,
                                      float farthestDist,
                                      bool& outBoostLeft, bool& outBoostRight) const {
    // ブーストフラグ初期化
    outBoostLeft = false;
    outBoostRight = false;

    // 隣接センサーのインデックスを決定
    int left_idx = (farthestIdx > 0) ? farthestIdx - 1 : -1;
    int right_idx = (farthestIdx < NUM_SENSORS - 1) ? farthestIdx + 1 : -1;

    bool has_left = (left_idx >= 0 && data[left_idx].valid);
    bool has_right = (right_idx >= 0 && data[right_idx].valid);

    if (has_left && has_right) {
        // 両側に隣接センサーあり: 三角形面積按分
        // 面積 = d1 × d2 × sin(角度差)、(1/2)は比率計算で消えるので省略
        float angle_diff_left =
            fabs(SENSOR_ANGLES[farthestIdx] - SENSOR_ANGLES[left_idx]);
        float angle_diff_right =
            fabs(SENSOR_ANGLES[right_idx] - SENSOR_ANGLES[farthestIdx]);

        float area_left = data[left_idx].distance * farthestDist *
                          sin(angle_diff_left * DEG_TO_RAD);
        float area_right = farthestDist * data[right_idx].distance *
                           sin(angle_diff_right * DEG_TO_RAD);

        // 近い隣接センサーの重みをブースト（コーナー脱出時のイン突き改善）
        if (data[left_idx].distance < CLOSE_NEIGHBOR_THRESHOLD) {
            area_left *= CLOSE_NEIGHBOR_BOOST;
            outBoostLeft = true;
        }
        if (data[right_idx].distance < CLOSE_NEIGHBOR_THRESHOLD) {
            area_right *= CLOSE_NEIGHBOR_BOOST;
            outBoostRight = true;
        }

        float total_area = area_left + area_right;
        if (total_area < 0.001f) {
            // ゼロ除算防止: 最遠センサーの角度を返す
            return SENSOR_ANGLES[farthestIdx];
        }
        return (SENSOR_ANGLES[left_idx] * area_left +
                SENSOR_ANGLES[right_idx] * area_right) / total_area;
    } else {
        // 片側のみ or 隣接なし: 距離重み付け
        float weighted_sum = SENSOR_ANGLES[farthestIdx] * farthestDist;
        float weight_total = farthestDist;

        if (has_left) {
            weighted_sum += SENSOR_ANGLES[left_idx] * data[left_idx].distance;
            weight_total += data[left_idx].distance;
        }
        if (has_right) {
            weighted_sum += SENSOR_ANGLES[right_idx] * data[right_idx].distance;
            weight_total += data[right_idx].distance;
        }

        if (weight_total < 0.001f) {
            // ゼロ除算防止: 最遠センサーの角度を返す
            return SENSOR_ANGLES[farthestIdx];
        }
        return weighted_sum / weight_total;
    }
}

// ============================================================================
// パブリックメソッド
// ============================================================================

GapResult GapFinder::find(const SensorData* data) {
    GapResult result = {0.0f, false, false};

    // Step 1: 最も遠いセンサーを見つける
    float farthest_dist;
    int farthest_idx = _findFarthestSensor(data, farthest_dist);

    // 有効なセンサーがない場合は直進
    if (farthest_idx < 0) {
        return result;
    }

    // Step 2: ヒステリシス処理を適用
    farthest_idx = _applyHysteresis(data, farthest_idx, farthest_dist,
                                   farthest_dist);

    // Step 3: 目標角度を計算（ブースト適用情報も取得）
    result.target_angle = _calculateTargetAngle(data, farthest_idx,
                                               farthest_dist,
                                               result.boost_left,
                                               result.boost_right);

    return result;
}
