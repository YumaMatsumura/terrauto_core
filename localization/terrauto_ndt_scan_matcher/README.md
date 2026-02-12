# terrauto_ndt_scan_matcher

## Purpose / Role

`terrauto_ndt_scan_matcher`は、CUDAベースのNDT（Normal Distributions Transform）によるScan Matchingノードである。

本ノードは以下を実行する：

- GPUで高速に位置合わせ
- フィットネススコアの計算
- スコアに応じた共分散の自動スケーリング
- `PoseWithCovarianceStamped`の出力
- 必要に応じてTFの発行

## Input topics

| Name | Type | Descreiption |
| --- | --- | --- |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 初期姿勢 |
| `input_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 外部からの姿勢更新入力 |
| `map` | `PointCloudAdaptedType` | 基準となる地図点群 |
| `points` | `PointCloudAdaptedType` | LiDAR入力点群 |

## Output topics

| Name | Type | Descreiption |
| --- | --- | --- |
| `output_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 推定姿勢 |

## Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| `publish_tf` | [-] | bool | TFをpublishするか | false |
| `set_initial_pose` | [-] | bool | 起動時に初期姿勢を設定するか | true |
| `min_map_points` | [points] | int | map点群の最小点数 | 300 |
| `min_sensor_points` | [points] | int | 入力点群の最小点数 | 100 |
| `ndt.max_iterations` | [-] | int | NDT最大反復回数 | 35 |
| `ndt.resolution` | [m] | double | NDTボクセル解像度 | 1.0 |
| `ndt.step_size` | [-] | double | NDTステップサイズ | 0.1 |
| `ndt.transform_epsilon` | [-] | double | 収束判定閾値 | 0.01 |
| `score.threshold` | [m] | double | フィットネス許容閾値 | 2.0 |
| `score.cutoff_ratio` | [-] | double | 上位除外割合 | 0.1 |
| `initial_pose.x` | [m] | double | 初期位置X | 0.0 |
| `initial_pose.y` | [m] | double | 初期位置Y | 0.0 |
| `initial_pose.z` | [m] | double | 初期位置Z | 0.0 |
| `initial_pose.qx` | [-] | double | 初期姿勢Qx | 0.0 |
| `initial_pose.qy` | [-] | double | 初期姿勢Qy | 0.0 |
| `initial_pose.qz` | [-] | double | 初期姿勢Qz | 0.0 |
| `initial_pose.qw` | [-] | double | 初期姿勢Qw | 1.0 |
| `cov.base.xy` | [m^2] | double | XY方向基準分散 | 0.25 |
| `cov.base.z` | [m^2] | double | Z方向基準分散 | 0.25 |
| `cov.base.rpy` | [rad^2] | double | RPY基準分散 | 0.0075 |
| `cov.scale_max` | [-] | double | 最大スケール倍率 | 100.0 |
| `global_frame_id` | [-] | string | グローバル座標系 | map |
| `base_frame_id` | [-] | string | ロボット基準座標系 | base_link |
