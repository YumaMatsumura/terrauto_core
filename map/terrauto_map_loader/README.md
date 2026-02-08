# terrauto_map_loader

## Purpose / Role

3次元点群マップ（PCDファイル）をロードし、ROS 2の`sensor_msgs/msg/PointCloud2`として配信するノードを提供する。

本パッケージでは、以下の処理を行うことが可能である。

- PCDファイルのロード
- マップ全体に対する平行移動・回転の適用
- ダウンサンプリング（任意）
- latched（`transient_local`）配信によるマップ共有

## Map Loader

### Output topics

| Name | Type | Description |
| --- | --- | --- |
| /map | sensor_msgs/msg/PointCloud2 | ロードした3次元点群マップ |

> `/map`トピックは**transient_local QoS**で配信されるため、後から起動したノードもマップを受信可能

### Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| use_voxel_grid_filter | [-] | bool | ダウンサンプリングを行うかどうか | true |
| voxel_leaf_x | [m] | double | ダウンサンプリングのリーフサイズ（x） | 0.1 |
| voxel_leaf_y | [m] | double | ダウンサンプリングのリーフサイズ（y） | 0.1 |
| voxel_leaf_z | [m] | double | ダウンサンプリングのリーフサイズ（z） | 0.1 |
| translation_x | [m] | double | マップ全体の平行移動量（x） | 0.0 |
| translation_y | [m] | double | マップ全体の平行移動量（y） | 0.0 |
| translation_z | [m] | double | マップ全体の平行移動量（z） | 0.0 |
| rotation_x | [rad] | double | マップの回転角（roll, x軸周り） | 0.0 |
| rotation_y | [rad] | double | マップの回転角（pitch, y軸周り） | 0.0 |
| rotation_z | [rad] | double | マップの回転角（yaw, z軸周り） | 0.0 |
| global_frame_id | [-] | string | 出力点群マップの`frame_id` | map |
| map_file | [-] | string | ロードするマップファイルのパス（pcd/ply対応） | "" |
| file_type | [-] | string | ロードするマップファイルの拡張子（pcd/ply対応） | pcd |
