# terrauto_core

"terrain"+"autonomy"を組みあせて命名。
不整地の自律移動を可能にするパッケージ。

開発中。

## 開発環境

- Jetson Orin Nano Super開発者キット
- Jetpack6
- ROS 2 Humble

## ネットワークの設定

`/etc/sysctl.d/10-cyclonedds.conf`に以下を記述し、PCを再起動。

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
```
