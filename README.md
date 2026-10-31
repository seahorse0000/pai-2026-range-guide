# 距離センサで相手に合わせて速度を調整する案内ロボット

ROS 2 JazzyとGazeboを使った直線案内の実行コードです。2D LiDARで後方の円柱形の移動物体1個を検知し、相手の動きに合わせて先行ロボットの速度を調整します。

物体の位置・速度はLiDARの反射点から推定し、自分の位置にはGazeboのオドメトリを使用します。AIは既存の学習済みQ表で速度候補を選び、距離に応じた速度上限を併用します。`models/q_final.json`は実行に必要な固定モデルです。追加学習は行いません。

## 実行

Dockerを起動した状態で、このフォルダから実行します。

```bash
docker build -f Dockerfile.base -t ward-guide-prototype:0.1.0 .
docker build -f Dockerfile.gui -t ward-guide-range-gui:20261031 .
mkdir -p outputs/run-01
docker run --rm --name ward-range-run \
  -v "$PWD:/workspace/range:ro" \
  -v "$PWD/outputs/run-01:/workspace/output" \
  --entrypoint bash ward-guide-range-gui:20261031 \
  -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/range/scripts/run.py --gui'
```

`--gui`を省くと画面録画を行わずに実行します。出力は`outputs/run-01`に保存されます。再実行は`run-02`など新しい出力先を使ってください。GUIはコンテナ内の仮想画面で動作し、`--gui`指定時はMP4が出力されます。

`src/ward_range`が検知・制御・物体動作・評価の処理、`worlds/ward-lidar.sdf`が使用した環境、`config/bridge.yaml`が通信設定です。依存関係はROS 2、Gazebo、NumPy、Xvfb、ffmpeg、xdotoolです。

既知の直線経路と理想化した距離センサを使うシミュレーションです。人物識別や複数物体の追跡、実機での動作は扱いません。

センサ設定の参考：Gazebo公式 https://gazebosim.org/docs/harmonic/sensors/
