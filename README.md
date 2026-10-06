# pingpong_tracker_v2

这是一个追踪并预测乒乓球轨迹的视觉项目。

## 轨迹预测任务总览

- 任务清单：`docs/trajectory_prediction_task_checklist.md`
- 坐标与单位契约：`docs/reconstruction_coordinate_contract.md`
- 回放基线报告：`docs/replay_baseline.md`

## 快速开始（与 CI 对齐）

CI 环境为 Ubuntu 24.04，编译器为 `gcc-14/g++-14`，C++ 标准为 C++23。

首次拉取后，请先初始化子模块（`third_party/hikcamera` 为必需依赖）：

```bash
git submodule update --init --recursive
```

本地验证建议严格使用与 CI 一致的顺序：

```bash
bash tests/download_assets.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure --no-tests=error
```

## 指标定义（冻结）

离线回放默认关注以下三个主指标：

- `Filtered RMSE`：每次成功更新后的滤波位置误差均方根（单位：米）。
- `Horizon RMSE`：给定预测时域（如 50/100/150ms）末端位置误差均方根（单位：米）。
- `update_success/update_attempts`：更新成功率，表示在所有尝试更新中成功更新的比例。

## 在线运行

默认配置文件为 `config/config.yaml`，路径由编译时 `PINGPONG_TRACKER_SOURCE` 决定，不依赖进程启动目录。

当前默认采集源是 `capturer.source: local_video`，请根据本机环境修改 `capturer.local_video.location`。

当前在线主循环已串联：

- 检测（`identifier`）
- 单目重建（`reconstruction.mono`）
- 预测器更新/外推（UKF 或 IMM+UKF）
- 可视化叠加（更新状态、当前估计、100ms 预测点）

时间戳统一使用 `Image::get_timestamp()` 驱动重建与预测更新。

运行：

```bash
./build/pingpong_tracker_runtime
```

## 离线回放评测

可执行程序：`pingpong_tracker_predictor_replay`。

输入 `CSV(t,x,y,z)` 的三维观测轨迹，输出更新成功率、滤波 RMSE 与多步预测 RMSE。

### 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target pingpong_tracker_predictor_replay --parallel
```

### 运行

```bash
./build/pingpong_tracker_predictor_replay /path/to/trajectory.csv \
  --config config/config.yaml \
  --horizon-ms 100 \
  --step-ms 10 \
  --warmup 10
```

`config/config.yaml` 中 `predictor` 节点支持 UKF / IMM 参数配置。

## 批量回放脚本

新增脚本 `scripts/evaluate_replay.sh`，用于批量执行多个 CSV 的回放并汇总结果。

示例：

```bash
bash scripts/evaluate_replay.sh \
  --input-dir tests/data/replay/eval \
  --config config/config.yaml \
  --build-dir build \
  --horizons 50,100,150 \
  --step-ms 10 \
  --warmup 10
```

脚本会输出：

- `docs/replay_eval_summary.csv`
- `docs/replay_eval_summary.md`

## 回放数据目录约定

按照以下结构组织回放数据集：

```text
tests/data/replay/
  train/
  tune/
  eval/
```

每个集合建议包含 `metadata.md`，记录帧率、光照、机位与场景说明。
