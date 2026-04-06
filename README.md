# pingpong_tracker_v2
这是一个追踪并预测乒乓球轨迹的视觉项目

## 轨迹预测开发任务清单

单目与双目路线的详细任务拆分见：

- `docs/trajectory_prediction_task_checklist.md`

## IMM+UKF 离线回放评测

新增可执行程序：`pingpong_tracker_predictor_replay`  
输入 `CSV(t,x,y,z)` 的三维观测轨迹，输出滤波 RMSE 与多步预测 RMSE。

### 构建

```bash
cmake -S . -B build
cmake --build build --target pingpong_tracker_predictor_replay -j4
```

### 运行

```bash
./build/pingpong_tracker_predictor_replay /path/to/trajectory.csv
```

可选参数：

```bash
./build/pingpong_tracker_predictor_replay /path/to/trajectory.csv \
  --config /path/to/config.yaml \
  --horizon-ms 100 \
  --step-ms 10 \
  --warmup 10
```

`config.yaml` 中 `predictor` 节点支持 UKF / IMM 参数配置。
