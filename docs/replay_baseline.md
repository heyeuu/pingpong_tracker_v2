# Predictor Replay Baseline

## 1. 基线目的

固定当前 `predictor.filter_type = imm_ukf` 配置，产出 50/100/150ms 三个时域下的可复现 RMSE 基线。

## 2. 数据与配置

- 轨迹数据：`tests/data/replay/eval/synthetic_ballistic.csv`
- 配置文件：`config/config.yaml`
- 可执行：`build/pingpong_tracker_predictor_replay`

## 3. 复现实验命令

```bash
cmake --build build --target pingpong_tracker_predictor_replay --parallel

./build/pingpong_tracker_predictor_replay tests/data/replay/eval/synthetic_ballistic.csv \
  --config config/config.yaml --horizon-ms 50 --step-ms 10 --warmup 10

./build/pingpong_tracker_predictor_replay tests/data/replay/eval/synthetic_ballistic.csv \
  --config config/config.yaml --horizon-ms 100 --step-ms 10 --warmup 10

./build/pingpong_tracker_predictor_replay tests/data/replay/eval/synthetic_ballistic.csv \
  --config config/config.yaml --horizon-ms 150 --step-ms 10 --warmup 10
```

或使用批量脚本一键输出汇总：

```bash
bash scripts/evaluate_replay.sh \
  --input-dir tests/data/replay/eval \
  --config config/config.yaml \
  --build-dir build \
  --horizons 50,100,150 \
  --step-ms 10 \
  --warmup 10
```

脚本输出：`docs/replay_eval_summary.csv`、`docs/replay_eval_summary.md`。

## 4. 基线结果

| Horizon (ms) | Filtered RMSE (m) | Horizon RMSE (m) | update_success / update_attempts |
|---:|---:|---:|---:|
| 50  | 0.001821 | 0.005402 | 56 / 56 |
| 100 | 0.001821 | 0.009764 | 56 / 56 |
| 150 | 0.001821 | 0.014760 | 56 / 56 |

## 5. 说明

- 本基线用于验证工具链与指标口径，数据为合成抛体轨迹。
- 后续应补充真实场景数据集（平击、上旋、弹台后高速）并更新此报告。
