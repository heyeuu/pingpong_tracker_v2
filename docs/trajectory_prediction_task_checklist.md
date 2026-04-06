# 轨迹预测任务清单（单目 + 双目）

## 0. 使用说明

- 本文档用于把“单目路线”和“双目路线”的工作项拆到可执行粒度。
- 勾选规则：
  - `[ ]` 未开始
  - `[-]` 进行中
  - `[x]` 已完成
- 建议先完成“通用任务”，再分支执行单目或双目路线。

## 1. 通用任务（两条路线共用）

### G0. 基线与评测契约（预计 2~3 天）

- [ ] **G0-1 指标定义冻结**（P0）
  - 任务：固定 `Filtered RMSE`、`Horizon RMSE`、`update_success/update_attempts` 三个主指标。
  - 落地：`README.md` 增补指标定义；评测命令记录在本文档。
  - 验收：团队内复现实验时，指标口径完全一致。

- [ ] **G0-2 坐标系与单位契约**（P0）
  - 任务：统一 `t,x,y,z` 含义、世界坐标原点、单位（秒/米）。
  - 落地：新增 `docs/reconstruction_coordinate_contract.md`（后续创建）。
  - 验收：在线日志和离线 CSV 不再出现单位歧义。

- [ ] **G0-3 回放基线跑通**（P0）
  - 任务：在当前 `imm_ukf` 参数下输出 50/100/150ms RMSE 基线。
  - 落地：使用 `src/predictor_replay.cpp`，结果写入 `docs/replay_baseline.md`（后续创建）。
  - 验收：至少 1 份 baseline 报告可复现。

- [ ] **G0-4 数据集分层**（P1）
  - 任务：按场景整理训练/调参/验收三个集合（例如：平击、上旋、弹台后高速）。
  - 落地：约定 `tests/data/replay/{train,tune,eval}/` 目录结构（后续创建）。
  - 验收：每个集合均有元信息（帧率、光照、机位说明）。

- [ ] **G0-5 一键评测脚本**（P1）
  - 任务：新增批量回放脚本，自动汇总 RMSE 与更新成功率。
  - 落地：`scripts/evaluate_replay.sh`（后续创建）。
  - 验收：单命令可输出汇总表（CSV/Markdown 任一）。

### G1. 工程与 CI 对齐（预计 1~2 天）

- [ ] **G1-1 本地验证流程与 CI 对齐**（P0）
  - 任务：统一使用以下顺序验证：
    1) `bash tests/download_assets.sh`
    2) `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
    3) `cmake --build build --parallel`
    4) `ctest --test-dir build --output-on-failure --no-tests=error`
  - 落地：本文档 + `README.md`。
  - 验收：本地执行流程与 `.github/workflows/ci.yml` 一致。

- [ ] **G1-2 增加子模块初始化检查**（P1）
  - 任务：在开发说明中强调 `git submodule update --init --recursive`。
  - 落地：`README.md`。
  - 验收：新环境首次构建不因 `third_party/hikcamera` 缺失失败。

## 2. 单目路线任务清单（MVP，预计 2~3 周）

> 目标：先打通“检测 -> 单目重建 -> 滤波预测 -> 可视化 -> 回放评测”闭环，快速得到可用结果。

### M0. 需求冻结与排期（预计 0.5~1 天）

- [ ] **M0-1 单目假设冻结**（P0）
  - 任务：明确使用的先验（桌面高度、球半径、相机姿态假设）。
  - 落地：`docs/reconstruction_coordinate_contract.md`。
  - 验收：任何人能据此实现同一版 2D->3D 公式。

- [ ] **M0-2 单目排期拆分到 Issue**（P0）
  - 任务：把 M1~M5 的任务分为可合并 PR（每个 PR 1~3 个任务）。
  - 落地：仓库 Issue/项目板（外部执行）。
  - 验收：每个任务有负责人、截止日期、验收标准。

### M1. 单目重建模块（预计 4~5 天）

- [ ] **M1-1 新增重建模块骨架**（P0）
  - 任务：新增 `src/module/reconstruction/mono_reconstructor.hpp/.cpp`。
  - 内容：输入 `Ball2D + timestamp`，输出 `ReplaySample` 风格 3D 观测结构。
  - 验收：模块可被单测直接调用。

- [ ] **M1-2 新增重建配置结构**（P0）
  - 任务：定义 `fx/fy/cx/cy`、畸变参数、球半径、桌面高度、质量阈值。
  - 落地：`config/config.yaml` 新增 `reconstruction.mono` 节点。
  - 验收：缺省配置可加载，参数缺失有错误提示。

- [ ] **M1-3 去畸变与归一化坐标**（P0）
  - 任务：对检测点做去畸变，映射到归一化像平面。
  - 落地：`src/module/reconstruction/mono_reconstructor.cpp`。
  - 验收：给定标定参数时，测试点误差在阈值内。

- [ ] **M1-4 深度估计（先验驱动）**（P0）
  - 任务：基于球像素半径 + 相机内参估计深度，并给出失败分支（半径异常、越界）。
  - 落地：`src/module/reconstruction/mono_reconstructor.cpp`。
  - 验收：合成数据回归中深度误差可控。

- [ ] **M1-5 横向坐标恢复**（P0）
  - 任务：由深度与像面坐标恢复 `x,y,z`，并约束到约定坐标系。
  - 落地：`src/module/reconstruction/mono_reconstructor.cpp`。
  - 验收：与离线真值对齐后误差分布合理。

- [ ] **M1-6 观测质量评分**（P1）
  - 任务：融合检测置信度、半径一致性、边界距离，输出 quality score。
  - 落地：`src/module/reconstruction/mono_reconstructor.cpp`。
  - 验收：低质量观测可被门控策略识别。

- [ ] **M1-7 单元测试覆盖**（P0）
  - 任务：新增 `tests/mono_reconstructor_test.cpp`。
  - 验收：至少覆盖正常输入、越界输入、低置信度输入三类场景。

### M2. 在线链路接入（预计 3~4 天）

- [ ] **M2-1 新增 kernel 重建封装**（P0）
  - 任务：新增 `src/kernel/reconstruction.hpp/.cpp`，封装 module 级重建逻辑。
  - 验收：runtime 可以通过 kernel 接口获取 3D 观测。

- [ ] **M2-2 新增 kernel predictor 封装**（P0）
  - 任务：新增 `src/kernel/predictor.hpp/.cpp`，封装 `make_predictor` 与 update/predict 流程。
  - 验收：支持 UKF 与 IMM+UKF 两种配置切换。

- [ ] **M2-3 串联 runtime 主循环**（P0）
  - 任务：在 `src/runtime.cpp` 串联“检测 -> 重建 -> 更新滤波 -> 轨迹预测”。
  - 验收：本地运行可持续输出预测结果，不影响原有可视化开关。

- [ ] **M2-4 时间戳统一**（P0）
  - 任务：使用 `Image::get_timestamp()` 作为观测时标，消除 wall-clock 混用。
  - 落地：`src/runtime.cpp`、`src/kernel/predictor.cpp`（后续创建）。
  - 验收：无时间倒退更新，预测步长稳定。

- [ ] **M2-5 预测结果可视化叠加**（P1）
  - 任务：叠加当前估计位置、未来 N 步轨迹、更新状态（成功/失败）。
  - 落地：`src/runtime.cpp` + `src/utility/image/ball.cpp`（按需扩展）。
  - 验收：能直观看到预测轨迹与实测点关系。

### M3. 观测门控与容错（预计 2~3 天）

- [ ] **M3-1 观测门控策略**（P0）
  - 任务：按速度上限、加速度上限、空间边界做异常测量拒绝。
  - 落地：`src/kernel/predictor.cpp`（后续创建）。
  - 验收：异常点注入测试中，轨迹不发散。

- [ ] **M3-2 自适应测量噪声**（P1）
  - 任务：根据 quality score 动态缩放 `measurement_noise_std`。
  - 落地：`src/module/predictor/ball_state.cpp` 与 `src/module/predictor/imm_ball_state.cpp`（按需扩展）。
  - 验收：遮挡/模糊场景下 RMSE 不恶化或恶化受控。

- [ ] **M3-3 丢观测回退策略**（P0）
  - 任务：连续丢失时仅 predict，不 update；超过 reset_interval 后重置。
  - 落地：复用 `set_reset_interval` 机制并补日志。
  - 验收：长时间丢观测后可自动恢复，不崩溃。

### M4. 离线回放与调参（预计 3~4 天）

- [ ] **M4-1 在线观测落盘**（P0）
  - 任务：将在线重建观测按 `t,x,y,z` 写入 CSV。
  - 落地：建议新增 `src/module/predictor/replay_logger.*`（后续创建）。
  - 验收：CSV 可直接被 `pingpong_tracker_predictor_replay` 使用。

- [ ] **M4-2 参数扫描脚本**（P1）
  - 任务：批量扫描 UKF/IMM 参数（Q/R、bounce gate、transition matrix）。
  - 落地：`scripts/tune_predictor.sh`（后续创建）。
  - 验收：输出参数与指标对照表。

- [ ] **M4-3 最优参数回写**（P0）
  - 任务：将调参结果回写 `config/config.yaml` 的 `predictor` 节点。
  - 验收：回归集上达到目标指标。

### M5. 单目验收与文档（预计 1~2 天）

- [ ] **M5-1 验收报告**（P0）
  - 任务：产出单目验收报告（场景、命令、指标、失败样例）。
  - 落地：`docs/mono_acceptance_report.md`（后续创建）。
  - 验收：报告可独立复现实验。

- [ ] **M5-2 README 运行章节补全**（P1）
  - 任务：补充单目在线运行与回放评测命令。
  - 落地：`README.md`。
  - 验收：新同学按文档可跑通。

## 3. 双目路线任务清单（高精度，预计 4~6 周）

> 目标：在单目闭环基础上，提高 3D 观测精度与鲁棒性，收紧中远时域预测误差。

### S0. 架构与硬件准备（预计 1~2 天）

- [ ] **S0-1 双路采集拓扑冻结**（P0）
  - 任务：确定双相机接入方式（双 Hikcamera / 视频文件对）。
  - 落地：`docs/stereo_capture_contract.md`（后续创建）。
  - 验收：采集方案可稳定产出左右帧。

- [ ] **S0-2 同步策略冻结**（P0）
  - 任务：定义硬同步优先、软同步容差（例如 <= 5ms）。
  - 验收：策略在代码与文档中一致。

### S1. 双路采集与同步（预计 4~5 天）

- [ ] **S1-1 双路 Capturer 接口**（P0）
  - 任务：扩展 `src/kernel/capturer.*` 支持左右路帧对输出。
  - 验收：输出结构中包含左右图像和统一时间戳。

- [ ] **S1-2 本地双视频适配器**（P1）
  - 任务：扩展 `src/module/capturer/local_video.*` 支持双文件回放模式。
  - 验收：文件回放可模拟双目开发调试。

- [ ] **S1-3 软同步队列与对齐**（P0）
  - 任务：按时间戳匹配左右帧，超时/缺帧时执行降级策略。
  - 落地：建议新增 `src/module/capturer/stereo_sync.*`（后续创建）。
  - 验收：帧对对齐误差满足容差要求。

### S2. 标定与几何（预计 4~6 天）

- [ ] **S2-1 标定参数配置**（P0）
  - 任务：新增左右内参、畸变、外参、投影矩阵配置。
  - 落地：`config/config.yaml` 新增 `reconstruction.stereo.calibration`。
  - 验收：配置可加载并校验维度合法性。

- [ ] **S2-2 极线矫正工具链**（P1）
  - 任务：提供标定结果验证脚本（重投影误差、极线误差）。
  - 落地：`scripts/validate_stereo_calibration.sh`（后续创建）。
  - 验收：输出可读报告，失败有明确提示。

- [ ] **S2-3 三角化模块骨架**（P0）
  - 任务：新增 `src/module/reconstruction/stereo_reconstructor.hpp/.cpp`。
  - 验收：输入左右 2D 点，输出 3D 点与质量分数。

### S3. 检测关联与三角化（预计 5~7 天）

- [ ] **S3-1 左右目标匹配器**（P0）
  - 任务：按极线距离、半径相似度、置信度做匹配。
  - 落地：建议新增 `src/module/reconstruction/stereo_matcher.*`。
  - 验收：多球场景下错配率受控。

- [ ] **S3-2 三角化 + 质量评估**（P0）
  - 任务：输出重投影误差、视差稳定性、深度可信度评分。
  - 落地：`src/module/reconstruction/stereo_reconstructor.cpp`。
  - 验收：低质量结果可被明确标记。

- [ ] **S3-3 失败回退接口**（P0）
  - 任务：无法三角化时，支持短时回退到单目估计或仅预测外推。
  - 落地：`src/kernel/reconstruction.cpp`（后续创建）。
  - 验收：遮挡下轨迹连续、不突变。

- [ ] **S3-4 双目单元测试**（P0）
  - 任务：新增 `tests/stereo_reconstructor_test.cpp` 与 `tests/stereo_matcher_test.cpp`。
  - 验收：覆盖匹配成功、错配拒绝、退化几何三类场景。

### S4. 在线融合（预计 3~4 天）

- [ ] **S4-1 runtime 双目链路打通**（P0）
  - 任务：在 `src/runtime.cpp` 接入“左右检测 -> 关联 -> 三角化 -> predictor”。
  - 验收：运行时可稳定输出 3D 与预测轨迹。

- [ ] **S4-2 可视化扩展**（P1）
  - 任务：显示左右目检测、匹配线、重投影误差、主模式概率。
  - 落地：`src/runtime.cpp` + `src/kernel/visualization.cpp`（按需扩展）。
  - 验收：调试时可定位误差来源。

### S5. 调参与鲁棒性（预计 4~5 天）

- [ ] **S5-1 双目专属参数扫描**（P1）
  - 任务：扫描匹配阈值、视差阈值、质量门控阈值。
  - 落地：`scripts/tune_stereo_reconstruction.sh`（后续创建）。
  - 验收：输出最优参数集与失败案例。

- [ ] **S5-2 遮挡/失步场景压测**（P0）
  - 任务：验证单路丢帧、左右时戳偏移、强反光误检场景。
  - 验收：更新成功率与轨迹连续性满足阈值。

- [ ] **S5-3 性能预算控制**（P0）
  - 任务：统计双目链路新增耗时，目标不破坏实时性（例如 60fps）。
  - 验收：链路新增平均耗时在预算内。

### S6. 双目验收与交付（预计 1~2 天）

- [ ] **S6-1 双目验收报告**（P0）
  - 任务：输出双目指标对比（vs 单目）与推荐配置。
  - 落地：`docs/stereo_acceptance_report.md`（后续创建）。
  - 验收：报告包含命令、数据集、结果、结论。

- [ ] **S6-2 运维文档补齐**（P1）
  - 任务：补充标定更新流程、故障排查（不同步/错配/漂移）。
  - 落地：`README.md` 或 `docs/stereo_ops.md`（后续创建）。
  - 验收：可按文档完成一次从标定到上线的全流程。

## 4. 验收门槛（Definition of Done）

### 单目 DoD

- [ ] 100ms Horizon RMSE 达到目标（建议先 8~12 cm，再逐步收紧）。
- [ ] `update_success / update_attempts >= 98%`。
- [ ] 60fps 目标下新增链路延迟 <= 10ms（根据硬件可调整）。
- [ ] 连续运行 30 分钟无崩溃、无明显漂移发散。

### 双目 DoD

- [ ] 100ms Horizon RMSE 相比单目显著下降（建议目标 4~8 cm）。
- [ ] 遮挡与失步场景下更新成功率 >= 97%。
- [ ] 三角化失败时可平滑回退，不出现轨迹跳变。
- [ ] 连续运行 30 分钟稳定，资源占用在预算内。

## 5. 建议执行顺序

1. 先完成 G0/G1 通用任务，建立统一评测口径。
2. 优先执行单目 M0~M5，快速形成闭环并沉淀 baseline。
3. 在单目可用基础上推进双目 S0~S6，提升精度与鲁棒性上限。
4. 最后统一回放评测脚本，形成“单目 vs 双目”横向对比报告。

## 6. 常用验证命令（抄录）

```bash
bash tests/download_assets.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure --no-tests=error
```

```bash
./build/pingpong_tracker_predictor_replay /path/to/trajectory.csv \
  --config config/config.yaml \
  --horizon-ms 100 \
  --step-ms 10 \
  --warmup 10
```
