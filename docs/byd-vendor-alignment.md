# BYD 宋 PLUS：厂商方案对齐路线（op_byd 解密成果 → 落地）

> 面向新 session 的专项文档。2026-09-29 确立"采纳厂商成熟方案"路线后的一切结论都在这里。
> 调试全史见 `byd-song-plus-lkas-debug.md`，运维规程见 `byd-song-plus-ops.md`。

## 一、背景与已确认决策

厂商 op_byd（carrot 系 fork，PyArmor 8.5.9 pro 加密，2025-04-27 构建）是**唯一被证实在本车长期稳定控车**的系统：横向走 790 (0x316) 扭矩通道（与我们相同），纵向全套 OP 控制（伪造 ACC_CMD/ACC_HUD/AEB + 按钮），手感好（实测扭矩 p50=67 / p90=128 / max=193）。

用户决策（2026-09-29，已确认）：
1. **先横向后纵向**：横向对齐 + 实车复测干净后，再上纵向
2. **门控 + 厂商曲线叠加**：保留我们的 EPS 包络门控（角度门/静默守卫/follow），叠加厂商平滑让位曲线
3. **移植 NN 前馈**：厂商 BYD_LM_PARAMS 宋 22 模型

## 二、解密产物（数据源，勿重复逆向）

| 资产 | 位置 | 说明 |
|---|---|---|
| 运行时解密产物 | `/Users/wujiafu/Documents/op/cp_byd/docs/pyarmor_decrypted/` | 13 个 PyArmor 文件，以 `.disasm.txt` + `.values.txt` 为权威；`.functions.py` 部分函数体可用；**update() 主循环等核心函数体是 PyArmor JIT 壳，不可读** |
| 运行时参数 dump | `docs_site/op_byd_data/values.json` | values 模块全部常量真值 + BYD_LM_PARAMS NN 权重 |
| 厂商源码树 | `/Users/wujiafu/Documents/op/cp_byd/` | 完整 op_byd openpilot 树（可重跑 dump，工具在 `dump_tool/pydump.py`） |
| 厂商实车日志 | `docs_site/op_byd_logs/` | route 37 等，字节模板出处 |

**解密已定论的事实**（不要再查）：
- **790 (0x316) 是宋 22 唯一验证过的转向通道**。"宋 22 应走 337" 的旧结论已被推翻（0x151 只在 RX 指纹，不在厂商 TX）。厂商 DBC（byd_tang_dm_18_19，老社区命名）把 0x316 叫 MPC_LKAS_CMD：Config↔LKAS_Config（常量 3）、LKASPrepare↔LKAS_ReqPrepare、LeftLane/RightLane↔LaneState、Keep_Hands_On_Wheel↔ReqHandsOnSteeringWheel——与我们现实现逐字节一致
- **482 角度路径在宋上是死路径**（22/23 指纹均无 0x1E2；厂商 MPC_LKAS_CMD_ANGLE 从不发出）
- **0x122 非轮速实锤**：厂商日志 70% 帧是同一静态模式 `0000000000300040`；0x1F0 ESP_SPEED 正常变化。我们现有车速源正确
- **0x11F (EPS 100Hz) 里还有信号位可挖**：厂商状态机谓词（is_steering_activated/need_activate/standby/lose_frame）全部读 0x11F 的 LKSPrepare + Cruise_Activated + Steer_Torque_Sensor——我们目前只解析角度+速率，0x318 源够用但 0x11F 可作备份
- 厂商横向手感 = 三件套：**NN 前馈**（18→7→13→3→1，test_loss 0.031）+ **平滑让位曲线** + 非线性扭矩拟合（后者与 NN 冗余，我们只采前两者）
- 厂商 STEER_MAX=300 / ALLOWANCE=120 / DELTA 16-10（低速 150/10-10）；**ALLOWANCE 120 是它 emergent-follow（由控制器输出自然跟随）的必要余量；我们用显式 follow，不需要**——固件保持 68/10/12 未动

## 三、Phase 1 横向对齐（已落码 `ec28881e7b`）

### 3.1 让位曲线（values.py + carcontroller.py）

```python
STEER_YIELD_DRV_BP = [50., 120., 186.]
STEER_YIELD_FACTOR = [1.0, 0.5, 0.0]
# 正常路径: new_torque = round(demand * np.interp(|drv|, BP, FACTOR))
```

设计要点（数学已验证，勿回退）：
- **只作用于正常路径**。对向请求仍走"检测窗 3 帧 0 → 持续对向翻 follow ±20"——厂商曲线若作用于对向路径会造出 armed-零（缩放到 0 的对向请求 = 静默锁存条件）
- 曲线终点 186 与 driver-limit 截断零点（68 + 200/3 ≈ 186.7）共位——死区在构造上消除
- ALLOWANCE 保持 68，**固件零改动**（byd.h 未动，无需重刷）
- 附带修复：静默守卫在对向检测/follow 计数期间冻结——原先 follow 翻转（0.15s）对静默退场（0.16s）只有 1 帧竞态余量

### 3.2 NN 前馈（已入库，默认关）

- `sunnypilot/neural_network_data/neural_network_lateral_control/BYD_SONG_PLUS_DMI_22.json`（从 values.json 的 BYD_LM_PARAMS 提取）
- 与 sunnypilot NNLC 框架（`sunnypilot/selfdrive/controls/lib/nnlc/`）**即插即用**：模型按 carFingerprint 文件名自动匹配；真实 NNTorqueModel 验证过直道 ≈0.001、±1.0 lat_accel → ±0.6 对称、全量程 ±0.9，与官方模型同符号约定，**无需缩放**
- `friction_override=True` 是框架预期路径（自动补经典摩擦项）
- 早期"输出恒正 0.8~2.3"是探针伪影（历史输入填 0 → sigmoid 饱和），勿再误判
- **启用**：设备 Params `NeuralNetworkLateralControl=True`（默认关，可 A/B 对比手感）

### 3.3 验证结果

- 回放 13/13 干净（`docs_site/hour_logs_2/replay_latches.py`，R16 对向暴露 0.10s 持平）
- 安全测试 24 failed 为**存量** libsafety 漂移（stash 对照确认与本改动无关，即待办 #5）

## 四、Phase 2 纵向设计（未实施；实车复测干净后启动）

目标形态：`openpilotLongitudinalControl=True`，OP 完全接管纵向，伪造 ACC 三件套；原车 ACC 被替代（厂商同样行为）。

厂商还原细节（置信度标注）：
- **TX bus0 50Hz×4**：0x316（已有）/ 0x32D ACC_HUD_ADAS / 0x32E ACC_CMD / 0x32F ACC_AEB；bus2 0x3B0 按钮间歇
- **ACC_CMD 0x32E**（15 字段）：ACCEL_CMD（DBC 0.05/-5，物理 [-4,2]）；ComfortBandUpper=0.2/Lower=-0.25；JerkUpper/LowerLimit=12（raw）；minimal_brake=-0.8（stopping）；STANDSTILL_RESUME（starting 时 1）；ACC_REQ_NOT_STANDSTILL=15；**counter 与原厂雷达 ACC_CMD 帧对齐**（acc_initial_counter_delta=-1 初始化，radar_acc_msg 取基准）(70-95%)
- **ACC_HUD 0x32D**（16 字段）：SET_SPEED（hudControl.setSpeed）、HAS_LEAD、SET_DISTANCE、LEAD_DISTANCE、ACC_ON1/ON2 双标志、TOO_CLOSE/NOTIFY/ERROR、CRUISE_STATE、SET_ME_XFF=0xFF/XF=0xF (75%)
- **ACC_AEB 0x32F**：PAYLOAD 心跳（实车 `05 80 02 0f...`）+ SETME_0xF=0xF (55-80%)
- **interface 纵向参数**：longitudinalActuatorDelay=0.44、vEgoStarting=0.3、stoppingDecelRate≈0.03、radarUnavailable=True（vision lead；本车指纹无 MRR 884，`EnableRadarTracks` 真雷达功能暂不做）(65-100%)
- **SNG**：BYDSnG param；静止后 starting 每 350ms 发 resume；BTN_ACC_DEC {NONE=0,DEC_1=1,DEC_10=2,ACC_1=3,ACC_10=4}；长按 30 帧→±10km/h (85%)

实施要点（探索已完成）：
- **engage 链切换**：pcmCruise=False → buttonEnable 路径。需 byd carstate 补 buttonEvents 解析（当前不解析）、`car_specific.py` byd 分支 `pcm_enable=self.CP.pcmCruise`（仿 hyundai:126-128）；MADS（mads.py:134）同时接受 pcmEnable/buttonEnable 无需改；cruiseMismatch 语义变化（pcmCruise=False 时 stock ACC 开着即触发，告警映射已注释为静默）；controlsd.py:171-173 的 cancel/resume/override 语义自动生效
- **byd.h**：TX 白名单加 0x32D/0x32E/0x32F；0x32E 走 `longitudinal_accel_checks`（参考 hyundai.h:247-266，限幅用 raw 单位 [-80, 140] = [-4,2] m/s²）；fwd_hook 改为 block 相机 0x32D/E/F（OP 长控时）；0x32F PAYLOAD 心跳帧按白名单放行即可
- **carstate**：aEgo 用 KF 推导即可（DBC 无加速度传感器消息，已确认）；radar_acc_msg/adas_msg/aeb_msg 原帧缓存（counter 对齐 + echo）
- **plan 取值**：本仓库 controlsd 已按 actuator delay 在 plan 上插值（CC.actuators.accel 即延迟补偿后目标）——**不需要**厂商的 get_accel_and_jerk_from_plan；JerkLimit 字段先用固定 12（厂商常量）
- **测试**：`opendbc/safety/tests/test_byd.py` 扩 TX_MSGS + 纵向 accel 测试（参考 test_hyundai.py 模式）+ `_pcm_status_msg` 语义调整
- 可选复用：`opendbc_repo/opendbc/sunnypilot/car/hyundai/longitudinal/` 的 jerk-limited 控制器层
- 风险：刹车安全关键（首次 OP 控刹车）；纵向标定（delay 0.44 需实车验证）；固件 TX 白名单扩充需重刷；engage 链切换需全场景重验

验证路线建议：低速封闭场地起步/跟停 → 开放道路低速 → 高速；全程对比原车 ACC 基线。

## 五、待办（优先级序）

1. **推送 + 设备同步**（`ea93962414` review 修复 + `ec28881e7b` 厂商对齐，均未推送）——路径见 ops 文档第二节
2. **实车复测**（按 ops 第三节规程）：重点 ①弯道助力连续性（让位曲线后 ⑤ 项观感）②对抗让位平滑度 ③开 `NeuralNetworkLateralControl=True` 后手感 A/B
3. Phase 2 纵向（启动条件：2 干净）——按第四节设计实施
4. 手感调参弹药（控车稳定后）：THRESHOLD 80→60-70、NN 与 override.toml 因子对比、厂商 DELTA 16（需固件放宽 10→16）与 24s 脱手计时器
5. 0x11F 扩展解析（LKSPrepare/扭矩传感器位）作 0x318 备份源——低优先
