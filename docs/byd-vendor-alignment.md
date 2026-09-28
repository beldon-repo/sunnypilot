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

## 四、Phase 2 纵向（已落码，未实车验证；开关默认关）

> 2026-09-29 落码。解密产物升级到 100% 覆盖 + route 37 TX 字节级验证后实施，本节是实施后的定论。

### 4.1 厂商架构定论（推翻本节旧推断的三点）

厂商是**透明替换（transparent replacement）**架构，不是"OP 完全接管"：
1. **engage 链不切换**（推翻旧"pcmCruise=False→buttonEnable"设计）：雷达帧在 bus2 照常活跃（fwd 只挡 bus2→bus0），雷达仍是 engage/cancel 权威，`cruiseState.enabled` 的 is_cruise_latch 与 MADS 零改动
2. **counter 50Hz 自由递增**（推翻旧"与雷达 ACC_CMD 对齐 delta=-1"）：厂商 counter 连续走 0-f 穿过 echo/active 切换，与雷达无关
3. **ACC_HUD/ACC_AEB 是纯 echo**（旧"16 字段构造"是高估）：HUD 全字段透传相机帧（Notify 12=ENGAGE/8=DISENGAGE 可见），AEB 是静态 `05 80 02 0f ff ff` + counter + checksum；只有 ACC_CMD 在 active 时覆写加速度字段

### 4.2 字节级定论（route 37 TX 实测，冒烟脚本 /tmp/byd_long_smoke.py 全对上）

- **0x32E ACC_CMD**：雷达帧 14 字段回声基底；active（enabled and active）时覆写：AccelCmd=OP accel（clip [-4,2]，raw [20,140]，raw100=0 m/s²）＋ ComfortBand 0.1/0.05（仅 accel≥0）＋ JerkUpper 1.0/raw5（697 帧中 692）＋ JerkLower clip(jerk,-4,-0.8)（我们无 plan jerk，固化为 -0.8）＋ AccControlActive=1；byte5 状态分布 0x48（待机 echo：control=0, accreq=1, esp=1）/0x58（active）/0x00（全关）；ResumeFromStandstill 全程 0（route37 无静止段），我们按 starting 状态发脉冲
- **0x32D ACC_HUD**：纯 echo + counter/checksum 重打；active 顶值 `5a0c5c01f4fff059` 与厂商逐字节一致
- **0x32F ACC_AEB**：PAYLOAD 0|48**@1+（小端——@0+ 大端在 parser/packer 两端位序不保真，往返丢 byte0）**+ counter + SETME4_0xF + checksum；echo 出 `0580020ffffff07b` = 厂商日志 x166 模式
- **checksum 全家统一 byd_checksum(0xAF)**（0x32D/E/F 三种均用厂商字节验算吻合）
- 雷达主动制动时（OP off、AccControlActive=1 echo）固件若按 hyundai 模式封死会拦掉原厂刹车 → byd.h 0x32E 用 echo-relay 规则：**active=1 需 controls_allowed 且限内；active=0 限内即放行**；陈旧 echo 幻影由 0x32D rx liveness 兜底

### 4.3 落码清单（本分支）

| 文件 | 内容 |
|---|---|
| `byd_general_pt.dbc` | 新增 BO_ 815 ACC_AEB（PAYLOAD LE 48bit） |
| `values.py` | `BydSafetyFlags.LONGITUDINAL=2`；ACCEL_MIN/-4.0、COMFORT_BAND、JERK 常量 |
| `bydcan.py` | create_accel_command 重写（回声基底+active 覆写）、create_acc_hud_command、create_acc_aeb_command |
| `carstate.py` | adas parser +ACC_AEB@50Hz；radar_acc_msg/adas_msg/aeb_msg 缓存 |
| `interface.py` | `openpilotLongitudinalControl=alpha_long`（AlphaLongitudinalEnabled param，无需新 UI、无需重刷切换）；startingState/startAccel 0.4/vEgoStarting 0.3/vEgoStopping 0.2/longitudinalActuatorDelay 0.5/stoppingDecelRate 0.03；get_pid_accel_limits=(-4,2) |
| `carcontroller.py` | _update_longitudinal：frame%2==0 发三件套，resume=LoC starting |
| `safety/modes/byd.h` | BYD_PARAM_LONGITUDINAL=2；TX_MSGS_LONG（0x316/32D/32E/32F/3B0）；0x32E echo-relay accel 检查；fwd 加挡 0x32D/E/F（双向）；**MADS 修复**（byd 全部 check_relay=false，stock_ecu_check 不触发 → rx_hook 末尾显式驱动 mads_state_update） |
| `safety/tests/test_byd.py` | TestBydSafetyLong（118 passed 全绿，含 echo-relay 门控测试） |
| `tests/common.py` | no_lockout += TestBydSafetyLong |

注：本仓库不带 scons 构建文件（工作区曾有 4 个未跟踪 SConstruct/SConscript，分支切换会被清掉，也不入库）。编译 libsafety 用手动命令：`cd opendbc_repo && cc -shared -fPIC -DCANFD -I. -Iopendbc/safety/board opendbc/safety/tests/libsafety/safety.c -o opendbc/safety/tests/libsafety/libsafety.so`，测试 `PYTHONPATH=$PWD/opendbc_repo:$PWD .venv/bin/python -m pytest opendbc_repo/opendbc/safety/tests/test_byd.py -q`（PYTHONPATH 必须带，否则解析到错误包路径）。

### 4.4 验证状态与风险

- 回归：byd 安全测试 118 passed/0 failed（旧 24 个存量漂移一并修复）；横向 latch 回放 13/13；冒烟三帧逐字节一致
- **待办**：固件重刷（TX 白名单+accel 检查+fwd 门控进 panda）；`docs/firmware-safety_byd.h` 快照待同步
- 实车首验重点：①低速封闭场地起步/跟停（delay 0.5 需标定）②TOO_CLOSE→-4 m/s²（厂商行为，route37 零触发证据，慎验）③JerkLower 固化 -0.8 在急减速的手感 ④静止段 ResumeFromStandstill 首验（route37 无样本）
- JerkLower/JerkUpper 固化与厂商"plan jerk 派生"的偏差：平顺驾驶完全一致（厂商 692/697 帧同值），急减速时厂商会放宽到 -4.0，我们待 plan jerk 接入后再对齐

## 五、待办（优先级序）

1. **推送 + 设备同步**（`ea93962414` review 修复 + `ec28881e7b` 厂商对齐 + 本次纵向落码，均未推送）——路径见 ops 文档第二节
2. **实车复测横向**（按 ops 第三节规程）：重点 ①弯道助力连续性（让位曲线后 ⑤ 项观感）②对抗让位平滑度 ③开 `NeuralNetworkLateralControl=True` 后手感 A/B
3. **纵向实车首验**（Phase 2 已落码，见第四节）：先刷固件（TX 白名单+accel 检查+fwd 门控）→ 设备开 `AlphaLongitudinalEnabled` → 低速封闭场地起步/跟停 → 开放道路低速 → 高速；重点验证 4.4 节四项
4. `docs/firmware-safety_byd.h` 快照同步（刷入后）
5. 手感调参弹药（控车稳定后）：THRESHOLD 80→60-70、NN 与 override.toml 因子对比、厂商 DELTA 16（需固件放宽 10→16）与 24s 脱手计时器、JerkLower 接 plan jerk
6. 0x11F 扩展解析（LKSPrepare/扭矩传感器位）作 0x318 备份源——低优先
