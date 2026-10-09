# BYD 打灯变道移植方案

> 状态：**方案定稿 + T2/T3 已落码待路测**（2026-10-04）。T1 符号标定已离线完成（§四.1）。
> 实现与本文设计稿的差异：fork 内**不存在 NO_ACTUATION→latActive 传播链**（grep 零命中），
> 故案 A 以「selfdrived 第二实例同谓词出事件 + controlsd_ext 实例出让位」落地；
> L2 的 `=0` 语义落码时映射为 inf（等效厂商禁用且有中途 abort，§三 描述仍成立）。
> 分支：main-lon-tici。本文所有行号均已实测复核。

---

## §〇 结论速览

1. **厂商的「打灯变道」核心就是原生 openpilot DesireHelper 状态机**——厂商解密树 `op_byd/selfdrive/controls/lib/` 下**没有 desire_helper.py**（未修改、直接继承），其 `legacy_lateral_planner.py:9` 原样 import 上游 DesireHelper。拨转向灯 → 车速达标 → 司机同向轻转方向盘 → MPC 横向轨迹带入邻道，盲区有车则拦截。这套机制**本仓库已完整具备且架构更新**（state machine 在 modeld 内跑，desire 进 modelV2.meta）。
2. 厂商在原生之上只有**两条自定义**：
   - **①可配速度阈值** `dp_lat_lane_change_assist_speed`（MPH，替换原生固定 20 mph；=0 关闭辅助变道）；
   - **②assist-less 让位模式**（阈值=0 时生效）：打灯 + 司机同向带舵 → `CC.latActive=False`，openpilot 横向**完全退出**让司机手变道，灭灯解锁，期间仅弹 laneChange 提示。
3. BYD 侧**输入信号已全部就绪**：`opendbc_repo/opendbc/car/byd/carstate.py` 已解析 STALKS 左右转向灯（:229-230）与 BSD_RADAR 盲区（:235-236），`steeringTorque=ACC_EPS_STATE.SteerDriverTorque`（:151）。理论上现在就能用——**符号问题已离线排除**（§四.1：实测左=正，原生触发条件直接成立），唯一留白是旧日志无拨灯记录，留路测验证（§六 T4）。
4. 移植 = L1 验证（零代码）+ L2 阈值参数 + L3 让位模式，三条独立可回滚。

---

## §一 厂商实现还原（实证）

路径前缀 `/Users/wujiafu/Documents/work/op/op_byd/`（PyArmor 解密产物完整代码树）。

### 1.1 状态机 = 原生 DesireHelper（未改）

`selfdrive/controls/lib/legacy_lateral_planner.py:77-83`：

```python
# Lane change logic
lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, self._dp_lat_lane_change_assist_speed)

# Turn off lanes during lane change
if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
    self.LP.lll_prob *= self.DH.lane_change_ll_prob
    self.LP.rll_prob *= self.DH.lane_change_ll_prob
```

（厂商给 DH.update 多传了第 4 参=速度阈值；原生是 3 参。状态机本体逻辑与上游一致：off→preLaneChange→starting→finishing，10 s 超时回 off。）

### 1.2 自定义①：可配速度阈值

`legacy_lateral_planner.py:38`：

```python
self._dp_lat_lane_change_assist_speed = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) * CV.MPH_TO_MS
```

`controlsd.py:121-122`：

```python
self._dp_lat_lane_change_assist_disabled = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) == 0
self._dp_lat_lane_change_assist_disabled_active = False
```

语义：参数=MPH 阈值（替代原生 20 mph）；**=0 时辅助变道整体禁用**并进入 §1.3 模式。设备上实际下发值未知（厂商 params 默认定义不在解密树内；本仓库 `docs_site/op_byd_data/values.json` 亦无此键）——实现时按「默认 20（=原生行为），0=让位模式」设计，不猜厂商在线值。

### 1.3 自定义②：assist-less 让位模式

`controlsd.py:691-703`（厂商注释自称 "rick - assist-less lane change"）：

```python
# rick - assist-less lane change
if self._dp_lat_lane_change_assist_disabled:
    # de-activate
    if not CS.leftBlinker and not CS.rightBlinker:
        self._dp_lat_lane_change_assist_disabled_active = False

    # activate
    if not self._dp_lat_lane_change_assist_disabled_active and CS.steeringPressed and \
        ((CS.steeringTorque > 0 and CS.leftBlinker) or
         (CS.steeringTorque < 0 and CS.rightBlinker)):
        self._dp_lat_lane_change_assist_disabled_active = True

    if self._dp_lat_lane_change_assist_disabled_active:
        self.events.add(EventName.laneChange)
        CC.latActive = False
```

要点：**锁存**（灭双灯才解锁，与 §1.5 的灯镜像独立）；激活瞬间 openpilot 横向退出，司机全程手动变道；提示复用 `laneChange` 事件。动机推断：BYD EPS 在司机对抗/低速场景有弃权-锁存史（根因16/17/18，见 byd-lateral-lifecycle.md），与其带着 LKAS 请求打架，不如打灯+带舵时干脆让位——**与我们路测实证的方向一致，值得完整移植**。

### 1.4 盲区拦截与事件（厂商=原生行为）

`controlsd.py:376-388`：preLaneChange 时 `CS.leftBlindspot/rightBlindspot` 命中方向 → `laneChangeBlocked`，否则 `preLaneChangeLeft/Right`；starting/finishing → `laneChange`。

### 1.5 变道期间灯镜像（厂商=原生行为）

`controlsd.py:730-733`：`laneChangeState != off` 时把 CC.leftBlinker/rightBlinker 置为变道方向（HUD 显示）。

---

## §二 本仓库现状：架构映射表

本 fork 为拆分架构（selfdrived/modeld/lateral MPC 分进程），厂商两参对照如下——**能力一一对应，无缺失**：

| 厂商（老架构） | 本仓库（新架构） | 差异 |
|---|---|---|
| DesireHelper 状态机（legacy_lateral_planner 调用） | `selfdrive/controls/lib/desire_helper.py:41-139`，由 **modeld** 调用（`selfdrive/modeld/modeld.py:348` `DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)`，publish→`modelV2.meta.laneChangeState/Direction` :349-353） | 触发多了 sunnypilot ALC 免舵路径（§二.1） |
| 速度阈值 20 mph 硬编码（厂商改参数） | `desire_helper.py:10` `LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS` 硬编码 | L2 移植点 |
| controlsd 事件（§1.4） | `selfdrive/selfdrived/selfdrived.py:314-329` 逐字同款 | 已有 |
| events 定义 | `selfdrive/selfdrived/events.py:399-428`：preLaneChangeLeft/Right、laneChangeBlocked、laneChange（全 ET.WARNING） | 已有 |
| CC 灯镜像（§1.5） | `selfdrive/controls/controlsd.py:124-128` 同款 | 已有 |
| assist-less hack（§1.3） | **无对应 → L3 移植点**。理想落点 `sunnypilot/selfdrive/controls/controlsd_ext.py:36-45 get_lat_active()`：该钩子已为 BlinkerPauseLateral（`sunnypilot/selfdrive/controls/lib/blinker_pause_lateral.py`，拨灯+低速→暂停横向）而建，与厂商 hack 同语义域，加锁存分支即为仓库惯例写法；事件在 selfdrived 补 `laneChange`（§五.2 给了两案） | 缺 |

### 二.1 sunnypilot 对状态机的既有扩展（移植时必须共存，不改其行为）

- **ALC 自动确认变道**（`sunnypilot/selfdrive/controls/lib/auto_lane_change.py`）：`AutoLaneChangeTimer` ≥ NUDGELESS 且非刹车/非连续变道时，**免司机转舵**直接进 starting。默认 NUDGE=0（原生行为：需轻舵）。desire_helper.py:93 `(torque_applied or self.alc.auto_lane_change_allowed)`。
- **LaneTurnController**（导航路口方向盘向，desire_helper.py:62-64,124-127）。
- **BlinkerPauseLateral**（`BlinkerPauseLateralControl` 参数，拨灯且低于 `BlinkerMinLateralControlSpeed` → latActive 暂停）——**与 L3 的语义区别**：它=拨灯即停（不看舵向），L3=拨灯+同向带舵才让位且锁存到灭灯。并存矩阵见 §五.3。

### 二.2 BYD 车型侧

- 信号：`carstate.py:229-230`（STALKS 左右灯）、:235-236（BSD_RADAR 盲区）、:151 `steeringTorque`、:155-156 `steeringPressed`（|torque|>STEER_THRESHOLD，5 帧防抖）。
- 无车型能力位需要开：本 fork 的 LC 链路对全车型通用（原生 openpilot 亦然，无 `laneChangingEnabled` 之类 CarParams），BYD 不需要动 interface。
- 横向授权链：`controlsd.py:115-122` `CC.latActive = get_lat_active(...) and not steerFault* and (not standstill or steerAtStandstill)`——BYD torque 路径的根因16/17/18 守卫全在 carcontroller 内，与本链路正交。

---

## §三 移植设计（L2：可配速度阈值）

1. **参数**：`common/params_keys.h` 新增（模式照抄 :136 `AutoLaneChangeTimer`）：
   `{"LaneChangeAssistSpeed", {PERSISTENT | BACKUP, INT, "20"}},`
   单位 MPH，语义照厂商：`>0` 作为状态机速度阈值；`=0` 禁用辅助变道并启用 L3。默认 "20"=现状零行为变化。
2. **读取点**：`selfdrive/controls/lib/desire_helper.py` 的 `DesireHelper` 增加 `update_params()`（照 ALC 的 `auto_lane_change.py:63-64` 模式：初始化读 + 每 N 帧刷新），`update()` 内 `below_lane_change_speed` 的比较值由 `LANE_CHANGE_SPEED_MIN` 改为 `max(self._speed_threshold_mph * speed_factor, 0)`；**`=0` 时 `below_lane_change_speed` 恒真 → preLaneChange 永不进 → 状态机自然关闭**（厂商同款效果，无需显式 if）。
3. **UI**：厂商 dp_ 参数无设备端 UI。本仓库参数系统带 UI 注册文件时（sunnypilot 设置页 json）暂不注册——USB `params put` 可改即达厂商形态；若用户要 UI 再加，单列 todo。

## §四 移植设计（L3：assist-less 让位模式）

### 四.1 符号标定：**已完成（2026-10-04 离线，docs_site 存档 rlog），结论=无需修正**

`desire_helper.py:81-83` 与厂商 L3 激活条件同式：**左=steeringTorque>0**。经五层实证链确认 BYD `SteerDriverTorque` 原生即左=正：

1. `bydcan.py:74` `LKAS_Output = apply_torque` 原值上总线（CarController 内无取反）+ 实车跟线正常 → 总线请求系=左正；
2. 根因18 回显守卫 `abs(apply_torque_last − steeringTorqueEps)` 路测通过（carcontroller.py:148）→ `MainTorque` 回显=左正；
3. 日志实测（r00000011_1）：`MainTorque`×`SteeringAngleDeg`同号 2328/反 241 → 角传感器=左正；
4. 日志实测：`SteerDriverTorque`×角 同号——1f--1 **1279/0**、r00000011_1 **2300/263**（反号样本=回盘瞬态，物理正常）→ **司机力矩=左正，无需取反**；
5. 旁证：厂商 route 7--12e 分析「-153 against a +166 driver yank」（请求与司机力矩反号=对抗）与本系一致；`carcontroller.py:102-104` Guard A `demand*drv<0` 的「对向」语义成立（既有代码无符号 bug）。

勘误记录：初测「符号反了」是假象——`controlsd.py:89` 的 `self.curvature = -VM.calc_curvature(...)` 使日志 `controlsState.curvature` 字段与转向角**全局反号**（实测 0 同/5096 反，纯由该负号产生），该 fork 曲率约定与 LC 逻辑无关（desire_helper 不读曲率，只读 steeringTorque），移植勿被此字段误导。
转向灯字段在 1f--1 / R10_000 / r00000011_1 三段存档中恒 0（当时无拨灯动作，不能证伪解析）→ 保留路测清单（§六 T4）。

### 四.2 落码位置

全部在 **`sunnypilot/selfdrive/controls/controlsd_ext.py:36-45 get_lat_active()`** + 一个新锁存类（仿 `blinker_pause_lateral.py` 结构、同目录 `assist_less_lane_change.py`）：

```
class AssistLessLaneChange:
  update(CS) -> bool:
    if speed_threshold != 0: return False        # 仅 L2=0 时启用（参数复用）
    if not left and not right: latch = False     # 灭灯解锁
    if not latch and steeringPressed and ((torque>0 and left) or (torque<0 and right)):
      latch = True
    return latch                                  # True → get_lat_active 返回 False
```

`get_lat_active` 中排在 `blinker_pause_lateral` 之后、MADS 分支之前（两者优先级并列，命中即 `latActive=False`）。

### 四.3 事件（提示司机「正在让位」）

- **案 A（首选，实现时验证）**：在 selfdrived 加事件 `EventName.laneChange` 的 assist-less 分支——但让位判定在 controlsd 进程，跨进程传播依赖 `carControl.latActive` 回读：selfdrived 订阅 carControl，可判「`modelV2.meta.laneChangeState==off` 且 `not CC.latActive` 且 CS 拨灯」还原场景——**状态回环有 1 帧延迟，可接受**。
- 案 B（保底）：不弹事件，纯让位。功能完整性优先于提示，路测反馈再补。

---

## §五 风险与并存矩阵

1. **根因16/17/18 守卫交互**：辅助变道（starting）≈2 s、同向、低力矩（<120 允许带内），不触 c=0 守卫 A/B（要求 c=0 对向 >140 或 >9 s）、不触 24 s 窗；根因18 的 EPS 回显守卫对变道轨迹照常生效（预期内，静默弃权时本就该硬停）。L3 让位模式**主动退出横向**，与守卫方向一致（司机接管即零指令），无冲突。
2. **L2=0 时**：desire 机死 → `selfdrived.py:314-329` 的 preLaneChange/blocked 事件全部失去触发源（原生厂商同款，非 bug）；灯镜像（controlsd.py:124-128）同理。
3. **ALC 共存**（L2>0）：`AutoLaneChangeTimer`≥NUDGELESS 时拨灯免舵自动变道=厂商没有的超集能力，保持现状默认 NUDGE 即可，矩阵备注进 UI 文案待办。
4. **BlinkerPauseLateral 共存**：两者同开时，低速拨灯 → pause 先命中（不等舵向）；L3 在 ≥其速度阈值或带舵时接管。语义为「更保守的谁先命中谁管」，无对抗。
5. **standstill**：BYD `CC.latActive` 的 standstill 条件（controlsd.py:113-118）与变道无关，低速 <20 mph 本就被 L2 阈值挡在 preLaneChange 外。
6. **法规/心理**：厂商双模式全保留、默认走辅助（原生行为），让位模式仅 `LaneChangeAssistSpeed=0` 显式选择，风险面与厂商一致。

## §六 实现待办拆分（未来落码，按序、各自独立 commit）

- [x] **T1 符号标定**（§四.1）：**离线已完成**——docs_site 存档 rlog 实证 `SteerDriverTorque` 左=正、与 openpilot 约定一致，**不需要任何取反改动**；路测仅剩拨灯信号在场确认（T4 内含）。
- [x] **T2 L2 参数**（§三）**已落码 2026-10-04**：`params_keys.h` `LaneChangeAssistSpeed`(INT,"20")；`desire_helper.py` update_params 读参、`=0` 映射为 `inf`（v<inf 恒真=机器全关，含**中途改 0 会解除进行中的 preLaneChange**，与厂商一致且更干净）。单测 `test_lane_change_assist_speed.py` 5 例。
- [x] **T3 L3 让位模式**（§四）**已落码 2026-10-04**：新类 `sunnypilot/selfdrive/controls/lib/assist_less_lane_change.py`（厂商 :691-703 逐行对齐，含当帧激活/双灯灭解锁/跨灯保持锁存）；接线 `controlsd_ext.get_lat_active`（排在 blinker_pause 后、MADS 分支前=任何模式下都让位）；事件走**案 A**（selfdrived 第二实例同谓词→`EventName.laneChange`，两进程同一 carState 流一帧内收敛；`params_thread` 10Hz 刷参，与 mads 同钩子）。单测 `test_assist_less_lane_change.py` 9 例。
  - **本机跑法**（仓库只带设备端 .so，pytest 在本机 import Params 即挂，既有 sunnypilot 测试同样跑不了）：`docs_site/byd_lane_change_2026-10-04/run_host_tests.py`（stub params → 5 套件 **69/69**，含把三个历史套件首次跑通）。设备/CI 原生环境直接 `pytest` 两个新文件。
- [x] **T4 路测首测 checklist**（2026-10-05..07 部分完成，docs_site/byd_logs_2026-10-05_07/ 415 段 rlog/qlog）：
  - **拨杆灯位语义已定案（推翻 §1.4 的 5=左 猜测）**：`TURN_SIGNAL_SWITCH` 是**成对集合**不是单值——厂商 disasm 10706/10756：`leftBlinker = in (2,3)`、`rightBlinker = in (4,5)`；实车逐段回放 2886 帧/段确认 **右拨 = 1→4(0.3-0.4s)→5(保持)**、**左拨 = 1→2(0.4s)→3(保持)**（瞬态是拨杆机械经过位，与保持位同侧，故厂商把两侧各两值并集）。旧单值猜测 5=左把两侧整体反了。
  - **旧映射的实测后果**：左拨（raw 2/3，采样 32 段里共 100s）`carState.leftBlinker` **恒 0** → 左变道、UI 左灯、BlinkerPauseLateral、L3 让位全部对左灯失效；右拨的 60 次 `laneChangeStarting right` **全部落在 0.2-0.3s 瞬态上**（85 个瞬态→60 次发起，命中 71%），415 段里**没有一次长 hold 被识别成右灯**（>1.2s 的右 run = 0，左 run = 81）。
  - **待复测**：修完映射后的左灯 preLaneChange→带舵→starting→finishing、盲区 blocked（BSD 位已核对无异常：`RIGHT_APPROACH` 13.3% / `LEFT_APPROACH` 5.5% 帧占、`APPROACH`=两侧之或，与字节位一致）、L3 让位、低速 <32km/h 不触发。
- [ ] T5（可选）：UI 注册 + 英文/中文文案。
- [ ] **T6 拍板项（路测数据带出的两条，均属控制行为，先不动）**：
  1. **发起窗口被放大**：旧映射只在 0.3s 瞬态里发起（85→60，命中 71%）；映射修正后一次拨灯 = 连续 2-9s 的 `one_blinker`（左 hold 实测中位 2.6s、最长 20.3s），整段都是 preLaneChange 待命——**只想示意不想变道时，方向盘稍给力矩即开变道**的暴露时间放大 ~10 倍。候选（择一）：preLaneChange 需持续 N 帧才允许 starting / 提高 torque 门限 / 保持现状（与厂商一致）。
  2. **灯不灭就再来一次**：`laneChangeFinishing → one_blinker → preLaneChange` 是 stock 语义，数据里已见 seg --51 38s 内 3 连发、--25 2 连发；叠加 §1.4 的方向翻转旧象（同一拨灯内 preLaneChange right→left 翻转 4 次、由此产生 2 次意外的 `starting left`）→ 修完映射后翻转消失，但**连续变道/变完再变回**的机制仍在。候选：starting 后要求灯重新来过（灭灯边沿）才允许下一次。

## §七 本文档的证据链

- 厂商：op_byd/selfdrive/controls/controlsd.py:121-122,376-388,691-703,730-736；lib/legacy_lateral_planner.py:9,38,77-83；lib/ 下无 desire_helper.py（未改实证）。
- 本仓库：selfdrive/controls/lib/desire_helper.py:10,41-139；selfdrive/modeld/modeld.py:220,348-353；selfdrive/controls/controlsd.py:115-128；selfdrive/selfdrived/selfdrived.py:314-329；selfdrive/selfdrived/events.py:399-428；sunnypilot/selfdrive/controls/controlsd_ext.py:14,21,34,36-45；sunnypilot/selfdrive/controls/lib/auto_lane_change.py:13-36,45,63-64,85-106；opendbc_repo/opendbc/car/byd/carstate.py:151-156,229-236；common/params_keys.h:136。
- 路测背景：docs/byd-lateral-lifecycle.md §七A/B（根因16/17、守卫 A+B）、byd-control-lessons.md、docs_site/byd_roadtest_2026-10-03/。
