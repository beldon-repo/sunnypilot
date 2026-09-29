# BYD 宋 PLUS DM-i 移植 · 代码 Review 报告

> Review 范围：`main-c3l-tici` 分支 2026-09-28 ~ 2026-09-30 的 30 个提交
> （`c510f23aae` ~ `5da4f5d396`），18 个文件 +2003 / −176。
> 配套阅读：`byd-current-status.md`（现状快照）、`byd-eps-reference.md`（EPS 协议层）、
> `byd-vendor-alignment.md`（厂商对齐路线）、`byd-song-plus-ops.md`（部署与路测规程）。
> 最后更新：2026-09-30。

---

## 一、Review 概览

本轮提交的主题是横向控制架构重写为**厂商会话架构**（根因 15/16）、Phase 2 纵向
（透明 ACC 替换）、固件与参数对齐厂商真值，以及 5 篇文档。

### 变更面

| 层 | 文件 | 变更量 |
|---|---|---|
| 控制器 | `car/byd/carcontroller.py` | +229 |
| 报文 | `car/byd/bydcan.py` | +154 |
| 参数 | `car/byd/values.py` | +137 |
| 状态 | `car/byd/carstate.py`、`interface.py` | +50 |
| 固件安全 | `safety/modes/byd.h` | +113 |
| 安全测试 | `safety/tests/test_byd.py`、`common.py` | +121 |
| DBC | `dbc/byd_general_pt.dbc`（新增 BO_ 815 ACC_AEB） | +11 |
| NN 权重 | `sunnypilot/neural_network_data/.../BYD_SONG_PLUS_DMI_22.json` | +547 |
| 固件产物 | `panda/board/obj/panda{,_h7}.bin.signed` | 二进制 |
| 文档 | `byd-control-lessons.md` 等 6 篇 | +817 |

### 已执行的验证

- `opendbc_repo/opendbc/safety/tests/test_byd.py` → **118 passed / 30 skipped**
- `opendbc_repo/opendbc/car/tests/test_car_interfaces.py -k BYD` → **1 passed**
- DBC 位域与 `byd.h` 解码逐条比对（`LKAS_Output 16|11`、`MainTorque 8|12`、
  `SteerDriverTorque 24|12`、`AccControlActive 44|1`、`AccState 19|3`）→ **全部正确**
- 分支**已全部推送** origin（HEAD = `origin/main-c3l-tici` = `5da4f5d396`），
  文档中"未推送"已过时

### 结论一句话

架构方向（照搬厂商会话状态机、参数锚定真值）是正确的，且控制器状态机本身无逻辑漏洞；
**但固件侧有两个会在实车上真正咬人的回归**：`vehicle_moving` 被死信号 0x122 驱动、
以及 Phase 2 纵向的 echo-relay 规则拦掉了它自己声称要保护的原厂制动。

---

## 二、高危问题

### H1. `byd.h` 重新引入 0x122 驱动 `vehicle_moving`，与已刷固件和文档快照相反

```52:57:opendbc_repo/opendbc/safety/modes/byd.h
    if (msg->addr == 0x122U) {
      uint16_t left_rear = ((msg->data[5] << 8) | msg->data[4]);
      uint16_t right_rear = ((msg->data[7] << 8) | msg->data[6]);
      vehicle_moving = (left_rear | right_rear) != 0U;
      UPDATE_VEHICLE_SPEED(((left_rear + right_rear) / 2.0) * 0.1 * KPH_TO_MS);
    }
```

已刷固件的快照 `docs/firmware-safety_byd.h` 明确写的是**不能用**：

```39:41:docs/firmware-safety_byd.h
    // WHEEL_SPEED: 0x122 reads all zeros on Song Plus DM-i (verified on real
    // vehicle logs), so it must NOT be used for speed or motion detection.
    // Kept in the RX checks only for liveness.
```

实车上 0x122 恒为全零 **50 Hz**，0x1F0 ESP_SPEED 是 **20 Hz**，所以"最后一次写入"
绝大多数来自 0x122 → `vehicle_moving` 基本恒为 `false`。`byd.h` 第 48-51 行的注释
"on the real car the 0x1F0 branch below overrides it every 50 ms" 是**事实错误**
（0x1F0 只有 20 Hz，覆盖不了 50 Hz 的 0x122）。

**后果（三条独立的防御被削弱）**：

1. `byd_button_checks` 的"仅静止允许 resume 伪造"固件防线失效——行驶中
   `vehicle_moving=false` 也会放行：

```130:132:opendbc_repo/opendbc/safety/modes/byd.h
    if (other_btns || vehicle_moving || (updown_cmd != 0U && updown_cmd != 3U)) {
      violation = true;
    }
```

2. `safety.h:354/360` 的刹车 / 动能回收退出条件
   `brake_pressed && (!brake_pressed_prev || vehicle_moving)` 语义退化为纯上升沿。
3. `byd_mads_update()`（`byd.h:118`）把错误的 `vehicle_moving` 喂给 MADS 状态机。

**建议**：删掉 0x122 的速度 / `vehicle_moving` 解析，只保留 RX liveness（0x122 已在
`byd_rx_checks_*` 中，删解析不影响 liveness）；或在两个源之间做"只升不降"合并。
同时修正注释。

---

### H2. Phase 2 纵向的 echo-relay 规则，与文档自述的风险直接矛盾

文档 `byd-vendor-alignment.md` §4.2 明确识别出这个风险：

> 雷达主动制动时（OP off、AccControlActive=1 echo）固件若按 hyundai 模式封死会拦掉
> 原厂刹车 → byd.h 0x32E 用 echo-relay 规则

但落码的规则**恰恰就是它说的"封死"**：

```220:223:opendbc_repo/opendbc/safety/modes/byd.h
    if (acc_control_active) {
      violation |= !controls_allowed;
    }
    violation |= max_limit_check(desired_accel_raw, 40, -80);
```

而 `_update_longitudinal` **没有 engagement 门控**，无条件每帧发送：

```240:250:opendbc_repo/opendbc/car/byd/carcontroller.py
    if self.frame % 2 == 0:
      raw_cnt = (self.frame // 2) % 16
      resume = CC.actuators.longControlState == LongCtrlState.starting
      can_sends.append(bydcan.create_accel_command(
        self.packer, CC.actuators.accel, CC.enabled, CC.longActive, resume,
        CS.radar_acc_msg, raw_cnt))
```

`create_accel_command` 在 `!enabled || !active` 时**原样回显雷达帧**，包括雷达自己的
`AccControlActive=1`（`bydcan.py:201-205, 217`）。同时 `byd_fwd_hook` 已把 0x32E 的
bus2→bus0 挡死（`byd.h:264`），OP 是 bus0 上**唯一**的 0x32E 源。

于是：OP 未介入但原车 ACC 正在主动制动时（`controls_allowed=0`），OP 的 echo 被 tx hook
拦掉 → **bus0 上彻底没有 ACC_CMD**，车辆失去原厂 ACC 制动。可达场景包括刹车瞬态、
OP 因故未 engage 而驾驶员用原车 ACC 巡航等。

现有测试测不到这个洞——`test_accel_cmd_echo_relay` 只测了 `acc_control_active=0`。

**建议**：区分"纯 echo"与"OP 覆写"。两个可选方案：

- **A（固件侧）**：缓存最近一次雷达帧的 accel 原始值，若 TX 的 accel 与之相同（未覆写）
  则只做范围检查、不要求 `controls_allowed`。
- **B（python 侧）**：在未 active 时清掉 `AccControlActive`，固件对该形态只做范围检查。

无论选哪个，都要补一条"OP off + 雷达 `AccControlActive=1` + 负加速度"的回归测试。

---

## 三、中危问题

### M1. 仓库 `byd.h` 与实际刷写固件不一致，而纵向开关是运行时 param

`interface.py` 允许**不刷固件**就打开纵向：

```48:50:opendbc_repo/opendbc/car/byd/interface.py
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= int(BydSafetyFlags.LONGITUDINAL)
```

但 `panda/board/obj/*.bin.signed` 是从仓库外的 `~/Documents/op/panda_fw_base`（0.9.x）
构建的，`byd-current-status.md` §四 说得很清楚：那棵树**没有** Phase 2 纵向、也没有 MADS。

**后果**：param=2 下发给不认识该 flag 的固件 → 0x32D/E/F 不在 TX 白名单（全部被拦）→
纵向静默失效；且 fwd 不挡 bus2→bus0 → OP 帧与雷达帧在 bus0 撞帧、counter 重复。
目前"自洽"仅因为 param 默认关 + 文档 §五 C-2 写了"开之前必须移植"，**代码零防护**。

**建议**：启动时校验固件能力（如 `boardd` 上报的 safety 版本 / hash），不匹配就拒绝启用
纵向并报警；或把纵向从纯 param 改为"param + 固件版本门禁"。

---

### M2. `STEER_THRESHOLD=80` 落在"轻握 60–150"区间内，造成"能脱不能进"的滞环

`carstate.py:105-106` 用 `STEER_THRESHOLD=80` 判定 `steeringPressed`
→ `car_specific.py:168-169` 产生 `steerOverride`
→ `events.py:548-553` 是 `ET.OVERRIDE_LATERAL` → `latActive=False`。

而控制器的武装门要求 `|drv| < 50`：

```157:157:opendbc_repo/opendbc/car/byd/carcontroller.py
      if allow and abs(drv) < CarControllerParams.STEER_ARM_DRV_TORQUE:
```

`byd-eps-reference.md` §八 自己定义"轻握 60–150"。于是轻握（如 100）
→ `steeringPressed=True` → 横向脱离 → 但 100 > 50 → **永不重武装**。
表现为"轻扶方向盘即失去助力且不复归"。

文档 §五 待办 5 想把它调到 60–70，**仍在轻握带内**，不解决问题。

**建议**：`STEER_THRESHOLD` 提到轻握带之上（≈150–160）；或让 `steeringPressed` 与 arm 门
共用同一阈值并加显滞环（如 engage 150 / release 50），避免死锁区。

---

### M3. 文档漂移已足以误导下一次实车

| 位置 | 问题 |
|---|---|
| `byd-current-status.md:23` | HEAD 记 `0cfd780854`，实际 `5da4f5d396` |
| `byd-current-status.md:83` | "推送 origin：全部未推送" —— 现已全部推送 |
| `byd-song-plus-ops.md` §三 / §四 | 仍在描述**已删除**的旧架构：对向 ±20 跟随、>40° 退让、>50° 锁 10s、"armed 只在需求 >6 时进入"、`STEER_MAX=200`、固件速率 10/12。`13b51bb5c4` 后全部相反（现行：满权限出力、无角度/速率门、`STEER_MAX=300`、rate 18/18） |
| `byd-eps-reference.md:26` vs `carstate.py:187-191` vs `byd.h:299` | 0x32D/0x32E 频率出现三个值：20 Hz / 50 Hz / 10 Hz |
| `byd-song-plus-ops.md:72` | 待办 #7 "24 个测试失败" 已修复（现 118 passed） |

---

### M4. 二进制固件 blob 入库且不可复现

`panda/board/obj/panda.bin.signed`（57696 → 57700）与
`panda_h7.bin.signed`（**83236 → 67044，−16 KB**）被跟踪提交；构建树在仓库外，
仓库内无 SConstruct，无构建 provenance 记录。

**建议**：提交信息 / 文档记录构建树 commit + 编译命令 + 产物 sha256，或把产物移出 git。

---

## 四、低危 / 整洁性

- **死代码**：`bydcan.create_fake_eps_state()` 从未被调用；`CarState.lkas_prepared` /
  `eps_state_msg` 只写不读；`CarControllerParams` 中 `STEER_ERROR_MAX*`、
  `STEER_DEACTIVE_INTERVAL_MS`、`STEERING_TORQUE_LIMIT_SPEED`、
  `STEERING_INNER_EPS_SCALE_POINT`、`NON_LINEAR_TORQUE_PARAMS`、`*_LOW` 系列均无引用。
- **死字段**：`BYD_TORQUE_STEERING_LIMITS.max_torque_error = 350`（`byd.h:186`）对
  `TorqueDriverLimited` 无效——`lateral.h:82-89` 只在 `TorqueMotorLimited` 分支使用
  `dist_to_meas_check`。
- **隐式约束**：`USE_ANGLE_STEERING` 是模块级常量 `False`，
  `BydSafetyFlags.ANGLE_STEERING` 永不被 `interface.py` 设置；`byd_init` 的 if/else-if
  在 `ANGLE|LONG` 同时置位时会**静默丢弃** LONG（`byd.h:313-322`）。当前组合不可达，
  但属未文档化的隐式前提。
- **测试覆盖缺口**：MADS 测试全部 skip（`_acc_state_msg is not implemented for this car`），
  而 `byd.h:112-119` 是 BYD 唯一的 MADS 接线点；无 param=3（ANGLE|LONG）组合测试；
  `test_steer_req_bit` 因 `NO_STEER_REQ_BIT=True` 被跳过。
- **`no_lockout_modes` 扩容**：`common.py:942-943` 把 `TestBydSafetyLong` 加进上游标注
  `# TODO: this should be blocked` 的白名单，等于为一个新模式延续了已知缺陷（与前两个
  BYD 模式一致，但应显式说明）。
- **变量遮蔽**：`byd.h:219` 的 `bool violation = false;` 遮蔽外层同名变量；该块走 `tx=false`、
  其余走外层 `violation`，两条路径混用易读漏。
- **魔数**：`carcontroller.py:197` 的 `CS.out.steeringTorqueEps > 15`（死路径，建议连同
  角度路径一起清理或加 TODO）。
- **冗余**：`bydcan.py:42-48` 的回显字段里 `LKAS_Output` / `LKAS_Active` / `LKAS_Config` /
  `LKAS_State` / `LeftLaneState` / `RightLaneState` 在三个分支中被全部覆写，
  "回显"语义可读性打折。

---

## 五、做得好的地方

1. **参数纪律扎实**：所有标量锚定厂商 `values.json` / ELF struct，并有"禁止日志反推"的
   明规则；固件与 python 的关键量保持一致（driver-limit 120/3、rate 18 vs 16、
   max_torque 300）。
2. **位域解码正确**：逐条比对 DBC 与 `byd.h`，`LKAS_Output`、`MainTorque`、
   `SteerDriverTorque`、`AccControlActive`、`AccState` 全部无误。
3. **`byd_fwd_hook` 双向阻断**（`byd.h:241-267`）的理由——同时防止 OP 自环撞帧——
   注释清楚，是这类移植里容易漏的点。
4. **ACC_AEB 用 48-bit `@1+` 小端 PAYLOAD** 并注明"`@0+` 大端往返丢 byte0"
   （`bydcan.py` 与 `byd-vendor-alignment.md:76`），是有实证支撑的坑位记录。
5. **根因 16 的二次修正**（`51b1a25a5f`）在代码、文档、注释三处同步，并明确写了 v1 错在
   哪（把 EPS 的相位标志当许可 → 双向等待死锁）。这种"错因留档"对新 session 极有价值。
6. **控制器状态机本身无逻辑漏洞**：已追过 `lkas_active` 真假 × `allow` 真假 ×
   `torque_failed` 的全部组合，`new_torque` 在每个分支都有赋值，无 `UnboundLocalError`
   风险；`silence_counter` 背停与 re-burst 的交互也自洽。

---

## 六、建议的下一步（按优先级）

| # | 项 | 级别 | 说明 |
|---|---|---|---|
| 1 | 修 H1（0x122 → `vehicle_moving`） | 高 | 纯删减、零风险，与已刷固件快照对齐 |
| 2 | 修 H2（0x32E echo-relay）+ 补回归测试 | 高 | 必须在 Phase 2 上实车之前完成 |
| 3 | M1：给 `AlphaLongitudinalEnabled` 加固件能力门禁 | 中 | 防止不刷固件开纵向 |
| 4 | M2：`STEER_THRESHOLD` 与 arm 门解滞环 | 中 | 直接影响手感验收 |
| 5 | M3：刷新 `byd-song-plus-ops.md` §三/§四 到现架构 | 中 | 顺手修正 current-status 的 HEAD 与推送状态 |
| 6 | M4 + 低危项：固件 provenance、清死代码、补 MADS / param 组合测试 | 低 | — |

> H2 需先确认采用方案 A（固件缓存雷达 accel 值）还是方案 B（python 侧清
> `AccControlActive`），再动代码。
