# BYD 宋 PLUS DM-i 移植 · 控车专项深度 Review

> 范围：在 `byd-code-review.md`（广域代码 review）基础上，针对"设备能否真正、可靠、安全地
> 控住车"做专项深挖。重点读：横向/纵向状态机、扭矩限幅、会话握手、状态解析，并对照
> `byd-eps-reference.md` §三/§四/§六/§九/§十 的 EPS 行为规则逐条核对。
> 适用分支：`main-c3l-tici`（HEAD = `5da4f5d396`）。最后更新：2026-09-30。

---

## 一、总评

**好消息**：横向会话状态机与 EPS 协议是**逐条对齐**的，实现质量高——

- `Config=3` 全态恒流（idle 含）、3 帧 `ReqPrepare` 握手后 `Active=1` 不等待 ack、
  `armed-零`背停（21 帧）、手轻武装门（`|drv|<50`）、刹车抑制（3 入/5 出）、
  `TorqueFailed` 硬切且不复归——全部对得上 `byd-eps-reference` 的厂商实测。
- `apply_driver_steer_torque_limits` 的 driver-limit 数学核对无误：
  `allowance=120 / mult=3 / max=300` → 驾驶员拽 +166 时 OP 可反向到 −162，
  与厂商 `-153 vs +166` 同量级（`lateral.py:27-47`）。
- 50 Hz 连续流、`Counter` 连续 `(frame//2)%16`、`CheckSum(0xAF)`、echo 字段完整
  （0x316 的全部信号都被 `_ACC_MPC_STATE_ECHO_FIELDS` 覆盖，无缺省）——字节级正确。

**真正的控车风险**集中在 7 项（按影响排序）：①驾驶员扭矩阈值滞环（轻握即脱且不归）
②`cam_lkas` 丢失导致 0x316 断流 → EPS fault ③纵向 echo 拦掉原厂制动 ④部署固件
扭矩限速集与 python 的 16/frame 步进可能不匹配 ⑤MADS 横向门控解耦未测
⑥SNG 自动 resume 的 lead 锁死 ⑦`STEER_MAX_REQUEST=200` vs `STEER_MAX=300` 缩放
导致横向控制器满程只用到 67%。

> 注：本 review 跑过的验证与前一份一致（`test_byd.py` 118 passed/30 skipped；
> `test_car_interfaces` 1 passed），未重复列。

---

## 二、关键发现（按控车影响排序）

### 控车#1 ［高］`STEER_THRESHOLD=80` 落在"轻握 60–150"带内 → 轻握即脱且永不归

机理（`carstate.py:105-106`）：

```105:106:opendbc_repo/opendbc/car/byd/carstate.py
    ret.steeringPressed = self.update_steering_pressed(
      bool(abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD), 5)
```

`steeringPressed=True` → `selfdrive/car/car_specific.py:168` 产生 `steerOverride`
→ `selfdrive/selfdrived/events.py:548-553` 是 `ET.OVERRIDE_LATERAL` → `latActive=False`。

而控制器的武装门要求 `|drv| < 50`（`carcontroller.py:157`）：

```157:157:opendbc_repo/opendbc/car/byd/carcontroller.py
      if allow and abs(drv) < CarControllerParams.STEER_ARM_DRV_TORQUE:
```

`byd-eps-reference.md` §八 自己把 60–150 定义为"轻握"。于是：驾驶员轻扶（如 100）
→ `steeringPressed=True` → 横向脱离 → 但 100 > 50 → **永不重武装**。
表现为"手一搭上去就失去助力、且一直不复归"，与厂商"正面出力不退让"的语义完全相反
（EPS 参考 §六 R2：对向本身不触发锁存，厂商 −153 vs +166 零投诉）。

`docs/byd-song-plus-ops.md` §五 待办 5 想把它调到 60–70，**仍在轻握带内**，不解决问题。

**建议**：把 `STEER_THRESHOLD` 提到轻握带之上（≈150–160，或直接用 `STEER_ARM_DRV_TORQUE`
同一阈值），并对"脱/归"加显式滞环（如 engage 150 / release 50），消除"能脱不能进"
的死锁区。这是**直接决定"能不能控住"**的一项，优先级最高。

---

### 控车#2 ［中-高］`cam_lkas` 为空时 `_update_torque_lateral` 返回 None → 0x316 断流 → EPS fault

```92:93:opendbc_repo/opendbc/car/byd/carcontroller.py
    if self.frame % 2 != 0:
      return None
```

```180:184:opendbc_repo/opendbc/car/byd/carcontroller.py
    if CS.cam_lkas:
      return bydcan.create_lkas_request(
        self.packer, CS.cam_lkas, self.apply_torque_last, self.lkas_active,
        lkas_req_prepare, CarControllerParams.STEER_SESSION_CONFIG, (self.frame // 2) % 16)
    return None
```

`CS.cam_lkas` 来自 `carstate.py:50`：`self.cam_lkas = cp_adas.vl["ACC_MPC_STATE"]`。
当 `cam_lkas` 为空（`{}`）时，`update()` 里 `steer_send is None` → 不发送 0x316。

EPS 参考 §三 规则 6 明确：**0x316 断流 → SteerWarning 锁存 → 拒绝 armed →
"check multifunction video controller"**。触发窗口包括：

- 启动最初期（adas 总线首帧未到）；
- **adas 总线抖动 / 相机掉线**（实车常见）——此时 OP 应兜底发 idle echo，却反而停发；
- `cam_lkas` 被清空（本进程异常路径）。

**建议**：缓存上一帧 `cam_lkas`（`self.cam_lkas_last`），缺帧时用它（或合成一个合法
idle 帧：Config=3 / Active=0 / Lanes 0/0 / SETME 取上次值）继续 50 Hz 发送，而不是
返回 None。这是"断流不 fault"的兜底，优先级高。

---

### 控车#3 ［高］纵向 echo-relay 规则拦掉原厂主动制动（控车视角）

`byd.h:220-223` 的 `AccControlActive=1` 无条件要求 `controls_allowed`：

```220:223:opendbc_repo/opendbc/safety/modes/byd.h
    if (acc_control_active) {
      violation |= !controls_allowed;
    }
    violation |= max_limit_check(desired_accel_raw, 40, -80);
```

而 `_update_longitudinal` **无 engagement 门控**（`carcontroller.py:240-250`），
`create_accel_command` 在 `!enabled || !active` 时**原样回显雷达帧**（含雷达的
`AccControlActive=1`，`bydcan.py:198-234`）。同时 `byd_fwd_hook` 已把 0x32E 的
bus2→bus0 挡死（`byd.h:264`），OP 是 bus0 上唯一 0x32E 源。

**控车后果**（比"框架层面"更严重）：OP 未介入但原车 ACC 正在主动制动时
（`controls_allowed=0`），OP 的 echo 被 tx hook 拦掉 → **bus0 彻底没有 ACC_CMD** →
车辆失去原厂 ACC 制动（含 AEB 触发时的纵向减速）。可达场景：刹车瞬态、OP 因故未 engage
而驾驶员用原车 ACC 巡航、MADS 横向脱离但纵向保留等。

`byd-vendor-alignment.md:78` 自己写明要避免"拦掉原厂刹车"，落码却恰恰实现成封死——
见 `byd-code-review.md` H2。此处只强调**控车影响**：这是 Phase 2 上实车前必须修的
安全项。

---

### 控车#4 ［中-高］部署固件扭矩限速集与 python 的 16/frame 步进可能不匹配

python 侧每帧步进上/下限 = `STEER_DELTA_UP/DOWN = 16`（`values.py:34-35`），
经 `apply_driver_steer_torque_limits` 施加（`lateral.py:40-45`）。

但厂商 ELF 里**存在两套**限速集（`byd-control-lessons.md:17`）：

> ELF：`BYD_STEERING_LIMITS=[300,18,18,…,46]`、`LOW=[300,9,9,46]`、`HIGH error=200`、
> `ALT=[150,50,50]`；`004_get_byd_torque_limits` 按 flag 选 LOW/HIGH。

EPS 参考 §九/§十 说"现架构固件实测限 18""allowance 120 / rate 18/18"——即宋 22 走
HIGH 集。若如此，python 的 16 ≤ 18，**固件放行，安全**。

**风险**：激活集由固件内部 flag 决定。若部署固件对宋 22 实际选了 `LOW=[300,9,9]`
（rate 9），则 python 的 16/frame 会被固件安全层**整帧拒绝**——表现为 ramp 起步
（0→目标）期间 0x316 被零星丢弃 → EPS 看到不连续扭矩 → 触发 SteerWarning / 锁存。

**建议**：实车抓 0x316，确认从 idle→active 的爬坡是否被固件丢帧；核对部署固件对宋 22
激活的是 HIGH(rate 18) 而非 LOW(rate 9)。若确认 LOW，则把 python `STEER_DELTA_UP/DOWN`
降到 ≤9，或推动固件固定为 HIGH 集。

---

### 控车#5 ［中］`steer_error` 门控过粗：err=2（可恢复预警）即整会话退出

```121:122:opendbc_repo/opendbc/car/byd/carcontroller.py
    allow = CC.latActive and not self.lkas_brake_inhibit and not CS.out.standstill \
      and not CS.torque_failed and not CS.steer_error
```

`CS.steer_error` 是原始 `SteerErrorCode`（`carstate.py:43`）。只要非零（含 err=2，
约 0.5 s 可恢复预警），`allow=False` → 整会话 stand-down。

张力：EPS 参考 §七/§六 明确 err=2 是**可恢复**的，"我方停止请求后 ~0.3 s 内全部清零、
无锁存"——**停请求本身就是 doc 记载的恢复路径**（预防 = 不制造 rail）。所以 stand-down
于 err=2 与 doc 自洽。但厂商**全程不停请求也能自清**，而我们一旦退出再重武装，又过
手轻门（`|drv|<50`）；若此刻驾驶员正持盘（err=2 的常见诱因），就**不再归** → 会话犹豫，
正是 EPS 参考 R16 把"会话犹豫"判为锁存真凶的点。

**建议**：区分 err=2（缩幅/降权而非整会话退出）与 err=4（硬切，已与 `torque_failed`
同帧）。err=2 时把请求按比例收缩（如 ×0.5）而非归零，既满足"停止违规"又避免退出犹豫。
需权衡：这是行为调参，建议实车先观察 err=2 出现频率再定。

---

### 控车#6 ［中］MADS 横向门控（`is_lat_active`）与 python `CC.latActive` 解耦未测

固件放行判据是 `is_lat_active() = controls_allowed || mads_is_lateral_control_allowed_by_mads()`
（`lateral.h:10-12`），而 BYD 是**唯一**显式在 rx_hook 驱动 `mads_state_update` 的接线点
（`byd.h:112-119`）。python 侧用的是 `CC.latActive`。

- 安全方向（固件放行但 python 发 0）：不会出事。
- **危险方向**（python `CC.latActive=True` 但固件 `is_lat_active=False`，例如 MADS 状态
  与 selfdrived 短时错位）：固件会**拦掉非 0 的 0x316** → 控车瞬间丢失，且 python 侧
  毫无感知（仍以为在控）。

而 MADS 相关测试**全部 skip**（`_acc_state_msg is not implemented for this car`），
这条链路**零覆盖**。

**建议**：补 MADS 测试（至少覆盖 `controls_allowed=False` + MADS 允许 lateral 时 0x316
仍放行）；上线前打印两侧 `controls_allowed` / `is_lat_active` / `CC.latActive` 做一致性核对。

---

### 控车#7 ［中-低］SNG 自动 resume 的 `lead_valid` 一旦 False 即永久锁死

```215:215:opendbc_repo/opendbc/car/byd/carcontroller.py
      self.lead_valid = CC.hudControl.leadVisible and self.lead_valid
```

```227:229:opendbc_repo/opendbc/car/byd/carcontroller.py
      elif self.lead_valid and self.frame > self.sng_next_press_frame:
        can_sends.append(bydcan.send_buttons(self.packer, (CS.counter_pcm_buttons + 1) % 16))
        self.resume_counter += 1
```

`lead_valid` 初始 False；进入静止（`is_sng_check` 置位）瞬间被设 True，此后
`leadVisible and lead_valid`——**只要任意一帧 `leadVisible=False`，`lead_valid` 永久变
False**，直到下次 `is_sng_check` 重置。后果：路口短暂无前车（lead 检测丢失一瞬）→ 之后
即便前车起步也不再自动 resume，需人工踩油门/按 SET。

**建议**：改成"最近 N 帧内有过 lead"的滑窗，而非永久锁。或对无 lead 的场景也允许
起步（停车取消防护已由 `ret.cruiseState.standstill` + `startingState` 覆盖）。

---

### 控车#8 ［中-低］`STEER_MAX_REQUEST=200` vs `STEER_MAX=300` 缩放 → 横向满程只到 67%

```96:103:opendbc_repo/opendbc/car/byd/carcontroller.py
    demand = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
    ...
    demand = max(-CarControllerParams.STEER_MAX_REQUEST, min(CarControllerParams.STEER_MAX_REQUEST, demand))
```

`CC.actuators.torque` 由横向控制器输出，标定为 ±1.0 = `STEER_MAX=300`。但被
`STEER_MAX_REQUEST=200` 截到 ±200 → **横向控制器满程只用到 67%**。急弯 / 高速变道 /
泊车大转角时，控制器以为已给满，实际只到 200，可能出力不足、跟不上曲率。

EPS 参考 §九 说厂商实测 max=193、p90=128——200 包络确实覆盖真实需求，所以**正常驾驶
够用**；但这是"安全包络"牺牲了"峰值余量"。

**建议**：要么把 `STEER_MAX` 也设成 200（让缩放与包络一致，横向 PID 满程即真实满程），
要么确认横向 PID 增益已按 200 调过。当前若 PID 还按 300 标定，会系统性偏弱。

---

### 控车#9 ［低，记录］`standstill` 退出横向会话，每次起步重做握手

```121:121:opendbc_repo/opendbc/car/byd/carcontroller.py
    allow = CC.latActive and not self.lkas_brake_inhibit and not CS.out.standstill \
```

`allow` 含 `not CS.out.standstill` → 停车时 `lkas_active=False`，起步后需重新 3 帧
握手（~60 ms）。厂商在低速/泊车保持会话（EPS 参考 §二："2-23 km/h 能控"）。影响：
频繁停车起步的场景有约 60 ms 横向空档。可接受，记录备查。

---

### 控车#10 ［澄清］H1（0x122 `vehicle_moving`）对本机控车影响有限

前一份 review 的 H1 位于 **panda 固件安全层**（`byd.h:52-57`）。python 这边的
`ret.vEgo` / `ret.standstill` 用的是 **0x1F0**（`carstate.py:65,73`），与 0x122 无关，
**计算正确**。

因此 H1 只影响固件层的 `controls_allowed` 刹车退出细节、`byd_button_checks` 的
"仅静止可 resume" 伪造门、以及 MADS 状态机输入——**不直接影响 python 横向逻辑**。
且 `controls_allowed` 因刹车上升沿跌落时，固件临时挡非 0 扭矩，而 python 的
`_handle_brake_inhibit`（`carcontroller.py:43-56`）已**独立**覆盖该场景，二者重叠安全。
（H1 仍建议修，但优先级可按"框架层正确性"而非"控车失效"评估。）

---

## 三、控车链路正向核对（确认正确的部分）

| 规则（EPS 参考） | 代码落点 | 结论 |
|---|---|---|
| `Config=3` 全态恒流（含 idle） | `bydcan.py:69` + `values.STEER_SESSION_CONFIG=3` | ✓ |
| 3 帧 `ReqPrepare` 握手后 `Active=1`，不等 ack | `carcontroller.py:156-169`（`retry_burst`） | ✓ |
| `Active=1/State=1/Lanes 2/2` 才出力 | `bydcan.py:72-82` | ✓ |
| idle = `Config=3/Active=0/Lanes 0/0`，50 Hz 恒流 | `bydcan.py:94-103` + `update():268-281` | ✓ |
| 手轻武装门 `|drv|<50` | `carcontroller.py:157` | ✓ |
| `armed-零`背停 21 帧 | `carcontroller.py:137-142, 150-155` | ✓ |
| 刹车抑制 debounce（3 入/5 出） | `carcontroller.py:43-56` | ✓ |
| `TorqueFailed` 硬切且不复归 | `carcontroller.py:124-132` + `carstate.py:164` | ✓ |
| driver-limit 数学（`±166 → ∓162`） | `lateral.py:33-37` | ✓ |
| 步进 ≤ 固件（16 ≤ 18） | `values.py:34-35` vs 固件 18 | ✓（前提：固件选 HIGH 集，见 #4） |
| `CruiseActivated` 不作为许可门 | 代码从不读它做门控 | ✓ |
| echo 字段完整（0x316 无缺省） | `bydcan.py:42-48, 66` | ✓ |
| 纵向介入时 accel clip `[-4,2]`、ComfortBand/Jerk 与厂商字节级一致 | `bydcan.py:198-234` | ✓ |

---

## 四、建议落地顺序（控车优先）

| # | 项 | 级别 | 一句话理由 |
|---|---|---|---|
| 1 | **控车#1** `STEER_THRESHOLD` 提到 150+ 或加滞环 | 高 | 直接决定"手搭上去还控不控得住" |
| 2 | **控车#2** `cam_lkas` 缓存兜底，缺帧发 idle 而非停发 | 中-高 | 决定"总线抖动/掉线不 fault" |
| 3 | **控车#4** 实车确认部署固件对宋 22 激活 HIGH(rate 18) 而非 LOW(9) | 中-高 | 决定"ramp 不被固件丢帧" |
| 4 | **控车#3** 纵向 echo 修复（方案 A/B 见 H2） | 高 | Phase 2 上实车前必修，防丢失原厂制动 |
| 5 | **控车#5/#6/#7** err=2 缩幅而非退出 / 补 MADS 测试 / lead 滑窗 | 中 | 行为调参与覆盖补全 |
| 6 | **控车#8** `STEER_MAX` 与包络一致或确认 PID 按 200 调 | 中-低 | 急弯峰值余量 |

> 控车#1/#2/#4 是"能不能控住"的硬前提，建议在下次实车前优先处理；控车#3 是 Phase 2
> 纵向专属，纵向开关默认关、尚未刷固件，可排在纵向实车前。
