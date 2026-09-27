# BYD 宋 PLUS DM-i（2022 款）：LKAS 横向控制调试全记录

> 时间：2026-09-27 ~ 09-28 · 前置文档：`byd-panda-fw-build.md`（固件构建）、`byd-panda-recovery.md`（固件排障）
> 本文整理横向控制（LKAS）从"方向盘完全不动 + 两个仪表故障"到逐步收敛的全部根因、修复与遗留问题。

---

## 一、车辆与架构背景

- 车辆：比亚迪宋 PLUS DM-i 2022 款（**低配，无天神之眼**，DiPilot 相机 + 前毫米波雷达）
- 设备：c3l 兼容板（tici 类），自建 0.9.x 基线 panda 固件（官方 0.10.1 在该板 assert_fatal）
- harness：截断相机/雷达的车辆 CAN，panda bus 2 接相机侧，bus 0 接车身侧

### 总线拓扑（实车 CAN 日志实证，segment 00000016）

| 总线 | 内容 | 地址数 |
|---|---|---|
| bus 0 | 底盘/动力/车身全部报文（EPS 0x11F/0x318、ESP 车速 0x1F0、轮速 0x122、挡位 0x242、踏板 0x342、BCM 0x12D、按键 0x3B0、BSD 0x418…） | 103 |
| bus 2 | **仅 MPC 自己发的 5 条**：0x316（LKAS 命令）、0x32D（HUD）、0x32E（ACC 命令）、0x32F、0x432 | 5 |
| 交集 | **0 个** —— 原车不直接桥接两条总线，MPC 靠原车网关白名单获取车辆状态 | — |

**结论：panda 必须全量转发 bus0↔bus2**（相机和雷达都靠它喂）：
- 全量转发：相机/雷达活着、按键有效、原车 ACC 正常（仅剩"视频控制器"故障）
- 白名单转发（10 条）：**雷达饿死**（"请检查前置毫米波雷达"）——已回退
- 全部 block：相机 + 雷达双饿死

---

## 二、最终确认的 0x316 协议（LKAS 命令帧）

ACC_MPC_STATE（0x316，50Hz，8 字节），是 **EPS 的唯一 LKAS 命令源**。协议要点全部来自实车日志的字节级对比（sendcan 中 OP 帧 vs bus2 上相机原发帧）：

1. **SETME 字段不可缺省**：`SETME2_0x1=1`、`SETME5_0x1=1`、`SETME7_0x3=3`、`MPC_State`、`AutoFullBeamState/OnOff` 等，相机原发非 0。**从零构造（填 0）= 非法帧** → ADAS 域故障。
   → **必须 echo 相机原发帧字段**（carstate 从 bus2 解析 `cam_lkas`，bydcan 只 override 扭矩相关字段）。
2. **CheckSum**：`byd_checksum(0xAF, dat)` 已验证正确（相机 5 帧全部 MATCH）。
3. **握手三步**：`LKAS_ReqPrepare=1` → EPS 置 `LKAS_Prepared`（0x318 bit0）→ 之后才接受 `LKAS_Active=1`。**跳过第一步 EPS 永远不 armed**。
4. **LeftLaneState/RightLaneState 必须 = 2**（无论 engage 与否），否则 EPS 不执行 LKA（"报文在发但方向盘不动"）。
5. **LKAS_Config 组合合法性**：`Config=1(ALARM) + LaneState=2` 是非法组合 → EPS 锁存 SteerWarning。请求 LKA 时 `LKAS_Config=2`。
6. **50Hz 连续流，不可断**：EPS 的 LKAS 子系统需要**持续的** 0x316 流。断流 → `SteerWarning=1` 锁存 → 拒绝 armed → MPC 报"请检查多功能视频控制器"。

---

## 三、根因链（按因果排序）

```
[根因 1] OP 只在 engaged 时发 0x316，且固件 block 了相机的 0x316（bus2→bus0）
         → OP 未 engage 时 EPS 完全收不到 0x316（每段未 engage 日志 SteerWarning=100%）
         → EPS 拒绝 armed → 横向不动 → MPC 报"视频控制器"
[修复] carcontroller 无条件 50Hz 发 0x316（idle 帧 echo 相机字段，LKAS_Active=0）

[根因 2] 0x316 从零构造，缺 SETME_* 等必填字段
         → EPS/MPC 收到非法帧
[修复] echo 相机 bus2 的 0x316 全字段，只 override LKAS_Output/Active/Counter

[根因 3] card 启动时的 UDS/isotp 固件查询帧（0x7e0/0x7e1/0x7d0/...）经 elm327
         阶段的 panda 默认转发到 bus2 → MPC 收到协议外诊断帧
[修复] SKIP_FW_QUERY=1（必须在 card 进程内设置——launch_env.sh 环境变量
       在该设备的启动链中到不了 card 进程，实测 /proc/<card>/environ 无该变量）

[根因 4] pandad 在 elm327 固件查询阶段停留 ~23 秒才切 byd safety，
         selfdrived 的 controlsMismatch 容忍只有 10s/200 帧
         → OP enable 后立刻 IMMEDIATE_DISABLE（"Controls Mismatch" 误报）
[修复] 容忍放宽到 60s/6000 帧

[根因 5] 相机 0x316 被 block 后按键 0x3B0 也曾被 block
         → stalk 按键到不了 MPC（"按钮无效"）
[修复] 0x3B0 保持转发（全量转发下自然满足）

[根因 6] c3l 无 DMS 摄像头，dmonitord 不发布 driverMonitoringState
         → "Communication Issue Between Processes driverMonitoringState"
[修复] selfdrived 对无 DMS 设备 ignore driverCameraState + driverMonitoringState

[根因 7] 安全带信号编码未验证（系了安全带仍报未系）→ 阻止 engage
[修复] seatbeltUnlatched = False（车端本身有安全带告警，OP 不重复把关）
```

---

## 四、修复清单与提交

| 提交 | 内容 |
|---|---|
| `1432de9` | 0x316 改为 echo 相机字段（SETME 修复） |
| `5fd62ef` | 固件 fwd_hook 不再 block 0x3B0（按键恢复） |
| `93f44ce` | selfdrived ignore driverMonitoringState |
| `557495f` | 安全带不检测 |
| `bbd39db`/`18ecb20` | controlsMismatch 容忍 10s→30s→60s |
| `ca7f7d8` | LeftLaneState/RightLaneState=2 + LKAS_Config/State echo |
| `3021ffb` | LKAS_ReqPrepare 握手 |
| `d9cf68f`/`b16c4a6`/`e0b6f80` | SKIP_FW_QUERY（launch_env 两份 + 进程内兜底） |
| `8496a71` | **固件恢复全量转发 + 无条件 50Hz 发 0x316**（本轮核心） |

**注意**：launch_env.sh 有**两份**——根目录版和 `sunnypilot/system/hardware/c3/launch_env.sh`（tici 实际用的是后者）。且环境变量在该设备启动链中不可靠，**card 进程内的 `os.environ.setdefault` 才是兜底**。

---

## 五、遗留问题 / 待验证

1. ~~本轮修复未实车验证~~ → **已验证（见第七节）**：仪表故障全部消失，0x316 持续供帧生效。
2. **横向生效后的标定**：`STEER_THRESHOLD=56`（steeringPressed 灵敏度）、`steerRatio=15`（占位）、`STEER_SOFTSTART_STEP`、LatControl 参数均为占位/未标定，实车调参时注意 steeringOverride 频率。
3. **libsafety.so 与源码漂移（既存）**：仓库的 `libsafety.so`（Sep26 构建）与当前 byd.h 源码不一致，重建后 byd 的 24 个 brake/mads 测试失败（mads 框架与 byd 模式结构性不兼容：`mads_state_update` 仅在 TX `check_relay=true` 时被调用，而 byd 的 0x3B0 原生在 bus0 不能开 check_relay，否则实车误触发 relayMalfunction 全车失能）。测试基线：旧 so + 旧 test = 8 failed；新 so = 12~24 failed。**此为测试框架问题，非车控代码缺陷**，后续单独处理。
4. **纵向**：原车 ACC 全权负责（OP 不控纵向），OP 只做静止 resume 按键伪造（SNG）。"纵向有效果"即原车 ACC 在工作。
5. **echo 空闲帧的风险待观察**：连续发 0x316 空闲帧是否影响 ACC 按钮（历史问题仅出现在缺 SETME 的从零构造帧，echo 帧理论上无碍，社区实现同样连续发）。
6. **cruiseMismatch（NO_ENTRY）**：stock ACC enabled 而 OP 未 enabled 持续 6 秒触发，会阻止 engage。它与"OP 为什么没 enable"互为因果，需在 engage 链排查中一并厘清。

---

## 六、进展追踪（2026-09-28）

**状态：全部仪表告警清零 ✅，横向仍不控车 ❌（卡在 engage 链）**

segment 0000001a 日志实证：
- `0x316` 已持续发送（141 帧 idle 帧，无条件 50Hz 供帧生效）✅
- 但全部为 disengaged idle 帧：`LKAS_Active=0`、`LKAS_ReqPrepare=0`——`selfdriveState` 全程 `disabled`
- **OP 从未进入 enabled/engaged 状态**，carcontroller 因此不进入 prepare/torque 流程

横向不控车的直接原因已从"EPS 拒绝（SteerWarning）"前移为"**engage 链未走通**"。下一步排查方向（按优先级）：
1. `cruiseState.enabled` 上升沿是否被 selfdrived 捕获（上升沿发生时 OP 是否已完成初始化、有无 NO_ENTRY 事件在场）
2. sunnypilot MADS 的启用语义（MADS 键 = cruise 主开关？是否需要额外 UI 开关）
3. `cruiseMismatch`（NO_ENTRY）与"OP 未 enable"的先后关系（互为因果，需抓上升沿瞬间帧）
4. buttonEvents 是否正确生成（PCM_BUTTONS 解析），pcmCruise 依赖按键上升沿

---

## 七、经验教训

1. **改 CAN 输出前，先做字节级对拍**：抓 bus2 上相机原发帧 vs sendcan 里 OP 帧，逐字节 diff（`od`/logreader）。0x316 的所有字段问题（SETME 缺失、LaneState、Config 组合）都能在对拍中直接看出来，比推理快一个数量级。
2. **CAN 节点要"持续供帧"而不是"按需发帧"**：车辆 ECU 对周期报文有存活监测（甚至 timeout 监测），接管某条报文的发送权后必须 50Hz 无条件续发，哪怕内容是"空闲"。断流=故障，这是这次最大的认知修正。
3. **实车验证过的参考实现 > 一切推理**：社区 BYD_Files 固件的 `byd_fwd_hook`（全量转发仅 block 0x316/0x32E）和 `carcontroller`（无条件发帧）就是标准答案，第一轮就该逐行看。
4. **两个疑似根因并存时，用总线拓扑统计区分**：bus0/bus2 地址交集为 0 直接证伪了"原车有桥接"的假设，把思路拉回正轨。
5. **`/proc/<pid>/environ` 只反映进程启动时的环境**，验证运行中 `os.environ` 修改要看行为（如 sendcan 内容），不能看 environ 文件。
6. **设备上有两份 launch_env.sh**（root 和 `sunnypilot/system/hardware/c3/`），tici 走 c3 版；且该设备环境变量传递链不可靠，关键开关建议进程内设置。
7. **阻止 engage 的告警是分层的**（commIssue → seatbelt → controlsMismatch → cruiseMismatch），要逐层剥掉，每剥一层才知道下一层是什么。
8. **libsafety.so 是测试的"隐式源"**：改了 byd.h 不重建 .so 等于没改；反过来 .so 与源码漂移会让测试结果误导排查方向。改 safety 后先 `clang -shared` 重建再跑测试。
9. **每轮测试后第一时间拉 rlog 解析**（sendcan/can/pandaStates），用户口述的"故障还在"缺少触发时序，日志里的 `states`/`events`/`SteerWarning` 时序才是定位依据。
