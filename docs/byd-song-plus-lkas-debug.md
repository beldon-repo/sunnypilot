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

[根因 8] 0x32d 的 AccState=1 是歧义值，被当成 "ACC_ACTIVE" 用作 enabled 判定
         → 点火瞬间（还挂着 P 档、SetSpeed=30 的残留态）enabled 就被置 True，
           pcmEnable 上升沿在 canValid 之前被吃掉；刹车断开后的待机态也是 1，
           每次松刹车还产生幻影上升沿 → 真正按 SET 时没有边沿，OP 永远不 engage
[修复] enabled = AccControlActive(0x32e) or AccState in (2,3,5)（821636b）

[根因 9] 0x316 扭矩请求量程按社区汉平台照搬 STEER_MAX=300，但宋的 EPS 只容忍很小的请求
         → route 0000001c seg0：握手 40ms 全通（ReqPrepare → LKAS_Prepared），激活后
           扭矩爬升 ~0.7s 到 64 时 EPS 锁存 TorqueFailed；OP 不监控该故障继续发
           Act=1 + 更大扭矩 → 整段路 TQFail 锁存，此后每次 engage 只是在向故障 EPS
           重发 ReqPrepare，永不 armed → engaged 绿框但方向盘零扭矩
[修复] STEER_MAX 300→20（原车包络）、软启动放缓、TorqueFailed 即完全退让、
       STEER_THRESHOLD 56→80（±356 原始量程下 56 导致全程 overriding）（1527e60）
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
| `821636b` | **cruiseState.enabled 改用 AccControlActive 判定**（AccState=1 歧义，engage 链收口） |
| `1527e60` | **STEER_MAX 300→20 + TorqueFailed 退让 + THRESHOLD 56→80**（EPS 扭矩故障锁存根因） |

**注意**：launch_env.sh 有**两份**——根目录版和 `sunnypilot/system/hardware/c3/launch_env.sh`（tici 实际用的是后者）。且环境变量在该设备启动链中不可靠，**card 进程内的 `os.environ.setdefault` 才是兜底**。

---

## 五、遗留问题 / 待验证

1. ~~本轮修复未实车验证~~ → **已验证（见第七节）**：仪表故障全部消失，0x316 持续供帧生效。
2. **横向生效后的标定**：`STEER_THRESHOLD=56`（steeringPressed 灵敏度）、`steerRatio=15`（占位）、`STEER_SOFTSTART_STEP`、LatControl 参数均为占位/未标定，实车调参时注意 steeringOverride 频率。
3. **libsafety.so 与源码漂移（既存）**：仓库的 `libsafety.so`（Sep26 构建）与当前 byd.h 源码不一致，重建后 byd 的 24 个 brake/mads 测试失败（mads 框架与 byd 模式结构性不兼容：`mads_state_update` 仅在 TX `check_relay=true` 时被调用，而 byd 的 0x3B0 原生在 bus0 不能开 check_relay，否则实车误触发 relayMalfunction 全车失能）。测试基线：旧 so + 旧 test = 8 failed；新 so = 12~24 failed。**此为测试框架问题，非车控代码缺陷**，后续单独处理。
4. **纵向**：原车 ACC 全权负责（OP 不控纵向），OP 只做静止 resume 按键伪造（SNG）。"纵向有效果"即原车 ACC 在工作。
5. **echo 空闲帧的风险待观察**：连续发 0x316 空闲帧是否影响 ACC 按钮（历史问题仅出现在缺 SETME 的从零构造帧，echo 帧理论上无碍，社区实现同样连续发）。
6. ~~**cruiseMismatch（NO_ENTRY）**~~ → **已厘清（见 6.1/根因 8）**：它是 enabled 幻影 True 的**果**；根因 8 修复后 ACC 待机态 enabled=False，计数器不再累积。注意其告警映射在本 fork 被注释（静默事件），后续若再现不会在 UI 提示，排查时直接看 onroadEvents。

---

## 六、进展追踪（2026-09-28）

**状态：全部仪表告警清零 ✅，engage 链已收口 ✅（根因 8 已修复），待有效路测 ❌**

### 6.1 上一轮遗留的 4 个排查方向——全部有答案了

1. **上升沿捕获**：上升沿不是丢在"OP 未初始化"，而是丢在 **enabled 在点火瞬间就被置 True**（AccState=1 残留态），此时 carState 刚出、canValid 未就绪，边沿被 `if CS.canValid` 门吞掉。见根因 8。
2. **MADS 语义**：设备实参 `Mads=1`、`MadsUnifiedEngagementMode=1`、`MadsMainCruiseAllowed=1`。UEM 开 → pcmEnable 不被 block，engage 走原链路 ✅；MADS 会摘掉 `pcmDisable`/`pedalPressed`（踩刹车不断横向，属 sunnypilot 设计行为）。
3. **cruiseMismatch 因果**：它是**果**不是因——enabled 幻影 True + OP 未 enable 满 6 秒即触发；且其告警映射在本 fork 被注释掉（events.py），**完全静默**，这就是"无告警但不控车"的观感来源。修掉根因 8 后不会出现。
4. **buttonEvents**：byd carstate **根本不解析 buttonEvents**（无按键类型枚举），engage 唯一入口就是 `cruiseState.enabled` 上升沿 → `pcmEnable`。此设计成立，无需补按键。

### 6.2 segment 19（09-28 凌晨，车库 D 档触发 ACC）实证——engage 链是通的

- `pcmEnable` 触发 3 次，selfdrived 进入 `enabled`（22 帧）+ `overriding`（226 帧，约 4 秒）
- **OP 确实 engage 了**。"没控车"的直接原因：全程 vEgo 0~1.4 m/s（0~5 km/h 车库挪车），且打方向触发 `steerOverride` → `overriding`，横向自然不动
- 按钮动作实测：`PCM_BUTTONS BTN_AccUpDown_Cmd=3` 三次，随后 `AccState 3 / SetSpeed 30 / CtlActive=1 / StandSS=1`（SET 生效）
- **至此所有日志里最高车速就是 5 km/h，从未做过一次"速度上来 + ACC 激活 + 松手"的有效路测**

### 6.3 AccState 字段实测语义（字节级，route 0000001a seg 19）

| 场景 | AccState | AccOn1 | CtlActive | 实际状态 |
|---|---|---|---|---|
| 点火瞬间（P 档） | **1** | 1 | 0 | 残留态，未激活（SetSpeed=30 是上次记忆） |
| 刹车断开后 | **1** | 1 | 0 | 待机 |
| 换 D 瞬间 | 2 | 1 | 1 | 过渡 |
| 按 SET（静止激活） | 3 | 1 | 1 | **真激活**（StandSS=1） |
| 激活后行车 | 1/3 | 1 | 1 | 真激活 |
| 主开关关 | 0 | 0 | 0 | 关闭 |

**结论：AccState=1 双义（残留/待机 vs 激活后），唯一可靠信号是 AccControlActive（0x32e）。**

### 6.4 修复后的回归（录制 CAN 场景回放新逻辑）

点火残留→enabled False ✅｜断开待机→False ✅｜SET→True ✅｜激活行车（AccState=1）→True ✅｜踩刹车→False ✅｜松刹车且 ACC 已断→False ✅（不再有幻影边沿）｜主开关关→available False ✅

### 6.5 下一步

1. **OTA 更新后做第一次有效路测**：≥30 km/h，按 ACC SET，松方向盘，观察 selfdriveState `enabled` 与方向盘扭矩（日志看 sendcan 0x316 `LKAS_Active=1` 帧是否出现）
2. 路测若 engage 成功但方向不对/不动，再进入标定项（见遗留问题 2：STEER_THRESHOLD=56、steerRatio=15 等占位值）

### 6.6 路测 2（route 0000001c，OTA 821636b 生效后）——engage 全通，倒在 EPS 扭矩锁存

821636b 修复实车生效：`pcmEnable` 触发 13 次，绿框（MADS enabled）出现，enabled+overriding 数千帧——**engage 链在实车上完全打通** ✅。

但方向盘零扭矩。逐帧解码（seg0 t=17.3-18.7）：

```
17.359 TX Prep=1 Cfg=2        ← ReqPrepare 请求
17.397 RX EPS_Prep=1          ← EPS 40ms 内 armed，握手全通 ✅
17.400 TX Act=1 Out=0         ← 激活，软启动爬升（100Hz 步进 9 → ~900/s）
17.4-18.1 Out 振荡 -41↔+64（驾驶员持续对抗 stq -14~-50，STEER_THRESHOLD=56 过低）
18.116 RX EPS_Prep=0 TQFail=1 ← 请求过 ~64 的瞬间 EPS 锁存扭矩故障
18.119+ TX Act=1 Out=105+     ← carcontroller 不监控 TQFail，继续锤
→ seg1-4 全程 TQFail=1，ReqPrepare 无人应答，零扭矩直到熄火
```

要点：
1. **握手协议本身是对的**，EPS 40ms 应答；故障点是请求量纲/速率，不是协议
2. EPS 锁存 TorqueFailed 后**直到下次点火循环才清除**（seg0 开头 TQFail=0 证明点火清除）——路测失败后必须熄火重启再测
3. STEER_THRESHOLD=56 在 ±356 原始量程下形同虚设（脱手噪声 <50，轻握 60-150），engaged 时长 70% 处于 overriding
4. 社区 BYD_Files 的 STEER_MAX=300 是汉平台的值；宋 EPS 实测 ~64 即故障，原车相机只用 ±10-14

1527e60 修复：STEER_MAX 300→20（原车包络）、软启动 0.4s 爬满、TQFail 完全退让、THRESHOLD 56→80。**预期路测观感：方向盘力度与原车 LKA 相当（轻）**——先验证链路，确认无 TQFail 后再按日志逐步放开 STEER_MAX。

### 6.7 路测 3（route 0000001e-22，OTA 1527e60 生效后）——EPS 包络图谱完成

1527e60 验证结果：新限幅/退让逻辑全部按设计工作（故障帧 1 帧内 Act=0 Out=0 ✅）。
用户熄火重置后（route 22）出现**首次无故障的扭矩输出窗口**：18 km/h、转角 -1~-21°、
扭矩 ≤15，**连续 11 秒 TQFail=0**（12.3-23.4s）——握手协议和请求格式彻底确认无误。

三次 TorqueFailed 锁存汇总成**故障包络图**（两个独立触发条件，任一越界即锁存）：

| 样本 | 车速 | 转角 | 扭矩峰值 | 触发 |
|---|---|---|---|---|
| r1c | 0.6-0.9 m/s | -5° ✅ | **55→78** ❌ | 扭矩 ~55+ 越界（步行速度+小转角） |
| r20 | 2-3 m/s | **43→58°** ❌ | ≤15 ✅ | 大转角越界 |
| r22 早期 | 1.4-7 m/s | -1~-21° ✅ | ≤15 ✅ | **无故障 11 秒**（包络内） |
| r22 后段 | 7 m/s | **53.2°** ❌ | ≤4 | 大转角越界 |

结论：
1. **大转角是触发条件**（~50°+，三次复现）——LKA 是车道保持功能，泊车级转角在其包络外
2. **扭矩 ~55+ 也是触发条件**（一次复现，步行速度+小转角；此前误判"量程 300 全错"——
   实际上 r22 证明 ≤15 可长期接受，55-78 之间是边界）
3. ≤15 单位在 18 km/h 下被接受但**推不动被握住的方向盘**——力度需要放开，但有包络上限

`282bda1` 修复：**转角门控**（>40° 退让 / <30° 重新激活，迟滞防抖）+ **STEER_MAX 20→50**
（距 55 的幅值故障线留 10% 余量，DELTA 4/6、软启动 2 仍在固件安全限 10/12 内）。

### 6.8 下一步（更新）

1. OTA 到 282bda1+，熄火重置清锁存
2. **路测要点变了：正常道路行驶中按 SET，不要在泊车/掉头大转角中激活**（门控会在 >40°
   自动退让、回正后自动恢复，属预期行为，不是故障）
3. 日志验证：TQFail 全程 0、|Out| 峰值 ≤50、方向盘在 40 km/h+ 应有可感知的保持力
4. 若 50 无故障且力度足够 → 收官；若力度不足 → 下一步试探 55-78 之间的真实边界（每次 +10）

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
10. **上升沿触发型 engage，判定信号必须绑定"真实意图"**：把双义枚举值（AccState=1 既是点火残留又是激活态）当上升沿来源，等于在系统还没就绪时就把唯一一次边沿花掉——之后每个真实操作都"无事件"。碰到"某操作永远不触发"类问题，先画该信号在**每个车辆状态下的实测值表**（6.3），再谈逻辑。

---

## 八、参考实现情报（可信度分级）

| 来源 | 内容 | 验证边界 | 结论 |
|---|---|---|---|
| **yysnet/opendbc**（/Users/wujiafu/Documents/op/opendbc） | 完成度最高的开源 BYD 移植：790 扭矩+echo+握手+伪 318+MRR 雷达，git 历史有字节级调试 | **仅汉 DM/EV、唐 DM 实车可控**（其余 dashcamOnly）；宋 Plus 只是识别占位 | 交叉印证源：其 `steerFaultPermanent=TorqueFailed`（注释"EPS give up all inputs until restart"）、`enabled=AccState in (3,5)`、LKASConfig 枚举与我们实测完全一致；STEER_MAX=300 从未在宋 Plus 790 通道验证。**已吸收 ddf29f8**（故障上报 UI + pressed 防抖） |
| github_value.py（docs_site，即 yysnet 的 values.py） | 同上仓库的参数文件 | 同上 | `TORQUE_LAT_CAR` 只含汉/唐——证明 300 是汉/唐 Veoneer 通道的数 |
| pro_values.py（docs_site） | 宋 Pro 参数草稿（"基于您提供的 CAN ID"） | 无实车验证痕迹，STEER_STEP/MSG_HZ 自相矛盾 | 仅 STEER_MAX=100 可作"族内幅值"旁证；ALLOWANCE=15 与实测矛盾，勿采纳 |
| **opendbc_repo.byd**（liruifeng1120，sunnypilot opendbc 的 fork） | yysnet BYD 品牌同步进 SP API 的桥接仓库（2025-06），我们 port 的同族直系前辈；曾把宋 Plus 21 解除 dashcamOnly 但 4 天后停更，未走完 | 同 yysnet（汉/唐）；其 PEDAL 0-255 修复经本车日志字节验证**不适用**（本车 0x342 是 0-100 百分比量纲，静止 0.000/深刹 0.62，现系数正确）；byd.h 的 SONG_STEERING_LIMITS 是汉值占位（注释 values to be check） | 无需改动，仅确认 lineage 与"宋 Plus 从未被任何社区实现验证过"这一判断 |
| **mouxangithub/opendbc**（/Users/wujiafu/Documents/op/mouxangithub/opendbc，master-c3 分支） | mouxan 2026-09-14（`3ce9c30b 适配c3`）独立 28 平台 BYD 移植，源自"高阶Python源码_v2"（社区反混淆 DiPilot 源码，CID_*/sig_* 混淆命名）；echo 相机字段 + 0xAF 校验 + ReqPrepare 握手 + 伪 318/伪 1FC 发 MPC + 0x3B0 按键伪造 + 0x32E 全纵向（EXP_LONG）；**同 c3 硬件**，另有腾势 D9（tn 分支），活跃开发中 | **宋 Plus 21/22/23 全在 non_tested_cars，零实车验证**；"validated on real vehicles" 仅指 ATTO3（角度控制）；STEER_MAX=300/DELTA 17/ALLOWANCE 68 为汉平台值——即本车实测已推翻的那组数 | 协议层强交叉印证源：CID_HGZKCQ=0x316/CID_IQMESB=0x318/CID_RJDCMR=0x32E/CID_YHMGPU=0x3B0、校验 0xAF、echo+握手+Config/LaneState=2 全部与本文实测一致（泄露源与实车逆向互为背书）；**可借鉴**：伪 318/1FC 向 MPC 伪造 EPS LKAS 状态反馈（视频控制器故障再现时的下一杠杆）、params.toml 宋 Plus 扭矩标定参数 [2.2,2.5,0.145]（疑为原厂增益，标定阶段对照起点）、0x3B0 AccUpDown 注入；**幅值/包络勿抄**：其安全层无大转角/速率/锁定门控，落后于本分支 4324231 |

**方法论重申**：所有参考实现的参数对我们只有"旁证"价值；宋 Plus 的权威依据是本车实车日志（EPS 包络、AccState 语义、车速源均由本车数据定）。

### 6.9 路测 4（route 00000023-27，OTA ddf29f8 生效后）——**首次持续控车成功**，第 4 种触发条件补全

r6 开机残留锁存（steerUnavailable 23 次 = 新 UI 告警生效 ✅）→ 重启清除 → **r7_0 历史性窗口**：
22-25 km/h、扭矩打到 50 上限（p95=49）、持续 20+ 秒、**TQFail 全程 0**——门控/限幅/退让全部按设计工作，OP 第一次真正控住了方向（期间驾驶员 7 次抓盘 overriding，退让正常）。

**第 4 次锁存（r7_1 t=129.9）**：泊车回正时方向盘以 ~150°/s 扫过零点，瞬时角度重武装在过零瞬间开门，-48 扭矩打进全锁速摆动 → EPS 判定不合理。速率统计：正常控车 p90=55 / p99=93 / max=129°/s，致命回正 100-150°/s——瞬时值不可分，**但致命场景方向盘从未停稳**。

`9e5b890` 修复：重武装需"角度<30° 且速率<40°/s 持续 0.3s"（扫过零点永不满足）；速率>180°/s 任意时刻退让；carstate 发布 `steeringRateDeg`（EPS 0x11F，4°/s/bit）。

**EPS LKAS 包络终版（4 个触发条件，任一即锁存，断电清除）**：
1. |转角| > ~50°（三次复现）
2. 扭矩 ~55+ 单位（一次复现）
3. 全锁速摆动中的请求（~150°/s 过零，一次复现）
4. ~~待发现~~

下一步：OTA/manual pull 到 9e5b890 → 熄火清锁存 → 重复 40 km/h+ 正常道路测试（**不要在泊车/掉头中激活**）。若 50 单位力度可接受且无锁存 → 进入标定阶段。

### 6.10 路测 5（route 00000028-30，STEER_MAX=70 探针）——幅值理论被推翻，五次故障共性确认

r29（STEER_MAX=50 旧代码）：24 秒持续扭矩（p50=10、峰值 50 上限）、25 km/h、零故障——但驾驶员仍感觉"没控"（力度不足）。→ 70 探针上设备。

r30（STEER_MAX=70）：**第 5 次锁存推翻幅值理论**——故障时扭矩仅 **-14**、手完全离开（stq=0）、直道 9.1°、32 km/h。逐帧重建：门控在方向盘 18-20° 停稳 0.9s 后合法重武装 → 驾驶员又向右打盘（+160）到 31° → 松手 → 方向盘以 ~60°/s 回弹扫过中心 → OP 同时 ramp 到 -39 反向助力 → EPS 锁存。70 上限根本没被碰到（峰值 -39）。

**五次锁存的真正共性：全部发生在 >50° 大转角偏移后的数秒内**（#2/#3/#4/#5 明确；#1 车库场景几乎必然）；两次长时间干净窗口（r7_0/r29 共 44+ 秒有效控车）都是纯前进行驶。EPS 在方向盘大偏转后的一段时间内拒绝 LKAS 请求——与原车相机 LKA 状态机行为一致。之前"55-78 幅值线"（故障 1）同样改判：那次是步行速度+车库大转角。

`4324231` 修复：**>50° 采样后锁定 10 秒**（叠加既有停稳条件）；boot 期不锁定。STEER_MAX 维持 70（幅值仍未被真正测试到，非被否定）。

**EPS LKAS 包络终版（2026-09-28 深夜）**：
1. >50° 大转角偏移后 ~10s 内拒绝请求（5 次锁存共性，4324231 已门控）
2. 全锁速摆动（>150°/s）中的请求（9e5b890 已门控）
3. |转角|>40° 期间退让（282bda1 已门控）
4. 幅值：50 持续安全（两次 44s 验证），70 未测完——待下一轮

### 6.11 下一步

OTA/manual pull 到 4324231 → 熄火清锁存 → **正常行驶中激活，激活后 10 秒内别打大方向** → 验证 |Out| 能否到 70 且无锁存。若干净 → 标定阶段（力度主观评价、steerRatio、LatControl）。若再锁存 → 看日志里当时的 |Out|：>55 则幅值线确认回退 50；≤50 则还有第 6 个条件。

### 6.12 路测 6（route 0000002b-c）——**根因 10：LKAS_State 出力许可位，整个症状史的最终解释**

用户反馈"松手了方向盘完全不动"。帧级审计揭开真相：

- **MainTorque(0x318) 与 LKAS_Output 相关性 0.99、幅值一致 → 它是 EPS 对收到指令的回显，不是实际助力**
- **全部 1121 帧 |Output|≥30 的高扭矩帧都带着 LKAS_State=1（相机待机态的 echo 值）发出**
- EPS 对这种帧：接收 ✓ 校验 ✓ 回显 ✓ 不锁存 ✓ **但绝不驱动电机**

对照 yysnet/opendbc 的激活分支发现缺失字段：**激活时必须 LKAS_State=2**（LKA 转向会话状态；4=取消中）。`c510f23` 修复：激活帧强制 LKAS_State=2（怠速帧保持相机 echo）。

此前所有"控车了又没感觉"的现象：握手/限幅/包络门控全部正常工作，扭矩请求也真实发出了——只是每一帧都被标记成"待机"，EPS 从未获准出力。走廊日志里的五次锁存反而是 EPS 对待机帧的高幅值/大转角请求做的合理性保护。

**0x316 协议终版（在第二节基础上补充第 7 条）**：
7. **LKAS_State=2 是出力许可位**：LKAS_Active=1 但 LKAS_State=1 的帧会被 EPS 接受并回显、但助力不介入。LKAS_State 的含义：0=关闭，1=待机，2=转向中，4=取消中。

### 6.13 下一步

熄火清锁存 → 正常行驶激活松手。**这次方向盘应该真的会动**。重点观察：
1. 助力方向是否正确（OP 向车道中心修正）
2. 力度感受（LKAS_State 修复后 50 单位是真实施加的，之前从未发生过）
3. 若 EPS 对 state=2 帧的包络更敏感（此前从未发过这种帧，故障线可能不同）→ 有 LKAS Fault 提示立即停，回来拉日志

### 6.14 路测 7（route 00000032，LKA 开关 ON + state=2）——**EPS 出力确认 + 幅值线确认**

**历史性突破**：LKA 开关打开（系统上下文 Config=2/armed）+ LKAS_State=2 帧 → **EPS 第一次真正驱动电机**。seg0 时间线实证：65.3-65.6s 手离开（drv≈0）、MT -41~-50、转角 0.7→-1.1°；63-67s 转角整体 +10°→-8.5°，全程跟随扭矩。

**幅值线确认**：seg1 故障时扭矩 **-62**（直道 42 km/h、手轻扶、state=2）——state=2 帧的幅值线在 50（多次干净+出力）与 62 之间。`ecccc3b` 把 STEER_MAX 收回 50（已验证干净且出力）。

**最终配置 = LKA 开关 ON + state=2 帧 + STEER_MAX 50 + 全部门控。**

### 6.15 下一步

1. **验证性路测**：熄火清锁存 → 点火 → 开 LKA → 激活 OP → 松手。预期：方向盘持续修正、无故障
2. 力度主观评价：50 单位 ≈ 原车 LKA 的 3-5 倍，若不够再探 55-60（每次 +5，有 LKAS Fault 立停）
3. 力度 OK → 进标定阶段（steerRatio=15、LatControl、STEER_THRESHOLD=80）
4. 速度信号待核对：bus0 ESP_SPEED 与 GPS 稳态比 0.896（10% 低估），影响控制精度，标定阶段一并处理

### 6.16 根因 11（终局）：LKAS_Config=3 出力会话模式——字节级实锤

用户刷回 op_byd 厂商系统（证实能控车），route 00000037 的日志终于给出字节级模板。高扭矩激活帧逐字段对比：

| 字段 | 厂商（能控车） | 我们（此前） |
|---|---|---|
| **LKAS_Config** | **3**（ALARM_AND_LKA 会话） | 2 ❌ |
| **LKAS_State** | **1**（相机待机值，别动） | 2 ❌（6.12 的 state=2 改错了方向） |
| MPC_State / Active / LaneState / SETME / AlarmType / 扭矩编码 | — | **全部一致 ✓** |

**EPS 的出力许可 = LKAS_Config=3**（yysnet 枚举里的 ALARM_AND_LKA；原车相机怠速只发 1/2，从不发 3——所以 echo 永远拿不到出力许可）。此前 10 个根因全部真实存在且已修复，但都只是"让帧被接受"；最后这道会话模式门在 Config 上。

厂商系统的完整画像（route 37）：
- bus0 整套伪造 0x32D/0x32E/0x32F/0x316 各 3347 帧（50Hz 同步）+ bus2 发 0x3B0 喂相机（1339 帧，间歇）
- 激活 0x316 = `(State=1, MPC=0, Config=3, Active=1, Lane 2/2)`；待机 = `(1,0,3,0, Lane 0/0)`
- 0x32D AccState 跟随（1/3/0/2）、0x32E CtlActive 跟随

`da3c4e2` 修复（纯 Python）：激活帧 State=1 + Config=3；待机帧 Config=3 + Lane 0/0（完整对齐厂商模板）。我们的固件本就转发相机真实 0x32D/0x32E/0x32F（byd.h fwd_hook 只 block 0x316/0x1E2），ACC 会话内容由真实相机覆盖，无需固件改动。

### 6.17 最终验证清单

1. 刷回我们系统（OTA 或 bundle，≥da3c4e2）
2. 熄火清锁存 → 点火 → **开 LKA 开关**（armed 上下文，r32/r33 已证明必要）→ 激活 OP → 松手
3. 预期：方向盘真实修正（这次是拿标准答案改的）
4. 力度评价 → 标定阶段

### 6.18 厂商日志深挖——标定参数全套对齐（84e2ae8）

op_byd 工作日志（route 37）的 carParams 与实时流量给出了完整标定基准：

| 参数 | 厂商实跑 | 我们原值 | 修正 |
|---|---|---|---|
| **转向扭矩** | p50=67 / p90=128 / **max=193** | 上限 50 | **STEER_MAX=200** |
| **steerRatio** | **19.5** | 15 | 19.5 |
| **mass** | **1926** | 1785 | 1926 |
| LAT_ACCEL_FACTOR | 2.5 | 2.2 | 2.5 |
| FRICTION | 0.10 | 0.145 | 0.10 |
| steerActuatorDelay | 0.3 | 0.2 | 0.3 |
| steerLimitTimer | 0.5 | 0.6 | 0.5 |
| 帧间步进 | p99≈16（其固件限 17） | — | delta 8/10（我们固件限 10/12 内） |
| MainTorque 语义 | **= 请求回显**（相关 0.981，真实出力时亦然） | — | 定论：回显，非助力反馈 |
| openpilotLongitudinalControl | True（它连纵向也控，0x32E 是它的） | False | 不动（我们用原车 ACC） |

关键认知修正：**50-62"幅值线"是 Config=2 错标帧的合理性规则，不是真实包络**——Config=3 会话下厂商用到 193。我们的门控（角度/速率/锁定）在厂商干净转向中不存在也不触发，保留作为额外保守层。

### 6.19 最终验证步骤

设备刷回我们系统（≥84e2ae8）→ 熄火清锁存 → 开 LKA → 激活 OP → 松手。预期：以厂商同级扭矩（67-128 典型值域）真实修正车道。

### 6.20 厂商日志再挖——进场/退场编排与控制器内部（f3926a3）

**进场编排**（route 37 seg0 逐帧）：怠速帧**不发 ReqPrepare** → engage 时 3 帧短促 prepare（60ms）→ EPS 50ms 应答 → 立即 Act=1 → 扭矩 0.24s 内 0→107（~445/s）→ 稳态保持 101-113。**退场**：Act=1 保持下斜坡 52→0（~80ms）再切 Act=0——扭矩阶跃直落是原车相机不会有的行为。

**控制器内部**：lagd 关闭（useParams=False，静态 2.5/0/0.1 直接生效）、扭矩 PID 几乎不积分（i≈0）全靠前馈、1854 激活帧**零饱和**、liveParameters angleOffset=0（未做方向机偏差补偿）。

**已采纳（f3926a3）**：优雅退场——latActive 消失后保持 Act=1 斜坡降扭到零（DELTA_DOWN 速率）再切 Act=0；TorqueFailed 仍瞬时硬切。进场编排暂不改（我们的持续 prepare 已验证能用）。

**调参弹药归档**（尚未采纳，按需取用）：yysnet 速度相关转角速率限制器（132→64°/s 防高速画龙）；byd2 离手防退出 hack（HANDSOFF_ANGLE/PERIOD）；比亚迪3 方向机偏差通病（STEER_ANGLE_OFFSET_DEG，paramsd 可活补）；STEER_THRESHOLD 三源交叉 56/59/60（我们 80 偏高）。

### 6.21 路测 8（flash 后 route 0-2）——根因 12：对抗性助力被拒，"完全让位"策略落地

用户刷回我们系统（OTA 到 19a15135）后两段测试（LKA 开/关）。真时序（route 1/2 实为连续一段，t=0-937s）：

- t=0-72：测试 1（LKA 开），OP **未 enable**——用户点火后很快按 SET，首个 enabled 上升沿被 selfdrived 就绪窗口吞掉，随后 ACC 持续 enabled 6s+ → cruiseMismatch（静默 NO_ENTRY）锁死后续 3 次 enable 尝试 → **经典 catch-22 回归**
- t=72.6-98.9：测试 2（LKA 关），OP enable，**Config=3 帧首次真实发出**（37 帧，|Out|max=46）
- t=98.9：TQFail 锁存——OP ramp 到 -46 时驾驶员正 +87~+138 向右打方向+踩油门（转角 20-23° 直行）。**对抗性助力被 EPS 拒绝**

**8 次锁存的最终共性：OP 扭矩与驾驶员输入对抗的瞬间**（5 次直接对抗 + 3 次门控未覆盖的边角）。标准 driver-limit 公式在对抗侧仍允许 ~100 单位（对丰田成立，对宋的 EPS 不成立）。

`8923742` 修复：**完全让位**——|驾驶员扭矩|>68（allowance）时请求斜坡归零并保持，安静 0.25s 后经软启动恢复。

另：s0 全程 disabled 但无 TorqueFail（CAN 级确认），早前"开机残留"判断更正为 cruiseMismatch 锁死；速度对拍 ESP/GPS=0.966（正常，之前的 140 是雷达帧污染 vmax 统计的假象）。

### 6.22 测试规程（固化）

1. 点火后**等 15 秒**再按 SET（避开 selfdrived 就绪窗口，防 cruiseMismatch 锁死）
2. 若 cruiseMismatch 挡住：先取消 ACC，等 3 秒，重新 SET
3. **LKA 开关保持打开**
4. 激活后松手，**手完全离开**（让位逻辑生效期间 OP 不与手对抗）
5. 出现 LKAS Fault → 熄火清锁存再试，间隔至少 2 分钟

---

## 七、里程碑（2026-09-28 深夜）：成功控车 ✅

`8923742`（完全让位）+ 全部前置修复部署后，实车验证：**方向盘真实接管，持续行驶一小时**。宋 PLUS DM-i 2022 低配的 openpilot 横向控制移植宣告成功。

十一个根因的修复链全部得到实车验证：CAN 全量转发 → 帧构造 → 会话模式(Config=3) → 出力许可 → 厂商标定 → 驾驶员让位。

遗留：行驶中出现过 1-2 次 "LKAS Fault: Restart the car to engage"（EPS TorqueFailed 锁存），原因待查（6.23）。锁存后需熄火重启恢复。

### 标定状态快照（成功时）

| 项 | 值 | 来源 |
|---|---|---|
| 会话模式 | LKAS_Config=3, State=1 | 厂商字节模板 |
| STEER_MAX | 200（实测用到 193 上限内） | 厂商 |
| steerRatio / mass | 19.5 / 1926 | 厂商 |
| torque map | factor 2.5 / friction 0.10 / offset 0 | 厂商 |
| 让位阈值 | \|drv\|>68 → 扭矩归零,安静 0.25s 恢复 | 8 次锁存教训 |
| 门控 | 角度 40/30 + 速率 180/40 + 大转角 10s 锁定 | 保守层 |

### 6.23 根因 13（最终）：armed 会话零扭矩静默 → EPS 判定 MPC 无响应

成功一小时里 4 次中途锁存（route04 seg16、08 seg08、09 seg12、0a seg02）逐帧对比后共性：**故障前 2 秒 |request|=0**（驾驶员让位后或直线段），而 Config=3 会话仍 armed。EPS 把"会话声称转向中但 MPC 静默"读作故障。厂商从不静默——它的请求即使在驾驶员对抗时也持续跟随（Act=1 + 非零值贯穿 drv>150 的时段）。

`478cd93` 修复：
1. **静默守卫**：armed 会话内 |request|<2 持续 2s → 主动退场（走既有斜坡）
2. 退场后待机帧 **Config echo 相机**（1/2）——正是原车相机不转向时的真实状态
3. 再入需**真实需求**（≥6 单位）——防止零需求下立即重新 armed

至此 EPS 的三重验收规则全部建模完毕并有对应门控：**帧合法性**（校验/SETME）→ **会话合法性**（Config=3 + 真实扭矩流）→ **行为合理性**（不对抗驾驶员/包络）。测试规程 6.22 不变。
