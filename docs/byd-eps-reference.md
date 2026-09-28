# BYD 宋 PLUS DM-i EPS（LKAS 执行器）参考手册

> 汇总自 14 个根因的实车调试全记录（`byd-song-plus-lkas-debug.md`）。所有结论以**本车实车 CAN 日志**为依据，参考实现（yysnet/opendbc、op_byd 厂商系统、mouxangithub）仅作旁证并标注验证边界。
> 本文是"现在时"快照：每条规则给出证据与复现次数，未定论项集中在 §11。最后更新：2026-09-28（根因 14 修复，47b8c5cb）。

---

## 一、在系统中的位置

- DiPilot MPC（前相机）是原车 LKA/ACC 的大脑；**EPS 是唯一横向执行器**。
- EPS 的 LKAS 命令唯一来源 = bus0 的 **0x316（ACC_MPC_STATE）**。相机原发的 0x316 在 bus2，panda byd 固件 `fwd_hook` 阻断 bus2→bus0 的 0x316/0x1E2 → **OP 是 EPS 唯一的 0x316 提供者**（调试记录根因 1 实证：OP 不发时 EPS 端 SteerWarning=100%、"请检查多功能视频控制器"）。
- 启动阶段 pandad 尚未切入 byd safety 模式的 ~8s 内，相机 0x316 会短暂透传到 bus0（R10_000 实测：384 帧 @48Hz，之后归零）——不是持续桥接。
- 总线拓扑（实车 CAN 统计）：bus0 = 底盘/动力/车身 + EPS 反馈；bus2 = 相机域仅 5 条（0x316/0x32D/0x32E/0x32F/0x432）；两总线地址交集为 0 → **必须全量转发 bus0↔bus2**（白名单饿死雷达，全 block 相机+雷达双饿死，根因 1 前置实验）。

## 二、报文清单（频率为实车实测）

| 报文 | ID | 频率 | 总线 | 方向 | 作用 |
|---|---|---|---|---|---|
| ACC_MPC_STATE | 0x316 | 50 Hz | bus0（OP 发送） | OP→EPS | LKAS 命令：扭矩 / 会话模式 / 握手 |
| ACC_EPS_STATE | 0x318 | 50 Hz | bus0 | EPS→OP | 反馈：Prepared / TorqueFailed / SteerWarning / 扭矩 |
| EPS | 0x11F | 100 Hz | bus0 | EPS→ | SteeringAngle / SteeringAngleRate（4°/s/bit，0-1020） |
| PCM_BUTTONS | 0x3B0 | 10 Hz | bus0 | 车身→ | ACC 按键（OP 伪造 resume；fwd 不阻断） |
| ACC_HUD_ADAS / ACC_CMD | 0x32D / 0x32E | 20 Hz | bus2 | 相机→ | ACC 状态 / 激活；真实转发（厂商曾整套伪造，我们不需要） |
| STEERING_MODULE_ADAS | 0x1E2 | — | bus0 | — | 角度命令（实验路径，宋 Plus 不发送，未启用） |

DBC：`opendbc_repo/opendbc/dbc/byd_general_pt.dbc`；固件侧安全模型：`opendbc/safety/modes/byd.h`。

## 三、0x316 命令帧协议（字节级）

| 字段 | 位定义 | 语义 |
|---|---|---|
| LeftLaneState / RightLaneState | 4\|2 / 34\|2 | 请求 LKA 时必须 =2（有车道目标）；idle 0/0 |
| **LKAS_Config** | 6\|2 | 1=ALARM，2=LKA，3=ALARM_AND_LKA；**3 = 出力会话许可** |
| SETME2/3/4/5/7、MPC_State、AutoFullBeam*、AlarmType、TrafficSign* | 各位 | **必须回显相机原发值**（不可缺省） |
| LKAS_Output | 16\|11 有符号 | 扭矩请求（厂商 p50=67 / p90=128 / max=193） |
| LKAS_ReqPrepare | 27\|1 | 握手请求 |
| LKAS_Active | 28\|1 | 激活位 |
| LKAS_State | 36\|4 | 0=关 / 1=待机 / 2=转向中 / 4=取消中；**用 1**（厂商模板） |
| Counter | 52\|4 | 0-15 循环 |
| CheckSum | 56\|8 | `byd_checksum(0xAF, dat)`——与相机原发帧逐字节对拍全部 MATCH |

**九条规则（全部实车字节级验证，括号为调试记录出处）**：

1. SETME_*/MPC_State 等回显字段不可缺省——从零构造 = 非法帧 → ADAS 域故障"check multifunction video controller"（根因 2，`1432de9`）。
2. 校验算法 0xAF 已对拍验证（调试记录 二.2）。
3. 握手三步：`ReqPrepare=1` → EPS 置 `LKAS_Prepared`（0x318 bit0，实测 38-50ms 应答）→ 之后才接受 `LKAS_Active=1`；**跳过第一步 EPS 永不 armed**（调试记录 二.3）。
4. LaneState 必须 =2，否则 EPS 不执行 LKA（调试记录 二.4）。
5. Config 组合合法性：`Config=1(ALARM)+LaneState=2` 是非法组合 → EPS 锁 SteerWarning、拒绝 armed；请求 LKA 时 Config=2（调试记录 二.5）。
6. **50Hz 连续流不可断**——断流 → SteerWarning 锁存 → 拒绝 armed → MPC 报"视频控制器"（根因 1，`8496a71` 无条件发送）。
7. **出力许可 = LKAS_Config=3**。厂商模板：激活帧 `(State=1, MPC=0, Config=3, Active=1, Lane 2/2)`，待机帧 `(1,0,3,0, Lane 0/0)`；相机待机只发 1/2、从不发 3——echo 永远拿不到出力许可（根因 11，`da3c4e2`）。
8. LKAS_State 用 1：6.12 曾改 2 是错误方向（state=2 帧会被接受回显但助力不介入，且包络变窄），厂商字节模板 State=1（根因 10 → 11 更正）。
9. **armed 会话（Config=3 + Active=1）的行为约束**：请求不得长时静默、不得与驾驶员对向——见 §6 锁存规则（根因 13/14）。

## 四、会话生命周期（厂商编排，route 00000037 逐帧实测）

| 阶段 | 厂商行为 | 我们的行为 |
|---|---|---|
| 怠速 | 不发 ReqPrepare；帧 `(State=1, MPC=0, Config=3, Active=0, Lane 0/0)` | echo 相机 Config（1/2）+ Lane 0/0 + Out=0（6.23 改定——正是原车相机不转向时的真实状态） |
| 进场 | engage 时 3 帧短促 prepare（60ms）→ EPS 50ms 应答 → 立即 Act=1 | 持续发 ReqPrepare 直到 Prepared（已验证可用，`3021ffb`） |
| 激活 | 扭矩 0.24s 内 0→107（~445/s ≈ 9/帧@50Hz）→ 稳态 101-113 | 软启动 10/帧（0.4s 到满 STEER_MAX=200），armed 需真实需求 ≥6 单位（根因 14） |
| 稳态 | Config=3 + Act=1 + State=1 + Lane 2/2，**请求非零贯穿全程** | 同左 + 静默守卫 0.16s + 对向跟随（§6/§8） |
| 退场 | Act=1 保持下斜坡 52→0（~80ms）再切 Act=0 | 同左（`f3926a3` 优雅退场）；TorqueFailed 时硬切 |

## 五、0x318 反馈语义（50Hz）

| 字段 | 位定义 | 语义 |
|---|---|---|
| LKAS_Prepared | 0\|1 | 握手应答；会话退场后 ~0.2s 清除（单例观察：Rf_015） |
| CruiseActivated | 1\|1 | |
| **TorqueFailed** | 2\|1 | **锁存故障**：EPS 放弃全部转向输入，**直到点火循环**（route 0000001c/20/22 + yysnet 同注 "EPS give up all inputs until restart"）；OP 收到即硬切退让 + UI "LKAS Fault: Restart the car to engage" |
| SteerWarning | 4\|1 | 见 §7 |
| SteerErrorCode | 5\|3 | 锁存时非零（码表未逆向） |
| **MainTorque** | 8\|12 有符号 | **= EPS 对收到指令的回显**（与 LKAS_Output 相关 0.98，差值 p50≈1）——不是实际助力，不能当反馈用（调试记录 6.12/6.18 定论） |
| ReportHandsNotOnSteeringWheel | 21\|1 | |
| SteerDriverTorque | 24\|12 有符号 | 驾驶员扭矩 raw 量程，见 §8 |
| Counter / CheckSum | 52\|4 / 56\|8 | XOR 校验（开源 port 说法，未字节验证；我们只收不发） |

## 六、锁存规则（TorqueFailed；armed 会话 = Config=3 + LKAS_Active=1）

第二次驾驶 6 处真锁存逐帧时序（`docs_site/hour_logs_2/latch_report.txt`）：

| 案例 | 不良状态开始 | SteerWarning | TorqueFailed | SW→锁存 |
|---|---|---|---|---|
| Rf_015（ACC 重 SET 时驾驶员正打盘） | armed-零 −0.72s | −0.50s | 0.00s | 0.50s |
| R10_000（出力 24 时被抓盘，让位归零） | 零请求 −0.48s | −0.50s | 0.00s | 0.50s |
| R15_001（方向盘停在 −34° 时 armed） | armed-零 −0.71s | −0.51s | 0.00s | 0.50s |
| Re_014（armed 时驾驶员持盘 −13.5°） | armed-零 −0.71s | −0.51s | 0.00s | 0.50s |
| R13_003（armed 时驾驶员持盘 stq 110~150） | armed-零 −0.64s | −0.50s | 0.00s | 0.50s |
| R16_000（OP −70 vs 驾驶员 +46 对向） | 对向 ~−0.63s | −0.49s | 0.00s | 0.49s |

**规则终版**：

| # | 触发条件 | 证据 | 现行防御 |
|---|---|---|---|
| R1 | **armed 会话请求 ≈0 持续 ~0.5s**（容忍度实测 ≤0.48s；一小时内 4 处 + 第二次驾驶 5 处） | 6.23、6.24；route 33 第 7 次锁存同属此类（armed+零+驾驶员剧烈操作） | 静默守卫 0.16s 退场 + engage 门（armed 需真实需求 ≥6 单位） |
| R2 | **armed 会话请求与驾驶员输入对向**（幅度与 68 让位阈值无关：drv +21~46 即触发；root cause 12 的 87~138 同类） | R16_000、route 32/33 | 对向跟随（~0.06s 翻转为同向 ±20） |
| R3 | 错标帧时代（Config=2/State=2）的合理性规则：\|转角\|>50° 后 ~10s、全锁速摆动 >150°/s、幅值 ~55+ | 路测 3-5；厂商 Config=3 下用到 193 且无门控也干净 → **已改判为错标帧规则**，保留为保守门控 | 角度门 40/30、速率门 180/40、大转角 10s 锁定 |
| R4 | 非会话类：断流（根因 1）、非法 Config 组合（二.5） | 调试记录 一/二 | 无条件 50Hz 发送 + Config 逻辑 |

**恢复**：仅点火循环清除。锁存后每次 engage 只是向故障 EPS 重发 ReqPrepare、永不 armed——仪表症状为"绿框但方向盘零扭矩"。OP 侧 `TorqueFailed → steerFaultPermanent` 硬切 + UI 告警（`ddf29f8`）。

## 七、SteerWarning（0x318 bit4）

- 触发：armed 会话进入零请求后 **0~0.22s**（R10 的 SW 在归零前 0.02s——抓盘瞬间即触）；armed 对向 **~0.14s**（R16）；错标帧非法组合。
- **不变量：SW → TorqueFailed 恒 ~0.50s（6/6，无一例外）**。尚未观察到 SW 出现后自行清除的案例 → 当前设计按"必须不让 SW 的触发条件成立"执行。
- 会话退场（idle echo）后 SW=1 **不**进展为锁存：大量 disarmed 段 SW=1 长期存在、零锁存（实证）。
- stock LKA 类比：手扶 → SW → 相机退出会话 → SW 清除 → 无锁存（推断，解释了原车 LKA 手扶即释放却从不锁存）。

## 八、驾驶员扭矩量程（SteerDriverTorque，raw ±2048）

| 状态 | 量程 |
|---|---|
| 脱手噪声 | <50（峰值，振荡） |
| 轻握 | 60-150 |
| 用力/对抗 | >150，剧烈 200-240（第二次驾驶实测） |

- `STEER_THRESHOLD=80`（steeringPressed，5 帧防抖）。
- driver-limit 公式（lateral.py，ALLOWANCE=68×MULTIPLIER=3）**只限制对向且 \|drv\|>68 的请求**——对向 ≤68 完全不限，这正是 R16 的漏洞，现由对向跟随逻辑兜底；同向助力公式不削减（vendor 同向跟随也如此）。

## 九、出力标定（厂商 Config=3 实跑，route 00000037）

| 参数 | 厂商实跑 | 我们 |
|---|---|---|
| 扭矩 | p50=67 / p90=128 / max=193 | STEER_MAX=200 |
| 帧间步进 | p99≈16（其固件限 17） | DELTA 8/10（我们固件安全限 10/12，max 300 未动） |
| 进场 ramp | ~9/帧（0.24s 到 107） | 软启动 10/帧（0.4s 到满） |
| steerRatio / mass | 19.5 / 1926 | 同（`84e2ae8`） |
| 横向模型 | factor 2.5 / friction 0.10 / delay 0.3 | 同 |

## 十、现行门控矩阵（代码 ↔ 规则）

| 门控 | 参数（values.py） | 防的规则 | 来源 |
|---|---|---|---|
| engage 门 | armed 需 \|需求\| ≥6 单位 | R1（对抗中 armed 零请求，4/6 案例） | 根因 14 |
| 静默守卫 | 8 命令@50Hz（0.16s）\|请求\|<2 → 退场 | R1 | 根因 13→14 |
| 对向跟随 | \|drv\|>15 持续 6 计数（~0.06s）→ 同向 ±20 | R2 | 根因 12→14 |
| 角度/速率门 | 40°退让/30°重arm、180/40°/s、>50° 后 10s 锁定 | R3（保守层） | 路测 3-5 |
| 刹车抑制 | 3 帧按下/5 帧释放 | R1 变体 | — |
| TorqueFailed 处理 | 硬切 + steerFaultPermanent 告警 | 恢复路径 | 路测 2 |
| 无条件 50Hz 发送 | — | R4 断流 | 根因 1 |
| 软启动 | 10/帧 | R1（复 armed 零窗口）+ 力度连续性 | 根因 14 |

## 十一、未定论 / 调参旋钮

1. **SW→锁存的确切机制**：是"SW 持续 0.5s"还是"SW 出现即无条件 0.5s 倒计时"——无清除案例。现按前者假设设计（退场可解）；若实车出现退场后仍锁存，改按后者（必须避免 SW 触发）。
2. **对向跟随参数**（15 单位 / 6 计数 / ±20 跟随幅值）是工程判断值，非厂商实测——出现助力晃动或残留锁存时首先调这里。
3. SteerErrorCode（0x318 bit5-7）码表未逆向。
4. 角度/速率门（R3）在 Config=3 时代可能可放宽或移除（厂商无门控也干净）——保留为保守层，弯道若仍受限再评估。
5. LKAS_Prepared 退场后 ~0.2s 清除为单例观察。
6. armed-零容忍度实测区间 0.48~0.72s，EPS 内部阈值未知（方差暗示与 SW 时刻/驾驶员活动相关）。

## 十二、工具与证据索引

| 内容 | 位置 |
|---|---|
| 第二次驾驶 6 锁存逐帧重建 | `docs_site/hour_logs_2/analyze_latch.py` → `latch_report.txt`（本地，gitignore） |
| 修复回归（6 时间线灌回控制器） | `docs_site/hour_logs_2/replay_latches.py`（9/9 干净） |
| 厂商系统工作日志（字节模板来源） | `docs_site/op_byd_logs/`（含 vendor_diff.py） |
| 调试全史（根因 1-14） | `byd-song-plus-lkas-debug.md` |
| 部署/测试规程 | `byd-song-plus-ops.md`（测试规程 = 调试记录 6.22） |

根因速查：1 断流 / 2 SETME / 3 UDS 查询帧 / 4 controlsMismatch 容忍 / 5 按键 0x3B0 / 6 DMS / 7 安全带 / 8 AccState 双义 / 9 扭矩量程 300 / 10 LKAS_State / 11 Config=3 会话 / 12 对抗性助力 / 13 armed-零静默 / 14 守卫输掉比赛 + 对向残余。
