# BYD 宋 PLUS DM-i · 横向/纵向生命周期重构：armed = ACC main-on

> 2026-09-30。提交 `3381318edd`（本分支 `main-c3l-tici`）。固件已部署并签名验证
> （设备 `get_signature()` 头 8 字节 `25e1b0be…` == 入库 `panda.bin.signed`，sha256
> 前缀 `c19b5359`；本文早先写的 `01dac823` 是重建前的旧标签，作废）。
> **部署后复验见 §八（route 1c/1d/1e + 补丁 `447a360e53`）。**

## 一、症状与根因（一个根因，两层同病）

用户三症状：

1. 开 MADS 实现不了全程横向，需要 SET ACC；
2. 设了 ACC、踩刹车退横向，等几秒恢复；
3. 从 ACC 激活退出（但没取消设置）退横向。

`scripts/long_lifecycle.py`（route 11/12 全帧时间线）实证：**MADS 状态机 +
panda 横向闸门都被绑在 stock ACC 的"session 边沿"上，而不是"main 开关"上**：

- **OP 侧**：`is_cruise_latch` 里 AND 了 `brakePressed` 和 `stock_acc_on`
  → 刹车/CANCEL/bounce 掉 → `cruiseState.enabled` 掉 → `pcmDisable`(USER_DISABLE)
  → `mads.state_machine` 进 disabled。carstate 又不出 buttonEvents，MADS 唯一
  武装入口是 `pcmEnable`=SET → 症状1。
- **panda 侧**：`acc_main_on` 喂给 `pcm_cruise_check`（直接驱动 `controls_allowed`
  横向扭矩闸门）。仓库 byd.h 旧值是 `AccState∈{2,3,5}`（session）；部署 0.9.x fw
  旧值是 `(state!=0 && state!=7)`——后者在相机 main glitch（route 11 t=156.4 /
  route 12 t=200.3：AccState 与 AccOn1 一起掉 ~2s，无按钮事件）时把横向杀 2 秒
  = 症状3 的时间窗。
- **恢复机制**：松刹车雷达自动 resume（5/5 刹车窗口，期间无 BTN，~40ms 内
  AccState 回 2/3/5）→ 两层同时活 → "等几秒"=踩刹车时长；CANCEL 型不自动 resume，需 RES。

## 二、设计模型

`available`（`AccOn1 || AccState∈{1,2,3,5}`）是两轴共同的 arm 信号：它经实证
对刹车-cancel / CANCEL / bounce **全免疫**（AccOn1 全程=1），只在真 main-off 时掉。

| 用户状态 | 新模型行为 | 落点 |
|---|---|---|
| 1 MADS 全程横向 | 拨 ACC main ON 即武装横向，不退出 | latch 驱动 pcmEnable→unified MADS |
| 2 设置未激活(非MADS) | 同上，main-on 即横向 | 同上 |
| 3 激活 | 横向纵向都在，不退出 | session→纵向出力 |
| 4 取消激活(刹车/CANCEL) | 横向不退；纵向让位、session 回来无缝续 | latch 不随 session 掉；controller 出力闸门 |
| 5 取消设置(main-off) | 退出（第一版接受，含 MADS） | latch 掉 + fw fall-hold 后掉 |

三层分离：**armed = main-on（防抖）** / **纵向出力 = session && !brake** /
**退出 = main-off**。

## 三、落码（提交 `3381318edd`）

| 层 | 文件 | 改动 |
|---|---|---|
| a | `opendbc/car/byd/carstate.py` | latch = `debounced_main && ever_engaged`（去 `brakePressed`/`stock_acc_on`）；`AVAIL_FALL_DEBOUNCE_TIME=0.5`；`ever_engaged`=本循环见过一次真 session（防点火残留）；boot hold 保留 |
| b | `opendbc/safety/modes/byd.h` + `~/Documents/op/panda_fw_base/board/safety/safety_byd.h` | `acc_main_on = AccOn1‖AccState∈{1,2,3,5}` + 掉沿 hold（fw `BYD_ACC_MAIN_FALL_HOLD=60` 帧@100Hz=0.6s > OP 0.5s，保证 OP 先停发→无 0x316/0x32E 空洞）。两棵树语义同步 |
| c | `selfdrived/selfdrived.py` | BYD 刹车不再加 `pedalPressed`（main-on latch 无上升沿可救 softDisable）；BYD 仅支持 `MadsSteeringMode=REMAIN_ACTIVE`（PAUSE/DISENGAGE 依赖 pedalPressed，本 port 不触发） |
| d | `opendbc/car/byd/carcontroller.py` | 纵向出力闸门 `long_active = longActive && !brakePressed && radar AccControlActive`；SNG 自动 resume 加 `!brakePressed` |
| — | `sunnypilot/mads/mads.py` | BYD 旁路 `block_unified_engagement_mode`：MADS 存活后 UEM 的"重武装丢弃 pcmEnable"会把纵向锁死，pcmEnable 必须始终达 SM |

## 四、验证（host 可跑的部分）

- `test_byd_carstate.py` **7 passed**：boot hold 边沿、SET 落入 hold、无 session 无边沿、
  刹车不退、standby 不退、main-off 退、bounce blip 不 flap available。
- `test_byd.py`（safety）**124 passed + 96 subtests**：fall-hold（59 帧不掉/60 帧掉）、
  standby 即 main-on、单帧 main glitch 保权限。
- `test_car_interfaces.py -k BYD` passed。
- MADS 状态机测试本机跑不了：`params_pyx.so`/`msgq/ipc_pyx.so` 都是设备 ARM 产物，
  Mac 加载失败（非代码问题；BYD 旁路只加函数首行早返回，非 byd 路径不变）。
- 固件：fw_base scons 重建成功，bin `01dac823`。

## 五、部署清单（设备上线后）

1. **刷固件**：`panda/board/obj/panda.bin.signed`（hash `01dac823`）；get_signature 128B
   == bin 尾 128B 核对（`/usr/local/venv/bin/python3`）。
2. **参数**：`AlphaLongitudinalEnabled=1`（纵向出力闸门才有意义）；`Mads=1`、
   `MadsUnifiedEngagementMode=1`（现状）；**不需 MadsMainCruiseAllowed**（arm 走
   latch→pcmEnable，非 available-边沿）；`MadsSteeringMode` 保持 REMAIN_ACTIVE。
3. 设备 GitCommit 应为 `3381318edd` 之后。

## 六、路测重点（新模型首验）

- **症状2**：行驶中踩刹车 → 横向应**不掉**（latch 跟 main，AccOn1 保 1）；松刹车
  纵向自动续。若仍掉：查 `cruiseState.enabled` 是否被 brakePressed 以外的东西拉低、
  查 fw 是否真刷成 `01dac823`。
- **症状3**：CANCEL（session 退、main 在）→ 横向**不退**、纵向让位；RES 后纵向续。
- **症状1**：拨 ACC main ON（未 SET）→ 横向武装（MADS active）；SET 后纵向加入。
- **camera glitch**：复现 route 11 t=156.4 那种 ~2s main 掉——>0.5s(OP)/>0.6s(fw) 会退
  （预期，是真 main-off 语义）；<hold 的短 blip 应免疫。
- **点火残留**：boot 时 ACC 开着（AccState=1 无 session）→ 15s boot hold 内不武装、
  出 hold 也不凭空 engage（ever_engaged 挡着）；司机 SET 后才 engage。
- 顺带：armed-零背停、EPS 锁存（横向手感层，与本重构正交，见 v2-validation §八）。

## 七A、部署后复验（2026-09-30 晚，route 1c/1d/1e；数据 `docs_site/byd_brake_2026-09-30/`）

设备跑 `3a257f1eac`+新固件（签名核对 ✓），实车参数 `Mads=0`、
`AlphaLongitudinalEnabled` 未开（与 §五 清单不同，是本次复验的真实语境）。
1c=下午旧代码（刹车时 CSEN/SD/FWCA 三层随 session 全掉=旧模型基线），
1d/1e=晚上新代码。**生命周期重构实证通过**：21+16 次刹车按压中
`cruiseState.enabled`/`selfdriveState.enabled`/fw `controlsAllowed` **无一随刹车掉**；
取消→重新 SET 的 CSEN/FWCA 恢复也正常（1e 33.4→34.26 CSEN 防抖-恢复链吻合设计）。

**但用户报"踩刹车依然退出/取消后重 SET 无助力非按 RES 不可"。根因是本文档漏掉的第四层**——
carcontroller 自己的两个否决（13b51bb5c4 架构收敛时刻意保留的旧硬停）：

1. `lkas_brake_inhibit`：踩刹 0.06s（3 帧防抖）→ 0x316 `LKAS_Active=0`。
   用户拍板：**可接受，保留不动**。（后被推翻——见 §七C，已删除）
2. `STEER_ARM_DRV_TORQUE=50` 手离盘武装门（根因15/16 遗产）：释放刹车/重 SET 后
   司机正在转向（实测 drv 83~182）→ burst 反复被拒 → **1.5~4.5s 无助力**
   （1d/1e 全部 27 个空窗量化在 `scripts/brake_lateral.py` 输出里）。
   **RES 不是恢复原因**：腾手去拨杆的瞬间 drv<50 门才开（1d t=162.48 未按任何
   按钮、手离盘自动武装；1e t=36.01 弯中松手 0.03s 后 `c=1 mt=81` 真实出力）。
   用户拍板：**手力门彻底删除**（厂商在任何手力下武装且从未因此锁存；锁存源
   rail/振荡已被 13b51bb5c4 + STEER_MAX_REQUEST=200 结构性移除）。
   修复 `447a360e53`：replay 14/14（cold-arm 在 cmd2 武装、|out|≤200）、
   carstate 7 + 接口 1 全绿。**下次路测重点=删门后冷/热武装的 EPS err 频率**。

分析注意：TX 0x316 必须走 `can`（src=128）解析，`sendcan` 不带 src 会把
bus=128 parser 喂瞎（本轮曾因此误判"Active 恒 0"）。

## 七B、删门后第二次路测（2026-09-30 深夜，route 1f/20/21；数据 `docs_site/byd_lkas_2026-09-30/`）

用户三报："时快时慢 / 先 SET 后上电不跟着激活 / 一次 LKAS Fault"。**代码版本先澄清**：
设备所有 boot banner（swaglog 1197/1201/1208）均 `dc41b4b5dc dirty=false`——删门代码
`447a360e53` 确实在跑（用户"没更新到"的猜测不成立；reflog：13:32:51 pull，随后重启，
1f/20/21=13:42–13:57 UTC 三次点火）。

### 1. 刹车再武装实证通过
20+21 全部 23 个刹车窗，释放→0x316 `LKAS_Active=1` **0.08–0.12s**（旧门 1.5–4.5s 空窗
全消失）。三个"不行"案例逐个定性，**均非生命周期回归**：
- 20 t=174–186：刹车 10.7s 停稳后 **"Gear not D"→soft-disable 设计内退出**（185.66
  sd_en 掉；司机停车换挡），横向随之；
- 21 t=311+：靠边停车、route 结束；
- 1f：t=85.7 EPS 锁存（见下）后 `steerUnavailable`=NO_ENTRY，**点火前再也启不动**——
  这就是"中途出现一次 LKAS Fault 之后时好时坏"的主线。

### 2. 唯一一次 LKAS Fault：1f t=85.21 err=2 → 85.71 err=4+TorqueFailed（锁存）
触发签名（`window_dump.py` 全程量化）：**低速 18–27 km/h 持续段**（90s 行程全程低速）。
EPS 对本 port 的会话按车速分频段拒绝——c=1 占比：0–9km/h≈0%、10–19≈20–35%、20–29≈28–53%、
40+≈67–100%（三 route 聚合）。1f 里 c=0/mt=0 时段 OP 仍把 TX 请求 **±200 满幅反复翻转**
（rail 21 段/14.6s，含 2.2s/1.6s 连续段），且司机手力 120–229 对抗；锁存前 7s 链条=
80.3 刹车切断→83.4 再 burst→+200 压司机→1.2s 内翻到 −200→EPS err。锁存后 mt 恒 0
直到点火。对照组：20（rail 仅 6.5s）与 21（rail 48 段/36s 但多在 30km/h+、c=1 率 66–100%、
req 与司机多同向）均无 err。**锁存风险窗口 = 低速 EPS 未接受会话(c=0) + |req|=200 满幅
翻转 + 与大司机手力对向**——正是删门时预留观察的"第四层残余路径"。

### 3. "先 SET ACC 后设备启动"链成立
三次 boot（1f=boot 时 main-off、10.3s 才 ON、14.4s session；20/21=boot 即 session=2）
全部在 **boot hold 末端 ~17.2s latch 边沿自动 engage**（`sd_en` 17.15–17.30，SET 落入
hold 被 hold-末端边沿吸收 ✓）。"没跟着激活"的真实成分=**15s BOOT_LATCH_HOLD 固有延迟**
（+1f 前 10s ACC main 实际是关的），机制本身无 bug。

### 4. 拍板结果 = A+B 组合，已落码（本分支，见下一节）

### 五A(补)、低速对抗守卫 A+B 落码（根因 17）
`carcontroller._update_torque_lateral` 两道新守卫 + `carstate.cruise_activated` 透传 +
values 四参数（`STEER_YIELD_OPPOSING_TORQUE=140`/`STEER_YIELD_DRV_RELEASE=90`/
`STEER_C0_WAIT_FRAMES=450`/`STEER_C0_RETRY_HOLD_FRAMES=300`）：
- **A. 对向让位（仅 c=0 时段！）**：EPS 未接受会话（`CruiseActivated=0`）且
  `demand*drv<0 and |drv|>140` 持续 0.1s（防 demand 正常翻向误触发）→ `allow=False`
  走既有斜坡退场；迟滞出=手力<90 / demand 转同向 / **c 翻 1**（EPS 接手即解除）。
  **c=1 会话内不守卫**：厂商满权限对抗（-153 vs +166 @c=1）是合法状态，若在那让位
  = 重造 13b51bb5c4 删掉的"对抗中 armed-零"锁存类。
- **B. 死会话超时**：Active=1 且 c=0 累计 9s（vendor 等待上限 2.86s，我们实测合法
  等待 clean 4.1s/邻 bounce 7.7s → 阈值取 9s 不误伤）→ 斜坡撤 Active + 重试 burst
  压 6s（c 提前翻 1 立即解禁）。c=0 期间 mt=0 本零出力，退场不损失任何助力，只把
  1f 的 45s 满幅流切成有界占空比。
验证：replay_latches 场景 15–18（1f 复现=c=0 对向 150ms 内缴械且不再武装；c=1 满权限
对抗不受扰；c=0 10s 超时退场；2.4s 正常等待不误触发）**18/18**；carstate 7 passed；
接口 BYD 1 passed（本机 params_pyx 为 ARM .so，用 /tmp/stub_params.py 插件打桩跑 host）。
纯 OP 侧改动，固件不动。**路测观察项**：①低速对抗时段的退让观感（预期=司机接管方向、
松手即回）②c=0 超时退场在起步等待期的误伤频率（9s 线）③err=2 频率（守卫的升级链应
只出现在守卫反应前）。

## 七C、刹车不再退横向（2026-09-30 深夜追加，用户拍板推翻 §七A-1）

用户："设置 ACC 后不退出控车，不管刹车还是不刹车，除非退出设置。"→ 删除
`lkas_brake_inhibit`（carcontroller 纯 OP 侧改动，固件不动，本分支提交见 git log）：

- 删 `_handle_brake_inhibit` + 两个防抖常量 + allow 项。横向的刹车耦合清零：
  **刹车期间 `LKAS_Active` 与请求流不断**，原"释放后 0.08–0.12s 再武装"整段
  消失（replay 新场景 `brake-through`：按压全程 armed 无掉落、出力连续）。
- 横向退出路径收敛为：**main-off 取消设置（latch+fw fall-hold）/ EPS 自报
  TorqueFailed·SteerErrorCode / standstill 门 / 守卫 A（c=0 对向让位）·
  守卫 B（c=0 死会话 9s 超时）**。纵向的 `!brakePressed` 让位与 SNG 刹车守卫
  **保留**（脚刹车时 OP 不能同时给油，属让位非退出）。
- 风险交代：长刹期间若 EPS 自行把会话判死（c=0），守卫 B 的 9s+6s 占空比是
  安全网（§五A 已落码，18/18 replay 含 1f 复现场景不变）；**路测观察项**：
  ①刹车期间 c 位是否保持 1（决定"刹车不断力感"是否真实达成）②长刹（>9s）
  退场/重试节奏观感。
- 验证：replay 18/18（场景 6 改为 brake-through 断言）；carstate/values/safety
  未动；`selfdrived.py` 注释同步（原"lateral via lkas_brake_inhibit"作废）。

## 七、已知取舍

- **状态1 字面义（MADS 下 main-off 也不退横向）不做**（用户拍板①）。第一版 main-off
  = 全退。若要独立"关横向"动作，候选是车机 LKA 设置开关（bus-2 0x316 `LKAS_Config`
  弱证据 on=3/off=0，需实车确认）。
- **状态2 冷启动（main-on 但本循环从未 SET）暂不武装横向**——`ever_engaged` 故意拦
  点火残留 phantom engage。安全方向；实际流程几乎都是 SET 起步。
- 0.9.x 部署 fw 无 sunnypilot MADS C 状态机，横向闸门=`controls_allowed`
  （main-on pcm_cruise_check + 0xf3 心跳双路径）；latch 保持 engaged 贯穿 standby
  → 心跳 engaged=1 → controls_allowed 保 → 横向存活。逻辑自洽，但**路测须实测确认
  fw 层未意外收紧横向**。

## 七D、用户三天路测四问题复盘 + 根因 18(2026-10-02/03 数据,fix 2026-10-04)

用户报告:①设 65 实跑 ~76(表显) ②纵向"设定就接管、没设速一直往上加" ③"车控"
易退出、退出后几秒才恢复(要求秒内,可参考厂商;困难则接受"启 ACC 才控车")
④三天两次 "LKAS Fault"。设备在线跑 1c49a6e(=本分支 HEAD),rlog 全量分析见
`docs_site/byd_roadtest_2026-10-03/`(audit_3d.py / speed_fit.py / fault_scan.py /
tx_scan.py / can_window.py,均在设备 /tmp 同源)。

### 1. 车速偏差 = ESP_SPEED 信号 0.9 km/h/bit(问题①,定案)
6 个 boot 会话 ~19k 个 1Hz 样本,`GPS/ESP = 1.112-1.118` 且 20-120 km/h 全速域平坦
(0000002a/2b/2c/2d/31/32)。OP 把 raw 当 1 km/h 用 → 控制目标 = 表显/1.04 的 ~0.9 倍
真速,"设 65 实 76"完全复现(65→真 72.3→表显 ~76)。**fix:HUD_MULTIPLIER=10/9**
(values.py,wheelSpeedFactor 通道;carstate 注释里的"相机里程计验证"是循环论证,已改)。

### 2. 油门噪声假接管(问题②③的机制层,定案)
PEDAL `AcceleratorPedal` 0.01/bit,判据 `>0.01` = 1 个 LSB 噪声即触发:SD 状态
enabled↔overriding 秒级翻 8 次(2d t=1485、31 t=1486),每次假油门松开纵向执行器;
真油门压下降时厂商雷达进 `AccState=5 (FORCE_ACCEL)/AccControlActive=0`,OP 让位
(session gate)并**原样回播雷达 AccelCmd** → 车在"没设速/低目标"下持续爬
(31 t=1365-1372:tgt=40, gap 中 vEgo 46→60)。3.7h 会话 32 里 longActive 空窗
140 次 >2s(v 全速域)。**fix:PEDAL_PRESS_THRESHOLD=0.05**(油门+刹车模拟量共用)
+ **session 内 SetSpeed==0 时 accel 只许 ≤0**(carcontroller `_update_longitudinal`,
"没设速永不加速",接管时机语义不变:仍只在真会话控纵向)。

### 3. 根因 18:EPS 会话中扭矩静默弃权 → 开环冲高 → err4 锁存(问题④,定案)
两次 LKAS Fault(28 t=2053.8 v=69、2b t=1095.4 v=59)同签名(CAN 窗口转储
`/tmp/win28.txt`/`win2b.txt` 同款于 docs_site/byd_roadtest_2026-10-03/):
`MainTorque 75→0 + SteerWarning=1` 而 **err 仍 0、c 仍 1**(守卫栈全瞎),
OP 无回显反馈继续积分,请求 0.4-0.7s 内 121→132/148→180,随后 err=4+TorqueFailed
锁存、ACC 进 st=7。手力仅 -14~+9(非 1f 对抗类)。**与厂商差的那环 =
STEER_ERROR_MAX=46 回显核对**(厂商 values 里有、python 侧从未实现)。**fix(guard C)**:
c=1 且 |apply−MainTorque|>46 持续 10 命令帧(0.2s)→ stand-down + 3s 保持重试
(仅 c=1 生效:厂商 c=0 等待流 193@mt=0 合法,7--12e,归守卫 B 管);
另 **SteerWarning 进 allow 硬停**(本案 warn 与 mt=0 同帧,1 帧即缴械,0.2s 退出
< err4 前 0.42s 窗口)。27 t=1527 同类 warn 0.5s 自愈场景亦被 warn 闸收住。
恢复节奏:warn 清除 → hold 走 → 3 帧 burst 重武装(亚秒级,满足"秒内")。
守卫 B(9s+6s)不动——tx_scan 实测其触发极罕(32 会话 armed 期 c-flap 未及)。

### 4. 横向"退出"的用户感知归因(问题③)
latActive 指标在 31/32 全程无掉(掉的那几次全是 main-off/park);用户体感的
"退车控几秒"主体 = 上面 §2 的纵向空窗(油门不跟了)+ 真会话 3↔5 弹跳让位。
守卫 A/B/C + warn 闸处理横向侧真实 stand-down,replay 21/21(新增场景 19 warn 弃权
缴械+清除即复、20 静默分歧 growth-stop@14cmd、21 满速率正弦 100% 不误伤;
旧 18 项含 brake-through/c0 全保留)。**厂商对照**:厂商从不出 mt=0 还冲请求的
状态(它有回显核对),本案补齐这一层即对齐。

### 5. 验证与设备侧数据链
- 新码 shadow-import(不碰 /data/openpilot)在设备跑:replay 21/21;真 rlog 段
  (28--34/35)CarState.update() 2001 帧无异常、vEgo×1.1111 生效、steer_warning 真实触发。
- 待办:OTA 后首轮路测观察 ①定速 65=真 65(表显 ~68)手感 ②油门/刹车不再抖
  overriding ③偶发 err=2/warn 是否 1-2s 内自恢复不再锁 ④"没设速"上电接管不再冲。
