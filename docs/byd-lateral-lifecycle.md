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
   用户拍板：**可接受，保留不动**。
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
