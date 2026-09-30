# BYD 宋 PLUS DM-i · v2 实车复验报告（route 0000000e/f）

> 数据：2026-09-30 两段路测。route `0000000e`（4 seg，281s）+ `0000000f`（5 seg，302s），
> 设备 GitCommit=`9a46d312e4`（根因 16 v2 = `51b1a25a5f` 在跑），固件 fec63fda。
> 配置：`AlphaLongitudinalEnabled` 参数不存在 = **OFF（纵向 = 原车 ACC）**；NNLC=ON（未标定）。
> rlog + 分析脚本存档：`docs_site/byd_v2_validation/`（gitignore，本地保存）。
> 状态：v2 核心目标全部达成；新发现 4 项待拍板（§六）。最后更新：2026-09-30。

## 一、v2 核心验证结论（对照 `byd-control-lessons.md` §五决策树）

| 验证项 | 结果 | 证据 |
|---|---|---|
| 整体能控 / c 门死锁解除 | ✓ | route e armed ≈234s/281s，其中 **72% EPS 真执行**（c=1）；route f 84% |
| 零 TorqueFailed 锁存 | ✓ | 9 seg 全程 TorqueFailed=0（v2 前的历史：10 次锁存） |
| err=2 不升级 err=4 | ✓ | 全程无 err≥2 记录（含 bounce 风暴时段） |
| 手轻武装门（根因 15） | ✓ 工作 | SET 后手扶盘：engage→burst 延迟 0.6-3.2s；手离开：0.02-0.06s 秒装 |
| 请求包络 200（根因 16v2） | ✓ 顶格即止 | armed 期间 MT max=200（= 包络），不再出现 ±300 rail |
| MT 分布 vs 厂商 | ⚠️ 偏热 | 我方 p50 61-73（厂商 52）/ **p90 159-180（厂商 115，1.5×）** / max 200（厂商 193） |
| sign flips（遗留观察项） | ✗ **坐实** | 见 §五 |
| ACC bounce | ✗ **新发现** | 见 §四 |

用户体感"整体有控车、效果不错"与数据一致（EPS 真执行占 armed 大头，MT 中位数与厂商同量级）。

## 二、症状 2 定案：boot-mid-cruise engage 边沿吞噬（第 2 段）

**现象**：ACC 已开 → 设备才启动 → 迟迟不控车，需 ACC 多次重新进入才能控车。

**机理**（`car_specific.py:215` + `carstate.py:144`）：engage 唯一入口 = `cruiseState.enabled`
**上升沿**（pcmEnable）。OP 启动前 ACC 已真活跃（AccControlActive=1）时，`is_cruise_latch`
第一帧即 True——边沿在 OP 醒来之前已经发生完，永不再来 → OP 永不 engage，无绿框。

**route f 证据**：开头 1.3→21.9s 共 **20.6s "ACC-on-OP-off"**（ACC 持续 on、selfdriveState.enabled
恒 False），21.9s 司机取消→重 SET 造出边沿后才 engage。

**"多次重新进入"的解释**：每次取消→重 SET 只造一次边沿；前几次失败 = 撞 selfdrived
就绪窗口（点火后 ~15s，ops 文档 §三.2 已知）。就绪后的重进一次即成功。

**修复候选（v3-1，不碰 EPS 面向行为）——✅ 已实现（见 `selfdrive/car/car_specific.py` BYD 分支
`BOOT_ENGAGE_STEADY_TIME`）**：stock ACC 的严格 latch（AccControlActive 基）连续保持 ≥3s 且 OP
仍未 enabled → 补发一次 pcmEnable。设计要点：
- **所有 NO_ENTRY 门照常**（gear/door/标定/故障）——只恢复"司机意图边沿"，不绕任何检查；
  就绪窗口（NO_ENTRY）期间的补发**不消耗**，每帧重试直到门清（单测覆盖）。
- **手轻不要求**（与真 SET 按压对齐；手轻由 EPS 武装门 `STEER_ARM_DRV_TORQUE` 在会话层管）；
  standstill 同理允许（controller 侧 standstill 本就挡武装）。
- **自限**：CC.enabled 一旦 True 永不再发（真边沿激活→合成器永久退役；之后的取消/重 SET
  走真边沿，不自动重engage）。
- 单测：`selfdrive/car/tests/test_car_specific.py` 5 场景（steady 时序/无 ACC/刹车重置/
  NO_ENTRY 重试/真 enable 后退役），本地 `pytest --noconftest -c /dev/null`（arm-only .so
  需 stub，见测试文件头）。待实车复验：boot 时 ACC 已开 → 就绪后 ~3s 应自动出绿框。

## 三、症状 1：绿框早、控车晚 = 三层延迟叠加（第 1 段）

绿框 = OP engaged（`selfdriveState.enabled`），到 EPS 真出力（c=1）之间实测三层：

| 环节 | 实测 | 性质 |
|---|---|---|
| ① engaged → 武装 burst | 手扶盘 0.6-3.2s；手离开 0.02-0.06s | 手轻武装门 `STEER_ARM_DRV_TORQUE=50`（根因 15），by design 但 UX 可感。与 deep-review 控车#1（STEER_THRESHOLD=80 落轻握带 60-150 → 轻握既 pressed 又挡武装）同族 |
| ② burst → EPS 接受（c=1） | 常态 0.2-1.1s，最长 2.8s | 厂商语义（等待相位）；**drv=0 时 EPS 也等 → 等待条件非纯手**，条件未逆向 |
| ③ 等待相位 windup | c=0 期间 demand **饱和在 200 包络** | 我方独有：等待相位占 armed 时间 16%（route f）-28%（route e）。厂商等待相位 demand ≤193 从不 rail（其环几乎不积分全靠前馈）；我们 rail 满 200 |

**风险**：③ 的 windup 在 EPS 激活瞬间全额释放。本两段未捕捉到明确的"rail 态激活猛给"
样本（234.5s 的 +23° 转角经 MT=0 证伪为司机打盘，非 EPS 释放），量级未定，但机理存在
——根因 16 v1（0000000a）的"c 翻 1 瞬间甩盘"就是同类形态，包络 200 已把最坏值从 300 压到 200。

## 四、新发现 A：ACC bounce 风暴 → OP enable 抖动 → 0x316 流空洞

两段路测最重要发现。**armed 时段雷达 ACA（ACC_CMD.AccControlActive，bus2 原发）以
60-110ms 打摆**，伴随 ACC_HUD_ADAS `Notify=8`；gas/brake/按键/steeringPressed 全零 = 非人为。
- 分布：route e 12 次 + route f 4 次，**16/16 全部发生在我方 armed 时段**；idle 时段零。
- 节奏：间隔 500→460→240→120ms **递增**（雷达渐怒），每次 ACA=0 持续 60-110ms。
- 厂商对照：3 条厂商 route（7--12e/6--ff9/5--44a）**零 bounce**；但厂商首个 armed 会话
  启动 6.8s 后 ACA 也让位过一次（40s 长让位，随会话结束恢复）——"LKA active 时 ACC
  短暂让位"疑为车的正常联动，厂商是低频长让位，我们被自己的循环放大成 2Hz 抖动。

**连锁反应（帧级实证，route e seg3 222.8-226.8s）**：

```
雷达 ACA 跌零 60-110ms + Notify=8
  → AccState 3→1，stock_acc_on=False → is_cruise_latch 跌零
  → pcmDisable → OP disable → controls_allowed=0
  → 固件拒发我方 Active=1 帧（byd.h：active=1 需 controls_allowed）
  → 0x316 流空洞：退场斜坡期间帧全被 REJECTED（src=192），4 处，
    最长 21 帧 / 0.42s 连发拒收 —— EPS 收不到优雅退场（斜坡+Active=0 收尾）
  → ACA 恢复 → latch 回 1 → pcmEnable 上升沿 → OP 重 engage → re-burst
  → 循环，直到雷达风暴自行平息（~4s）
```

route e 单段因 bounce 产生 **20 次 enable 循环**（0.3-0.6s 间隔的假"重新激活"）。
本次未升级为 err 的原因：0.42s 空洞 < 实测 armed-零锁存带 0.48-0.72s——**贴线幸免**，
再长一档就是 EPS 断流投诉（SteerWarning/根因 14 族）。

**触发源未定位**。嫌疑排序：①需求环振荡（§五，雷达/watchdog 对转向活动不满）→ 修振荡
可能连带消失；②armed-零背停的 0.45s 会话重启节奏可疑但时序不符（bounce 先于我方退场）；
③雷达对 0x316 内容的校验（我方 armed 帧与厂商逐字节同分布，含低频罕见模式，已排除内容差异）。

**修复方向（v3-5，先修上游再看）**：
- 上游：§六-1 振荡杠杆 + §六-2 NNLC A/B。
- 空洞本身：退场斜坡（~80ms，out>0 时 Active=1）与 controls_allowed 抖动冲突。
  选项 a：controller 侧 latActive 掉时 Active=0 硬切（放弃斜坡，违背厂商退场编排）；
  选项 b：固件放宽（controls_allowed 撤销后允许 Active=1→0 斜坡 N 帧）；选项 c：不动，
  待上游修复后复测。倾向 c → a 备选。

## 五、新发现 B：需求环振荡坐实（sign flips 观察项结案）

根因 16 v2 遗留观察项"0000000d 回放 sign flips 46>40（疑似零输出期规划器追人手的回放
伪影）"——**实车改判为真实存在**：

- 直道、方向盘静止（0.5s 窗口转角 range ≤0.6°）时，我方 LKAS_Output 在 ±50 摆动
  （0.5s range 42-76，route e seg3 222.5-227.5s）。
- armed 时段 EPS 转角速率 p90=96-160°/s、p99=272-288°/s、max=464°/s——历史"正常控车"
  max=129°/s、p90=55（根因 9 时代），现在**整个分布上移 2-3 倍**。
- MT p90 159-180 vs 厂商 115：出力整体偏热，与振荡互为印证。

§五决策树第二行触发条件达成。注意状态文档 C-4：**NNLC=ON 且从未标定**——先做
NNLC 关/开 A/B 再上 governor，避免两个变量混在一个路测里。

## 六、待拍板清单（v3 候选，按建议优先级）

| # | 项 | 改动面 | 说明 |
|---|---|---|---|
| 1 | NNLC A/B（先关 NN 跑一段基线） | param，无代码 | 振荡的第一嫌疑人；C-4 已列，现在有了实车证据 |
| 2 | 振荡杠杆：yysnet 转角速率 governor（param 默认关）或厂商 EMA | 控制行为 | §五决策树第二行已触发；可能连带消 ACC bounce |
| 3 | boot-engage（症状 2） | engage 链 | 非控制行为；守卫见 §二；或维持规程 workaround |
| 4 | 等待相位 windup 抑制（c=0 时冻结/衰减 demand） | 控制行为 | 与 v2"stream and wait"有交互，需设计：参考厂商"几乎不积分全靠前馈"的环形态 |
| 5 | 0x316 流空洞（§四修复方向 c→a） | 视复测结果 | 上游（1/2）修复后复测，仍出现再动 controller/固件 |
| 6 | STEER_THRESHOLD 标定 | 参数 | ops 文档说 60-70、deep-review 说提到轻握带之上（≈150-160），**两说冲突**，需按本车 drv 分布定（本次数据：轻握样本 60-160+） |

纪律提醒：2/4/5 触碰控制行为，逐项拍板、单项路测（勿多项同改——根因 14-16 的教训）。

## 七、分析工具与数据（docs_site/byd_v2_validation/）

```
r0000000e_{0..3}.zst / r0000000f_{0..4}.zst   # 两段路测 rlog（设备 /data/media/0/realdata/）
scripts/
  analyze_engage.py   # 逐 engage 事件时序：enabled→latActive→burst→armed→EPS c=1 + 手扭矩
  scan_bounce.py      # 全程 ACA 跌零统计（bounce vs 长让位分类、armed 关联、Notify）
  fine_trace.py       # 帧级三路并显：我方 TX(src=128)/相机 0x316(src=2)/EPS 0x318
  dump_trans.py       # 状态变化行打印（latch/ACA/op/lat/c/tx 摘要）
  eps_exec.py         # EPS 执行判别：c + drv + MainTorque + 转角/转率
  exec_stats.py       # armed 时段 c=1 执行率 + MT 分布（支持厂商 sendcan 对照）
  dump_acc.py         # ACC 域信号 + 踏板/按键（排除人为）
  accel_trace.py      # 我方 ACC_CMD echo vs 雷达原帧 accel diff（OP long 离题验证）
  angle_trace.py      # 0.5s 桶：转角 range / 出力 range / 翻向数（§五 振荡证据）
  rej_timing.py       # REJECTED 0x316 帧时间 vs bounce/armed 时段（§四 流空洞证据）
  vendor_overlap.py   # 厂商 sendcan armed 会话 vs ACA 时间线（§四 厂商让位对照）
  diff316.py          # 我方 vs 厂商 armed 0x316 全字段模式 diff（§四 内容排除）
  rate_compare.py     # armed/idle 转角速率分布（§五）
```

用法：`PYTHONPATH=$PWD/opendbc_repo:$PWD .venv/bin/python scripts/<x>.py <rlog> [w0 w1]`。

**src 编码备忘（本次踩坑修正）**：can 流 `src` = bus 号 + 特殊位。
`0`=bus0 RX、`2`=bus2 RX、**`128`=bus0 TX loopback——我方 TX 与 fwd_hook bus2→bus0 转发副本
混在一起**（0x316@128=我方（790 转发被 block）；0x32D/E/F@128=雷达帧转发副本，非 OP 纵向！），
`130`=fwd bus0→bus2 转发副本（0x11F/0x318 等全量 1:1），`192`=固件拒收。
CANParser 按 `src==bus` 过滤（`parser.py:221`），跨 src 同址帧（0x316/0x32E 双源）必须分 parser。

重新拉取：`ssh comma@192.168.31.44 -i ~/Documents/op/menmen.key
"cat /data/media/0/realdata/<route>--<seg>/rlog.zst" > <route>_<seg>.zst`
（本地文件名含 `-` 会让 logreader 误判路由名，存成 `r{route}_{seg}.zst`。）
