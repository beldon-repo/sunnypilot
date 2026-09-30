# BYD 宋 PLUS DM-i 移植 · 现状总览（新 Session 入口）

> 最后更新：2026-09-30 深夜。本文是**当前状态的唯一权威快照**；历史根因全录见
> `docs/byd-song-plus-lkas-debug.md`（6.1-6.30），厂商对齐设计见 `docs/byd-vendor-alignment.md`，
> **v2 实车复验结果见 `docs/byd-v2-validation.md`（2026-09-30，4 项新发现待拍板）**。
> 冲突时以本文为准。

## 一、一句话现状

横向 = **厂商会话架构 v2**（`51b1a25a5f`：撤 c 门 + 手轻武装门 + 请求包络 200），
**实车复验已通过**（2026-09-30 两段 route 0000000e/f：整体能控、零 TorqueFailed、
err 不升级、armed 时段 72-84% EPS 真执行——死锁/锁存史上的 10 个根因全部关闭）。
**下一步 = v3 小改拍板**（复验暴露 4 项，全不锁存）：①NNLC A/B（振荡第一嫌疑）
②需求振荡杠杆（governor/EMA，sign flips 已坐实）③boot-mid-cruise engage 边沿吞噬
（**✅ 已按厂商同款修复**——carstate 层 `BOOT_LATCH_HOLD_TIME=15` 压 latch 出迟到真边沿，
厂商 7--12e 实录解码，car_specific 零改动；见复验报告 §二，待路测）
④等待相位 windup（c=0 时 demand rail 200）。
另有 ACC bounce 风暴观察项（armed 时雷达 ACA 2Hz 打摆 → 我方 0x316 退场流空洞，
贴线未升级），详见复验报告 §四。

## 二、状态快照

| 项 | 值 |
|---|---|
| 分支 / HEAD | `main-c3l-tici`（本地含复验后文档；推送状态见 §五C-1） |
| 设备 | comma@192.168.31.44，GitCommit=`9a46d312e4`（= v2 `51b1a25a5f` + 文档提交） |
| 部署固件 | fw_base(0.9.x) 构建，bin sha=fec63fda…，uno+h7 已入库（`panda/board/obj/`，gitignore 需 -f）；签名比对 ✓ |
| AlphaLongitudinalEnabled | **OFF**（纵向走原车 ACC；2026-09-30 设备实测参数不存在；固件无 Phase 2 纵向，自洽） |
| NNLC（NeuralNetworkLateralControl） | ON（权重 json 在设备，从未路测标定；**复验发现需求振荡，A/B 是 v3 第一候选**） |

## 三、横向控制器现状（`opendbc_repo/opendbc/car/byd/`）

**架构 = 厂商会话状态机**（route 00000037 帧级实证，勿再加防御塔——10 次锁存的教训）：

- idle：`Config=3/Act=0/lanes 0/0` 常开（厂商 standby 姿态，勿回退到"回显相机 Config"）
- engage：3 帧 ReqPrepare 突发（lanes 2/2）→ 50ms 无条件 Act=1，**不等 EPS ack**
- 会话内：满权限 demand（STEER_MAX=300 / ALLOWANCE=120 / DELTA 16/16，全部=厂商 values.json 真值）
- 退场：斜坡出 → 立即 re-burst 重连；TF 硬切不复活；SteerErrorCode≠0 退场不重试
- 仅存硬停：刹车抑制、standstill、TF、err≠0、armed-零 0.42s 背停（实测 0.48-0.72s 锁存带）

**已删除（勿复活）**：让位曲线、对向探测/follow 翻转、软启动、静默守卫旧逻辑、
角度/速率门、10s 锁定、武装门（根因15）、角度 holdback（根因 6.30 的 100s 静默来源）。
**EPS 锁的是"犹豫的会话"（armed-零、振荡小出力），不是对抗**（厂商 -153 vs drv+166 实证）。

**参数纪律**：一切参数以 `docs_site/op_byd_data/values.json`（厂商运行时真值）+
厂商 ELF struct 直读为准，禁止日志反推。

## 四、固件现状（两棵树，注意区分）

| 树 | 用途 | 现状 |
|---|---|---|
| `~/Documents/op/panda_fw_base`（0.9.x） | **当前部署固件的构建树** | allowance 120、rate 18/18、**0xf3 心跳 `controls_allowed = engaged`（立即放行/3-strike 下降）= mismatch 死锁修复**；无 MADS、无 Phase 2 纵向 |
| `opendbc_repo/opendbc/safety/modes/byd.h`（新版式） | 未来 repo 构建固件用 | 含 MADS 接线且已修 **engaged 喂入（AccState 2/3/5，勿回退 AccOn1——宋上 AccOn1 全程 1 无边沿会死锁）** |

构建：`export PATH="$HOME/Documents/op/arm-tc/bin:$HOME/.local/bin:$PATH"`，
`cd ~/Documents/op/panda_fw_base && scons -j8 board/obj/panda.bin.signed board/obj/panda_h7.bin.signed`，
产物 cp 回仓库 `panda/board/obj/`（提交需 `git add -f`）。

## 五、待验证清单（按优先级）

### A. 设备验证（✓ 2026-09-29 晚完成，设备 192.168.31.44）
1. GitCommit = `0cfd780854…` ✓（deploy 确认）
2. **固件刷写 ✓**：设备 `get_signature()` 128 字节与本地 `panda.bin.signed` 尾 128 字节
   逐位一致（首 8 字节 uno `51ccab3e3af4c7ff` / h7 `2b09882f8041d8ad`；签名块在 bin 尾部
   offset ~57572）。注意用 `/usr/local/venv/bin/python3`（系统 python3 无 usb1 模块）。
   远程通道不可用：DongleId = `UnregisteredDevice`（未注册 comma 账号）；
   0580020f… 等三个 id 是厂商 route 的，不是本机。
3. CAN 静态 ✓（点火前：pandaStates 10Hz、faults=[]、safety=noOutput/0 正常姿态）；
   **byd@35 + bus0 流量需点火后确认**（随 §五 B 首次路测顺带看）。

### B. 实车验证（✓ v2 复验 2026-09-30 完成，详见 `byd-v2-validation.md`）
1. **mismatch 死锁修复 ✓**：route f 多次"行驶中取消 → 重 SET"循环（22.9/216.3/226.7s
   ACC 中断后重进）全部正常恢复控车，全程无 "Controls Mismatch"。
2. **爬行大弯/泊车全程有辅助**：未专项验证（route e 235s 见过 +23° 时段但为司机打盘）。
3. **armed-零背停平顺性**：退场/re-burst 机制工作，但被 ACC bounce 风暴放大成断续
   （复验报告 §四）；体验问题并入 v3 拍板清单。
4. **err=2 警告频率 ✓**：9 seg 零 err≥2 记录。
5. 手感：用户反馈"整体有控车、效果还不错"；对抗时正面出力 = 厂商原味。

### C. 遗留工程（实车干净后再动）
1. **推送 origin**：`28145a1c3b`（Phase 2 纵向）到 `0cfd780854` 全部未推送。
2. **Phase 2 固件同步**：开 AlphaLongitudinalEnabled 前必须把 repo byd.h 的纵向
   （TX_MSGS_LONG/0x32E accel/fwd 挡）+ MADS 移植进 fw_base（0.9.x API 不同，手工移植），
   或切换到 repo opendbc_repo panda 树构建。
3. **横向环稳定性**：route c9f1698c82 出现过 demand ±130 饱和振荡（3Hz）。
   架构重写后环已闭环，但未实车确认；下次 route 直接查我方 TX 的 sign-flips
   （厂商包络：27s 内 8 次）与 armed-silence。
   **本地回放已过（2026-09-29）**：c9f1698c82--0 与 909633d7ed--5（根因 15 泊车
   场景）灌新控制器均 CLEAN——armed-silence 最差 0.02s/0.08s（锁存带 0.48-0.72s），
   sign flips 均 5 次（包络 8/27s）；replay_latches.py 11 场景全绿。仅剩实车确认。
4. NNLC A/B：NN 权重从未标定，若振荡复现先关 NeuralNetworkLateralControl 对比。
5. 厂商行为还原（ELF `byd_adjust_steer_torque` 可反汇编）：get_byd_torque_limits 的
   LOW flag 选择、24s 连续转向降额+SDA、STEERING_TORQUE_LIMIT_SPEED 速度曲线。
6. `sync-20251218-tici` 分支同步：今天的架构/固件/参数改动都要合过去（另一条产品线）。

## 六、资料与工具索引

| 资源 | 路径 | 说明 |
|---|---|---|
| 厂商参数真值 | `docs_site/op_byd_data/values.json` | values 模块运行时 dump，**参数第一权威** |
| 厂商装机树+固件 ELF | `docs_site/op_byd` | panda.elf 带符号：BYD_STEERING_LIMITS=[300,18,18,…,46]，LOW=[300,9,9,46]，HIGH error=200，ALT=[150,50,50]；`byd_adjust_steer_torque`/`byd_steer_torque_cmd_checks` 可反汇编 |
| 厂商实车日志 | `docs_site/op_byd_logs/`（route 35/36/37） | 会话生命周期/标定全部出自 7--12e_0 |
| 厂商解密源码 | `/Users/wujiafu/Documents/op/cp_byd/docs/pyarmor_decrypted/` | disasm+values 100% 覆盖 |
| **三方对照+手册经验库** | `docs/byd-control-lessons.md` | 厂商/yysnet/官方手册可吸收项与决策树（新 session 先读） |
| **v2 实车复验报告** | `docs/byd-v2-validation.md` | 2026-09-30 route 0000000e/f 全量分析：v2 达成项 + 4 项新发现（ACC bounce/流空洞/windup/振荡/boot 边沿吞噬）+ v3 待拍板清单 |
| **v2 复验数据+脚本** | `docs_site/byd_v2_validation/` | rlog 9 段 + 21 个脚本（13 分析 + 8 调试迭代）+ src 编码备忘（gitignore 本地） |
| **代码 review 报告** | `docs/byd-code-review.md` | 09-28~09-30 提交的全量 review：2 高危（0x122 vehicle_moving / 0x32E echo-relay）+ 4 中危 + 低危清单 |
| **控车专项深度 review** | `docs/byd-control-deep-review.md` | 聚焦"能否真正控住车"：会话状态机核对（正确）+ 7 项控车风险（轻握即脱/0x316 断流/纵向拦原厂制动/固件限速集不匹配/MADS 解耦/lead 锁死/缩放 67%） |
| 官方维修手册文本 | `docs_site/pdf/2021年款比亚迪宋PLUS DMi-01-维修手册-*.txt` | ACC+MPC 分册（EPS 分册缺，SteerErrorCode 码表仍在找） |
| 回放套件 | `docs_site/hour_logs_2/replay_latches.py` | 11 场景（厂商语义），本地跑：`PYTHONPATH=$PWD/opendbc_repo:$PWD .venv/bin/python …` |
| route 回放 | `docs_site/hour_logs_2/replay_routes_new.py` | 真实 route 灌新控制器（本地，rlog 样本在 /tmp/byd_routes/ 会丢可重拉） |
| route 总览审计 | `docs_site/hour_logs_2/route_audit.py` | FAULT/armed 时段/告警一览（需 scp 到设备 /tmp 跑，/tmp 重启即清） |
| 固件构建树 | `~/Documents/op/panda_fw_base` + `~/Documents/op/arm-tc` | 见 §四 |

## 七、新 session 快速上手

```bash
# 设备状态
ssh comma@192.168.31.44 -i /Users/wujiafu/Documents/op/menmen.key \
  "cat /data/params/d/GitCommit; ls -t /data/media/0/realdata | head -3"

# 最新 route 一览（先 scp docs_site/hour_logs_2/route_audit.py 到设备 /tmp）
ssh … "/usr/local/venv/bin/python3 /tmp/route_audit.py /data/media/0/realdata <route_prefix>"

# 设备 python = /usr/local/venv/bin/python3；日志在 /data/media/0/realdata/<route>--<seg>/rlog.zst
# 我方 TX 在 can 流里以 src=128 出现（LOOPBACK），fwd 拒收=192（REJECTED），bus0 真 RX=src0
```
