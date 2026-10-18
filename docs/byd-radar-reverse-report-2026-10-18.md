# BYD 宋 bus1 雷达逆向 — 整体报告（2026-10-18）

> 一句话：**宋的 bus1 确实有雷达原始数据**——0x101-0x10A 是 Continental ARS410 家族的目标帧，
> 其中 0x109 b7 是领先 HUD 档位的连续前向距离（候选缩至 feet 0.3048 vs 0.25，1.0/0.5/0.45 已被 13 起跟停事件硬上界排除）、0x10A b3[7:5] 是目标存在位、
> b0 已破译为 SAE J1850 CRC-8；但**没有相对速度场**，完整 track 语义仍不可覆盖。
> 六帧已入 dbc（HYPOTHESIS，无任何在车实例化），控制行为零变更。
> 细节定案见 `docs/byd-radar-010x.md`，本报告为全程汇总。

## 1. 起因与结论

- 起点问题：分析 `docs_site/hour_logs`（2026-09-28 深夜首次 40 分钟连续控车的 4 段全量 rlog）的 can1 有无有效雷达数据。
- 首轮结论（按既有 tangdm 布局判）：0x351-354 槽位恒空、0x374 零帧 →「无有效目标流」，与 10-11 定案一致。
- 翻案：抛开 IsValid 与已猜布局做字节级挖掘后，**0x101-0x10A 被证明携带真实目标信息**，
  推翻 10-05_07 复扫「bus1 新面 0x101-0x10A=状态广播/计数器伪影」中涉及 0x109 的部分。

## 2. 方法演进（五个阶段，工具全部入库 tools/byd/）

| 阶段 | 方法 | 工具 | 产出 |
|---|---|---|---|
| ① 字节级挖掘 | 0x32D HUD 档位当监督,对 bus1 全地址×全字节算互信息 MI | radar_re_mine.py | 命中 0x109 b7 MI=1.28bit,两段独立单调 |
| ② 跨段验证 | 416 段 rlog 逐段档位序检验(防新鲜度伪影) | radar_0109_validate.py | 严格单调 102/113=90.3%,c0 归零 96.3% |
| ③ 对抗审查 | 三路独立证伪:时间假象/HUD 回声/单位自洽 | radar_echo_probe/checks.py, radar_unit_check*.py | 计数器说死、回声说死(8/10 翻转 b7 先行)、米制被疑 |
| ④ 监督反解 | can2 为标(档位/时距带+vEgo),单/双字节×LE/BE×符号全变体回归 | radar_sup_mine.py | 0x109 b7 唯一跨段稳定;无 vRel;b0 确定性函数;b6/b7 平行 |
| ⑤ 钉定收口 | 静止逼近硬上界+翻转回归+CRC 试打,486 段全量+独立日 | radar_unit_pin.py + radar_unit_aggregate.py + radar_415_diversity.py | u≤0.467(米制否)、CRC 破(100%×2 组)、多样性终检成立 |
| ⑥ 跟停等式 | 13 起 ACC 跟停事件(standstill+驻停平台+存在位),逐窗 u≤D/Δb7(前车不可倒车) | radar_stop2lead.py (/tmp/radar_stop2lead.json) | **u≤0.42-0.44 → 0.5/0.45 判死,候选缩至 feet vs 0.25** |

方法论教训两条：**单调性检验必须跨段**（同段内新鲜度/时间共线都会造出伪相关）；
**单位标定必须有物理恒等式锚**（前车静止时 `−d(dist)/dt = vEgo`），日志内即可完成，不必等路测。

## 3. 最终字段定案

| 字段 | 定案 | 置信 | 关键证据 |
|---|---|---|---|
| 0x109 b7 | **连续前向目标距离**，u∈[0.25,0.44] m/计数，**候选缩至 feet(0.3048) vs 0.25 二选一**；0=无目标哨兵，247 疑超量程 | 量高/值未钉 | 单调 90.3%×2 组归档、distinct 240、领先 HUD 翻转、**13 起跟停事件硬上界 u≤0.42-0.44**、驻停平台 15c→feet 4.6m/0.25 3.7m、feet 下 τ=0.87/1.24s 更自然 |
| 0x10A b3[7:5] | **目标存在位**：111=前向可见目标 | 高 | 全 384 位唯一干净分离：lead 96.5% vs no-lead 0.0% |
| 全 6 帧 b0 | **CheckSum = SAE J1850 CRC-8**（poly 0x1D/init 0xFF/无反射/xorout 0xFF，载荷 b1..b7） | 确凿 | 2933 唯一载荷 100% + 独立日 5 帧型 80100/80100 |
| 全 6 帧 b1 | AliveCounter 滚动计数（低 nibble 每帧 +1） | 高 | 跨帧恒 +1 规律 |
| 0x101/0x105 | 同一帧的**纯传输重发**（含 b0 全等 99.7-100%）；但各自 b7 是另一路信号（非 0x109 镜像） | 重发高/含义低 | 无跟车时 b7 均值 30.6 非零、单调仅 24.8%、11 distinct 集中 |
| 0x102/0x106 | 传感器健康 A/B 对（b4 慢量 ~152/~103） | 角色中/义 UNKNOWN | 与 lead/no-lead 无关 |
| vRel | **不存在**于这 6 帧 | 高 | 全体字节对 −db7/dt 监督无一 \|r\|>0.4（486 段） |

**重要推论**：0x109 帧存在 10 段「ACC 未跟车但 b7/存在位非零」的真实目标（HUD 路看不见的增量）
→ 下游窗过滤规则=**存在位 AND 档位**双条件，只看 HUD 会丢真实目标。

## 4. 与既有定案的修订关系

- **维持**：0x38x 状态表证伪、0x35x 槽恒空、0x374 零帧、"宋无完整 track（无 vRel）"。
- **缩小**：§8.5.7「bus1 0x101-0x10A 皆状态广播」→ 仅对 0x102/106/10A 状态字节成立，0x109=距离。
- **新增**：`RADAR_HUD_BARS_TO_M{2:13,3:22,4:35}` 不可作米标定真值——HUD 档位是**时距带**
  （类内 corr(b7,vEgo)=0.70-0.95），8.5.4 档位→米标定的前提本身要重做。
- **社区 ARS410OD dbc 定位**：地址段 0x20-0x70 与宋 bus1 不相交，不可直套；
  其 CheckSum/AliveCounter 惯例是本次 CRC 破译的钥匙，两段式对象结构支持 0x109+0x10A 同目标配帧假说。

## 5. 落码与产物（commit ec0d3d04e7，已推 origin/dev-tici）

- **dbc**：`byd_general_pt.dbc` +BO_ 257/258/261/262/265/266（RADAR_DIST_109 `LongDist_109:63|8@0+ (0.3048)` + HYPOTHESIS CM_、RADAR_PRESENCE_10A `Presence_10A:31|3@0+`、CheckSum/AliveCounter、其余 MSG 占位零编造）。
- **控制**：radar_interface.py / values.py / 双路 track+档位回退 **一字未动**（路测门禁纪律）。
- **测试**：`selfdrive/car/tests/test_byd_radar_010x_dbc.py` 10 例（真 CANParser 位序/邻字节/重发一致性/地址冲突）；byd 套件 38/38（host 须 `pytest --noconftest -o addopts=`）。
- **工具**：10 个挖掘/验证脚本入库 `tools/byd/`（见 §2 表）。
- **文档**：本报告 + `docs/byd-radar-010x.md`（定案草案+补录+风险台账）。
- **数据**：`/tmp/radar_1005_1005.json`、`/tmp/radar_415_diversity.{txt,json}`、`/tmp/up_*.json`、`/tmp/radar_unit_pin.json`。

## 6. 风险台账（10-18 晚更新）

| # | 风险 | 状态 |
|---|---|---|
| 1 | 单位 0.3048 vs 0.25 未绝对钉 | **开口**——13 起跟停事件把上界收到 u≤0.42-0.44（1.0/0.5/0.45 全排除），剩二选一；路测实测一次跟停驻停距分辨（驻停 15c：feet≈4.6m / 0.25≈3.7m） |
| 2 | 0/247 哨兵语义 | 部分销——0=无目标已证（无跟车段整段恒零）；247 超量程未证 |
| 3 | 多样性证据薄 | **销**——415 段+hour_logs_2 独立日复验完成 |
| 4 | 无 vRel | **维持**——track 原语义不可覆盖，档 3 路（差分 vRel）噪声大仅备选 |
| 5 | b0 CRC UNKNOWN | **销**——SAE J1850 CRC-8 已破并双组泛化 |
| 6 | 0x101 语义 | 开口——疑似第二目标槽，未建模 |
| 9 | HUD_BARS_TO_M 依赖路径 | 开口——档位=时距带的结论尚未回写 values.py 注释（纪律：路测前不动） |

## 7. 下一步

1. **跟停标定**（唯一硬开口）：10-05 归档的 13 起跟停已把候选缩到 **feet 0.3048 vs 0.25**；下次路测实测一次跟停驻停距即可定夺（15c 平台→4.6m=feet / 3.7m=0.25），顺带观察 247 超量程码。
2. **T8 落地**：BSD 发起前 clear 窗/starting abort 可升级用 0x109 连续量——**只做单调序与相对阈值**（比例于档位穿越提前量），米制阈值等①钉死后引入；过滤用存在位+档位双条件。
3. **设备探针**（T7 残余）：bus3/后方 SRR 是否可及仍待 `radar_bus_probe.py` 上机。
4. 0x101/0x105 的"另一路 b7"与 0x109 b6/b7 平行递减（段偏置 -6/-12）继续建模（粗细分解 or 双目标）。

## 8. 复现

```bash
# ①挖掘  ②跨段验证  ③回声  ④监督反解  ⑤钉定
.venv/bin/python tools/byd/radar_re_mine.py docs_site/hour_logs/radar_rlogs
.venv/bin/python tools/byd/radar_0109_validate.py docs_site/byd_logs_2026-10-05_07/realdata --workers 10
.venv/bin/python tools/byd/radar_415_diversity.py   # 见脚本头注释
.venv/bin/python tools/byd/radar_sup_mine.py extract && .venv/bin/python tools/byd/radar_sup_mine.py fit
.venv/bin/python tools/byd/radar_unit_pin.py docs_site/hour_logs_2 --pattern 'rlogs_flat/*.zst' --json /tmp/up_hl2.json
.venv/bin/python tools/byd/radar_unit_aggregate.py /tmp/up_hl2.json /tmp/up_hl.json /tmp/up_1005.json
.venv/bin/python tools/byd/radar_stop2lead.py docs_site/byd_logs_2026-10-05_07/realdata --workers 10   # 13 起跟停,候选缩至 feet vs 0.25
# 测试
.venv/bin/python -m pytest selfdrive/car/tests/test_byd_radar_010x_dbc.py selfdrive/car/tests/test_byd_radar_interface.py selfdrive/car/tests/test_byd_carstate.py -q --noconftest -o addopts=
```
