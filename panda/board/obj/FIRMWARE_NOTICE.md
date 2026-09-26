# panda 固件说明（本目录二进制来源）

当前状态：**本目录固件 = 自建固件（含 BYD byd 安全模式 @35）**，与本仓库
`opendbc/safety/modes/byd.h` 同源构建（0.9.x 时代上游 panda 基线 7287ff0c，
UNO 板型，DEV 签名）。

| 文件 | 说明 |
|---|---|
| `panda.bin.signed` | 主固件（DEV 签名，含 byd@35 + UNO 板型强制 + USB VID 0x3801）|
| `bootstub.panda.bin` | 配套 dev bootstub（同 VID，接受 DEV 签名 app）|
| `panda_h7.*` | H7 构建（tici 未使用）|

**USB VID 0x3801（2026-09-26 起）**：0.10.1 的 C++ pandad 数据路径只匹配
comma 注册 VID（`idVendor == 0x3801`），旧 VID 0xbbaa 的固件会被无视并落入
SPI 兜底死循环（`SPI: timed out waiting for ACK`）→ UI"无 PANDA"。python 库
两个 VID 都收，故此前 python 层测试全通、唯独 UI 无 panda。详见
`docs/byd-panda-fw-build.md` 第六节。

**0x32D 总线修正（2026-09-27，app 哈希 4be260bc）**：`safety_byd.h` 的
`pcm_cruise_check` 与 0x32D RX 检查从 bus 0 改到 bus 2（Song Plus DM-i 的
DiPilot 相机在相机侧总线上发送 0x32D，bus 0 上永远收不到）。bus 0 上的旧检查
超时导致 pcm_cruise_check 永远认为原厂 ACC 未激活 → pcmCruise 模式下
controls_allowed 被强制清除，OP 无法保持发动。激活判定同步改为
`AccState != 0 && != 7`（7 = 主开关开/待机）。

## 刷机恢复

刷机/重装系统后：安装本 fork 分支（custom URL）→ OTA → pandad 签名校验
自洽（运行固件 = 仓库固件）→ **无需任何手动操作**。panda 芯片固件不受
系统刷机影响；若 panda 意外进 DFU/bootstub，pandad 自动刷回本目录固件。

## 历史备注

- 官方 v0.10.1 固件在此兼容板（hw_type 检测 UNKNOWN）会按安全设计挂起，
  不可用；厂商固件（DEV-3a515542）可启动但 byd 模式为私有协议（@29）。
- 因此固件必须自建：UNO 板型强制（board/stm32f4/board.h）+ byd.h 移植
  （0.9.x safety 结构）+ USB VID 0x3801 + 签名链自洽。
- 0.10.1 基线（3dc2138 + 强制 DOS）构建通过但 app 在此板不枚举 USB，弃用，
  崩溃根因未查（见 fw-build 文档第六节）。
- `/data/panda_skip_flash` 标志仅在固件-仓库不一致的过渡期需要；签名
  自洽后可删除（删除后 OTA 固件更新恢复正常）。
