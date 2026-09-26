# panda 固件说明（本目录二进制来源）

当前状态：**本目录固件 = 自建固件（含 BYD byd 安全模式 @35）**，与本仓库
`opendbc/safety/modes/byd.h` 同源构建（0.9.x 时代上游 panda 基线 7287ff0c，
UNO 板型，DEV 签名）。

| 文件 | 说明 |
|---|---|
| `panda.bin.signed` | 主固件（DEV 签名，含 byd@35 + UNO 板型强制）|
| `bootstub.panda.bin` | 配套 dev bootstub（接受 DEV 签名 app）|
| `panda_h7.*` | H7 构建（tici 未使用）|

## 刷机恢复

刷机/重装系统后：安装本 fork 分支（custom URL）→ OTA → pandad 签名校验
自洽（运行固件 = 仓库固件）→ **无需任何手动操作**。panda 芯片固件不受
系统刷机影响；若 panda 意外进 DFU/bootstub，pandad 自动刷回本目录固件。

## 历史备注

- 官方 v0.10.1 固件在此兼容板（hw_type 检测 UNKNOWN）会按安全设计挂起，
  不可用；厂商固件（DEV-3a515542）可启动但 byd 模式为私有协议（@29）。
- 因此固件必须自建：UNO 板型强制（board/stm32f4/board.h）+ byd.h 移植
  （0.9.x safety 结构）+ 签名链自洽。
- `/data/panda_skip_flash` 标志仅在固件-仓库不一致的过渡期需要；签名
  自洽后可删除（删除后 OTA 固件更新恢复正常）。
