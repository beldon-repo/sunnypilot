# panda 固件说明（本目录二进制来源）

| 文件 | 来源 | 说明 |
|---|---|---|
| `panda.bin.signed` | 第三方 BYD 定制固件（cp_byd 仓库，`DEV-3a515542` 构建）| 本设备（tici 兼容板，hw_type 上报 UNO/0x05）验证可启动、CAN RX 正常；官方 v0.10.1 固件在此板无法启动。**不含本仓库的 byd 安全模式**，设备以未识别 + 静默监听状态运行 |
| `bootstub.panda.bin` | sunnypilot v0.10.1 官方 | 已验证可与上述固件配合工作（DFU 恢复刷入 + app 刷写均经它完成）|
| `panda_h7.bin.signed` 等 | sunnypilot v0.10.1 官方 | tici 未使用 |

- pandad 的期望签名直接读本目录 `panda.bin.signed`，替换后自洽，无重刷循环
- 跳刷开关：`/data/panda_skip_flash`（存在即跳过刷写与签名校验）
- 自建固件（含 `opendbc/safety/modes/byd.h` 的 SafetyModel.byd@35）就绪后替换
  `panda.bin.signed`，并恢复 `opendbc/car/byd/fingerprints.py` 中的指纹匹配
- 原始 sunnypilot 固件见 git 历史（本文件替换前的版本）
