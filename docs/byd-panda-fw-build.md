# BYD 宋 PLUS DM-i：panda 自建固件构建记录

> 时间：2026-09-26 · 配套文档：`docs/byd-panda-recovery.md`（排障过程）
> 本文记录自建固件的构建环境、源码基线选择、构建过程与当前状态。

## 一、为什么必须自建固件

| 固件 | 结论 |
|---|---|
| 官方 v0.10.1 固件 | ❌ 在此兼容板上 `assert_fatal(hw_type != UNKNOWN)` 故意挂起（板型 GPIO 不匹配任何官方型号）|
| 厂商固件（DEV-3a515542）| ✅ 能启动、CAN RX 正常；**但不含我们 openpilot 侧需要的 byd 安全模式 @35**（其 byd 为私有协议 @29）|
| **自建固件** | ✅ 唯一能同时满足"能启动 + 含 byd@35"的方案 |

构建源码基线的选择经过（重要）：

1. **0.9.x 时代基线**（commaai/panda `7287ff0c`，2024-06-12）——已验证可启动 + CAN RX 正常，但 SPI 协议是旧版，与 0.10.1 主机栈握手失败（`SPI: timed out waiting for ACK, waiting for 0x1f`），C++ pandad 连不上 → UI"无 PANDA"
2. **0.10.1 时代基线**（3dc2138，2025-08-19，F4 支持被删除前的最后提交）——SPI 协议匹配主机栈；该时代的 F4 板型检测只认 DOS，其余板型 `assert_fatal` 挂起，因此**必须强制 DOS 板型**（待实测：判断此兼容板的 panda 到底是 SPI 还是 USB 接线）

## 二、构建环境（Mac ARM64）

### 1. 交叉编译工具链

brew cask 安装会在后台卡死（需要 sudo 交互），**用官方 tarball 直下 + 断点续传**：

```bash
# Arm 官方服务器限速（~45KB/s），用循环续传直到完成
URL="https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
cd /tmp
until curl -sL -C - --retry 5 --retry-all-errors -o arm-gcc.tar.xz "$URL"; do sleep 3; done
mkdir -p /tmp/arm-tc && tar -xJf arm-gcc.tar.xz -C /tmp/arm-tc --strip-components=1
export PATH="/tmp/arm-tc/bin:$PATH"
```

总大小 129,557,320 字节 ≈ 129.6MB。GCC 13.2 编译 2024/2025 年的 panda 代码无 `-Werror` 问题。

### 2. 构建工具

```bash
uv tool install scons --force --with pycryptodome
# pycryptodome 是签名步骤（crypto/sign.py）必需
```

**三个坑**（都会导致构建中断）：
- `crypto/sign.py` 由 SConscript 以独立子进程调用，shebang 是系统 python3（无 Crypto）。
  修法：`SConscript:126` 的命令行前缀加 `{sys.executable}`（文件头部需 `import sys`）
- uv tool 重装后 site-packages 会重置，`--with pycryptodome` 必须与 `--force` 同用
- sign 步骤报 `ModuleNotFoundError: No module named 'Crypto'` 时优先查这里

### 3. 源码树

```bash
git clone https://github.com/commaai/panda /tmp/panda_fw_base
cd /tmp/panda_fw_base && git checkout 7287ff0c   # 0.9.x 基线（或 3dc2138 = 0.10.1 基线）
```

部署树（/data/openpilot）里的 panda 目录**没有板级源码**（只有 obj 产物），必须从上游拉。

## 三、byd 安全模式的移植

### 0.9.x 基线（旧式平铺结构）——已完成并验证

- 新建 `board/safety/safety_byd.h`：从主仓库 `opendbc/safety/modes/byd.h` 移植，
  适配老结构差异：
  - 统一 `SteeringLimits`（扭矩/角度共用，含 `max_steer`/`max_rt_interval` 字段）
  - 无 `current_safety_param` 全局 → 用 `static bool byd_angle_steering` +
    `byd_init()` 里 `GET_FLAG(param, ...)` 赋值（老结构惯例，参考 nissan）
  - `CanMsg` 无 `check_relay` 字段；`RxCheck` 用 `.frequency` 命名初始化
- 注册三处：`board/safety.h` 的 define 区（`SAFETY_BYD 35U`）、include、registry
- **AccState 编码修正**：Song 是 `1 = ACC_ACTIVE`（Han 是 3），此修正必须同步到
  固件版 safety_byd.h（曾因漏改导致 controls_allowed 永不置位）

### 0.10.1 基线（新结构）——直接复用主仓库文件

`opendbc/safety/modes/byd.h` 本身就是新结构，SConscript 的 `CPPPATH` 含
`opendbc.INCLUDE_PATH`，理论上直接拷入 `board/safety/` 即可；唯一要改的是
板型检测强制（见下）。

### 板型强制（两个基线都需要）

此兼容板的 GPIO 检测不匹配任何官方型号 → 官方代码 `assert_fatal(... UNKNOWN)`
挂起。固件层强制：

```c
// board/stm32f4/board.h 的 detect_board_type() 末尾（0.9.x 基线）：
hw_type = HW_TYPE_UNO;           // 0.9.x：UNO = USB 传输
current_board = &board_uno;

// 0.10.x 基线（board/stm32f4/board.h）：
hw_type = HW_TYPE_DOS;           // 0.10.x：DOS = SPI 传输（tici 内置）
current_board = &board_dos;
```

板型决定主机传输方式：UNO → USB，DOS/TRES/CUATRO → SPI。**选错传输方式 =
pandad `SPI: timed out` 死循环 / USB 无法握手**。此兼容板的实际接线方式
（SPI or USB）尚未实测确认，是当前待验证项。

## 四、构建命令

```bash
export PATH="/tmp/arm-tc/bin:$HOME/.local/bin:$PATH"
cd /tmp/panda_fw_base            # 或 /tmp/panda_fw_010
scons -j8 board/obj/panda.bin.signed
# 产物：board/obj/panda.bin.signed（DEV 签名，bootstub + app 签名链自洽）
# board/obj/version 应显示 DEV-<commit>-DEBUG
```

只构建主固件可跳过 libpanda 测试组件（其依赖 `gcc-13` 裸命令，未装会 Error 127）。

## 五、刷写流程（设备端）

```bash
# 1. 停 openpilot
sudo systemctl stop comma
# 2. （可选）跳刷开关：签名自洽后不需要；过渡期用于防止旧包覆盖
touch /data/panda_skip_flash    # 删除: rm /data/panda_skip_flash
# 3. 强制 DFU
python3 -c "from openpilot.system.hardware import HARDWARE; \
  HARDWARE.recover_internal_panda()"   # 需 capnp（/usr/local/venv/bin/python）
lsusb                            # 应见 0483:df11
# 4. DFU 刷 bootstub
sudo /usr/local/venv/bin/python -c "
from panda.python.dfu import PandaDFU
s = PandaDFU.list(); PandaDFU(s[0]).recover()"
# 5. ★ BOOT0 复位（关键！DFU 强制会把 GPIO134/BOOT0 拉高，不复位则永远回 DFU）
sudo sh -c 'echo 0 > /sys/class/gpio/gpio134/value'
# 6. 刷主固件（经 bootstub）
sudo /usr/local/venv/bin/python -c "
from panda import Panda
p = Panda(); assert p.bootstub
p.flash(fn='/tmp/panda.bin.signed')"
# 7. 验证
lsusb                            # bbaa:ddcc = 我们的固件运行中
```

**Python 环境**：`/usr/local/venv/bin/python`（有 usb1/capnp）；SSH 非交互 shell
的 `python3` 没有 usb1，直接跑 `from panda import Panda` 会 ModuleNotFoundError。

## 六、当前状态与待验证

| 项 | 状态 |
|---|---|
| 0.9.x 基线固件（UNO 强制 + byd）| ✅ 构建、刷写、启动、CAN RX（24450 帧）全部验证 |
| 0.9.x 固件 + 0.10.1 主机栈 | ❌ SPI 协议不匹配 → UI"无 PANDA" |
| 0.10.1 基线固件（DOS 强制 + byd 移植）| ⏳ 源码就绪（/tmp/panda_fw_010 3dc2138 + 强制 DOS 补丁已打好），构建命令同上，**待构建刷写实测**——此实验同时回答"兼容板的 panda 是 SPI 还是 USB 接线" |
| 指纹自动匹配 | 已恢复（实车验证 ✓）；固件稳定前曾临时禁用 |

## 七、签名链说明

- DEV 构建：`certs/debug` 密钥签名，配套 dev bootstub（同一次构建产出），
  bootstub 校验 app 签名通过 → 签名链自洽
- `panda.bin.signed` 提交进仓库后，pandad 的期望签名（读自同文件）与运行固件
  一致 → 不再需要跳刷标志，OTA 固件更新通道正常
- 官方 bootstub（v0.10.1）与 DEV 签名 app 的兼容性未验证；刷写时始终
  bootstub + app 成对刷
