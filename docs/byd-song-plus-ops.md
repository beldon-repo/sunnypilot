# 宋 PLUS DM-i 移植:部署与运维手册

> 面向新 session 的接手文档。调试全史见 `byd-song-plus-lkas-debug.md`(13 个根因),本文只放"现在怎么干活"。

## 一、当前状态(2026-09-28 深夜)

- **里程碑已达成:成功控车**。最长连续控车 ~14 min(高速 66-72 km/h),累计 ~40 min
- 设备代码:`45997ca`(含全部 13 根因修复 + 标定)
- **⚠️ GitHub origin 落后 2 个提交**(478cd93 静默守卫 + 45997ca 文档,笔记本代理故障未推上)——设备代码已通过 LAN bundle 同步,是新的。下次网络正常时先 `git push origin main-c3l-tici`
- 已知遗留:①行驶中 EPS 偶发 TorqueFailed 锁存(根因 13 的静默守卫已部署,**未路测验证**);②车速信号 bus0 ESP_SPEED 比 GPS 低 ~10%(稳态比 0.966);③libsafety.so 与 byd.h 漂移致 24 个测试失败(测试框架问题)

## 二、代码部署三路径(按优先级)

### 路径 1:设备直接拉取(首选,网络通时)
```bash
ssh comma@<设备IP> 'cd /data/openpilot && git fetch origin main-c3l-tici && git reset --hard FETCH_HEAD && git log --oneline -1'
```
- 等价于 OTA 做的事,不影响后续 OTA(updater 每次强制对齐 origin)
- **注意**:设备 git over https 曾报 TLS 证书错误——重试或换路径 3
- pull 后**必须重启进程**:sudo reboot(或熄火循环,若设备由车供电)

### 路径 2:笔记本 bundle 局域网直推(GitHub 不可达时,已验证多次)
```bash
# 笔记本:打增量包(基点 = 设备当前 commit)
git bundle create /tmp/b.bundle <设备当前commit>..main-c3l-tici
scp -i <key> /tmp/b.bundle comma@<设备IP>:/data/b.bundle
# 设备:
ssh comma@<设备IP> 'cd /data/openpilot && git fetch /data/b.bundle main-c3l-tici && git reset --hard FETCH_HEAD && rm /data/b.bundle && sudo reboot'
```
- 保持 git 树 = origin 内容,不违反"不改设备代码"原则
- 行驶中不要 reset(延迟 import 会新旧混跑);停车/熄火时做

### 路径 3:OTA(自动)
- push 后 updater 自动拉取,开机套用。慢/断网时用路径 1/2
- 设备 GitRemote=github.com/beldon-repo/sunnypilot,分支 main-c3l-tici

### 设备访问坑
- 刷机后 SSH host key 会变:`ssh-keygen -R <IP>` 再连
- 设备在手机热点(172.20.10.x)与家用网(192.168.1.12)间切换,IP 不固定
- 热点下大文件传输慢:用 `tar cf -` 流式一次拉多个文件,勿逐个 scp
- 本地文件名含 `-` 会让 logreader 误判为路由名,存日志用 `r{route}_{seg}.zst` 格式

## 三、路测规程(固化,照做)

1. **熄火 ≥2 分钟**清 EPS TorqueFailed 锁存(锁存只能断电清)
2. 点火后**等 15 秒**再按 SET(避开 selfdrived 就绪窗口)
3. 若 enable 被挡(cruiseMismatch,静默):取消 ACC → 等 3 秒 → 重新 SET
4. **LKA 开关保持打开**(armed 上下文是出力前提之一)
5. 激活后**手完全离开**(让位逻辑不与手对抗,|drv|>68 即让位)
6. 避免在泊车/掉头大转角中激活(>40° 退让、>50° 后锁 10s 是保护)
7. 出 LKAS Fault → 熄火 2 分钟 → 再测;反复出现 → 记时间点拉日志

## 四、EPS 交互规则终版(13 根因浓缩)

三层验收,任一不过即拒绝(静默拒绝或 TorqueFailed 锁存,锁存断电清):

1. **帧合法性**:echo 相机 SETME 字段、byd_checksum、50Hz 连续流、Counter 对齐
2. **会话合法性**:`LKAS_Config=3`+`State=1`+`Active=1`+`Lane 2/2` 才出力(会话模式是钥匙);**armed 会话零扭矩静默 >2s = 锁存**(根因 13,静默守卫已部署未路测);握手 ReqPrepare→Prepared→Active
3. **行为合理性**:不许对抗驾驶员(|drv|>68 完全让位);|转角|>40° 退让、>50° 后锁 10s;>180°/s 扫摆退让;重武装需停稳(<40°/s 持续 0.3s)

标定(全部锚定厂商实跑):STEER_MAX=200、steerRatio=19.5、mass=1926、factor=2.5、friction=0.10、delay=0.3。MainTorque(0x318)=请求回显,不是助力反馈。

## 五、待办(优先级序)

1. **路测验证静默守卫**:上一轮 4 次中途锁存均因 armed 静默,守卫已部署未验证。若仍锁存,拉日志看锁存前 |req| 是否=0 超 2s(守卫应已先退场)
2. **手感调参**(控车稳定后):THRESHOLD 80→60-70(三源参考 56/59/60)、LatControl 手感、速率限制器(参考 yysnet 132→64°/s)
3. **车速 10% 偏差**:bus0 ESP_SPEED vs GPS 稳态比 0.966,标定阶段修正
4. **固件速率限**:当前 10/12,厂商 17/17——若修正速度不够再改(需固件重编译+刷写)
5. libsafety.so 与 byd.h 漂移(24 测试失败),单独处理
6. echo 空闲帧对 ACC 按钮长期影响观察

## 六、参考资产

- `docs/byd-song-plus-lkas-debug.md`:13 根因全史 + 6.1-6.23 逐轮分析
- `docs_site/op_byd_logs/`:厂商工作日志(字节模板出处)+ README + vendor_diff.py
- 参考库已挖尽:op_byd(厂商,Pyarmor 加密但日志/参数已榨干)、yysnet/opendbc、opendbc_repo.byd、cankao 系列、高阶Python源码_v2(其 DELTA 17/17 与厂商固件互证)
- 分析脚本:/tmp 会丢,常用工具已收录 docs_site/op_byd_logs/vendor_diff.py
