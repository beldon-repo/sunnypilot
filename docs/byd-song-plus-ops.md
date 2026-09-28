# 宋 PLUS DM-i 移植:部署与运维手册

> 面向新 session 的接手文档。调试全史见 `byd-song-plus-lkas-debug.md`(13 个根因),本文只放"现在怎么干活"。

## 一、当前状态(2026-09-29)

- **里程碑已达成:成功控车**。最长连续控车 ~14 min(高速 66-72 km/h),累计 ~40 min
- 设备代码:`45997ca`(根因 13 版)。**本地已到根因 14 + 09-29 review 修复 + 厂商对齐 Phase 1(让位曲线+NN 前馈,`ec28881e7b`),设备待同步**
- **⚠️ GitHub origin 落后本地提交**(review 修复 `ea93962414` + 厂商对齐 `ec28881e7b` + 文档等)。下次网络正常时先 `git push origin main-c3l-tici`,设备用路径 1/2 同步
- 已知遗留:①行驶中 EPS 偶发 TorqueFailed 锁存(根因 14 守卫+同向跟随已部署,**未路测验证**);②车速信号 bus0 ESP_SPEED 比 GPS 低 ~10%(稳态比 0.966);③libsafety.so 与 byd.h 漂移致 24 个测试失败(测试框架问题)
- **新路线:厂商方案对齐**(2026-09-29 确立)——op_byd 已运行时解密,结论/设计/待办全在 **`byd-vendor-alignment.md`**,纵向(Phase 2)设计已备好未实施

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
5. 激活后**手完全离开**(保证观察基线。根因 14 起 |drv|>15 持续对向不再归零让位,而是请求翻转**同向 ±20 跟随**、|drv|<15 恢复车道控制——轻握方向盘 = 助力被压到 ±20 且方向跟手,这是预期行为不是故障)
6. 避免在泊车/掉头大转角中激活(>40° 退让、>50° 后锁 10s 是保护)
7. 出 LKAS Fault → 熄火 2 分钟 → 再测;反复出现 → 记时间点拉日志

## 四、EPS 交互规则终版(13 根因浓缩)

三层验收,任一不过即拒绝(静默拒绝或 TorqueFailed 锁存,锁存断电清):

1. **帧合法性**:echo 相机 SETME 字段、byd_checksum、50Hz 连续流、Counter 对齐
2. **会话合法性**:`LKAS_Config=3`+`State=1`+`Active=1`+`Lane 2/2` 才出力(会话模式是钥匙);**armed-零容忍实测 ~0.5s**(SteerWarning ~0.2s 先亮,根因 14),守卫 0.16s(8 命令@50Hz)退场回 idle echo;armed 只在真实需求(>6 单位)时进入;握手 ReqPrepare→Prepared→Active
3. **行为合理性**:不发对向帧(对向需求持续 ~0.06s → 请求翻转**同向 ±20 跟随**;探测用截幅前 demand,重对抗 |drv|>135 不盲);armed 中刹车/角度门(>40°)/静止**切断出力**斜坡退场;>50° 后锁 10s;>180°/s 扫摆退让;重武装需停稳(<40°/s 持续 0.3s)

标定(全部锚定厂商实跑):STEER_MAX=200、steerRatio=19.5、mass=1926、factor=2.5、friction=0.10、delay=0.3。MainTorque(0x318)=请求回显,不是助力反馈。

## 五、待办(优先级序)

1. **推送 + 设备同步**(`ea93962414` + `ec28881e7b` 均未推送;同步后路测前确认 `NeuralNetworkLateralControl` 参数——NN 前馈默认关,先不开做基线,再开做 A/B)
2. **路测验证**:①根因 14 + review 修复(静默守卫+同向跟随;若仍锁存,拉日志看锁存前 |req| 是否=0 超 0.16s、或对向帧是否存在)②厂商让位曲线手感(弯道助力连续性、对抗让位平滑度);顺带确认静默退场后 re-arm 是直接还是走 3 帧 prepare 突发
3. **Phase 2 纵向**(路测干净后启动):设计已备好,见 `byd-vendor-alignment.md` 第四节
4. **手感调参**(控车稳定后):THRESHOLD 80→60-70(三源参考 56/59/60)、LatControl 手感、速率限制器(参考 yysnet 132→64°/s)
5. **车速 10% 偏差**:bus0 ESP_SPEED vs GPS 稳态比 0.966,标定阶段修正
6. **固件速率限**:当前 10/12,厂商 17/17——若修正速度不够再改(需固件重编译+刷写)
7. libsafety.so 与 byd.h 漂移(24 测试失败),单独处理
8. echo 空闲帧对 ACC 按钮长期影响观察

## 六、参考资产

- `docs/byd-vendor-alignment.md`:**厂商对齐专项**(解密产物索引/已定论事实/Phase 1 设计/Phase 2 纵向完整设计/待办)——新 session 先读这个
- `docs/byd-song-plus-lkas-debug.md`:13 根因全史 + 6.1-6.23 逐轮分析
- `docs_site/op_byd_logs/`:厂商工作日志(字节模板出处)+ README + vendor_diff.py
- `docs_site/op_byd_data/values.json`:厂商运行时参数 dump(NN 权重出处);解密产物在本机 `/Users/wujiafu/Documents/op/cp_byd/docs/pyarmor_decrypted/`
- 参考库已挖尽:op_byd(厂商,已运行时解密+参数 dump)、yysnet/opendbc、opendbc_repo.byd、cankao 系列、高阶Python源码_v2(其 DELTA 17/17 与厂商固件互证)
- 分析脚本:/tmp 会丢,常用工具已收录 docs_site/op_byd_logs/vendor_diff.py
