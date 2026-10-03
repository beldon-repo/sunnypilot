# UI 中文（zh-CHS）改用 Noto 矢量字体

> 时间：2026-10-04 · 分支 `sync-20251218-tici` · 上游参考：commaai/openpilot `66b3590b63` (#36514 "Use Noto fonts for Asian languages")

## 一、现象与根因

Trips / Vehicle / Firehose 等设置页中文"粗糙"：

- 本分支为新版 Python/raylib UI。`selfdrive/assets/fonts/process.py` 构建时把 **unifont 按 16px 点阵烘焙**成 `unifont.fnt` atlas（上游 `47d0a95fd6` 特意去掉抗锯齿）。
- 中文语言下 `system/ui/lib/application.py::font_fallback()` 把全部文本切到 unifont，UI 以 60~74px 绘制 = 16px 位图放大 ~4.4 倍 → 锯齿块。
- `main-c3l-tici(_trans)` 上观感较好是因为 `12bb0f246f` 把 unifont 以 200px 烘焙再缩小绘制，点阵被"平均化"；本质仍是位图，不是矢量。

## 二、方案（backport #36514，适配本分支烘焙管线）

中文改走 **Noto Sans SC 矢量字体**，运行时按当前语言的翻译用字子集光栅化（`load_font_ex`，96px + mipmap + 三线性过滤），不再经过 16px unifont。其余脚本不变：

| 语言 | 渲染 |
|---|---|
| zh-CHS / zh-CHT | **NotoSansSC-Regular.ttf**（新；tc 共用 sc，码位全覆盖） |
| ar / th / ko / ja | unifont 16px 点阵（维持原状） |

字体文件用 **TrueType(glyf) 轮廓**（Google Fonts 可变字体，默认实例=Regular 400），而非上游的 CFF `.otf`——本分支 pyproject 钉 `raylib<5.5.0.3`，旧 stb_truetype 对 CFF 支持不可靠；glyf 在所有版本可读。

## 三、改动清单

- `system/ui/lib/multilang.py`：新增 `NOTO_LANGUAGES = ["zh-CHT","zh-CHS"]` + `requires_noto()`。
- `system/ui/lib/application.py`：
  - `NOTO_FONTS` / `NOTO_FONT_SIZE = 96`（可按需调大，zh 子集仅 ~540 字形，纹理开销小）。
  - `gui_app.noto_font()`：懒加载，字符集 = ASCII + `EXTRA_FONT_CHARS` + `app_{lang}.po` 全部字符；加载失败（glyphCount==0）cloudlog 报错并回退 unifont，不会白屏。
  - `font_fallback()`：zh 优先 Noto；**显式传入 UNIFONT 的字体保持不变**（语言选择对话框里"日本語/한국어/العربية"等原生名仍用 unifont 全字符集）。
  - `_load_fonts()` 预载当前语言 Noto，避免首帧卡顿；`close()` 释放。
- `selfdrive/assets/fonts/process.py` + `selfdrive/ui/SConscript`：烘焙 glob 跳过 `NotoSans*`（运行时加载，不烘 200px atlas）。
- `selfdrive/assets/fonts/NotoSansSC-Regular.ttf`：新增，17.8MB，LFS 指针入库。

unifont 的 16px 烘焙**保留不动**：`ar` 及语言对话框原生名仍依赖它。

## 四、本地验证（macOS，pyray raylib 6.0）

离屏 render texture 同串对照截图：`转向扭矩响应校准完成。` @70px

- unifont@16 烘焙 → 笔画像素块粘连；
- Noto@96 烘焙 → 矢量平滑曲线，30px 小字亦清晰。

加载成本：538/540 字形命中，`load_font_ex` ~0.02s（桌面）；设备 aarch64 预计 <0.5s，且仅 UI 启动时一次。

## 五、设备部署

1. 代码同步：`git pull`（本分支）。
2. 字体实体文件（二选一）：
   - `git lfs pull --include "selfdrive/assets/fonts/NotoSansSC-Regular.ttf"`（设备需装有 git-lfs，且 origin 为含该 LFS 对象的 fork）；
   - 或本机 `scp selfdrive/assets/fonts/NotoSansSC-Regular.ttf root@<设备IP>:/data/openpilot/selfdrive/assets/fonts/`。
3. 重跑 `scons -u .`（SConscript 已排除 Noto 文件，不会尝试烘焙）后重启 UI。
4. 若字体缺失/加载失败：日志出现 `Failed to load Noto fallback font`，界面自动回退到旧 unifont 渲染，不影响使用。

## 六、已知边界

- zh-CHT 使用简体字形（个别地区字形如 骨/內 笔画形态差异），如需严谨后续加 `NotoSansTC-Regular.ttf` 并在 `NOTO_FONTS` 映射即可。
- 混排中英全走 Noto 拉丁字形（与原上游行为一致），与 Inter 风格略有差异。
- 日/韩/泰仍是 16px 点阵；如需一并提升，补对应 `NotoSansCJKjp/kr` + `NotoSansThai` 进 `NOTO_FONTS` 即可（纹理上限注意 ko 字符集大，建议 48~64px）。
- `NOTO_FONT_SIZE=96`：UI 最大字号 ~74（60×FONT_SCALE 1.24），96 烘焙为缩小绘制，锐利；若设备纹理内存吃紧可降回上游的 48。
