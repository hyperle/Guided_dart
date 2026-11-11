# 录像逐帧播放器（主机 Qt 工具）

移植来源：远程 `origin/k230_v1` 的 `tools/video_player`，提交
[`a739026`](https://github.com/hyperle/Guided_dart/tree/a7390262203f2565876aef1d2a8b628b2722d79f/tools/video_player)。
保留 Qt Widgets 的滑块、左右键、图像缩放、像素点击和检测叠加，按当前 v2
的紧凑 DBV、逐帧 PNG、旧版 PGM/PBM 连续流及逐帧文件、录像 CSV 和检测 CSV 重写数据关联。与 v1 一样为手动逐帧浏览，
不按时间自动播放。支持 Qt 5 / Qt 6，不依赖 ROS、SDK 或板端构建。
支持 H.264/H.265（HEVC）Annex B 裸码流，可直接打开 `.h264`、`.h265`、`.hevc`
文件，或读取录像目录中的码流，保留逐帧浏览、检测叠加和 PNG 导出。
支持直接读取 `recording.pngs` 单文件无损灰度 PNG 容器，按 PNG 边界和 CSV 偏移关联，
截断的最后一帧会提示并忽略；无需先提取成独立 PNG。

## 构建与打开

构建需要 CMake >= 3.28、支持 C++23 的编译器、Qt Widgets、zlib、pkg-config 和
FFmpeg 开发库（libavformat、libavcodec、libavutil、libswscale）；脚本使用 Ninja。
Debian/Ubuntu 对应开发包为 `qtbase5-dev`、`zlib1g-dev`、`pkg-config`、
`libavformat-dev`、`libavcodec-dev`、`libavutil-dev`、`libswscale-dev`。
显示窗口需要图形桌面，在仓库根目录运行：

```bash
# 自动增量构建并打开；也可从其他目录调用 .script/player 的绝对路径。
bash .script/player records

# records 下有多段 rec_NNNN 时，按编号顺序连续浏览所有段。
# 也可只打开一段板端原始会话目录。
bash .script/player records/rec_0001

# 默认显示全部支持格式；--dbv/--png/--h264/--h265 筛选格式，-r 看 PGM，-f 看 PBM。
bash .script/player -r records/rec_0001

# 拷贝位置不同可显式指定两个 CSV；包含空格的路径加引号。
bash .script/player records/rec_0001/img \
  records/rec_0001/logs/frames.csv records/detection.csv

# 只浏览图片，跳过自动发现的录像和检测 CSV。
bash .script/player records "" ""
```

也可以独立构建，不使用任何交叉编译预设：

```bash
cmake -S tools/video_player -B build/video_player -G Ninja
cmake --build build/video_player --parallel
./build/video_player/video_player --help
./build/video_player/video_player records
```

从旧版缓存迁移时，先执行 `cmake --fresh -S tools/video_player -B build/video_player -G Ninja`。
`DART_CXX_COMPILER` 或 `CXX` 可指定 C++ 编译器，默认使用系统编译器。
`DART_PLAYER_BUILD_DIR` 可覆盖脚本的默认构建目录 `build/video_player`。
脚本透传参数，数据相对路径以调用时工作目录为准。指定一轮录像的目录，支持
原始 `rec_NNNN`、其 `img` 目录，以及当前仓库 `records` 这样的拷贝布局：

```text
records/
  frames.csv
  recording.pgm                     # 默认灰度格式；也支持 DBV 与 PNG 分组目录
  detection.csv                       # 可选
```

图片直接位于输入目录时，优先读取该目录的 `frames.csv`，其次是 `logs/frames.csv`；
原始会话布局优先读取 `logs/frames.csv`，其次是会话根目录的 `frames.csv`。
检测 CSV 优先取会话/拷贝目录内的 `detection.csv`；原始 `rec_NNNN/img` 布局
还兼容会话上一级的 `detection.csv`。图片直接位于输入目录时不会查找父目录的检测日志。
显式给出的 CSV 路径优先使用，不存在时直接报错。

输入目录直接包含多个 `rec_NNNN` 子目录时，按数字编号依次载入全部录像段。
每段使用自己的 `logs/frames.csv`，检测日志按上述优先级分别解析；各段相同的图片编号
和文件名不会串联到别段的日志。滑块可跨段浏览，右侧文件路径显示所属段。
`--check records` 检查全部段，并输出总计和各段的检查结果。目录自身含图片或帧索引时，
优先按单段/拷贝布局读取。多段输入不接受指定一份 `frames.csv`；需要覆盖时指定单段目录。

建议一并拷贝以下内容，保持相对目录：

```text
records/
  detection.csv
  rec_0001/
    img/recording.dbv
    logs/frames.csv
```

**检测日志必须与录像对应。** 多次运行或暂停恢复可能复用同一个 CSV 路径，
而旧 `rec_NNNN` 目录仍会保留；文件名相同不能证明属于同一轮采集。
每轮结束后将对应的 `detection.csv` 连同录像目录一起归档，或启动时通过
`--det-csv` 指定不同路径。缺失的检测记录无法从图片恢复，播放器会显示未匹配。

## DBV 浏览与导出

默认板端输出 `img/recording.pgm`，显式选择 DBV 时输出 `img/recording.dbv`，停止录像后再通过 MTP 拷贝此文件和 CSV。
播放器自动解码为黑白像素，界面按钮可导出当前帧的 PNG（不含叠加标记）。
批量导出不需要桌面环境，按整个输入序列连续编号，遇到已存在的图片会拒绝覆盖：

```bash
bash .script/player --export-png /tmp/dart-export records/rec_0001
```

文件格式、无校验记录及差分依赖的限制见 [录像格式说明](../../include/firmware/src/recorder/README.md)。

## H.264 / H.265 浏览

目录入口自动识别图像目录顶层的 `.h264`、`.h265`、`.hevc` 文件，扩展名不区分大小写。
也可以指定单个码流文件，避免载入同目录中的其他录像；CSV 仍按所在目录或原始
`rec_NNNN/img` 布局自动发现，显式 CSV 参数和空字符串跳过参数保持不变。
`--h265` 同时选择 `.h265` 与 `.hevc`；各格式筛选选项不能同时使用。
此入口针对 Annex B 裸码流，不读取 MP4/MKV 容器。

```bash
bash .script/player records/rec_0001/img/recording.h264
bash .script/player --h265 records/rec_0001
bash .script/player /tmp/clip.hevc "" ""
bash .script/player --export-png /tmp/video-frames /tmp/clip.h265 "" ""
```

首次打开会扫描、解码码流，建立显示顺序与压缩包字节偏移的索引，不把全部像素帧
保存在内存中，也不生成中间图片。浏览时缓存当前图像和解码器所需的参考帧；支持
包含 B 帧的码流，倒退或跨关键帧跳转从独立关键帧重新解码。H.265 开放 GOP 中的
CRA 帧不当作完全独立的跳转起点，必要时从更早的 IDR/BLA 帧开始，长录像跳转可能较慢。
颜色按码流的色彩空间和范围转换为 RGB。损坏或空码流会报错，文件在载入后被修改时
需要重新打开。

录像 CSV 仍用 `image_file + file_offset` 精确关联，并校验 `file_bytes` 与显示顺序的
`image_index`；不会把 SPS/PPS/VPS 等参数头单独算成图像，也不会用相邻日志猜测关联。
没有 CSV 时可以浏览和导出，但无法恢复板端的采集身份或检测结果，`--check` 仍按
原有规则返回关联不完整。

## 图片、文字与叠加如何对应

1. 对 `img/recording.dbv` 扫描共享文件头及变长记录，建立偏移和差分依赖索引；
   不依赖 CSV 解码图片，也不额外保存索引文件。顺序浏览缓存上一帧；旧版重复帧记录仍可读取。
   随机/向后跳转从最近完整帧重建，长差分链的跳转耗时会增加。对 `img/recording.pgm` / `.pbm` 扫描帧头并建立字节偏移索引，跳过像素负载；
   复位导致的不完整尾帧会提示并忽略。独立 PNG 支持 `img/bNNNNNNN/pNNNNNNN/00000001.png`，兼容旧版 `img/frame_*.pgm` / `.pbm`；按图片数字编号排序。
   图像格式只读取和解码当前选中帧；视频格式的索引建立方式见上一节，均不缓存整个录像的像素。
2. 用 `frames.csv` 的 **`image_file + file_offset`** 精确匹配连续帧，校验 `file_bytes`、
   `decision=accepted` 和 `image_index`。独立图片用包含分组目录的完整 `image_file`；PNG 同时校验 `file_offset=0` 与 `file_bytes` 等于磁盘文件长度，冲突时停用该帧关联。
   拷贝布局中 CSV 仍可保留板端的 `img/b.../p.../编号.png`，无需修改日志或移动图片；
   注释里的板端 `session=/data/app/...` 不影响主机路径解析。
   `image_index` 是输出帧的编号，不是输入帧号或 CSV 行号。
3. 用录像行的 **`source_sequence + received_us + source_pts`** 精确匹配检测行的
   **`sequence + received_us + source_pts`**，并核对检测的 `admission=accepted`。
4. 右侧显示对应 CSV 的路径、物理行号、原始文本及全部列，便于回查原始记录。

降采样、忙丢弃等输入有 CSV 行但没有图，浏览时不会占据一张图片；图片缺号也不会
使后续文字错位。接收时间和 PTS 一起核对，避免只凭重启后复用的序号串到其他轮次。
重复匹配、放行冲突或没有对应文字时明确提示，停用该帧的检测关联；不按最近时间
或相邻行补齐。metadata 队列本身也可能丢行，所以有图不等于一定有录像 CSV 行。

缺少默认 CSV，或 CSV 为空文件、仅含空白/注释、只有有效表头时，仍可查看图片。
面板会提示没有数据行；没有录像行时不关联检测结果，空的检测 CSV 不画检测叠加。
显式传入空字符串可跳过对应 CSV，例如 `bash .script/player records "" ""` 只看图片。
损坏的 CSV、错误的表头
或非法身份字段会报告文件和行号。当前关闭识别/识别出错时写出的短空字段行会按
`kind=none, valid=0` 的格式兼容，保留原文并正确识别最后的 `admission`。
图片读取失败时清空图像区，并显示错误，防止旧图与新文字同屏。

绿色十字仅表示 `valid=1` 且坐标有限、位于图内的目标；支持小数质心。
青色框是检测时实际扫描窗口 `win_x0,win_y0,win_x1,win_y1`，右下边界为开区间。
无目标时也可能有扫描框。非法坐标不绘制。显示使用原始图片坐标，不修改源文件。
`dart_app.log` 的周期状态是汇总信息，没有每图身份，不作为单帧检测结果叠加。

拖动底部滑块或按 **← / →** 切帧，**Home / End** 到首尾；点击图片会在状态栏
和文字面板显示原图像素坐标，黑边点击忽略。右侧文字可选中复制，面板可拖动调宽。

## 无窗口检查与回归

```bash
./build/video_player/video_player --check records/rec_0001
./build/video_player/video_player --check records
ctest --test-dir build/video_player --output-on-failure
python3 tests/check_cpp_structure.py
```

`--check` 不要求桌面环境，检查文件列表和文字关联；图像格式不解码全部图片，
H.264/H.265 为建立帧索引会解码整个码流。
输出图片数、录像匹配数、检测匹配数和 CSV 有记录但磁盘缺图的数量；还会列出
未匹配/冲突的图片。成功返回 0，关联不完整或解析失败返回 1，命令参数错误返回 2。
若没有检测 CSV 或其中没有数据行，仅以录像关联判断完整性；不代表已经验证了检测结果。
录像 CSV 为空或被跳过时，图片可以正常浏览，但 `--check` 仍因缺少录像关联返回 1。
图片数量可能多于录像 CSV 的记录数，播放器会逐张提示缺少录像行；若末尾 PNG 为
0 字节，选中时清空图像区并提示读取失败，不会为这些图片补配日志。

`recording_types.cpp` 定义帧和 CSV 行数据，`csv.cpp` 负责 CSV 解析，
`recording_paths.cpp` 解析原始会话和拷贝布局的默认路径，`image_reader.cpp` 负责逐帧解码，
`video_stream.cpp` 管理 FFmpeg 码流索引、解码和跳转，`recording.cpp` 扫描文件并关联录像/检测记录；
`image_view.cpp` 管理显示和原图坐标映射，`player.cpp` 组合窗口与命令入口，
`main.cpp` 仅转交入口。组件按需要提供 `.hpp` 声明，由 `.cpp` 实现并链接为静态库；
不要求每个源文件建立对应头文件。播放器复用应用的 DBV 编解码实现。

验证包括独立构建、最终链接和 Qt offscreen 回归；硬件录像格式以真实样本验证。
`video_stream` 回归需要带 `libx264` / `libx265` 编码器的 `ffmpeg` 命令，临时生成
包含多 GOP、B 帧的码流，与 FFmpeg 顺序解码的像素逐帧对比，并验证倒退/随机跳转、
CSV 关联、格式筛选、无窗口导出及 GUI 切帧。正常运行播放器只需要 FFmpeg 库，不调用
`ffmpeg` 命令；只构建播放器时可配置 `-DBUILD_TESTING=OFF`。
