# PBM 逐帧播放器

这是一个轻量的 Qt Widgets 主机工具，用于查看 `records/img` 中的 PBM，并将
`records/logs/frames.csv` 中与图片文件名末六位数字对应的数据行显示在右侧（表头后的第一条记录算第 1 行）。它不自动播放，只在滑块、
左右键或鼠标点击时切换帧。

```sh
cmake -S tools/video_player -B build/video_player
cmake --build build/video_player -j
./build/video_player/video_player
```

仓库提供了可自动构建并启动播放器的入口（从任意工作目录调用均可）：

```sh
./scripts/player -f    # 播放 f*.pbm（默认模式）
./scripts/player -r    # 播放 raw*.pgm
```

也可以直接指定目录：

```sh
./build/video_player/video_player records/img records/logs/frames.csv
```

点击图像会在状态栏和右侧显示 PBM 像素坐标。CSV 的 `cx/cy` 用绿色十字标注；
`roi` 支持 `x,y,w,h`、`x1,y1,x2,y2` 及 `x:y:w:h` 等矩形格式。当前仓库样本中的
`roi=1` 是状态标志而不是矩形，因此只显示在日志面板中，不会虚构 ROI 框。
