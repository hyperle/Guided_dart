# 业务代码存放目录（核心！）

本仓库**只负责业务代码**（跑在板端 RT-Smart 上的用户态应用），与固件无关。

## 业务代码放哪里？

```
apps/
├── <应用名A>/          ← 一个目录 = 一个板端可执行程序
│   ├── CMakeLists.txt  ← 定义 APP_NAME 与源文件（从 app_template 复制修改）
│   ├── main.c          ← 入口，业务代码写在这里（可加任意 .c/.cpp）
│   └── ...
└── <应用名B>/          ← 可以有多个应用，互不影响
```

## 约定

- 每个 `apps/<应用名>/` 目录编译为一个独立的板端 ELF 可执行文件。
- 目录结构刻意与官方 SDK 的 `src/applications/<应用名>/` 保持一致：
  需要时可将整个目录拷贝进 SDK（`src/applications/` 下），即可用官方
  `make` 流程打包进固件镜像（`/sdcard/app/<应用名>`）。本仓库则用
  CMake 独立编译，产物直接拷贝到板子运行，两边产物等价。
- 应用名建议：小写字母 + 下划线，如 `my_app`、`face_detector`。

## 新增一个业务应用（三步）

```bash
cp -r apps/app_template apps/my_app     # 1. 复制骨架
# 2. 编辑 apps/my_app/CMakeLists.txt 把 APP_NAME 改为 my_app
# 3. 在 apps/my_app/ 下写业务代码
bash scripts/build.sh                   # 编译
bash scripts/deploy.sh my_app <板子IP>  # 部署到板子
```
