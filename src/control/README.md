# 四路舵机坐标映射

`servo_mapping.cpp` 是 L4 纯 C++23 映射，构建目标为 `dart_control`。
文件声明命名模块 `dart.control.servo_mapping`，独立编译并链接为静态库；不包含 SDK，不操作 PWM 设备，也不分配堆内存。

四个数组元素始终按已有 PWM 测试的 CH1～CH4 排列：

| 数组下标 | 通道 | PWM / 引脚 |
|---|---|---|
| 0 | CH1 | PWM1 / IO61 |
| 1 | CH2 | PWM2 / IO7 |
| 2 | CH3 | PWM3 / IO8 |
| 3 | CH4 | PWM5 / IO59 |

每路 `initial_pulse_us` 独立决定初始角度，以实际脉宽表示，单位 µs。
不假定脉宽与角度之间已经有标定关系。默认初始值 1520 µs、范围
1420～1620 µs 来自已有摆动测试；机械零位、范围和映射系数均为 **UNVALIDATED**。
默认两个方向的系数都是 0，设置系数后才产生对应方向的增量。

## 两个方向函数

两个函数都输入装甲板 `Target`、灯 `Target` 和预设像素 `Point`。
沿用检测器的选择规则：装甲板 `valid=true` 时优先，否则选择有效的灯。
两者均无效时返回四路零增量和 `source=kNone`，不沿用上次坐标。
原点 `(0, 0)` 是有效坐标，不能用坐标是否为零判断目标存在。
目标坐标和预设点必须来自同一幅图像的坐标系；ROI 检测应传全图坐标。

| 函数 | 像素偏差 | 每路输出 |
|---|---|---|
| `horizontal_increment(armor, light, preset)` | `target.x - preset.x`，向右为正 | `偏差 × horizontal_us_per_pixel` |
| `vertical_increment(armor, light, preset)` | `preset.y - target.y`，向上为正 | `偏差 × vertical_us_per_pixel` |

图像坐标 x 向右、y 向下。系数的绝对值表示比例，正负号表示该通道方向；
每路、每个方向分别设置。未参与某个方向的通道，其对应系数设为 0。
两个方向使用同一套通道顺序和单位，支持同时修正。

返回 `std::expected<Increment, Error>`。`Increment::delta_us` 是相对初始角度的
四路 PWM 脉宽增量，单位 µs，允许小数；不是相对上一拍的增量，不应逐拍累加。
`source` 区分装甲板、灯和无目标。配置不合法、所选坐标不是有限数、增量溢出时
返回明确错误，调用方负责决定恢复动作；无目标是正常结果。

## 配置和合成

下面的数值只是展示各字段的含义，方向和比例须自行修改：

```cpp
import dart.control.servo_mapping;

// 以下配置和调用放在所属业务函数内。以下参数为 UNVALIDATED。
dart::control::ServoMapping::Config config{};
//               初始µs  最小µs  最大µs  左右µs/px  上下µs/px
config.channels = {{
    {1520.0F, 1420.0F, 1620.0F, +1.0F, +1.0F}, // CH1
    {1520.0F, 1420.0F, 1620.0F, -1.0F, +1.0F}, // CH2
    {1520.0F, 1420.0F, 1620.0F, -1.0F, -1.0F}, // CH3
    {1520.0F, 1420.0F, 1620.0F, +1.0F, -1.0F}, // CH4
}};
const dart::control::ServoMapping mapping{config};
const auto initial = mapping.pulses(); // 四个独立的初始位置；检查 expected 后读取。

// detected 是本帧 detection::DetectResult，preset 是自行设置的预设点。
const auto horizontal = mapping.horizontal_increment(detected.armor, detected.light, preset);
const auto vertical = mapping.vertical_increment(detected.armor, detected.light, preset);
if (horizontal && vertical) {
    const auto output = mapping.pulses(horizontal->delta_us, vertical->delta_us);
    if (output) {
        // output->pulse_us：最终绝对脉宽，交给下层舵机输出接口。
        // output->delta_us：合成、限幅后相对初始位置的实际增量。
        // output->saturated[i]：该通道是否触发了行程限制。
    }
}
```

`pulses()` 不带参数时输出初始位置；带参数时先将两个方向与初始值相加，
再按各路上下限统一钳位。不要将两个方向分别限幅后再相加，也不要把裸增量
当成绝对脉宽发送。每路必须满足 `0 < min_pulse_us <= initial_pulse_us <= max_pulse_us`，
所有参数必须为有限数；最终范围还应与实际设备周期、机械行程匹配。

`dart_app` 已通过 `src/vision/visual_servo.cpp` 将两个方向的增量合成后，发布给
`include/firmware/src/pwm/session.cpp` 的独立输出任务。K230 后端沿用 PWM 摆动测试的
333 Hz 和引脚路由，实际转向和机械角度仍待台架验证。

## dart_app 纯视觉链路

每个成功检测的帧按 `armor` / `light` 的有效位选择目标，使用全图坐标计算两轴偏差，
相对于各路初始位置合成限幅后发布绝对脉宽。IMU 可独立采样，但不参与控制计算。
输出任务每 10 ms 检查最新指令，仅脉宽发生变化时写设备；硬件持续生成 333 Hz 波形。
无目标、检测错误、取帧错误或无帧时请求初始位置；采集/日志阻塞超过默认 200 ms，
任务也会独立回初始位置。相机失效或进程退出时先停止任务、关闭四路 PWM，再收尾录像。
PWM 初始化或写入失败只停用 PWM，不停止识别、录像及可选 IMU。

默认 `--pwm on`，`--vision off` 时不初始化 PWM；`--pwm off` 可用于只采集/录像。
默认预设点为实际图像的 `(width/2, height/2)`，所有增益为 0。
因此默认可验证设备链路和初始位置，目标偏移不会改变脉宽；实际修正需填写标定增益。

| 参数 | 内容 |
|---|---|
| `--aim X,Y` | 自定义全图像素预设点，必须落在实际图像内 |
| `--servo-initial A,B,C,D` | CH1～CH4 初始脉宽，默认各 1520 µs |
| `--servo-min A,B,C,D` | 各路最小脉宽，默认各 1420 µs |
| `--servo-max A,B,C,D` | 各路最大脉宽，默认各 1620 µs |
| `--servo-horizontal A,B,C,D` | 各路左右增益，单位 µs/像素，默认各 0 |
| `--servo-vertical A,B,C,D` | 各路上下增益，单位 µs/像素，默认各 0 |
| `--servo-timeout-ms N` | 视觉指令有效时长，默认 200 ms，必须大于 0 |

数值列表用逗号分隔，不含空格，允许负增益和小数。配置同时校验机械限幅关系和
333 Hz 周期；图像范围在相机启动后校验。状态日志 `module=servo` 给出预设点、
各路参数、实际下发脉宽、饱和计数、超时计数与设备错误。
`runtime/launcher/startup.list` 已启用此链路；不能同时运行 `pwm_swing`。

## 板端零点标定

零点就是各路的 `initial_pulse_us`，即机械零位对应的脉宽。默认 1520 µs 只是 PWM
摆动测试的中心值，四路机械零位均为 **UNVALIDATED**。标定循环是"改四个数 → 下发 →
看舵面"，不需要重编或重新部署 ELF：每次换一条命令里的数值即可。

两个前置条件：

- PWM 由 `VisualServo::start` 在采集流水线里、`--vision on` 且相机 start 成功后启动，
  相机失败时四路没有波形。
- launcher 自启的 `dart_app` 和手跑实例会同时抢 VICAP 和 PWM。标定前先把清单里那条
  `dart_app` 行注释掉再重启（或直接改它的 `--servo-initial` 走"重启即标定"）。

标定期间两轴增益保持 0，检测到目标也不会叠加增量，四路始终停在初始脉宽：

```text
msh> /sdcard/app/dart_app --seconds 8 --record off --pwm on --servo-initial 1520,1520,1520,1520
```

- 启动约 1～2 秒后四路保持给定脉宽；`--seconds` 到期退出，退出即关闭 PWM，
  舵面失去保持力，下一次运行重新建立波形。进程不给舵机断电。
- 只改要动的那一路，其余保持；例如单独把 CH2 试到 1535：
  `--servo-initial 1520,1535,1520,1520`。
- 参数校验要求 `0 < min ≤ initial ≤ max`，且 `max × 1000 ns` 不超过 333 Hz 周期
  （3003 µs）。某路零点落在默认 1420～1620 之外时，同一条命令里放宽该路的 min/max，
  例如 `--servo-min 1450,1390,1420,1420 --servo-max 1590,1700,1620,1620`。
- 日志 `module=servo op=start ... initial_us=...` 是下发的初始脉宽，状态行的
  `pulse_us=...` 是输出任务实际写入的四路值，可用来核对没有写错通道。

标定完成后把四路数值写进清单那条 `dart_app` 行的 `--servo-initial`，部署清单并重启：

```bash
bash .script/deploy-rtos list     # 或直接改板端 /sdcard/app/startup_final.list
```

清单只在 launcher 下次启动时读取，改动要重启才生效。每行最多 31 个 token，
现有 `dart_app` 行 26 个，加 `--servo-initial` 后 28 个；若还要加 `--servo-min/--servo-max`，
先删掉与默认值相同的 `--rec-format dbv`、`--rec-keep 0` 或 `--raw-layout packed10-lsb`。

零点只决定中位。行程 `--servo-min/--servo-max`、左右/上下方向的符号和比例仍需分别
标定，当前全部为 UNVALIDATED。

## 验证

```bash
cmake --preset host
cmake --build --preset host
ctest --test-dir build --output-on-failure
cmake --build --preset cross --target dart_control
```

主机回归覆盖四路独立初始位置、四个方向的符号、目标选择和丢失、原点坐标、
无累积行为、合成后限幅、非有限输入和数值溢出。
