# IMU 采样与日志

## 当前配置：六轴 LN、关闭省电与 Bank 回读（v7）

依据 `docs/IIM-42652.pdf` 第 25、80 页，陀螺仪只有 OFF（00）、Standby（01）和
Low Noise/LN（11），10 是保留值，没有另一种普通测量模式。LN 是低噪声测量模式，
不是 Low Power/LP。加速度计支持 LP（10），本项目不使用。

所有探针入口和正式 C 驱动测量时均写入并验证 `PWR_MGMT0=0x0f`，即六轴 LN。
无参数入口改为 `low_noise_poll_v7`；`--diagnose` 只保留 `as_is/reset_ln/rc_clock`
三组 LN 对照，已移除 Standby 保持/转换及失败后退回 Standby 的路径。
初始化暂时 OFF、等待 200 ms、复位和配置后再启用 LN；这段关闭是配置时序要求。

第 86 页的 `APEX_CONFIG0 (0x56)` 默认 `DMP_POWER_SAVE=1`，v7 显式清零；
同时禁用 TAP/PED/TILT/FF 和 `SMD_CONFIG (0x57)[1:0]` 的 WOM/SMD 模式。
读改写掩码分别为 `0xf4`、`0x03`，保留保留位、DMP_ODR 和 WOM 比较设置。
每处均回读验证，不符则停止初始化。低功耗模式由模式位关闭，无需更改 LP 时钟选择位。

Bank 1 寄存器完成后，先写 `REG_BANK_SEL (0x76)=0` 并回读，再访问电源、配置与数据。
所有 Bank 选择均回读，配置快照及每组采样前也验证 Bank 0；不匹配时拒绝继续。
正式 C 驱动采用相同的选择后回读检查；Bank 1 选择回读失败时尝试恢复 Bank 0。

```text
/sdcard/app/imu_raw_probe /data/app/logs/imu_ln_v7_probe.log --self-test &
```

当前启动标识 `sensor_init=vendor_selftest_ln_v7`。普通 SPI4、基线、激励各 300 帧，
完成后清除自检激励，恢复普通 LN 持续轮询。`--external-clock` 仍运行三组外部时钟
LN 对照，标识 `external_clock_ln_v7`。以下 Standby/v2/v4/v5/v6 文本均为历史试验，
不代表当前程序行为。

2026-10-30 RESET 后确认 v7：19 次 Bank 回读全部正确，三次启用配置均为
`power=0x0f gyro_mode=3 accel_mode=3`；`0x56: 0x82→0x02`、`0x57=0x00`
回读通过。三组共 900 帧，加速度有效 900、陀螺仪有效 0、SPI 错误 0；激励清除成功。
57 项相关主机测试和探针/正式应用交叉编译通过。证据保存于 `records/imu-ln-bank-v7-20261030/`。

## 厂家启动流程与自检激励（v5/v6 历史配置）

用户已确认供电及外部 32.768 kHz 时钟正常。对照 InvenSense 官方仓库
`public.mcu.iim42652`（commit `5033be4754aa0c0e78d49d8ef9b0ea2dfc27c351`）发现并修正：

- 陀螺仪 OFF→ON 间隔需要严格大于 150 ms，当前等待 200 ms。原先 2 ms 的关闭间隔不足。
  这与软复位后至少 1 ms、开启后至少 200 µs 禁写、最短 ON 45 ms 是不同的要求。
- 厂家设置 INT1 推挽、高有效，注释说明用于避免软复位后进入 Test Mode。
  测试程序及正式 C 驱动均已加入该设置和回读确认。

```text
/sdcard/app/imu_raw_probe /data/app/logs/imu_vendor_selftest_probe.log --self-test &
```

该轮为 `sensor_init=vendor_selftest_v6`：先比较厂家精确 SPI4 设置，再设置陀螺仪自检
基线（±250dps、1 kHz 档、三阶 UI 滤波、ODR/10 带宽），最后启用三轴自检激励，
等待 200 ms 后轮询。三阶段各读取 300 帧；加速度计保持 LN/±16g/原 ODR，不改偏置。
全程读取数据寄存器、FIFO Bypass。激励结束或中断时均尝试清除激励，回读确认后恢复
普通无激励测量并持续轮询。本诊断只判断数据可用性，不报告标准自检合格。

2026-10-30 v5 启动时序/INT1/滤波三组共 900 帧，v6 SPI4/基线/激励三阶段共 900 帧，
SPI 错误均为 0，加速度全部有效，陀螺仪全部为 `0x8000`。出厂陀螺仪自检码为
178、180、182；v6 结束确认 `cleanup_ok=1 self_test_config=0x00`。
修正的软件遗漏并未恢复角速度；结合已确认的供电和时钟，证据支持陀螺仪子系统未工作，
但不能仅凭此确定芯片本体或焊接故障。该轮建议采用同一代码、更换器件作对照，结果见下文。

53 项相关主机测试通过，`imu_raw_probe` 与 `dart_app` 均交叉编译。
普通 `--external-clock` 入口保留 v5 三组对照；无参数/`--diagnose` 为旧 Standby 诊断入口，
其显式 OFF 后等待也已更新为 200 ms。正式驱动对所有时钟模式都保证该关闭间隔。
v6 对 Bank 1 `INTF_CONFIG4` 写厂家 SPI4 字节 `0x02` 属于独立诊断对照，正式驱动仍保留
原先对该寄存器保留位的处理。每次下载正在增长的板端日志前，刷新主机 GIO 挂载，
避免重复读取缓存内容。详见 `records/imu-vendor-startup-20261030-165932/`。

以下 v4/v2 为历史验证记录，其中未满足大于 150 ms 关闭间隔的对照不能排除该软件问题。

## 2026-10-30 更换 IMU 后复测

用户更换 IMU 后，板端 v6 二进制及启动清单回读与上一轮归档逐字节一致。
出厂陀螺仪自检码由 178/180/182 变为 179/182/177，与器件更换一致。
按最后一个启动标记隔离追加日志：最新完整启动的普通、基线及激励三组各 300 帧，
加速度有效共 900 帧、陀螺仪有效 0 帧、SPI 错误 0；连读和逐字节读取的三轴角速度
仍为 `0x8000`。激励结束确认 `cleanup_ok=1 self_test_config=0x00`，已恢复普通测量。

持续轮询快照覆盖约 72.897 秒、完成 7100 轮，累计 SPI 传输错误 0；72 条周期快照
陀螺仪均无效。周期快照不等于逐帧有效性验收。54 项相关主机测试通过。
供电和外部时钟沿用用户此前确认，本轮没有重新测量引脚波形。
同程序更换仍有相同现象，不能唯一归因于原芯片，也不能断言第二块器件损坏；
下一步需核对共用板级连接、电气条件及初始化条件。当前仍只运行独立探针。
日志、程序回读、统计及报告见 `records/imu-replacement-20261030/`。

## 外部 CLKIN 轮询验证（v4 历史配置）

用户确认 IIM-42652 的 CLKIN 由晶振提供 32.768 kHz 时钟。依据 `docs/IIM-42652.pdf`
第 79、98 页，需要同时配置 Bank 1 `INTF_CONFIG5 (0x7b)[2:1]=10` 和 Bank 0
`INTF_CONFIG1 (0x4d)[2]=1`，即选择 Pin 9 CLKIN 并开启 RTC_MODE。两处均读改写、
保留其它位并回读确认。板上 `0x7b` 原值为 `0x00`，实际写入 `0x04`；不能强行写入
手册默认值中的保留位 `0x20`。

```text
/sdcard/app/imu_raw_probe /data/app/logs/imu_external_clock_reset_probe.log --external-clock &
```

日志标识为 `sensor_init=external_clock_v4`。程序先按最短 ON 时长等待 50 ms，
关闭并确认双传感器，等待 2 ms；软复位后再等待 2 ms，然后设置 FIFO Bypass、CLKIN
及 RTC_MODE，最后写 `PWR_MGMT0=0x0f`，等待 50 ms 后轮询数据寄存器。
加速度计维持原 LN、±16g、ODR 编码 `0x06`，复位期间暂时关闭；陀螺仪使用能够输出
角速度的 LN 测量模式。全程轮询、不读 FIFO。每组轮询 300 帧、间隔 10 ms。

| 对照组 | 配置 |
|---|---|
| `external_reset_auto` | 软复位后启用 CLKIN，`RTC_MODE=1, CLKSEL=01`，板上 `0x4d=0x95` |
| `external_reset_rc` | 软复位后启用 CLKIN，`RTC_MODE=1, CLKSEL=00`，板上 `0x4d=0x94` |

若获得有效角速度且没有读错误，恢复首个成功配置；若两组均失败，保留 CLKIN、
`CLKSEL=01`，继续读取 ID 和原始数据，不把配置回读成功视为有效角速度。
不带参数或 `--diagnose` 仍保留下述旧 Standby/v2 入口，当前启动清单使用 `--external-clock`。

手册第 18 页规定 CLKIN 范围为 31–50 kHz；第 60 页说明 ODR 按外部频率/32 kHz 缩放。
32.768 kHz 下 `0x06` 档的实际 ODR 是 1024 Hz。本诊断约每秒轮询 100 次，读取最新值。
CLKIN 是输入引脚，寄存器配置不会产生外部时钟；晶振实际波形尚未测量。

2026-10-30 已完成部署校验与 RESET 后验证。v3 保留配置直接启用 CLKIN 的 300 帧、
v4 软复位后两组共 600 帧，SPI 错误均为 0、加速度全部有效，陀螺仪全部为 `0x8000`。
逐字节读取同样无效，因此软件使能已确认，角速度尚未恢复。
39 项相关主机测试与 `imu_raw_probe`、`dart_app` 交叉编译通过；正式驱动新增
`use_external_clock`，K230 配置为 1。板上目前运行独立 probe，未启动正式应用。
详细日志、部署文件与统计见 `records/imu-external-clock-20261030-162431/`。
下一步需测量 IMU 第 9 脚处的 32.768 kHz 波形与幅度。

## 上电轮询与 Standby 对照（v2 历史配置）

`raw_probe.cpp` 构建为 `imu_raw_probe`。SPI1、CS IO14 低有效、Mode 3、8 bit、1 MHz。
直接轮询 Bank 0 `0x1d..0x2a` 的 14 字节数据寄存器，**不读取 FIFO**；配置阶段显式将
`FIFO_CONFIG (0x16)[7:6]` 设为 `00`（Bypass），诊断回读日志给出 `fifo_config` 和 `access=polling`。

按最新要求，加速度计保持原来的 LN 模式、默认 ±16g、ODR=1 kHz。陀螺仪增加纯 Standby
保持实验，再测试 Standby → LN 的角速度输出。依据 `docs/IIM-42652.pdf` 第 80 页，
`GYRO_MODE=01` 为 Standby、`11` 为 LN；Standby 只保持驱动，不产生新的角速度测量。
陀螺仪没有可输出角速度的普通/低功耗模式，因此 LN 只在角速度对照组中启用。

### 修正后的关闭与配置顺序

1. 选择 Bank 0；若读到传感器已开，额外等待 50 ms，保证未知开启时刻下的最短 ON 时长。
2. 显式关闭加速度计和陀螺仪，等待 2 ms 并回读确认低四位为 0。
3. 需要复位的组此时才写 `DEVICE_CONFIG=0x01`，再等待 2 ms。
4. 只在双传感器 OFF 时修改 Bank 1 配置、时钟和 FIFO。保留保留位，显式设置 FIFO Bypass。
5. 重新开启后等待至少 50 ms，期间不访问寄存器，覆盖 OFF → ON 后至少 200 µs 的禁写要求。

关闭后的 2 ms 是诊断裕量；软复位后的 2 ms 覆盖手册第 68 页要求的至少 1 ms。
Standby → LN 对照只修改允许在线修改的电源模式，不复位、不修改加速度量程/ODR。
`op=quiesce` 日志给出原电源值、关闭后的回读和 `verified`；失败时退出，不继续配置。

### 五组对照与持续轮询

v2 启动命令（当前清单已切换为下述外部时钟 v4）：

```text
/sdcard/app/imu_raw_probe /data/app/logs/imu_standby_probe.log --diagnose &
```

启动日志为 `sensor_init=gyro_diagnostic_v2`，每组轮询 300 帧、间隔 10 ms。

| `case` | 配置 |
|---|---|
| `as_is` | 先关闭，保留既有量程/ODR/时钟，再以 `0x4e=0x0f` 开启六轴 |
| `reset_ln` | 关闭、确认、软复位后显式设置默认量程与 1 kHz，开启六轴 |
| `standby_hold` | 关闭、确认、复位配置后写 `0x4e=0x07`：加速度计 LN，陀螺仪 Standby，持续采样约 3 秒 |
| `standby_ln` | 保持 `0x07` 再等待 3 秒，然后写 `0x0f`，等待 200 ms 后采样，不复位，不改加速度计配置 |
| `rc_clock` | 关闭、确认、复位后设置内部 RC 时钟，再开启六轴作对照 |

所有组均保留加速度计原来的测量配置；复位/配置阶段会按手册暂时关闭双传感器。
每帧读取 INT_STATUS 和原始数据；INT_STATUS 读后清除，统计保留观察值的按位或。
每组结束另逐字节读取陀螺仪六字节。有效计数要求大端及默认无效码配置已确认、
整帧非全零/全 ff、对应三轴均非 `0x8000`。Standby 阶段没有角速度有效帧是预期行为。

若某组获得陀螺仪有效帧且没有读错误，对照结束后重新应用首个成功配置；若全部失败，
重新应用 `standby_hold`，按用户要求让陀螺仪停留在 Standby、加速度计继续测量。
随后进入无限轮询，首轮及每 100 轮记录 ID 和原始字节，直到退出信号或断电。

不带 `--diagnose` 时只运行 Standby 轮询：选择 Bank 0，等待 50 ms，写 `0x4e=0x00`，
等待 2 ms，设置 FIFO Bypass，再写 `0x4e=0x07`、等待 50 ms 并回读电源。
此时日志为 `sensor_init=standby_poll_v2`，默认写入 `/data/app/logs/imu_raw_probe.log`。

### 波形、日志与部署

每次读取 WHO_AM_I 发送 `f5 00`；14 字节数据连读发送 `9d` 后跟 14 个 `00`。
每次传输期间 CS 保持低电平，传输结束释放；丢弃 MISO 第一字节，其余原样输出。
不按 DATA_RDY 阻塞轮询；原始轮询中遇到异常 ID、全零/全 ff 或读失败仍继续记录。
诊断配置阶段保留 ID、格式、FIFO 及电源回读校验，失败时释放 SPI 并退出。
总线创建失败每秒重试。文件每行写入后同步并关闭，方便 USB/MTP 在线读取。

```bash
bash .script/build-rtos imu_raw_probe
bash .script/deploy-rtos imu_raw_probe
bash .script/deploy-rtos list
# 重启后运行新程序；不需要重新烧录固件。
```

自启实例不能与其它 IMU/SPI1 程序并行运行。SIGINT/SIGTERM 停止循环并释放主控 SPI，
退出时保留传感器当前电源状态。恢复视觉版本时注释 `startup.list` 的 IMU 行，
取消 `dart_app` 行注释，再部署清单并重启。以下为此前 v1 的历史验证记录，
不能替代关闭顺序修正后及 Standby v2 的板端验证。

### 2026-10-30 Standby v2 板端验证

29 项主机测试、交叉编译和部署 SHA-256 回读通过；RESET 后日志确认 v2。
各组复位前先关闭并回读 `0x00 verified=1`，FIFO 回读 `0x00 access=polling`。
五组各 300 帧，SPI 错误均为 0、有效加速度帧均为 300、有效陀螺仪帧均为 0。
Standby 回读 `0x07 gyro_mode=1 accel_mode=3`；切换 `0x0f`、等待 200 ms 后仍无角速度。
该轮验证结束后程序回到 `standby_hold`，保持加速度原配置并持续轮询；Standby 不输出新角速度。
详细证据保存在 `records/imu-standby-v2-20261030-155603/`。

### 2026-10-30 板端验证

新版经 MTP 上传并 SHA-256 回读校验、RESET 重启后，日志确认 `sensor_init=wake_only`。
Bank 0 选择和 `PWR_MGMT0=0x0f` 写入均返回 2，50 ms 等待后电源回读为 `0x0f`、
`verified=1`，WHO_AM_I 为 `0x6f`。连续运行 5400 轮，周期日志覆盖 55.457 秒，
约 97.35 轮/秒，SPI 累计错误为 0。温度与加速度三轴已更新；55 条抽样日志中
陀螺仪三轴均为 `0x8000`（-32768），因此六轴采样尚未通过验证。
详细原始日志和统计保存在本机 `records/imu-board-verify-20261030-144505/`。

### 2026-10-30 陀螺仪进一步板端对照

已分别在 RESET 后和完全断电冷启动后运行五组对照，每组 300 帧，共 3000 帧。
两轮所有组的 SPI 读错误均为 0，陀螺仪有效帧均为 0；逐字节读取同样为三轴 `0x8000`。
`SENSOR_CONFIG0=0x80`（轴未禁止）、ODR=1 kHz、电源模式、内部时钟均回读符合配置。
仅陀螺仪组未观察到 DATA_RDY，两轮均未观察到 PLL_RDY；六轴组加速度每组 300 帧有效。
RESET 后初始电源值仍为 `0x0f`，完全断电后为 `0x00`，证明两种启动状态确有区别，
但两者都未恢复陀螺仪。后续重点核查器件引脚处 VDD/VDDIO、去耦、地连接和替换对照，
尚不能仅凭软件确定芯片损坏。报告和原始日志保存在
`records/imu-gyro-diagnostic-20261030-150348/`；诊断完成后程序继续采用 `reset_ln` 六轴配置读数。

## 正常采样会话

`session.cpp` 的 `dart::imu::Session` 接收现有 `IImu&`，持有唯一的 `Estimator`。
独立采样线程调用 `read_sample()` / `Estimator::update()`，把返回的完整 `Estimate`
同时保存为上层快照并送入日志队列。日志线程只写快照，不进行第二次积分。
应用须保证设备已初始化，并且生命周期覆盖 Session。

`imu_working()` 仅读取线程安全的缓存状态，不访问设备：首次成功读取前为 false，
读取成功时为 true，读取失败或停止采样时为 false，后续读取成功会恢复为 true。
它表示最近一次采样读取是否成功；估计器质量、参考系是否就绪仍由 `Estimate` 表达，
具体读取错误继续写入日志。

```cpp
dart::imu::Session session{device};
if (!session.start("/data/app/logs/imu.log"))
    return 1;
// session.log_path() 为本轮实际路径，例如 /data/app/logs/imu_000001.log。
const auto seeded = session.seed(sensor_to_reference); // 取当前样本，要求静止。
if (seeded.accepted) {
    const auto estimate = session.current();          // 与日志中的数据一致。
    // 后续控制周期继续读取 session.current()。
}
session.reset();                                      // 清零并失去参考系，等待下一次 seed。
session.stop();                                       // 停采样、排空队列、同步关闭文件。
```

`current()`、`seed()`、`reset()` 可以由上层线程调用，同一把状态锁保证命令与样本
的先后顺序。重新积分应调用这个 Session 的 `seed()`，不要另建估计器。
成功 seed 的事件记录清零后的速度/累计旋转、新姿态和新 `generation`；失败则保留
当前状态，记录 `command_accepted=0` 和 `command_quality_events`。reset 也单独记录，
并递增 generation。上层读取完整质量位和时间戳；读取错误不会伪造新样本。

`start(path, period_us, initial_seed, log_limit_bytes)` 每个实例只允许成功启动一次。默认采样周期
1000 µs，`initial_seed=false`，由上层明确给出初始参考系。`dart_app` 传入 true，
启动阶段反复请求以单位四元数进行静止 seed，首次成功后停止自动 seed；上层任何
显式 seed/reset 都取消这个启动请求，reset 后不会自行重新积分。
start/stop 由应用生命周期线程串行调用；stop 后应停止消费者发出命令。

`start()` 的可选第五参数接收 `platform::WriteControl*`。应用使用该模式后，暂停日志与
恢复日志可以反复执行，采样和积分继续运行；每次恢复新建编号文件。
最终停止采样仍调用 `stop()`。详见 [写入控制](../WRITE_CONTROL.md)。

每次启动都创建新文件，参数 `path` 是日志基名。例如 `/data/app/logs/imu.log`
依次生成 `imu_000001.log`、`imu_000002.log`；启动时扫描同目录同基名文件，
使用现有最大编号加一，不依赖日期或上电计时。创建使用 `O_CREAT | O_EXCL`，
并发创建发生重名时继续尝试后续编号，不追加或覆盖旧文件；旧的 `imu.log` 也保留。
自定义扩展名会保留，没有扩展名时补 `.log`。编号用尽（999999）或创建失败时
`start()` 返回 false。`log_path()` 返回本轮实际文件名，应用的 IMU start/stop 状态行
也记录该路径。受控模式下每次恢复日志会更新路径，可在会话稳定时或最终 stop 后读取。

日志以 CSV 列写入。每个新文件以 `# imu_log` 元数据行和列名开头，
正常结束时追加 `# session_end` 汇总行。
元数据包含估计器门限、采样周期和固定队列内存大小。解析时跳过 `#` 注释并识别
每轮列名；generation 和 sequence 在新进程/新 Session 中重新计数。

每轮新文件默认限制为 **32 MiB**，上一轮文件不占用本轮额度。
达到上限后停止向该文件追加，保留已有内容；采样、估计和上层快照继续更新。
不会自动删除或覆盖日志。此时 `stats().log_limit_reached=true`，未写入事件计入
`dropped`，不计为 I/O 错误；文件可能以最后一条完整事件结束，没有结束汇总行。
应用的独立状态日志仍报告采样计数、文件字节数和限额标志。第四参数可指定其他
正数字节上限。旧文件持续保留，需要按可用空间定期归档清理。

| 列 | 含义 |
|---|---|
| `event` | `sample`、`seed`、`reset`、`read_error` |
| `sequence,recorded_ns,timestamp_ns` | 事件序号、处理时间、采样时间；ns，单调时钟域 |
| `generation` | 积分批次；成功 seed 和 reset 推进 |
| `raw_a*,raw_g*` | 传感器 ADC 原始计数，三轴各一组 |
| `a*_m_s2,g*_rad_s,temperature_c,sample_flags` | 原始样本的 SI 值、温度和标志；传感器轴 |
| `qw,qx,qy,qz,roll_rad,pitch_rad,yaw_rad` | 传感器到参考系的四元数与 RPY 姿态 |
| `w*_rad_s,linear_a*_m_s2,v*_m_s,rotation_*_rad` | 参考系角速度、去重力加速度、积分速度、累计旋转 |
| `quality_events,sample_accepted,integrated,has_reference_frame` | 原样保存 Estimate 的有效性和异常标志 |
| `command_accepted,command_quality_events,seed_q*` | 命令结果、拒绝原因、请求的参考四元数 |
| `read_error,dropped` | IImu 读取错误码（非读取错误为 -1）、累计日志事件丢弃数（队列满或达到文件限额） |

浮点值用 9 位有效数字保存，可还原 float。饱和、非有限和时间异常的样本也记录，
通过质量位和 integrated 区分，不能把未积分的旧状态当成新积分结果。
`read_error` 行的姿态/样本是上一次快照，应结合 event 和 timestamp 判断年龄。

文件只由日志线程写入，每秒刷新 stdio 缓冲并 fsync，退出时再次同步。固定 512 条队列在
启动期分配；队列锁只保护单条快照复制，写盘期间不持锁。队列满即丢弃该事件，
sequence 缺口、后续 dropped 列和结束汇总可定位缺失，命令也遵循此规则。
`stats()` 提供 samples/read_errors/written/dropped/io_errors/log_bytes/log_limit_reached；written 指写入 stdio
成功的事件数，是否全部落盘还须检查 io_errors。异常断电可能丢失未同步内容。
当前文件在 Session 停止前保持打开（达到限额也不会关闭），板端 FatFS 共享锁可能
阻止 USB/MTP 同时读取；复位后程序改写新文件，上一轮文件可读取。

主机 `imu_session_test` 覆盖日志与上层快照逐字段一致、重复时间戳、饱和、NaN、
gap、重积分成功/失败、参考系变换、reset、初始 seed、读错误、文件错误，以及
达到文件限额后仍能采样和 seed、每轮新建文件与独立额度、旧文件保留、编号缺口、
并发创建、扩展名和路径错误，以及磁盘写入错误。
真实 SPI 采样率、调度抖动、SD 卡吞吐和长时间记录仍待板端验证。
