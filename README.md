# QMC5883L

QST QMC5883L 三轴磁力计驱动模块。
Driver module for the QST QMC5883L 3-axis magnetometer.

构造时模块通过 I2C（7 位地址 `0x0D`）检查 `CHIP_ID`（期望 `0xFF`），开启寄存器指针
自动回卷并保持 DRDY 中断使能，写入推荐的 SET/RESET 周期，并配置为连续模式、±8 G、
200 Hz、OSR 512；初始化失败时每 100 ms 重试，直到成功。随后创建 `qmc5883l_thread`
线程（`REALTIME` 优先级）：等待 DRDY 中断（超时 100 ms 时查询一次状态寄存器的 DRDY
位，仍未就绪则告警），读取 6 字节原始数据，乘以 1000/3000 mG/LSB，经 `rotation` 旋转后
发布。原始值全为 0 的样本被丢弃，此时重新发布上一次的值。

During construction the module checks `CHIP_ID` over I2C (7-bit address `0x0D`,
expects `0xFF`), enables register pointer roll-over with the DRDY interrupt
enabled, writes the recommended SET/RESET period and configures continuous mode,
±8 G, 200 Hz, OSR 512; on failure it retries every 100 ms until it succeeds. It
then starts the `qmc5883l_thread` thread (`REALTIME` priority): it waits for the
DRDY interrupt (on a 100 ms timeout it checks the DRDY bit of the status
register once and logs a warning if no data is ready), reads the 6 raw bytes,
scales them by 1000/3000 mG/LSB, rotates the vector by `rotation` and publishes
it. A sample whose raw values are all zero is discarded and the previous value
is published again.

- Topic：`topic_name`（默认 `qmc5883l_mag`），类型 `Eigen::Matrix<float, 3, 1>`（x, y, z，mG）。
  / Topic `topic_name` (default `qmc5883l_mag`), type `Eigen::Matrix<float, 3, 1>` (x, y, z in mG).
- `OnMonitor()`：读取状态寄存器，溢出（OVL）时告警；数据出现 NaN 或 Inf 时告警。
  / Reads the status register and warns on overflow (OVL); warns when the data contains NaN or Inf.

### RamFS 命令 / RamFS command

模块在 `ramfs` 中注册命令 `qmc5883l`。/ The module adds the command `qmc5883l` to `ramfs`.

```sh
qmc5883l show <time_ms> <interval_ms>   # 每 interval_ms 打印一次磁场（mG），持续 time_ms / print the field (mG) every interval_ms for time_ms
```

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。
No other Modules; LibXR only.

## 构造接口 / Constructor

```cpp
QMC5883L(LibXR::GPIO& interrupt, LibXR::I2C& i2c, LibXR::RamFS& ramfs,
         LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
         const char* topic_name = "qmc5883l_mag",
         size_t task_stack_depth = 1536);
```

依赖 / Dependencies:

- `interrupt`：连接 DRDY 引脚的中断 GPIO（数据就绪时为高电平）。/ Interrupt GPIO connected to the DRDY pin (high when data is ready).
- `i2c`：芯片所在的 I2C 总线。/ The I2C bus the chip is on.
- `ramfs`：注册 `qmc5883l` 命令的 RamFS。/ RamFS that receives the `qmc5883l` command.

配置 / Configuration:

- `rotation`：安装姿态四元数 (w, x, y, z)，默认单位四元数。/ Mounting rotation quaternion (w, x, y, z), identity by default.
- `topic_name`：发布磁场数据的 Topic 名，默认 `qmc5883l_mag`。/ Name of the magnetic-field topic, default `qmc5883l_mag`.
- `task_stack_depth`：采集线程栈大小，默认 1536。/ Stack size of the acquisition thread, default 1536.

## 使用 / Use

```sh
xrobot module add xrobot-org/QMC5883L
xrobot setup
xrobot instance add xrobot-org/QMC5883L
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/QMC5883L
    id: qmc5883l_0
    args:
      - interrupt: qmc5883l_int
      - i2c: i2c1
      - ramfs: ramfs
      - rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
      - topic_name: '"qmc5883l_mag"'
      - task_stack_depth: '1536'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(qmc5883l_int, LibXR::GPIO);
XR_REGISTER(i2c1, LibXR::I2C);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/QMC5883L`
（在 BSP 中）打印当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/QMC5883L` in a BSP, prints the current
constructor.
