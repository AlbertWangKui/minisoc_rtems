# SMBus SDA Hold Time 配置指南

## 概述

SDA Hold Time 配置用于补偿长线传输导致的信号延迟问题。通过 Kconfig 可以动态选择适合不同线缆长度的配置，无需修改代码。

## 配置选项

### 可用模式

| 模式 | 适用线长 | 因子值 | 实际保持时间 (@12.5MHz) | 实际保持时间 (@8.5MHz) |
|------|---------|--------|----------------------|----------------------|
| **Short** | < 30cm | 15 | ~720ns | ~1059ns |
| **Medium** | 30-100cm | 35 | ~1680ns | ~2471ns |
| **Long** | > 100cm | 50 | ~2400ns | ~3529ns |

*注：当前系统使用 8.5MHz 时钟，实际保持时间请参考最后一列*

## 配置方法

### 方法 1：使用 menuconfig（推荐）

1. 运行 menuconfig：
   ```bash
   make menuconfig
   ```

2. 导航到配置选项：
   ```
   Device Drivers → SMBUS SDA Hold Time Configuration
   ```

3. 选择适合您线缆长度的模式：
   - `[*] Short line mode (<30cm)` - 短线模式
   - `[*] Medium line mode (30-100cm)` - 中线模式（默认）
   - `[*] Long line mode (>100cm)` - 长线模式

4. 保存配置并退出

### 方法 2：直接编辑 .config 文件

在 `minisoc_fw/.config` 文件中修改：

```bash
# 短线模式 (<30cm)
CONFIG_SMBUS_SDA_HOLD_SHORT=y
CONFIG_SMBUS_SDA_HOLD_FACTOR=15

# 或中线模式 (30-100cm) - 推荐
CONFIG_SMBUS_SDA_HOLD_MEDIUM=y
CONFIG_SMBUS_SDA_HOLD_FACTOR=35

# 或长线模式 (>100cm)
CONFIG_SMBUS_SDA_HOLD_LONG=y
CONFIG_SMBUS_SDA_HOLD_FACTOR=50
```

### 方法 3：使用 defconfig（适用于产品）

在板级 defconfig 文件中添加：
```bash
CONFIG_SMBUS_SDA_HOLD_MEDIUM=y
CONFIG_SMBUS_SDA_HOLD_FACTOR=35
```

## 验证配置

编译后运行程序，查看调试日志：

**@12.5MHz 系统示例：**
```
SMBus: SDA TX hold = (35x12500000)/(200x100000) = 21 cycles (~1680 ns)
```

**@8.5MHz 系统示例（当前配置）：**
```
SMBus: SDA TX hold = (35x8750000)/(200x100000) = 15 cycles (~1714 ns)
```

日志说明：
- `SMBUS_SDA_HOLD_FACTOR` 显示使用的因子值
- `cycles` 显示实际的时钟周期数
- `~XXX ns` 显示实际的纳秒时间（根据时钟频率计算）

## 技术细节

### 计算公式

```c
sdaHoldTime = (SMBUS_SDA_HOLD_FACTOR × ic_clk) / (200 × speed)
```

其中：
- `SMBUS_SDA_HOLD_FACTOR`: 从 Kconfig 配置的因子（15/35/50）
- `ic_clk`: SMBus 控制器时钟频率（如 12.5MHz 或 8.5MHz）
- `speed`: 总线速度（100kHz/400kHz/1MHz）

### 示例计算

**中线模式 @ 100kHz, ic_clk=12.5MHz:**
```
sdaHoldTime = (35 × 12,500,000) / (200 × 100,000)
            = 437,500,000 / 20,000,000
            = 21 周期
实际时间 = 21 / 12.5MHz = 1680ns
```

**中线模式 @ 100kHz, ic_clk=8.5MHz (当前系统):**
```
sdaHoldTime = (35 × 8,750,000) / (200 × 100,000)
            = 306,250,000 / 20,000,000
            = 15 周期
实际时间 = 15 / 8.5MHz = 1765ns
```

**长线模式 @ 100kHz, ic_clk=8.5MHz (当前系统):**
```
sdaHoldTime = (50 × 8,750,000) / (200 × 100,000)
            = 437,500,000 / 20,000,000
            = 21 周期
实际时间 = 21 / 8.5MHz = 2471ns
```

## 应用场景建议

### 短线模式 (15)
- 适用于开发板上的短距离连接
- 线缆长度 < 30cm
- 典型应用：板级 I2C 设备、EEPROM、传感器等

### 中线模式 (35) - 默认推荐
- 适用于一般工业应用
- 线缆长度 30-100cm
- 典型应用：设备间通信、控制面板等

### 长线模式 (50)
- 适用于长距离传输或噪声环境
- 线缆长度 > 100cm
- 典型应用：远程传感器、工业控制、长线电缆

## 故障排查

### 如果出现通信错误

1. **检查线缆长度**：确认实际线长是否与配置匹配
2. **尝试更高因子**：从 Short → Medium → Long 逐级尝试
3. **查看调试日志**：确认 SDA_HOLD 值是否正确设置
4. **使用示波器**：测量实际的 SDA 保持时间

### 调整自定义值

如果预设值不满足需求，可以直接修改 `.config` 文件中的 `CONFIG_SMBUS_SDA_HOLD_FACTOR` 值：

```bash
# 自定义值为 40（介于 Medium 和 Long 之间）
CONFIG_SMBUS_SDA_HOLD_FACTOR=40
```

## 相关文件

- **Kconfig 配置**: `drivers/smbus/Kconfig`
- **头文件定义**: `drivers/smbus/drv_smbus_dw.h`
- **计算逻辑**: `drivers/smbus/drv_smbus_dw_i2c.c`
- **配置文件**: `minisoc_fw/.config`

## 参考资料

- Synopsys DesignWare I2C 手册
- I2C-Bus Specification (UM10204)
- SMBus Specification (SMBus 3.0)
