# SMBus Spike Suppression 配置指南

## 概述

Spike Suppression（尖峰抑制）配置用于滤除SMBus/I2C总线上的信号尖峰和毛刺，提高长线传输的通信稳定性。通过 Kconfig 可以动态选择适合不同线缆长度的配置，无需修改代码。

## 什么是尖峰抑制？

尖峰抑制是SMBus控制器的一项硬件滤波功能，用于滤除总线上的短时干扰信号：

```
有效信号: ▁▁▁▔▔▔▁▁▁  (持续时间 > 抑制阈值)
尖峰噪声: ▁▁▁▃▁▁▁▁  (持续时间 < 抑制阈值) ← 被滤除
```

### 工作原理

```
尖峰抑制时间 = (SPKLEN + 1) × ic_clk_period
```

其中：
- `SPKLEN`: 寄存器配置值
- `ic_clk_period`: SMBus控制器时钟周期

**示例** (@200MHz ic_clk):
- SPKLEN=2: (2+1) × 5ns = **15ns**
- SPKLEN=6: (6+1) × 5ns = **35ns**
- SPKLEN=15: (15+1) × 5ns = **80ns**

**示例** (@8.5MHz ic_clk, 当前系统):
- SPKLEN=2: (2+1) × 114.3ns = **343ns**
- SPKLEN=6: (6+1) × 114.3ns = **800ns**
- SPKLEN=15: (15+1) × 114.3ns = **1829ns**

## 配置选项

### 可用模式

| 模式 | 适用线长 | FS_SPKLEN | HS_SPKLEN | 抑制时间 @200MHz | 抑制时间 @8.5MHz |
|------|---------|-----------|-----------|-----------------|------------------|
| **Short** | < 30cm | 2 | 1 | 15ns | 343ns |
| **Medium** | 30-100cm | 6 | 4 | 35ns | 800ns |
| **Long** | > 100cm | 15 | 9 | 80ns | 1829ns |

*注：当前系统使用 8.5MHz 时钟，实际抑制时间请参考最后一列*

### 模式选择建议

#### Short 模式 (短线)
- **适用场景**: PCB板内走线、短距离连接
- **线缆长度**: < 30cm
- **典型应用**: 板级I2C设备、EEPROM、传感器
- **特点**: 最小延迟，适合信号完整性好的场景

#### Medium 模式 (中线) - **默认推荐**
- **适用场景**: 一般工业应用
- **线缆长度**: 30cm - 100cm
- **典型应用**: 设备间通信、控制面板、机箱内连接
- **特点**: 平衡性能和抗干扰能力，适合大多数应用

#### Long 模式 (长线)
- **适用场景**: 长距离传输或噪声环境
- **线缆长度**: > 100cm
- **典型应用**: 远程传感器、工业控制、长线电缆、噪声大的环境
- **特点**: 最强抗干扰能力，适合恶劣环境

## 配置方法

### 方法 1：使用 menuconfig（推荐）

1. 运行 menuconfig：
   ```bash
   make menuconfig
   ```

2. 导航到配置选项：
   ```
   Device Drivers → SMBUS Spike Suppression Configuration
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
CONFIG_SMBUS_SPK_SHORT=y
CONFIG_SMBUS_FS_SPKLEN=2
CONFIG_SMBUS_HS_SPKLEN=1

# 或中线模式 (30-100cm) - 推荐
CONFIG_SMBUS_SPK_MEDIUM=y
CONFIG_SMBUS_FS_SPKLEN=6
CONFIG_SMBUS_HS_SPKLEN=4

# 或长线模式 (>100cm)
CONFIG_SMBUS_SPK_LONG=y
CONFIG_SMBUS_FS_SPKLEN=15
CONFIG_SMBUS_HS_SPKLEN=9
```

### 方法 3：使用 defconfig（适用于产品）

在板级 defconfig 文件中添加：
```bash
CONFIG_SMBUS_SPK_MEDIUM=y
CONFIG_SMBUS_FS_SPKLEN=6
CONFIG_SMBUS_HS_SPKLEN=4
```

## 验证配置

编译后运行程序，查看调试日志：

**@200MHz 系统示例：**
```
SMBus: FS_SPKLEN=6 (~35 ns) [Config: FS_SPKLEN=6, HS_SPKLEN=4]
SMBus: HS_SPKLEN=4 (~25 ns) [Config: FS_SPKLEN=6, HS_SPKLEN=4]
```

**@8.5MHz 系统示例（当前配置）：**
```
SMBus: FS_SPKLEN=2 (~343 ns) [Config: FS_SPKLEN=2, HS_SPKLEN=1]
SMBus: HS_SPKLEN=2 (~343 ns) [Config: FS_SPKLEN=2, HS_SPKLEN=1]
```

日志说明：
- `FS_SPKLEN`: Fast模式尖峰抑制值
- `HS_SPKLEN`: High Speed模式尖峰抑制值
- `~XXX ns`: 实际的抑制时间（纳秒），根据时钟频率动态计算
- `[Config: ...]`: 显示Kconfig配置的源值

## 技术细节

### 计算公式

```c
实际抑制时间 = (SPKLEN + 1) × (1 / ic_clk频率)
```

其中：
- `SPKLEN`: 从 Kconfig 配置的值（如 2/6/15）
- `ic_clk频率`: SMBus 控制器时钟频率（如 200MHz 或 8.5MHz）

### 示例计算

**中线模式 @ 200MHz ic_clk:**
```
FS_SPKLEN = 6
抑制时间 = (6 + 1) × (1 / 200MHz)
        = 7 × 5ns
        = 35ns
```

**中线模式 @ 8.5MHz ic_clk (当前系统):**
```
FS_SPKLEN = 6
抑制时间 = (6 + 1) × (1 / 8.5MHz)
        = 7 × 117.6ns
        = 823ns
```

**长线模式 @ 8.5MHz ic_clk (当前系统):**
```
HS_SPKLEN = 9
抑制时间 = (9 + 1) × (1 / 8.5MHz)
        = 10 × 117.6ns
        = 1176ns
```

### 不同速度下的SPKLEN选择

代码会根据总线速度自动选择合适的寄存器：

| 总线速度 | 使用的寄存器 | SPKLEN来源 |
|---------|-------------|-----------|
| Standard (100kHz) | IC_FS_SPKLEN | `CONFIG_SMBUS_FS_SPKLEN` |
| Fast (400kHz) | IC_FS_SPKLEN | `CONFIG_SMBUS_FS_SPKLEN` |
| Fast+ (1MHz) | IC_FS_SPKLEN | `CONFIG_SMBUS_FS_SPKLEN` |
| HS (3.4MHz) | IC_HS_SPKLEN | `CONFIG_SMBUS_HS_SPKLEN` |

## 应用场景建议

### 短线模式 (FS_SPKLEN=2, HS_SPKLEN=1)

**适用场景：**
- PCB板上的芯片间通信
- 线缆长度 < 30cm
- 信号完整性良好
- 低延迟要求

**典型应用：**
- EEPROM、Flash存储器
- 板级温度/湿度传感器
- ADC/DAC转换器
- RTC实时时钟

### 中线模式 (FS_SPKLEN=6, HS_SPKLEN=4) - **默认推荐**

**适用场景：**
- 设备间连接
- 线缆长度 30cm - 100cm
- 一般工业环境
- 平衡性能和可靠性

**典型应用：**
- 控制面板通信
- 机箱内设备互联
- 显示模块接口
- 键盘/触摸板接口

### 长线模式 (FS_SPKLEN=15, HS_SPKLEN=9)

**适用场景：**
- 长距离传输
- 线缆长度 > 100cm
- 高噪声环境
- 强干扰场景

**典型应用：**
- 远程传感器网络
- 工业现场设备
- 长线电缆连接
- 电机驱动应用

## 配合 SDA_HOLD 使用

尖峰抑制（SPKLEN）和 SDA保持时间（SDA_HOLD）可以配合使用以获得最佳的长线性能：

```bash
# 长线配置示例
CONFIG_SMBUS_SPK_LONG=y           # 尖峰抑制
CONFIG_SMBUS_FS_SPKLEN=15
CONFIG_SMBUS_HS_SPKLEN=9

CONFIG_SMBUS_SDA_HOLD_LONG=y       # SDA保持时间
CONFIG_SMBUS_SDA_HOLD_FACTOR=50
```

两者协同工作：
- **SPKLEN**: 滤除输入尖峰噪声
- **SDA_HOLD**: 补偿输出信号延迟

## 故障排查

### 如果出现通信错误

1. **检查线缆长度**：确认实际线长是否与配置匹配
2. **尝试更高抑制值**：从 Short → Medium → Long 逐级尝试
3. **查看调试日志**：确认 SPKLEN 值是否正确设置
4. **使用示波器**：测量总线上的实际尖峰宽度

### 常见问题

**Q: 配置了长线模式但仍然有错误？**

A: 可能需要同时调整 SDA_HOLD 时间：
```bash
CONFIG_SMBUS_SPK_LONG=y
CONFIG_SMBUS_SDA_HOLD_LONG=y
```

**Q: 如何知道应该使用哪个模式？**

A: 按照以下步骤：
1. 测量线缆长度
2. 选择对应模式（Short/Medium/Long）
3. 如果仍有问题，尝试升级一级配置
4. 使用示波器验证信号质量

**Q: 可以自定义 SPKLEN 值吗？**

A: 可以，直接修改 `.config` 文件：
```bash
# 自定义值（介于 Medium 和 Long 之间）
CONFIG_SMBUS_FS_SPKLEN=10
CONFIG_SMBUS_HS_SPKLEN=6
```

### 性能影响

| 配置 | 滤波强度 | 延迟影响 | 抗干扰能力 |
|------|---------|---------|-----------|
| Short | 低 | 最小 | 弱 |
| Medium | 中 | 小 | 中 |
| Long | 高 | 中等 | 强 |

**注意**: 过大的SPKLEN值可能导致信号边沿变缓，影响高速通信。

## 配置示例

### 开发板测试（短线）

```bash
# .config
CONFIG_SMBUS_SPK_SHORT=y
CONFIG_SMBUS_FS_SPKLEN=2
CONFIG_SMBUS_HS_SPKLEN=1
```

### 产品应用（中线）

```bash
# .config
CONFIG_SMBUS_SPK_MEDIUM=y
CONFIG_SMBUS_FS_SPKLEN=6
CONFIG_SMBUS_HS_SPKLEN=4
```

### 工业现场（长线）

```bash
# .config
CONFIG_SMBUS_SPK_LONG=y
CONFIG_SMBUS_FS_SPKLEN=15
CONFIG_SMBUS_HS_SPKLEN=9
```

## 相关文件

- **Kconfig 配置**: `drivers/smbus/Kconfig`
- **头文件定义**: `drivers/smbus/drv_smbus_dw.h`
- **计算逻辑**: `drivers/smbus/drv_smbus_dw_i2c.c`
- **配置文件**: `minisoc_fw/.config`
- **相关文档**: `drivers/smbus/SDA_HOLD_CONFIG.md`

## 参考资料

- Synopsys DesignWare I2C 手册 - IC_FS_SPKLEN / IC_HS_SPKLEN 寄存器
- I2C-Bus Specification (UM10204)
- SMBus Specification (SMBus 3.0)
- I2C 总线信号完整性与长线传输最佳实践

## 版本历史

- v1.0 (2025-01-22): 初始版本，基于Kconfig的SPKLEN配置支持
