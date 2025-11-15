# BSP 配置文件

## 如何使用

### 加载defconfig

```sh
# 加载配置文件
cp configs/sheshou_defconfig .config
# 编译
./docker_build.sh -C "make clean; make -j16"
```

### 新建defconfig

```sh
# 使用 make menuconfig 配置bsp
make menuconfig
# 保存后生成 .config, 将 .config 保存为 defconfig
cp .config configs/sheshou_defconfig
```

## 常用 defconfig 说明

| Name                              | 说明      |
| --------------------------------- | --------- |
| `<proj_name>_defconfig`           | ASIC 默认配置 |
| `<proj_name>_fpga_defconfig`      | FPGA 验证平台默认配置 |
| `<proj_name>_emu_defconfig`       | EMU 验证平台默认配置  |
| `<proj_name>_exportlib_defconfig` | 导出静态库时默认配置  |

## 特殊 defconfig 说明

有特殊defconfig时，请在下方注明具体用途

### 天鹰

| Name          | 说明     |
| ------------- | -------- |
| xxx_defconfig | 用于 ... |
