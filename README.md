# 关于该工程
该工程主要包含minisoc的代码、编译Makefile等文件。支持menuconfig配置和裁剪组件和驱动。

这一部分代码是基于RTEMS的minisoc驱动和测试代码，其中驱动和测试代码尽可能保持稳定，平台相关的差异通过bsp和配置文件进行配置。Bsp的配置可以参考KConfig的方式进行配置。

| 目录           | 说明                                                         |
| -------------- | ------------------------------------------------------------ |
| ./drivers       | 该目录主要是CPU和外设的驱动程序                              |
| ./configs     | 保存各个BSP的 defconfig |
| ./bsp          | 该目录是根据cpu架构分类的bsp相关代码和配置，其中每一个驱动和测试用例在该目录下都有一个移植相关的文件（xxx_porting.h/xxx_porting.c）,为驱动和测试用例提供各个bsp相关的底层接口。 |
| ./components   | 该目录主要是自研或第三方的通用组件，可以包含日志系统、网络协议栈等，部分组件可以通过submodule方式加入进来 |
| ./include      | 通用的头文件                                                 |
| ./scripts      | Kconfig相关的脚本                                            |
| ./tools        | 该目录是系统中使用的工具，如shell和python脚本等              |
| ./applications | 该目录包含minisoc的应用程序                                  |
| ./api          | 该目录主要包括minisoc驱动的api接口                           |



# 项目配置

make menuconfig----可选择具体的项目、ip 驱动、测试用例；


# git 提交
git checkout -b branch_name

git add .

git commit -m "添加描述" --no-verify

git push -u origin branch_name

本文档将在开发过程不断补充完善相关注意事项的提示工作。

## Release Checklist

1. 检查是否修改过menuconfig，是否因此涉及编译流程，是否将修改的menuconfig项目合入有关项目的defconfig
2. 所有对外头文件中，grep一下查看有没有包含非api和非系统库的其余头文件，原则上不允许包含他们
3. 在对外发布的.a中所集成的所有代码中，检查下没有调用nvlog**/NV_LOG之类的接口或者宏定义，其中带时间戳，避免和项目组打印格式有较多冲突；
    至于log_msg和LOG/LOG_INFO/LOG_DEBUG/LOG_ERROR/LOG_WARN 等接口，可以自由调用，其内部都被替换为归一化日志接口，不再带有时间戳；
4. 对于某个外部需求，修改代码时，如有疑问不确定之处，及时找需求有关人澄清需求理解再开发；但是难免会出现不知道自己理解有偏差的情况，即使
    通过外人review，也可能因理解问题或者review实际投入程度问题，而无法确保发现；从效率来说，责任主体还应该是开发人自身，多小心留意；
    此外，有些特殊情况，比如对于某次发布中存在多模块同类修改的问题，比如所有模块都加init/deinit等接口这种情况，每个修改人请关注参考下别的模块的代码，看有无大的原则差异，
    也是可以加大发现问题的概率；
5. 对于问题4, 还是需要在内部建立模拟外部集成环境的测试条件，比如外部不调用minisoc_init,我们自己也不能调用它，改成调用各模块的init接口，但是改动和测试工作量不小，近期没人力投入也是难题；
6 所有的模块核查下自己api.c中的接口的返回值，是否符合api函数注释头中提到了的返回值取值种类





# WAF编译命令集 ✅ 

==以tianhe fpga为例==

## 基础编译

## docker编译

### 不带参数编译

```bash
# prj-defconfig = .config
# prj-name和prj-platform从.config里提取
# 编译参数使用wscript里默认值
./docker_build.sh
```



### 带2个参数编译

```bash
# prj-defconfig 优先使用prj-name和prj-platform匹配configs里的defconfig；如果没有匹配到，则使用.config
# prj-name和prj-platform使用传入参数
# 编译参数使用wscript里默认值
./docker_build.sh --prj-name=tianhe --prj-platform=fpga
```



### 带3个参数编译

```bash
# prj-defconfig，prj-name和prj-platform使用传入参数
# 编译参数使用wscript里默认值
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig
```



### clean

```bash
./docker_build.sh -c './waf clean'
```



## 非docker编译

有一种情况是，minisoc_fw作为子模块参与编译，主模块编译时**已经进入docker环境**了，这时候不能调用docker_build.sh来编译，需要调用native_build.sh执行编译。

### 不带参数编译

```bash
# prj-defconfig = .config
# prj-name和prj-platform从.config里提取
# 编译参数使用wscript里默认值
./native_build.sh
```



### 带2个参数编译

```bash
# prj-defconfig 优先使用prj-name和prj-platform匹配configs里的defconfig；如果没有匹配到，则使用.config
# prj-name和prj-platform使用传入参数
# 编译参数使用wscript里默认值
./native_build.sh --prj-name=tianhe --prj-platform=fpga
```



### 带3个参数编译

```bash
# prj-defconfig，prj-name和prj-platform使用传入参数
# 编译参数使用wscript里默认值
./native_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig
```



### clean

```bash
./native_build.sh -c
```



### wscript里的默认编译参数

#### CFLAGS (默认)

```
-O2 -g0 -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields
```

#### LDFLAGS (默认) 

```
-march=armv7-r -fno-omit-frame-pointer -mapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections -qnolinkcmds -qrtems -march=armv7-r -mapcs -mfloat-abi=soft
```

#### LINKFLAGS (默认)

```
--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs -Wl,-T/home/scott/projects/minisoc_fw/build/tianhe.ld -Wl,--wrap,__getreent -Wl,-Map,tianhe.map
```





## 带编译参数

### 1. 仅自定义CFLAGS

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O2 -g0 -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields'
```

### 2. 仅自定义LDFLAGS  

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-ldflags='-march=armv7-r -fno-omit-frame-pointer -mapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections -qnolinkcmds -qrtems -march=armv7-r -mapcs -mfloat-abi=soft'
```

### 3. 仅自定义LINKFLAGS

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-linkflags='--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs -Wl,-T/home/scott/projects/minisoc_fw/build/tianhe.ld -Wl,--wrap,__getreent -Wl,-Map,tianhe.map'
```

### 4. 自定义CFLAGS + LDFLAGS

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O2 -g0 -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields' \
  --prj-ldflags='-march=armv7-r -fno-omit-frame-pointer -mapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections -qnolinkcmds -qrtems -march=armv7-r -mapcs -mfloat-abi=soft'
```

### 5. 自定义CFLAGS + LINKFLAGS

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O2 -g0 -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields' \
  --prj-linkflags='--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs -Wl,-T/home/scott/projects/minisoc_fw/build/tianhe.ld -Wl,--wrap,__getreent -Wl,-Map,tianhe.map'
```

### 6. 自定义LDFLAGS + LINKFLAGS

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-ldflags='-march=armv7-r -fno-omit-frame-pointer -mapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections -qnolinkcmds -qrtems -march=armv7-r -mapcs -mfloat-abi=soft' \
  --prj-linkflags='--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs -Wl,-T/home/scott/projects/minisoc_fw/build/tianhe.ld -Wl,--wrap,__getreent -Wl,-Map,tianhe.map'
```

### 7. 所有自定义编译参数

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O2 -g0 -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields' \
  --prj-ldflags='-march=armv7-r -fno-omit-frame-pointer -mapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections -qnolinkcmds -qrtems -march=armv7-r -mapcs -mfloat-abi=soft' \
  --prj-linkflags='--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs -Wl,-T/home/scott/projects/minisoc_fw/build/tianhe.ld -Wl,--wrap,__getreent -Wl,-Map,tianhe.map'
```



## 自定义测试场景

### 1. 调试构建 (高调试信息)

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O0 -g3 -DDEBUG -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields'
```

### 2. 优化构建 (O3优化)

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O3 -g1 -DRELEASE -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields'
```

### 3. 尺寸优化构建 (Os优化)

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-Os -g1 -DSIZE_OPTIMIZED -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields'
```

### 4. 链接时优化 (LTO)

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O2 -g1 -flto -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields' \
  --prj-ldflags='-march=armv7-r -fno-omit-frame-pointer -mapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections -flto -qnolinkcmds -qrtems -march=armv7-r -mapcs -mfloat-abi=soft'
```

### 5. 自定义宏定义

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-O2 -g0 -DCUSTOM_BUILD -DVERSION=100 -DPLATFORM_FPGA -Wno-missing-prototypes -Wno-strict-prototypes -fstrict-volatile-bitfields'
```

### 6. 自定义链接器脚本和映射文件

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-linkflags='--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs -Wl,-T/home/scott/projects/minisoc_fw/build/tianhe.ld -Wl,--wrap,__getreent -Wl,-Map,custom_tianhe.map'
```

### 7. 最小功能测试 (验证参数传递)

```bash
./docker_build.sh --prj-name=tianhe --prj-platform=fpga --prj-defconfig=tianhe_fpga_defconfig \
  --prj-cflags='-DTEST_CFLAGS' \
  --prj-ldflags='-DTEST_LDFLAGS' \
  --prj-linkflags='-Wl,-Map,test.map'
```

