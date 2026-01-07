# peacock

小孔雀3.0，恩师[ずっと徹夜中でいい](https://space.bilibili.com/3546647883680530)

> STM32G0 (STM32G030xx) MCU 项目，基于 STM32CubeMX 生成，使用 CMake 构建。

使用的是 VSCode 的 STM32CubeIDE 插件，[参考地址](https://blog.csdn.net/liu_yu_143482/article/details/155913195)。


## 目录
- [项目亮点](#项目亮点)
- [已实现功能](#已实现功能)
- [环境与依赖](#环境与依赖)
- [详细使用说明](#详细使用说明)
- [模式与参数说明](#模式与参数说明)
- [构建模式说明](#构建模式说明)
- [目录结构](#目录结构)
- [重新生成 CubeMX 代码](#重新生成-cubemx-代码)
- [常见问题](#常见问题)

## 项目亮点
- C11 + ASM，目标 Cortex-M0+
- 预置 Debug/Release 构建方案
- HAL/CMSIS 驱动完整集成
- 适配 `arm-none-eabi` 工具链

## 已实现功能
- 7 路 LED 软件 PWM（定时器中断扫描，BSRR 一次写入）
- 8 种灯效（流动/呼吸/爆闪/全亮）
- 伽马校正与双缓冲占空比切换，降低闪烁
- 按键消抖、短按切换模式、长按进入/退出设置
- 参数档位调节并显示档位
- 配置持久化（Flash 保存）

## 环境与依赖

| 组件 | 说明 | 备注 |
| --- | --- | --- |
| CMake | 3.22+ | 用于配置生成 |
| Ninja | 构建器 | 推荐 |
| ARM GNU Toolchain | `arm-none-eabi-gcc` | 需在 PATH 中 |
| STM32CubeMX | 可选 | 用于重新生成代码 |

## 详细使用说明

### 1. 配置与编译（Debug）

```sh
cmake --preset Debug
cmake --build --preset Debug
```

### 2. Release 构建

```sh
cmake --preset Release
cmake --build --preset Release
```

### 3. 清理构建产物

```sh
cmake --build --preset Debug --target clean
```

### 4. 生成编译数据库

工程默认导出 `compile_commands.json`，位于 `build/<PresetName>/`，可用于 clangd 或静态分析工具。

### 5. 产物说明

- `build/Debug/peacock.elf`：可执行映像
- `build/Debug/peacock.map`：链接映射文件

> 如果提示找不到 `arm-none-eabi-gcc`，请检查环境变量 PATH。

### 6. 运行与按键交互

按键为低电平有效（上拉输入），用于切换模式与调节参数：
- 短按：切换灯效模式
- 长按：进入/退出设置模式
- 设置模式下短按：切换当前模式的参数档位

## 模式与参数说明

### 模式列表

| 模式 | 灯效描述 | 参数含义 |
| --- | --- | --- |
| 1 | 正向单灯流动 | 流动速度 |
| 2 | 反向单灯流动 | 流动速度 |
| 3 | 中心向两侧分组流动 | 流动速度 |
| 4 | 呼吸流动（软亮圈环绕） | 流动速度 |
| 5 | 反向呼吸流动 | 流动速度 |
| 6 | 中心向外分组呼吸 | 流动速度 |
| 7 | 全亮/全灭爆闪 | 爆闪周期 |
| 8 | 常亮 | 亮度 |

### 参数档位与具体数值

参数档位范围为 1~7，档位越大变化越快/更亮：

| 档位 | 流动周期 (ms) | 爆闪周期 (ms) | 常亮亮度 (0~255) |
| --- | --- | --- | --- |
| 1 | 240 | 300 | 20 |
| 2 | 200 | 240 | 36 |
| 3 | 160 | 200 | 56 |
| 4 | 130 | 160 | 80 |
| 5 | 110 | 130 | 110 |
| 6 | 90 | 110 | 160 |
| 7 | 70 | 90 | 220 |

说明：
- 模式 1~6 使用“流动周期”作为速度参数。
- 模式 7 使用“爆闪周期”。
- 模式 8 使用“常亮亮度”。

### 设置模式与参数保存

进入设置模式后，当前档位会以单灯高亮方式提示（第 1~7 颗灯对应档位 1~7）。
退出设置模式后，参数会延迟写入 Flash，避免频繁擦写。

## 构建模式说明

| 模式 | 目标 | 主要差异 | 适用场景 |
| --- | --- | --- | --- |
| Debug | 调试优先 | `-O0 -g3` | 断点调试、问题定位 |
| Release | 体积/性能优先 | `-Os -g0` | 量产固件、尺寸敏感 |

说明：
- 模式由 CMake preset 控制，定义在 `CMakePresets.json`。
- 具体编译参数由 `cmake/gcc-arm-none-eabi.cmake` 统一设置。

## 目录结构

```
peacock/
├─ Core/                    # 应用代码 (main, ISR, system)
├─ Drivers/                 # CMSIS 与 HAL 驱动
├─ cmake/                   # 工具链与 CubeMX CMake 集成
├─ startup_stm32g030xx.s    # 启动文件
├─ STM32G030XX_FLASH.ld     # 链接脚本
└─ peacock.ioc              # CubeMX 工程文件
```

## 重新生成 CubeMX 代码

1. 使用 STM32CubeMX 打开 `peacock.ioc`
2. 重新生成代码（保持 CMake/Makefile 工程配置）
3. 按照 [详细使用说明](#详细使用说明) 重新构建

## 常见问题

**Q: 为什么编译失败提示找不到工具链？**  
A: 确保已安装 ARM GNU Toolchain，并把 `arm-none-eabi-gcc` 加入 PATH。

**Q: 重新生成后 CMake 出错怎么办？**  
A: 确认 CubeMX 工程输出未覆盖 `cmake/` 下的工具链配置文件。
