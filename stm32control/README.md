# STM32 舵机控制代码

> **目录**：`stm32control/`
>
> 配合 CubeMX 生成的 HAL 项目使用，实现 SG90 舵机自动扫舵。

---

## 文件结构

```
stm32control/
├── README.md
└── Core/
    ├── Src/
    │   └── main.c          ← 完整舵机扫舵代码
    └── Inc/
        └── main.h          ← 头文件
```

## 使用步骤

### 如果你还没配置 CubeMX

按 `stm32_servo_cubemx_guide.md` 配置并生成项目。

### 如果你已经用 CubeMX 生成了项目

```
① 打开 CubeMX 生成的项目文件夹
② 用本目录的 Core/Src/main.c → 覆盖项目中的 Core/Src/main.c
③ 用本目录的 Core/Inc/main.h  → 覆盖项目中的 Core/Inc/main.h
④ 重新编译烧录
```

> ⚠️ 如果你的 CubeMX 生成时勾选了 `Generate peripheral initialization as a pair of .c/.h files per peripheral`，会生成 `tim.c`。本 main.c 已自带 TIM2 初始化，需要：
> - **删除** `tim.c`（或从编译中排除）
> - **删除** `tim.h` 的引用

## 代码功能

上电后舵机自动循环：
```
0° → 1° → 2° → ... → 180° → 179° → ... → 0° → (停0.8秒) → 重复
```

## 引脚定义

| 功能 | 引脚 | 定时器 |
|------|------|--------|
| 舵机信号 | **PA0** | TIM2_CH1 |

## 编译烧录

| 工具链 | 操作 |
|--------|------|
| **MDK-ARM (Keil)** | 打开 `.uvprojx` → Build (F7) → Download (F8) |
| **STM32CubeIDE** | 打开项目 → Build → Run |
| **VS Code + GCC** | 在项目目录执行 `make` → 用 ST-Link 烧录 |
