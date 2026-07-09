/**
 * oled.c — SH1106 1.3寸 OLED (128x64) I2C 驱动。
 *
 * 接线: SCL→PB6, SDA→PB7, VCC→3.3V, GND→GND
 * I2C 设备地址: 0x3C (HAL 用 8 位形式 0x78)
 *
 * 用法:
 *   OLED_Init();                     初始化 + 清屏
 *   OLED_DrawString(0, 0, "PAN:");   在(列0,页0)画字符串
 *   OLED_DrawInt(30, 0, 85, 3);      画 "085"(3位前导0)
 *   OLED_Refresh();                  把显存刷新到屏幕
 *
 * 显存: oled_ram[8][128] (8 页 × 128 列)。OLED_Clear 后调 OLED_Refresh 刷全屏。
 * 字体: 5x7, 含 空格/数字/'-':'/A-Z,每字符宽 6 像素(5+1间隔)。
 */

#include "oled.h"
#include "main.h"   /* HAL 类型 */
#include <string.h>

extern I2C_HandleTypeDef hi2c1;   /* 在 main.c 中定义 */

/* ── 字体 ── 顺序对应 OLED_CHARS, 每字符 5 字节(列优先, bit0=最上行)─ */
static const char OLED_CHARS[] = " 0123456789-:ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const uint8_t OLED_FONT[][5] = {
    {0x00,0x00,0x00,0x00,0x00},  /* ' ' */
    {0x3E,0x51,0x49,0x45,0x3E},  /* '0' */
    {0x00,0x42,0x7F,0x40,0x00},  /* '1' */
    {0x42,0x61,0x51,0x49,0x46},  /* '2' */
    {0x21,0x41,0x45,0x4B,0x31},  /* '3' */
    {0x18,0x14,0x12,0x7F,0x10},  /* '4' */
    {0x27,0x45,0x45,0x45,0x39},  /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30},  /* '6' */
    {0x01,0x71,0x09,0x05,0x03},  /* '7' */
    {0x36,0x49,0x49,0x49,0x36},  /* '8' */
    {0x06,0x49,0x49,0x29,0x1E},  /* '9' */
    {0x08,0x08,0x08,0x08,0x08},  /* '-' */
    {0x00,0x36,0x36,0x00,0x00},  /* ':' */
    {0x7E,0x11,0x11,0x11,0x7E},  /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36},  /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22},  /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C},  /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41},  /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01},  /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A},  /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F},  /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00},  /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01},  /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41},  /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40},  /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F},  /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F},  /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E},  /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06},  /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E},  /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46},  /* 'R' */
    {0x46,0x49,0x49,0x49,0x31},  /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01},  /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F},  /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F},  /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F},  /* 'W' */
    {0x63,0x14,0x08,0x14,0x63},  /* 'X' */
    {0x07,0x08,0x70,0x08,0x07},  /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43},  /* 'Z' */
};

#define OLED_I2C_ADDR   0x78        /* 0x3C << 1 */
#define OLED_WIDTH      128
#define OLED_PAGES      8

static uint8_t oled_ram[OLED_PAGES][OLED_WIDTH];
static uint8_t oled_dirty;          /* 位掩码: 哪些页需要刷新 */

/* ── 底层 I2C 写 ── 用 MemAddress 作 control byte (0x00=命令, 0x40=数据) ── */
static void oled_write_cmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x00, 1, &cmd, 1, 100);
}

void OLED_Init(void) {
    /* SH1106 初始化序列(与 SSD1306 的差别:电荷泵用 0xAD 0x8B, 列偏移 2) */
    static const uint8_t cmds[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* display clock divide */
        0xA8, 0x3F,       /* multiplex ratio 1/64 */
        0xD3, 0x00,       /* display offset */
        0x40,             /* start line 0 */
        0xAD, 0x8B,       /* charge pump ON (SH1106) */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan direction */
        0xDA, 0x12,       /* COM pins hardware config */
        0xD9, 0x22,       /* pre-charge period */
        0xDB, 0x40,       /* VCOMH deselect level */
        0xA4,             /* display follow RAM content */
        0xA6,             /* normal (not inverted) */
        0xAF,             /* display ON */
    };
    for (uint16_t i = 0; i < sizeof(cmds); i++) {
        oled_write_cmd(cmds[i]);
    }
    HAL_Delay(50);        /* 等 charge pump 稳定 */
    OLED_Clear();
    OLED_Refresh();
}

void OLED_Clear(void) {
    memset(oled_ram, 0, sizeof(oled_ram));
    oled_dirty = 0xFF;    /* 全部页标记为脏 */
}

/* 把 dirty 的页刷新到屏幕 */
void OLED_Refresh(void) {
    for (uint8_t p = 0; p < OLED_PAGES; p++) {
        if (!(oled_dirty & (1 << p))) continue;
        uint8_t cmd[3] = {
            (uint8_t)(0xB0 + p),    /* 设页地址 0xB0~0xB7 */
            (uint8_t)(0x00 + 2),    /* 列地址低 4 位(SH1106 列偏移 2) */
            (uint8_t)(0x10 + 0),    /* 列地址高 4 位 */
        };
        HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x00, 1, cmd, 3, 100);
        HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x40, 1, oled_ram[p], OLED_WIDTH, 100);
        oled_dirty &= ~(1 << p);
    }
}

static int8_t font_index(char c) {
    const char *p = strchr(OLED_CHARS, c);
    return p ? (int8_t)(p - OLED_CHARS) : 0;   /* 找不到用空格 */
}

static void draw_char(uint8_t x, uint8_t page, char c) {
    const uint8_t *f = OLED_FONT[font_index(c)];
    for (uint8_t i = 0; i < 5; i++) {
        if (x + i < OLED_WIDTH) oled_ram[page][x + i] = f[i];
    }
    oled_dirty |= (1 << page);
}

void OLED_DrawString(uint8_t x, uint8_t page, const char *s) {
    uint8_t cx = x;
    for (; *s; s++) {
        if (cx + 6 > OLED_WIDTH) break;   /* 超出宽度,停止 */
        draw_char(cx, page, *s);
        cx += 6;                           /* 5 列字符 + 1 列间隔 */
    }
}

void OLED_DrawInt(uint8_t x, uint8_t page, int val, uint8_t width) {
    char buf[8];
    if (width > 7) width = 7;
    if (val < 0) val = -val;              /* 显示绝对值(角度都非负,防御) */
    for (int8_t i = width - 1; i >= 0; i--) {
        buf[i] = '0' + (val % 10);
        val /= 10;
    }
    buf[width] = 0;
    OLED_DrawString(x, page, buf);
}
