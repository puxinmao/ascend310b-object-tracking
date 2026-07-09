#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* SH1106 1.3寸 OLED (128x64) 驱动 — 接 PB6(SCL)/PB7(SDA) */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_DrawString(uint8_t x, uint8_t page, const char *s);
void OLED_DrawInt(uint8_t x, uint8_t page, int val, uint8_t width);

#endif
