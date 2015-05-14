/*
  SPLC792-I2C.h
  
  I2C LCD aitendo SPLC792-I2C用
  
  _delay_ms() 関数を使用しているためF_CPUでCPUクロックを定義してください
  
  ピンアサイン
  	PC3....RST(リセットピン）※省略可能
  	PC4....SDA
  	PC5....SCL
  
  */
 
#ifndef _SPLC792_H
#define _SPLC792_H

#include <stdint.h>

void I2C_LCD_init(void);
void I2C_LCD_clear(void);
void I2C_LCD_puts(const char* s);
void I2C_LCD_setpos(uint8_t x, uint8_t y);

#endif
