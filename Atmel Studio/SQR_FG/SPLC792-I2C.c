/*
 * SPLC792-I2C.c
 * 
 * I2C LCD aitendo SPLC792-I2C用
 * 
 * _delay_ms() 関数を使用しているため、F_CPUでCPUクロックを定義してください
 *
 *	ピンアサイン
 *		PC3....RST(リセットピン）※省略可能
 *		PC4....SDA
 *		PC5....SCL
 */

#include "SPLC792-I2C.h"

#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdint.h>

#define I2C_DIR		DDRC
#define I2C_PORT	PORTC
#define	I2C_LCD_RST	3

/*======================================================================
 * I2C
 *
 *======================================================================*/
 
// CPUクロック、I2Cクロックに対応してTWBRを修正してください
//
static void i2c_init(void)
{
	// TWBR = {(CLOCK / I2C_CLK) - 16} / 2;
	// I2C_CLK = 100kHz, CLOCK = 8MHz, TWBR = 32
	// I2C_CLK = 100kHz, CLOCK = 16MHz, TWBR = 72
	// I2C_CLK = 100kHz, CLOCK = 20MHz, TWBR = 92
	TWBR = 72;
	TWSR = 0;
}

static uint8_t wait_stat(void)
{
	while(!(TWCR & _BV(TWINT)));
	
	return TW_STATUS;
}

static void i2c_stop(void)
{
	
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
	while(TWCR & _BV(TWSTO));
}

static uint8_t i2c_start(uint8_t addr, uint8_t eeaddr)
{
i2c_restart:
i2c_start_retry:
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	switch(wait_stat()){
		case TW_REP_START:
		case TW_START:
			break;
		case TW_MT_ARB_LOST:
			goto i2c_start_retry;
		default:
			return 0;
	}
	TWDR = addr | TW_WRITE;
	TWCR = _BV(TWINT) | _BV(TWEN);
	switch(wait_stat()){
		case TW_MT_SLA_ACK:
			break;
		case TW_MT_SLA_NACK:
			goto i2c_restart;
		case TW_MT_ARB_LOST:
			goto i2c_start_retry;
		default:
			return 0;
	}
	TWDR = eeaddr;
	TWCR = _BV(TWINT) | _BV(TWEN);
	switch(wait_stat()){
		case TW_MT_DATA_ACK:
			break;
		case TW_MT_DATA_NACK:
			i2c_stop();
			return 0;
		case TW_MT_ARB_LOST:
			goto i2c_start_retry;
		default:
			return 0;
	}
	return 1;
}

static uint8_t i2c_write(uint8_t addr, uint8_t eeaddr, uint8_t dat)
{
	uint8_t rv=0;

restart:
begin:
	if(!i2c_start(addr, eeaddr))	goto quit;
	
	TWDR = dat;
	TWCR = _BV(TWINT) | _BV(TWEN);
	switch(wait_stat()){
		case TW_MT_DATA_ACK:
			break;
		case TW_MT_ARB_LOST:
			goto begin;
		case TW_MT_DATA_NACK:
		default:
			goto quit;
	}
	rv = 1;
quit:
	i2c_stop();
	_delay_us(50);	// １命令ごとに余裕を見て50usウェイトします。
	
	return rv;
}

// コマンドを送信します。HD44780でいうRS=0に相当
static void i2c_cmd(uint8_t db)
{
	i2c_write(0b01111100, 0b00000000, db);
}

// データを送信します。HD44780でいうRS=1に相当
static void i2c_data(uint8_t db)
{
	i2c_write(0b01111100, 0b01000000, db);
}

// （主に）文字列を連続送信します。
static void i2c_puts(uint8_t *s)
{
	while(*s){
		i2c_data(*s++);
	}
}

/*======================================================================
 * I2C LCD
 *
 *======================================================================*/
void I2C_LCD_init(void)
{
	// コントラストの設定
	// 3.0V時 数値を上げると濃くなります。
	// 2.7Vでは0b111000くらいにしてください。。
	// コントラストは電源電圧，温度によりかなり変化します。実際の液晶をみて調整してください。
	uint8_t contrast = 0b110000;
	
	I2C_DIR |= _BV(I2C_LCD_RST);
	
	i2c_init();
	_delay_ms(500);
	
	// RST信号の送出
	I2C_PORT &= ~_BV(I2C_LCD_RST);
	_delay_ms(1);
	I2C_PORT |= _BV(I2C_LCD_RST);
	_delay_ms(10);
	
	// LCDの初期化
	_delay_ms(40);
	i2c_cmd(0b00111000); // function set
	i2c_cmd(0b00111001); // function set
	i2c_cmd(0b00010100); // interval osc
	i2c_cmd(0b01110000 | (contrast & 0xF)); // contrast Low
	
	i2c_cmd(0b01011100 | ((contrast >> 4) & 0x3)); // contast High/icon/power
	i2c_cmd(0b01101100); // follower control
	_delay_ms(300);

	i2c_cmd(0b00111000); // function set
	i2c_cmd(0b00001100); // Display On
	
	i2c_cmd(0b00000001); // Clear Display
	_delay_ms(2);			 // Clear Displayは追加ウェイトが必要
}

void I2C_LCD_puts(char* s)
{
	i2c_puts((uint8_t *)s);
}

void I2C_LCD_setpos(uint8_t x, uint8_t y)
{
	//i2c_cmd(0b10000000 | (x + y * 0x40));
	i2c_cmd(0b1000000 | y << 7 | x);
}
