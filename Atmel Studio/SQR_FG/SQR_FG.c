/*
 * SQR_FG.c
 *
 * Created: 2015/05/07 15:41:32
 *  Author: gizmo
 * 
 * 2015.12.02 Ver.2
 */ 

#define F_CPU	16000000UL

#include <stdio.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>

#include "SPLC792-I2C.h"

//-----------------------------------------------------------------------------
// PORT 定義
//-----------------------------------------------------------------------------
// PWM
//
#define PWM_DIR		DDRB
#define PWM_PORT	PORTB
#define PWM_A	1
#define PWM_B	2

// Rotary Encoder
//
#define RE_DIR	DDRD
#define RE_PORT	PORTD
#define RE_PIN	PIND
#define RE_A	0
#define RE_B	1
#define RE_SW	2

// Potentiometer
//
#define POT_DUTY	0

//-----------------------------------------------------------------------------
// 定数定義
//-----------------------------------------------------------------------------
// 状態遷移
//
#define STAT_SQR_WAV	0
#define STAT_PRESCALER	1
#define STAT_IMPULSE	2
#define MAX_STAT		3

const char* state_str[] = {
	"SQR WAV    ",
	"PRE SCALER ",
	"IMPULSE    ",
};

// 周期設定
//
const uint16_t cycle_table[] = {
	50000, 20000, 10000, 5000, 2000, 1000, 500, 200, 100, 50, 20, 10, 5, 2, 1
};

#define CYCLE_IDX_MAX		(sizeof(cycle_table)/sizeof(uint16_t))
#define CYCLE_IDX_INIT		(5)

// プリスケーラ
//
#define PRESCALER_1			((0<<CS12)|(0<<CS11)|(1<<CS10))
#define PRESCALER_8			((0<<CS12)|(1<<CS11)|(0<<CS10))
#define PRESCALER_64		((0<<CS12)|(1<<CS11)|(1<<CS10))
#define PRESCALER_256		((1<<CS12)|(0<<CS11)|(0<<CS10))
#define PRESCALER_1024		((1<<CS12)|(0<<CS11)|(1<<CS10))

#define INITIAL_PRESCALER	(PRESCALER_8)

// デューティ
//
#define INITIAL_DUTY		(64)

//-----------------------------------------------------------------------------
// 帯域変数
//-----------------------------------------------------------------------------
volatile uint8_t g_cycle_idx;		/* 0 .. CYCLE_IDX_MAX */
volatile uint8_t g_duty;			/* 0 .. 127 */

/*------------------------------------------------------------------------/
 * PWM
 *
 ------------------------------------------------------------------------*/
void timer1_set_cycle_duty(uint16_t cycle, uint16_t duty)
{
	// Output Compare Register
	OCR1A = cycle;
	OCR1B = duty;
}

void timer1_init_PWM(uint16_t cycle, uint16_t duty)
{
	timer1_set_cycle_duty(cycle, duty);
	
	// Initialize Counter
	TCNT1 = 0;
	
	// Phase and Frequency Correct PWM Mode, TOP = OCR1A
	TCCR1B |= (1 << WGM13) | (0 << WGM12);
	TCCR1A |= (0 << WGM11) | (1 << WGM10);

	
	// Compare Output OC1A, OC1B
	TCCR1A |= (0 << COM1A1) | (1 << COM1A0);
	TCCR1A |= (1 << COM1B1) | (0 << COM1B0);
	
	// Timer1 Start, Set Prescaler to Initial Value
	TCCR1B |= INITIAL_PRESCALER;
	
	// Test Prescaler
	//TCCR1B |= (0 << CS12) | (1 << CS11) | (1<< CS10);
}

void timer1_stop_PWM(void)
{
	TCCR1A = 0;
	TCCR1B = 0;
}

/*------------------------------------------------------------------------/
 * Rotary Encoder
 *
 ------------------------------------------------------------------------*/
// 戻り値: ロータリーエンコーダーの回転方向
//        0:変化なし 1:時計回り -1:反時計回り
//
int8_t readRE_ROT(void)
{
	static uint8_t index;
	int8_t retVal = 0;
	
	index = (index << 2) | (RE_PIN & _BV(RE_A)) | (RE_PIN & _BV(RE_B));
	index &= 0b1111;
	
	switch (index) {
		// 時計回り
		case 0b0001:	// 00 -> 01
		case 0b1110:	// 11 -> 10
		retVal = 1;
		break;
		// 反時計回り
		case 0b0010:	// 00 -> 10
		case 0b1101:	// 11 -> 01
		retVal = -1;
		break;
	}
	
	_delay_ms(2);	// (とりあえず)チャタリング防止
	
	return retVal;
}

#define RE_SW_PRESS	(~(RE_PIN)&_BV(RE_SW))

// 戻り値: ロータリーエンコーダーのプッシュスイッチ
//        0:押されていない 1:押されている
//
int8_t readRE_SW(void)
{
	if (RE_SW_PRESS) {
		_delay_ms(10);
		while(RE_SW_PRESS)
			;
		_delay_ms(10);
		return 1;
	}
	else return 0;
}

void read_cycle_idx()
{
	g_cycle_idx += readRE_ROT();

	// 周期インデックスを 0..CYCLE_TABLE_ELEMENTS に制限
	if (g_cycle_idx < 0) {
		g_cycle_idx = 0;
	} else if (g_cycle_idex >= CYCLE_TABLE_ELEMENTS) {
		g_cycle_idx = CYCLE_TABLE_ELEMENTS - 1;
	}
}

/*------------------------------------------------------------------------/
 * Potentiometer
 *
 ------------------------------------------------------------------------*/
void adc_init(void)
{
	// ADC使用ピンをデジタル入力禁止 PC0, PC1
	DIDR0 |= _BV(ADC0D) | _BV(ADC1D);
	
	// ADCの基準電圧(AVCC)
	ADMUX =	_BV(REFS0);
	
	// ADC動作設定
	ADCSRA = 0b10000111		// bit2-0: 111 = 128分周	16MHz/128=125kHz (50k～200kHzであること)
		| _BV(ADSC);		// 1回目変換開始(調整)
}

// 戻り値: ADCの8bit読み取り値（POTの回転）
//         8bit (0..255)
//
uint8_t adc_convert8(uint8_t channel)
{
	ADMUX = _BV(REFS0)		// ADCの基準電圧(AVCC)
		| _BV(ADLAR)		// 左詰め
		| channel;			// AD変換チャンネル
	
	ADCSRA |= _BV(ADSC);	// 変換開始
	
	loop_until_bit_is_set(ADCSRA, ADIF); // 変換完了まで待つ
	
	return ADCH;
}

// 戻り値: ADCの8bit読み取り値（POTの回転）
//        7bit (0..127)
//
uint8_t read_duty(void)
{
	// 8bit -> 7bit
	return adc_convert8(POT_DUTY) >> 1;
}

/*------------------------------------------------------------------------/
 * generate wave
 *
 ------------------------------------------------------------------------*/
 void setPWM(uint_16 cycle_idx, uint_16 duty)
 {
	cycle = cycle_table[cycle_idx];
	
	// デューティー比の計算 
	// 1..cycle に制限
	duty = (((uint32_t)cycle * duty) >> 7);
	if (duty == 0)
		duty = 1;
		
	timer1_set_cycle_duty(cycle, duty);
 }

void setPreScaler(uint_8 prescaler)
{
	// レジスタを設定
}
 
uint16_t calcFrequency(uint8_t cycle_idx, uint8_t duty)
{
	// 周期とデューティから周波数を計算
	return 1000;
}
 
 
 /*------------------------------------------------------------------------/
 * main routine
 *
 ------------------------------------------------------------------------*/
 int main(void)
{
	// 状態変数
	int8_t state = 0;
	int8_t is_state_changed = 1;
	// LCD
	char LCD_line[17];
	
	//-------------------------------------------------------------------
	// Portの初期化
	//-------------------------------------------------------------------
	// PWM Output
	//
	PWM_DIR |= _BV(PWM_A) | _BV(PWM_B);
	
	// Rotary Encoder
	// PullUp
	RE_PORT |= _BV(RE_A) | _BV(RE_B) | _BV(RE_SW);
	
	//-------------------------------------------------------------------
	// デバイスのの初期化
	//-------------------------------------------------------------------
	// Initialize ADC
	//
	adc_init();

	// Initialize PWM
	//
	g_cycle_idx = cycle_table[INITIAL_CYCLE_IDX];
	timer1_init_PWM(g_cycle_idx, DUTY_MAX/2);
	
	// Initialize I2C-LCD
	//
	I2C_LCD_init();
	I2C_LCD_puts("Square Wave");
	I2C_LCD_setpos(1, 1);
	I2C_LCD_puts("Generator");
	_delay_ms(1000);
	
	I2C_LCD_clear();
	
	//-------------------------------------------------------------------
	// メイン・ループ
	//-------------------------------------------------------------------
	while(1)
    {
		// stateの遷移
		//
		if (readRE_SW() != 0) {
			state++;
			is_state_changed = 1;
			if (state >= MAX_STAT)
				state = 0;
			I2C_LCD_clear();
			I2C_LCD_puts(state_str[state]);
		}
		
		switch (state) {
		case STAT_SQR_WAV:
			// 周期を取得
			read_cycle_idx();
			// デューティを取得
			read_duty();
			// 矩形波を出力
			setPWM(g_cycle_idx, g_duty);
			// LCDに表示
			I2C_LCD_setpos(0, 1);
			sprintf(LCD_line, "F:%7d D:%2d", 
				calc_freq(cycle_table[g_cycle_idx], g_duty),
				g_duty * 100 / 128
			);
			I2C_LCD_puts(LCD_line);
			break;
		case STAT_IMPULSE:
			// 周期を取得
			// impulseを出力
			// LCDに表示
			break;
		case STAT_PRESCALER:
			// プリスケーラを取得
			// プリスケーラを設定
			// LCDに表示
			break;
		}
		
		is_state_changed = 0;
	}
}