/*
 * SQR_FG.c
 *
 * Created: 2015/05/07 15:41:32
 *  Author: gizmo
 */ 

#define F_CPU	16000000UL

#include <stdio.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>

#include "SPLC792-I2C.h"

// 状態遷移
//
#define STAT_SQR_WAV	0
#define STAT_IMPULSE	1
#define STAT_STEP		2
#define STAT_GND_LEVEL	3
#define MAX_STAT		4

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

// GND Switch
//
#define GNDSW_DIR	DDRB
#define GNDSW_PORT	PORTB
#define GNDSW_GND	3
#define GNDSW_VGND	4

// 周波数設定
//
const uint16_t cycle_table[] = {
	50000, 20000, 10000, 5000, 2000, 1000, 500, 200, 100, 50, 20, 10, 5, 2
};

#define CYCLE_TABLE_ELEMENTS	(sizeof(cycle_table)/sizeof(uint16_t))
#define INITIAL_FREQ	(4)
#define INITIAL_DUTY	(64)


// 状態遷移表示用
//
const char* state_str[] = {
	"SQR WAV",
	"IMPULSE",
	"STEP   ",
	"GND LVL"
};

const char* gnd_level_str[] = {
	"GND ",
	"VGND"
};

/*------------------------------------------------------------------------/
 * PWM
 *
 ------------------------------------------------------------------------*/
void timer1_start_PWM(void)
{
	// Timer1 Start, Set Prescaler to 8
	//TCCR1B |= (1 << CS11);
	
	// Test Prescaler
	TCCR1B |= (0 << CS12) | (1 << CS11) | (1<< CS10);
}

void timer1_stop_PWM(void)
{
	// clear CS10, CS11, CS12
	TCCR1B &= 0b11111000;
}

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
	
	timer1_start_PWM();	
}

/*------------------------------------------------------------------------/
 * Rotary Encoder
 *
 ------------------------------------------------------------------------*/
int8_t readRE(void)
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
	
	_delay_ms(5);	// (とりあえず)チャタリング防止
	
	return retVal;
}

#define RE_SW_PRESS	(~(RE_PIN)&_BV(RE_SW))

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

uint8_t adc_convert8(uint8_t channel)
{
	ADMUX = _BV(REFS0)		// ADCの基準電圧(AVCC)
		| _BV(ADLAR)		// 左詰め
		| channel;			// AD変換チャンネル
	
	ADCSRA |= _BV(ADSC);	// 変換開始
	
	loop_until_bit_is_set(ADCSRA, ADIF); // 変換完了まで待つ
	
	return ADCH;
}

// 7bit 0..127
uint8_t readDuty(void)
{
	// 8bit -> 7bit
	return adc_convert8(POT_DUTY) >> 1;
}

/*------------------------------------------------------------------------/
 * main routine
 *
 ------------------------------------------------------------------------*/
#if 0
static void wait_us(short t)
{
	while(t-->=0){
		asm volatile ("nop");
		asm volatile ("nop");
	}
}

// この関数でミリ秒のウェイトを入れます。引数 100 = 100ミリ秒ウェイト
// クロックスピードにより調整してください。
static void wait_ms(short t)
{
	while(t-->=0){
		wait_us(1000);
	}
}
#endif

int main(void)
{
	int8_t state = 0;
	int8_t is_state_changed = 1;
	
	int8_t freq = INITIAL_FREQ;	
	uint16_t cycle;
	
	uint8_t duty_val;
	uint8_t old_duty_val = INITIAL_DUTY; 
	uint16_t duty;

	int8_t RE_val;
	//int8_t gnd_level = 1;	// 0:GND 1:VGND
	
	char LCD_line[17];
	
	// PWM
	//
	PWM_DIR |= _BV(PWM_A) | _BV(PWM_B);
	
	// Rotary Encoder
	//
	// PullUp
	RE_PORT |= _BV(RE_A) | _BV(RE_B) | _BV(RE_SW);
	
	// GND Level Switch
	//
	GNDSW_DIR |= _BV(GNDSW_GND) | _BV(GNDSW_VGND);
	GNDSW_PORT |= _BV(GNDSW_VGND);
	
	// Initialize ADC
	//
	adc_init();

	// sei();
	
	// Initialize PWM
	//
	cycle = cycle_table[freq];
	timer1_init_PWM(cycle, cycle / 2);
	
	// Initialize I2C-LCD
	//
	I2C_LCD_init();
	I2C_LCD_puts("Square Wave");
	I2C_LCD_setpos(1, 1);
	I2C_LCD_puts("Generator");
	_delay_ms(1000);
	
	I2C_LCD_clear();
	I2C_LCD_puts(state_str[state]);
	
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
		
		RE_val = readRE();
		
		switch (state) {
		case STAT_SQR_WAV:
			duty_val = readDuty();
			if (RE_val != 0 || duty_val != old_duty_val || is_state_changed) {
				old_duty_val = duty_val;
				
				freq += RE_val;
				
				// 周波数インデックスを 0..CYCLE_TABLE_ELEMENTS に制限
				freq = freq < 0 ? 0 : (freq >= CYCLE_TABLE_ELEMENTS ? CYCLE_TABLE_ELEMENTS - 1 : freq);
				
				cycle = cycle_table[freq];
				
				// デューティー比の計算 
				// 1..cycle に制限
				duty = ((uint32_t)cycle * duty_val >> 7);
				if (duty== 0)
					duty = 1;
				
				timer1_set_cycle_duty(cycle, duty);
				
				// LCDに表示
				I2C_LCD_setpos(0, 1);
				sprintf(LCD_line, "F:%7ld D:%2d", 1000000/cycle, duty_val*100/128);
				I2C_LCD_puts(LCD_line);
			}
			// PWMの有効/無効のチェックと有効化
			if (is_state_changed) {
				timer1_start_PWM();
			}
			break;
		case STAT_IMPULSE:
			if (is_state_changed) {
				timer1_stop_PWM();
			}
			// ワン・ショットでH値を出力
			if (RE_val != 0) {
				PWM_PORT |= _BV(PWM_B);
				PWM_PORT &= ~_BV(PWM_B);
			}
			break;
		case STAT_STEP:
			if (is_state_changed) {
				timer1_stop_PWM();
			}
			// Hi/Loの切り替え
			if (RE_val != 0) {
				PWM_PORT ^= _BV(PWM_B);
			}
			break;
		case STAT_GND_LEVEL:
			// ToDo: GND/VGNDの切り替え
			// PWMの有効無効は不要？
			break;
		}
		
		is_state_changed = 0;
	}
}