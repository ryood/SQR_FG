/*
 * SQR_FG.c
 *
 * Created: 2015/05/07 15:41:32
 *  Author: gizmo
 * 
 * ATMega328P Fuse Bit: h:D9 l:B7
 *
 * AtmelStudio 6.2
 *
 * 2015.12.02 Ver.2
 */ 

#define F_CPU	16000000UL

#include <stdio.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>	/* for _BV() */
#include <util/delay.h>

#include "SPLC792-I2C.h"

//-----------------------------------------------------------------------------
// PORT ��`
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
// �萔��`
//-----------------------------------------------------------------------------
// ��ԑJ��
//
#define STAT_SQR_WAV	0
#define STAT_PRESCALER	1
#define MAX_STAT		2

const char* state_str[] = {
	"SQR WAV   ",
	"PRESCALER?",
};

// �����ݒ�
//
const uint16_t cycle_table[] = {
	50000, 20000, 10000, 5000, 2000, 1000, 500, 200, 100, 50, 20, 10, 5, 2
};

#define CYCLE_IDX_MAX		(sizeof(cycle_table)/sizeof(uint16_t))
#define CYCLE_IDX_INIT		(5)

// �v���X�P�[��
//
const uint16_t prescaler_table[] = {
	0, 1, 8, 64, 256, 1024
};

#define PRESCALER_1			((0<<CS12)|(0<<CS11)|(1<<CS10))
#define PRESCALER_8			((0<<CS12)|(1<<CS11)|(0<<CS10))
#define PRESCALER_64		((0<<CS12)|(1<<CS11)|(1<<CS10))
#define PRESCALER_256		((1<<CS12)|(0<<CS11)|(0<<CS10))
#define PRESCALER_1024		((1<<CS12)|(0<<CS11)|(1<<CS10))

#define PRESCALER_MAX		(sizeof(prescaler_table)/sizeof(uint16_t))
#define PRESCALER_INIT		(PRESCALER_8)

// �f���[�e�B
//
#define DUTY_INIT			(64)

//-----------------------------------------------------------------------------
// �ш�ϐ�
//-----------------------------------------------------------------------------
volatile int8_t g_cycle_idx;		/* 0 .. CYCLE_IDX_MAX */
volatile uint8_t g_duty;			/* 0 .. 127 */
volatile uint8_t g_prescaler;		/* 0b000 .. 0b111 */

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

void timer1_stop_PWM(void)
{
	TCCR1A = 0;
	TCCR1B = 0;
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
	TCCR1B |= PRESCALER_INIT;
	
	// Test Prescaler
	//TCCR1B |= (0 << CS12) | (1 << CS11) | (1<< CS10);
}

 void timer1_set_PWM(uint16_t cycle_idx, uint8_t duty)
 {
	 uint16_t cycle, duty_cnt;
	
	cycle = cycle_table[cycle_idx];
	
	// �f���[�e�B�[�䂩��f���[�e�B�[�̃J�E���g���v�Z 
	// 1..cycle �ɐ���
	duty_cnt = (((uint32_t)cycle * duty) >> 7);
	if (duty_cnt == 0)
		duty_cnt = 1;
		
	timer1_set_cycle_duty(cycle, duty_cnt);
 }

void timer1_set_prescaler(uint8_t prescaler)
{
	// �v���X�P�[���̃��W�X�^��ݒ�
	TCCR1B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
	TCCR1B |= prescaler;
}

/*------------------------------------------------------------------------/
 * Rotary Encoder
 *
 ------------------------------------------------------------------------*/
// �߂�l: ���[�^���[�G���R�[�_�[�̉�]����
//        0:�ω��Ȃ� 1:���v��� -1:�����v���
//
int8_t readRE_ROT(void)
{
	static uint8_t index;
	int8_t retVal = 0;
	
	index = (index << 2) | (RE_PIN & _BV(RE_A)) | (RE_PIN & _BV(RE_B));
	index &= 0b1111;
	
	switch (index) {
		// ���v���
		case 0b0001:	// 00 -> 01
		case 0b1110:	// 11 -> 10
		retVal = 1;
		break;
		// �����v���
		case 0b0010:	// 00 -> 10
		case 0b1101:	// 11 -> 01
		retVal = -1;
		break;
	}
	
	//_delay_ms(2);	// (�Ƃ肠����)�`���^�����O�h�~
	
	return retVal;
}

#define RE_SW_PRESS	(~(RE_PIN)&_BV(RE_SW))

// �߂�l: ���[�^���[�G���R�[�_�[�̃v�b�V���X�C�b�`
//        0:������Ă��Ȃ� 1:������Ă���
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

//------------------------------------------------------------------------
// �������擾
//
void read_cycle_idx()
{
	g_cycle_idx += readRE_ROT();

	// �����C���f�b�N�X�� 0..CYCLE_TABLE_ELEMENTS �ɐ���
	if (g_cycle_idx < 0) {
		g_cycle_idx = 0;
	} else if (g_cycle_idx >= CYCLE_IDX_MAX) {
		g_cycle_idx = CYCLE_IDX_MAX - 1;
	}
}

//------------------------------------------------------------------------
// �v���X�P�[�����擾
//
void read_prescaler()
{
	g_prescaler += readRE_ROT();

	// �v���X�P�[���� 1..PRESCALER_MAX�ɐ���
	if (g_prescaler < 1) {
		g_prescaler = 1;
	} else if (g_prescaler >= PRESCALER_MAX) {
		g_prescaler = PRESCALER_MAX - 1;
	}
}

/*------------------------------------------------------------------------/
 * Potentiometer
 *
 ------------------------------------------------------------------------*/
void adc_init(void)
{
	// ADC�g�p�s�����f�W�^�����͋֎~ PC0, PC1
	DIDR0 |= _BV(ADC0D) | _BV(ADC1D);
	
	// ADC�̊�d��(AVCC)
	ADMUX =	_BV(REFS0);
	
	// ADC����ݒ�
	ADCSRA = 0b10000111		// bit2-0: 111 = 128����	16MHz/128=125kHz (50k�`200kHz�ł��邱��)
		| _BV(ADSC);		// 1��ڕϊ��J�n(����)
}

// �߂�l: ADC��8bit�ǂݎ��l�iPOT�̉�]�j
//         8bit (0..255)
//
uint8_t adc_convert8(uint8_t channel)
{
	ADMUX = _BV(REFS0)		// ADC�̊�d��(AVCC)
		| _BV(ADLAR)		// ���l��
		| channel;			// AD�ϊ��`�����l��
	
	ADCSRA |= _BV(ADSC);	// �ϊ��J�n
	
	loop_until_bit_is_set(ADCSRA, ADIF); // �ϊ������܂ő҂�
	
	return ADCH;
}

//------------------------------------------------------------------------
// �f���[�e�B�[���擾
//
void read_duty(void)
{
	// 8bit -> 7bit
	g_duty = adc_convert8(POT_DUTY) >> 1;
}

/*------------------------------------------------------------------------/
 * main routine
 *
 ------------------------------------------------------------------------*/
uint32_t calc_freq(int8_t cycle_idx, uint8_t prescaler)
{
	// �����ƃv���X�P�[��������g�����v�Z
	return ((uint32_t)F_CPU / 2) / cycle_table[cycle_idx] / prescaler_table[prescaler]; 
}
 
uint16_t calc_freq_fraction(int8_t cycle_idx, uint8_t prescaler)
{
	// �����ƃv���X�P�[��������g���̏�����2�����v�Z
	uint32_t integer = calc_freq(cycle_idx, prescaler);
	uint32_t fraction =  ((uint32_t)F_CPU / 2) * 100 / cycle_table[cycle_idx] / prescaler_table[prescaler];
	
	return fraction - (integer * 100);
}

 int main(void)
{
	// ��ԕϐ�
	int8_t state = 0;
	int8_t is_state_changed = 1;
	
	// LCD
	char LCD_line[17];
	
	//-------------------------------------------------------------------
	// Port�̏�����
	//-------------------------------------------------------------------
	// PWM Output
	//
	PWM_DIR |= _BV(PWM_A) | _BV(PWM_B);
	
	// Rotary Encoder
	// PullUp
	RE_PORT |= _BV(RE_A) | _BV(RE_B) | _BV(RE_SW);
	
	//-------------------------------------------------------------------
	// �f�o�C�X�̂̏�����
	//-------------------------------------------------------------------
	// Initialize ADC
	//
	adc_init();

	// Initialize PWM
	//
	g_cycle_idx = CYCLE_IDX_INIT;
	g_duty = DUTY_INIT;
	g_prescaler = PRESCALER_INIT;
	timer1_init_PWM(g_cycle_idx, g_prescaler);
	timer1_set_prescaler(g_prescaler);
	
	// Initialize I2C-LCD
	//
	I2C_LCD_init();
	I2C_LCD_puts("Square Wave");
	I2C_LCD_setpos(1, 1);
	I2C_LCD_puts("Generator");
	_delay_ms(1000);
	
	I2C_LCD_clear();	
	I2C_LCD_puts(state_str[0]);
	
	//-------------------------------------------------------------------
	// ���C���E���[�v
	//-------------------------------------------------------------------
	while(1)
    {
		// state�̑J��
		//
		if (readRE_SW() != 0) {
			state++;
			is_state_changed = 1;
			if (state >= MAX_STAT)
				state = 0;
		}
		
		switch (state) {
		case STAT_SQR_WAV:
			read_cycle_idx();
			read_duty();
			timer1_set_PWM(g_cycle_idx, g_duty);
			// LCD�ɕ\��
			if (is_state_changed) {
				sprintf(LCD_line, "%sP:%4d", state_str[state], prescaler_table[g_prescaler]);
				I2C_LCD_clear();
				I2C_LCD_puts(LCD_line);
			}
			if (g_prescaler < PRESCALER_64) {
				sprintf(LCD_line, "F:%8ld D:%2d ",
					calc_freq(g_cycle_idx, g_prescaler),
					g_duty * 100 / 128
				);
			} else {
				sprintf(LCD_line, "F:%5ld.%02d D:%2d",
					calc_freq(g_cycle_idx, g_prescaler),
					calc_freq_fraction(g_cycle_idx, g_prescaler),
					g_duty * 100 / 128
				);
			}
			I2C_LCD_setpos(0, 1);
			I2C_LCD_puts(LCD_line);
			/*
			sprintf(LCD_line, "%d %u %u ", g_cycle_idx, cycle_table[g_cycle_idx], g_duty);
			I2C_LCD_setpos(0, 1);
			I2C_LCD_puts(LCD_line);
			*/
			break;
		case STAT_PRESCALER:
			// �v���X�P�[�����擾
			read_prescaler();
			// �v���X�P�[����ݒ�
			timer1_set_prescaler(g_prescaler);
			// LCD�ɕ\��
			if (is_state_changed) {
				I2C_LCD_clear();
				I2C_LCD_puts(state_str[state]);
			}
			sprintf(LCD_line, "1/%d   ", prescaler_table[g_prescaler]);
			I2C_LCD_setpos(0, 1);
			I2C_LCD_puts(LCD_line);
			break;
		}
		
		is_state_changed = 0;
		
		_delay_ms(1);
	}
}