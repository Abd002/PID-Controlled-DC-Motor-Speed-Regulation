/*
 * main.c
 *
 * Created on: Feb 12, 2024
 * Author: AbdElRahman Khalifa
 */

/*
 * freq=8M
 * motor has 360 count per revolution
 * need timer1 to get the speed of motor every .125 s and update control unit u
 * controll unite updates in ISR of Timer1
 *
 * wait around 10 secs to get the target
 */



#include<avr/io.h>
#include<util/delay.h>
#include<stdlib.h>
#include<avr/interrupt.h>
#include<math.h>

#define LCD_RS_PIN_ID 7
#define LCD_E_PIN_ID 6

void timer0_PWM_init(int);

void timer1_COC_init(void);

void MOTOR_init(void);

void INT0_init(void);

void PID(void);

void LCD_init(void);
void LCD_sendCommand(char);
void LCD_displayCharacter(char);
void LCD_intgerToString(int);
void LCD_moveCursor(char, char);

int counterEncoder = 0;
/*
 * 70 -> 90
 * ?  -> x
*/
float targetRPM = 50;
float RPM;
float ePrev = 0, integrationOfError = 0, deltaTime = 1 / 8.0;

int main(void){
	timer0_PWM_init(0);							/* speed of motor */
	timer1_COC_init();
	LCD_init();
	MOTOR_init();
	INT0_init();

	targetRPM=(targetRPM*70)/90.0;

	while(1);
}

void PID(void) {
	//float Kp = 2, Kd = .01, Ki = .5, derevativeOfError, e, u;

	float Kp = 1, Kd = .01, Ki = .5, derevativeOfError, e, u;

	e = -(RPM - targetRPM);
	derevativeOfError = ((e - ePrev) / deltaTime);
	integrationOfError += e * deltaTime;
	u = Kp * e + Kd * derevativeOfError + Ki * integrationOfError;

	u = fabs(u);

	LCD_moveCursor(0, 0);
	LCD_intgerToString((RPM * 90) / 70.0);
	LCD_displayString("  ");
	/*
	 * 70 -> 255
	 * x  -> ?
	 */
	u = (u * 255) / 70.0;
	if (u >= 256) {
		u = 255;
	}

	OCR0 = u; /* Update of PWM */

	ePrev = e;
}

ISR(INT0_vect){
	counterEncoder++;
	if(PIND&(1<<PD3)){
		/* ACW */

	}else {
		/* CW */

	}

}

/* interuupt every 1 sec */
ISR(TIMER1_COMPA_vect){

	RPM=(counterEncoder/360.0)*(60.0/deltaTime);

	counterEncoder=0;

	/* PID Controller */
	PID();
}

void INT0_init(void){


	MCUCR|=(1<<ISC01)|(1<<ISC00); 								/* enable INT0 at rising edge */

	GICR|=(1<<INT0);											/* enable INT0  */

	sei();														/* enable avr global interrupts */
}

void timer1_COC_init(void){
	TCCR1A =(1<<FOC1A);											/* non-PWM mode */
	TCCR1B=(1<<WGM12)|(1<<CS12);								/* CTC mode OCR1A TOP & prescaler 256 */
	/*
	 * freq = 8 MHZ
	 * 8 Mhz / 256 = 31250 Hz
	 * T = (1/31250)
	 * we want to get interrupt every 1 sec so (1 sec / T) = 31250 at which compare shoul occurred
	*/
	OCR1A=31250/8;

	TIMSK=(1<<OCIE1A);											/*  Output Compare A Match Interrupt Enable */
}

void timer0_PWM_init(int number){
	DDRB|=(1<<PB3);												/* configure PB3 as output for PWM gen */
	TCCR0=(1<<WGM01)|(1<<WGM00)|(1<<COM01)|(1<<CS01)|(1<<CS00);	/* DC motor with 500 HZ so prescaler will be 64 */
	TCNT0=0;
	OCR0=number;												/*  */
}

void MOTOR_init(void){
	DDRC|=(0x03);												/* configuration of IN1 and IN2 of motor as outputs*/
	PORTC|=(0x01);												/* MOTOR mode -> CW */
	DDRD&=~(0x06);												/* configuration of encoder pins as inputs */
}

void LCD_init(void){
	DDRA|=0xff;													/* configuration of LCD data pins as output */
	DDRB|=(1<<PB6)|(1<<PB7);									/* configuration of RS and E of LCD as output pins */
	_delay_ms(20); 		  										/* LCD Power ON delay always > 15ms */
	LCD_sendCommand(0X38);										/* 2-lines + 8bit mode  */
	LCD_sendCommand(0x01);										/* clear screen */
	LCD_sendCommand(0x0c);										/* cursor off */
}



void LCD_sendCommand(char command){
	PORTB&=~(1<<LCD_RS_PIN_ID);									/* RS = 0 ; send command mode */
	_delay_ms(1);
	PORTB|=(1<<LCD_E_PIN_ID);									/* E=1  */
	_delay_ms(1);
	PORTA=command;												/* send command */
	_delay_ms(1);
	PORTB&=~(1<<LCD_E_PIN_ID);									/* E=0  */
	_delay_ms(1);
}

void LCD_displayCharacter(char data){
	PORTB|=(1<<LCD_RS_PIN_ID);									/* RS = 1 ; send data mode */
	_delay_ms(1);
	PORTB|=(1<<LCD_E_PIN_ID);									/* E=1  */
	_delay_ms(1);
	PORTA=data;													/* send data */
	_delay_ms(1);
	PORTB&=~(1<<LCD_E_PIN_ID);									/* E=0  */
	_delay_ms(1);
}

void LCD_displayString(const char *str_ptr){
	while(*str_ptr != '\0'){
		LCD_displayCharacter(*str_ptr);
		str_ptr++;
	}
}

void LCD_intgerToString(int number){
	char buff[18];
	itoa(number, buff, 10);
	LCD_displayString(buff);
}
/* move cursor base 0 */
void LCD_moveCursor(char row, char col){
	int location;
	switch(row){
	case 0:
		location=0x00+col;
		break;
	case 1:
		location=0x40+col;
		break;
	case 2:
		location=0x10+col;
		break;
	case 3:
		location=0x50+col;
		break;
	default:
		break;
	}
	LCD_sendCommand(location|0x80);
}
