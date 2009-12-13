#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/**********************************************
              H a r d w a r e
 **********************************************
               _________
            1 /         |20
   Reset o---|A2     Vcc|---o VCC
 Data_IN o---|D0      B7|---o Block 7
Data_OUT o---|D1  AT  B6|---o Block 6
         o---|A1 tiny B5|---o Block 5
         o---|A0 2313 B4|---o Block 4
   Clock o-D2|Int0    B3|---o Block 3
      PS o-D3|Int1    B2|---o Block 2
   Check o---|D4      B1|---o Block 1
Enable-A o---|D5      B0|---o Block 0
     GND o---|GND_____D6|---o Enable-B
*/

#define BLOCK_A		(0)
#define BLOCK_B		(1)

#define Data_IN		(_BV(PD0))
#define Data_OUT	(_BV(PD1))
#define Check		(_BV(PD4))
#define Enable_A	(_BV(PD5))
#define Enable_B	(_BV(PD6))

#define Signals	GPIOR0
#define SIG_CLOCK	(1<<0)
#define SIG_LATCH	(1<<1)
#define SIG_TIMER	(1<<2)
#define SIG_CHECK	(1<<3)
#define SIG_SAVE	(1<<4)

#define Buffer	GPIOR1
#define DELAY_A		(1<<0)
#define DELAY_B		(1<<1)
#define BUFF_OUT	(1<<6)
#define BUFF_IN		(1<<7)

typedef union {
        uint8_t shift8[2];
        uint16_t shift16;
} shift_t;

register shift_t shifter asm("r2");
static shift_t status;

register uint8_t time_ms asm("r4");
register uint8_t counter_read asm("r5"), counter_ms asm("r6");
static uint8_t timers[2][8];

ISR(INT0_vect, ISR_NAKED) {
	Signals |= SIG_CLOCK;
	if (BUFF_OUT & Buffer) {
		PORTD |= Data_OUT;
		reti();
	} else {
		PORTD &= ~Data_OUT;
		reti();
	}
	reti();
}

ISR(INT1_vect, ISR_NAKED) {
#if 0 // done in IRQ
	Signals |= SIG_LATCH;
#endif
	asm volatile(
		"push r0"		"\n\t"
		"in r0, __SREG__"	"\n\t"
	);
	shifter.shift8[BLOCK_A] = status.shift8[BLOCK_A];
	shifter.shift8[BLOCK_B] = status.shift8[BLOCK_B];
	asm volatile(
		"out __SREG__, r0"	"\n\t"
		"pop r0"		"\n\t"
	);
	reti();
}

ISR(TIMER0_COMPA_vect, ISR_NAKED) {
	asm volatile(
		"push r0"		"\n\t"
		"in r0, __SREG__"	"\n\t"
		"sei"			"\n\t"
	);
	Signals |= SIG_TIMER;

	if (0 == (PIND & Data_IN)) Buffer &= ~BUFF_IN;
	else Buffer |= BUFF_IN;

	counter_read++;
	counter_ms++;
	asm volatile(
		"out __SREG__, r0"	"\n\t"
		"pop r0"		"\n\t"
	);
	reti();
}

void init (void) {
	uint8_t i;

	Signals = 0;
	time_ms = 1;
	counter_read = 0;
	counter_ms = 0;
	status.shift16 = 0x8421;

	for (i=0; i<8; i++)
	{
		timers[BLOCK_A][i] = 0;
		timers[BLOCK_B][i] = 0;
	}
	set_sleep_mode(SLEEP_MODE_IDLE);

	CLKPR	= 0x80; 	/* disable clock divider */
	CLKPR	= 0x00;

	/* configure port A */
// 	DDRA	= 0b00000011;
// 	PORTA	= 0b00000000;

	/* configure port B */
	DDRB	= 0b00000000;	/* all pins are input */
	PORTB	= 0b11111111;	/* activate pull up resistors */

	/* configure port D */
	DDRD	= Data_OUT | Enable_A | Enable_B;
	PORTD	= Check;

	ACSR	|= ACD;		/* disable analog comparator */

	/* initialize timer 0 */
	TCCR0A	= 0b00000010;	/* CTC mode */
	TCCR0B	= 0b00000010;	/* div8 -> timer runs with 1MHz */
	OCR0A	= 10;		/* interrupt every 10us */
	TIMSK	= 0b00000001;	/* enable OCF0A interrupt */

	MCUCR	= 0b00001111;	/* configure raising edge interrupt on INT0 and INT1 */

	GIMSK	= 0b11000000;	/* enable INT0 and INT1 */

	sei();
}

void do_shifting() {
	shifter.shift16 = shifter.shift16 >> 1;

	if (BUFF_IN & Buffer)
		shifter.shift8[BLOCK_B] |= 0x80;
}

void do_timer() {
	uint8_t i;
	if (10 <= counter_read) { /* 10us * 10 = 100us */
		counter_read = 0;
		Signals |= SIG_CHECK;
	}

	if (100 <= counter_ms) { /* 10us * 100 = 1000us */
		PORTA &= ~_BV(0);
		counter_ms -= 100;
		time_ms++;
		for (i=0; i<8; i++)
		{
			if (DELAY_A & Buffer) timers[BLOCK_A][i]++;
			if (DELAY_B & Buffer) timers[BLOCK_B][i]++;
		}
		Buffer |= DELAY_A;
		Buffer |= DELAY_B;
	}
}

void do_check_blocks() {
	static uint8_t AorB = BLOCK_A;
	uint8_t inputb, inputd, ltime, i, mask;

	ltime = time_ms - 1;
	inputb = PINB;
	inputd = PIND;

	for (mask=1, i=0; i<8; i++, mask=mask<<1)
	{
		if (0 == (inputb & mask)) /* low -> block occupied */
		{
			timers[AorB][i] = ltime;
			status.shift8[AorB] |= mask;
		}
	}

	if (0 == (inputd & Check)) /* low if power supply available */
	{
		if (BLOCK_A == AorB) Buffer &= ~DELAY_A;
		else Buffer &= ~DELAY_B;

		for (mask=1, i=0; i<8; i++, mask=mask<<1)
			if (time_ms == timers[AorB][i]) /* free vacant blocks */
				status.shift8[AorB] &= ~mask;
	}

	AorB ^= 1; /* switch blocks */
	if (BLOCK_A == AorB) {
		PORTD |= Enable_B;	/* Disable_B */
		PORTD &= ~Enable_A;	/* Enable_A */
	} else {
		PORTD |= Enable_A;	/* Disable_A */
		PORTD &= ~Enable_B;	/* Enable_B */
	}
}

int main(void) {

    init ();

    while (1) { /* loop forever, the interrupts are doing the rest */
	if (0 == Signals)
        	sleep_mode();

	if (SIG_CLOCK & Signals) {
		Signals &= ~SIG_CLOCK;
		do_shifting();
	}

	if (1 & shifter.shift8[0]) Buffer |= BUFF_OUT;
	else Buffer &= ~BUFF_OUT;

	if (SIG_TIMER & Signals) {
		Signals &= ~SIG_TIMER;
		do_timer();
	}

	if (SIG_CHECK & Signals) {
		Signals &= ~SIG_CHECK;
		do_check_blocks();
	}
    }

    return (0);
}
