/* GBM16smd (c) 2009-2010 by Hans-Peter Bock <hpbock@avaapgh.de>
 *
 * This source file is subject of the GNU general public license 2,
 * that is available at the world-wide-web at
 * http://www.gnu.org/licenses/gpl.txt
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <stdint.h>

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
 LOAD/PS o-D3|Int1    B2|---o Block 2
   Check o---|D4      B1|---o Block 1
Enable-B o---|D5      B0|---o Block 0
     GND o---|GND_____D6|---o Enable-A
*/

char string_1[] PROGMEM = "(c) 2009-2010 Hans-Peter Bock <hpbock@avaapgh.de>";

#define CYCLETIME	(64)	/* cycletime in us */

#define MS_CYCLES	(1024/CYCLETIME) /* 1024us ~ 1ms */
#define WAIT_CYCLES	(256/CYCLETIME) /* Wait between block switch and read. */

#define BLOCK_A		(0)
#define BLOCK_B		(1)

#define Data_IN		(_BV(PD0))
#define Data_OUT	(_BV(PD1))
#define CLOCK		(_BV(PD2))
#define LOAD		(_BV(PD3))
#define Check		(_BV(PD4))
#define Enable_A	(_BV(PD6))
#define Enable_B	(_BV(PD5))

#define Signals		GPIOR0
#define SIG_TIMER	(1<<0)
#define SIG_CHECK	(1<<1)
// #define SIG_SAVE_A	(1<<6)
// #define SIG_SAVE_B	(1<<7)

#define Buffer		GPIOR1
#define DELAY_A		(1<<0)
#define DELAY_B		(1<<1)
#define BUFF_IN		(1<<7)

typedef union
{
        uint8_t shift8[2];
        uint16_t shift16;
} shift_t;

register volatile shift_t shifter asm("r2");
static shift_t status;

register volatile uint8_t time_ms asm("r4");
register volatile uint8_t counter_read asm("r5"), counter_ms asm("r6");
static uint8_t timers[2][8];

void do_shifting() 
{
	shifter.shift16 = shifter.shift16 >> 1;

	if (BUFF_IN & Buffer)
		shifter.shift8[BLOCK_B] |= 0x80;
}

ISR(INT0_vect)
{
	if (CLOCK & PIND) // low->high: latch input
	{
		if (PIND & Data_IN)
			Buffer |= BUFF_IN;
		else
			Buffer &= ~BUFF_IN;

		if (0 == (LOAD & PIND)) // only shift if LOAD is low
			do_shifting();
		
	} else { // high->low: latch output
		if (1 & shifter.shift8[0])
			PORTD |= Data_OUT;
		else
			PORTD &= ~Data_OUT;
	}
}

ISR(INT1_vect)
{
	shifter.shift8[BLOCK_A] = status.shift8[BLOCK_A];
	shifter.shift8[BLOCK_B] = status.shift8[BLOCK_B];
}

ISR(TIMER0_COMPA_vect)
{
	sei();
	
	Signals |= SIG_TIMER;

	counter_read++;
	counter_ms++;
}

void init (void)
{
	uint8_t i;

	Signals		= 0;
	time_ms 	= 1;
	counter_read	= 0;
	counter_ms	= 0;
	status.shift8[0]= 'H';
	status.shift8[1]= 'P';

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
	PORTD	= Check | Data_IN;

	ACSR	|= ACD;		/* disable analog comparator */

	/* initialize timer 0 */
	TCCR0A	= 0b00000010;	/* CTC mode */
	TCCR0B	= 0b00000010;	/* div8 -> timer runs with 1MHz */
	OCR0A	= CYCLETIME;
	TIMSK	= 0b00000001;	/* enable OCF0A interrupt */

	MCUCR	= 0b00001101;	/* configure interrupts INT0 and INT1 */

	GIMSK	= 0b11000000;	/* enable INT0 and INT1 */

	sei();
}


void do_timer()
{
	uint8_t i;
	if ((WAIT_CYCLES) <= counter_read)
	{
		counter_read = 0;
		Signals |= SIG_CHECK;
	}

	if (MS_CYCLES <= counter_ms)
	{
		PORTA &= ~_BV(0);
//		cli(); counter_ms -= MS_CYCLES; sei();
		counter_ms -= MS_CYCLES; // with volatile definition, this is atomic
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

void do_check_blocks()
{
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
	if (BLOCK_A == AorB)
	{
		PORTD |= Enable_B;	/* Disable_B */
		PORTD &= ~Enable_A;	/* Enable_A */
	} else {
		PORTD |= Enable_A;	/* Disable_A */
		PORTD &= ~Enable_B;	/* Enable_B */
	}
}

int main(void)
{
    init ();

    while (1) /* loop forever */
    {
	if (0 == Signals)
        	sleep_mode();

	if (SIG_TIMER & Signals)
	{
		Signals &= ~SIG_TIMER;
		do_timer();
	}

	if (SIG_CHECK & Signals)
	{
		Signals &= ~SIG_CHECK;
		do_check_blocks();
	}
    }

    return (0);
}
