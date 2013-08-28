/* GBM16smd (c) 2009-2012 by Hans-Peter Bock <hpbock@avaapgh.de>
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
   Check o---|A1 tiny B5|---o Block 5
Enable-B o---|A0 2313 B4|---o Block 4
   Clock o-D2|Int0    B3|---o Block 3
 LOAD/PS o-D3|Int1    B2|---o Block 2
Enable-C o---|D4      B1|---o Block 1
Enable-D o---|D5      B0|---o Block 0
     GND o---|GND_____D6|---o Enable-A
*/

char string_1[] PROGMEM = "(c) 2009-2012 Hans-Peter Bock <hpbock@avaapgh.de>";

#define CYCLETIME	(64)	/* cycletime in us */

#define MS_CYCLES	(1024/CYCLETIME) /* 1024us ~ 1ms */
#define WAIT_CYCLES	(256/CYCLETIME) /* Wait between block switch and read. */

#define BLOCK_A		(0)
#define BLOCK_B		(1)
#define BLOCK_C		(2)
#define BLOCK_D		(3)

#define Data_IN		(_BV(PD0))
#define Data_OUT	(_BV(PD1))
#define CLOCK		(_BV(PD2))
#define LOAD		(_BV(PD3))
#define Enable_C	(_BV(PD4))
#define Enable_A	(_BV(PD5))
#define Enable_B	(_BV(PD6))

#define Enable_D	(_BV(PA0))
#define Check		(_BV(PA1))


#define Signals		GPIOR0
#define SIG_TIMER	(1<<0)
#define SIG_CHECK	(1<<1)
// #define SIG_SAVE_A	(1<<6)
// #define SIG_SAVE_B	(1<<7)

#define Buffer		GPIOR1
#define DELAY_A		(1<<0)
#define DELAY_B		(1<<1)
#define DELAY_C		(1<<2)
#define DELAY_D		(1<<3)
#define BUFF_IN		(1<<7)

typedef union
{
        uint8_t shift8[4];
        uint32_t shift32;
} shift_t;

register volatile shift_t shifter asm("r2");
static shift_t status;

register volatile uint8_t time_ms asm("r6");
register volatile uint8_t counter_read asm("r7"), counter_ms asm("r8");
static uint8_t timers[4][8];

void do_shifting() 
{
	shifter.shift32 = shifter.shift32 >> 1;

	if (BUFF_IN & Buffer)
		shifter.shift8[BLOCK_D] |= 0x80;
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
	shifter.shift8[BLOCK_C] = status.shift8[BLOCK_C];
	shifter.shift8[BLOCK_D] = status.shift8[BLOCK_D];
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
	status.shift8[1]= 'a';
	status.shift8[2]= 'P';
	status.shift8[3]= 'e';

	for (i=0; i<8; i++)
	{
		timers[BLOCK_A][i] = 0;
		timers[BLOCK_B][i] = 0;
		timers[BLOCK_C][i] = 0;
		timers[BLOCK_D][i] = 0;
	}
	set_sleep_mode(SLEEP_MODE_IDLE);

	CLKPR	= 0x80; 	/* disable clock divider */
	CLKPR	= 0x00;

	/* configure port A */
 	DDRA	= Enable_D;
 	PORTA	= Check;

	/* configure port B */
	DDRB	= 0b00000000;	/* all pins are input */
	PORTB	= 0b11111111;	/* activate pull up resistors */

	/* configure port D */
	DDRD	= Data_OUT | Enable_A | Enable_B | Enable_C;
	PORTD	= Data_IN;

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
//		cli(); counter_ms -= MS_CYCLES; sei();
		counter_ms -= MS_CYCLES; // with volatile definition, this is atomic
		time_ms++;
		for (i=0; i<8; i++)
		{
			if (DELAY_A & Buffer) timers[BLOCK_A][i]++;
			if (DELAY_B & Buffer) timers[BLOCK_B][i]++;
			if (DELAY_C & Buffer) timers[BLOCK_C][i]++;
			if (DELAY_D & Buffer) timers[BLOCK_D][i]++;
		}
		Buffer |= DELAY_A;
		Buffer |= DELAY_B;
		Buffer |= DELAY_C;
		Buffer |= DELAY_D;
	}
}

void do_check_blocks()
{
	static uint8_t ABCorD = BLOCK_A;
	uint8_t inputa, inputb, inputd, ltime, i, mask;

	ltime = time_ms - 1;
	inputa = PINA;
	inputb = PINB;
	inputd = PIND;

	for (mask=1, i=0; i<8; i++, mask=mask<<1)
	{
		if (0 == (inputb & mask)) /* low -> block occupied */
		{
			timers[ABCorD][i] = ltime;
			status.shift8[ABCorD] |= mask;
		}
	}

	if (0 == (inputa & Check)) /* low if power supply available */
	{
		if (BLOCK_A == ABCorD) Buffer &= ~DELAY_A;
		else if (BLOCK_B == ABCorD) Buffer &= ~DELAY_B;
		else if (BLOCK_C == ABCorD) Buffer &= ~DELAY_C;
		else Buffer &= ~DELAY_D;

		for (mask=1, i=0; i<8; i++, mask=mask<<1)
			if (time_ms == timers[ABCorD][i]) /* free vacant blocks */
				status.shift8[ABCorD] &= ~mask;
	}

	ABCorD = (ABCorD + 1) % 4; /* switch blocks */
	PORTD |= Enable_A;	/* Disable_A */
	PORTD |= Enable_B;	/* Disable_B */
	PORTD |= Enable_C;	/* Disable_C */
	PORTA |= Enable_D;	/* Disable_D */
	if (BLOCK_A == ABCorD) PORTD &= ~Enable_A;	/* Enable_A */
	else if (BLOCK_B == ABCorD) PORTD &= ~Enable_B;	/* Enable_B */
	else if (BLOCK_C == ABCorD) PORTD &= ~Enable_C;	/* Enable_C */
	else PORTA &= ~Enable_D;			/* Enable_D */
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
