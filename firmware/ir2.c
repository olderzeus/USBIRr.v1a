/*
 * $Id: ir2.c,v 1.12 2009/01/16 13:08:48 fvecoven Exp $
 */

#include <p18cxxx.h>
#include <capture.h>
#include <timers.h>
#include "stdint.h"
#include "ir2.h"

/* globals */
ir_flags_t ir_flags;
ir_t ir;

/* static prototypes */
static void ir_reset(void);
static void ir_reset_all(void);
static void ir_decode(void);

/* prototypes and globals per protocol */
#if IR_RAW_SUPPORT == 1
static uint8_t ir_raw[IR_RAW_SIZE];
static uint8_t ir_ptr;
#endif
#if IR_RC5_SUPPORT == 1
rc5_t rc5;
static void rc5_decode(void);
static void rc5_shift(void);
#endif
#if IR_RC6_SUPPORT == 1
rc6_t rc6;
static void rc6_decode(void);
static void rc6_shift(void);
#endif
#if IR_NEC_SUPPORT == 1
nec_t nec;
static void nec_decode(void);
#endif
#if IR_JVC_SUPPORT == 1
jvc_t jvc;
static void jvc_decode(void);
#endif
#if IR_SONY_SUPPORT == 1
sony_t sony;
static void sony_decode(void);
#endif
#if IR_SHARP_SUPPORT == 1
sharp_t sharp;
static void sharp_decode(void);
#endif
#if IR_PACE_SUPPORT == 1
pace_t pace;
static void pace_decode(void);
#endif


/* temp variables needed by some protocol handlers */
#if IR_SHARP_SUPPORT == 1 || IR_RC6_SUPPORT == 1
static uint8_t temp0;
#endif


/*
 * ir_init : this function must be called to initialize the library.
 */
void
ir_init(void)
{
	/* configure timer 1 */
	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT &
		T1_PS_1_1 & T1_SOURCE_CCP);
	T1CONbits.TMR1ON = 0;
	TMR1H = T1_RELOAD_H;
	TMR1L = T1_RELOAD_L;

	/* configure capture unit */
	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE);
	ir_flags.edge = 0;

	/* configure interrupts */
	PIE1bits.TMR1IE = 1;
	INTCONbits.PEIE = 1;

	/* init data */
	ir_reset();
}


/*
 * ir_enable : enable ir reception
 */
void
ir_enable(void)
{
	ir_reset();
	ir_reset_all();
	ir_flags.decoded = 0;
	PIE1bits.CCP1IE = 1;
}


/*
 * ir_disable : disable ir reception
 */
void
ir_disable(void)
{
	PIE1bits.CCP1IE = 0;
	T1CONbits.TMR1ON = 0;
}



/*
 * ir_interrupt : this function must be called when the capture interrupt
 *                occurs.
 */
void
ir_interrupt(void)
{
	/* reload timer 1 */
	TMR1H = T1_RELOAD_H;
	TMR1L = T1_RELOAD_L;

	/* change edge */
	if (ir_flags.edge) {
		OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE);
		ir_flags.edge = 0;
	} else {
		OpenCapture1(CAPTURE_INT_ON & C1_EVERY_RISE_EDGE);
		ir_flags.edge = 1;
	}

	if (ir_flags.first) {
		/* This is the first edge. Start timer1 */
		T1CONbits.TMR1ON = 1;
#if IR_RAW_SUPPORT == 1
		ir_ptr = 0;
#endif
		ir_decode();

		ir_flags.first = 0;
		return;
	}

#if IR_RAW_SUPPORT == 1
	ir_raw[ir_ptr] = CCPR1H;	// ir_raw in multiple of 64us
	ir_ptr++;
	if (ir_ptr == IR_RAW_SIZE - 1) {
		ir_ptr = 0;
		//ir_timeout();
	}
#endif

	ir_decode();
}


/*
 * ir_timeout : this function is called when timer1 overflows.
 */
void
ir_timeout(void)
{
#if IR_RAW_SUPPORT == 1
	ir_raw[ir_ptr] = 0xff;
#endif
	ir_flags.timeout = 1;

	/* stop timer 1 */
	T1CONbits.TMR1ON = 0;

	/* decode what we got ! */
	ir_decode();

	if (ir_flags.decoded) {
		/* wait for the caller to re-enable reception */
		ir_disable();
	} else {
		Nop();
	}

	/* reset capture edge */
	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE);
	ir_flags.edge = 0;

	ir_reset();
}



static void
ir_reset(void)
{
	ir_flags.edge = 0;
	ir_flags.timeout = 0;
	ir_flags.first = 1;
}


/* reset data for various protocol (if needed) */
static void
ir_reset_all(void)
{
#if IR_SHARP_SUPPORT == 1
	sharp.flags.second = 0;
#endif
}


static void
ir_decode(void)
{
#if IR_RC5_SUPPORT == 1
	rc5_decode();
#endif
#if IR_RC6_SUPPORT == 1
	rc6_decode();
#endif
#if IR_NEC_SUPPORT == 1
	nec_decode();
#endif
#if IR_JVC_SUPPORT == 1
	jvc_decode();
#endif
#if IR_SONY_SUPPORT == 1
	sony_decode();
#endif
#if IR_SHARP_SUPPORT == 1
	sharp_decode();
#endif
#if IR_PACE_SUPPORT == 1
	pace_decode();
#endif
}


#if IR_RC5_SUPPORT == 1
static void
rc5_decode(void)
{
	if (ir_flags.first) {
		rc5.flags.raw = 0;
		rc5.data = 0x0005;	// we use rc5.data as a bit counter
		return;
	}

	if (rc5.flags.invalid)
		return;

	if (rc5.flags.last2) {
		rc5.flags.last2 = 0;
		return;
	}

	if (rc5.flags.last) {
		if (ir_flags.timeout) {
			// confirm valid data !
			ir_flags.decoded = 1;
		} else {
			rc5.flags.invalid = 1;
		}
		return;
	}

	if (CCPR1H < RC5_SHORT_MIN || CCPR1H > RC5_LONG_MAX) {
		rc5.flags.invalid = 1;
		return;
	}

	if (CCPR1H <= RC5_SHORT_MAX) {
		// short pulse or gap
		if (rc5.flags.x) {
			rc5.flags.same = 1;
			rc5_shift();
			rc5.flags.x = 0;
		} else {
			rc5.flags.x = 1;
		}

	} else if (CCPR1H >= RC5_LONG_MIN) {
		// double pulse or gap
		if (rc5.flags.x) {
			rc5.flags.invalid = 1;
			return;
		}
		rc5.flags.same = 0;
		rc5_shift();

	} else {
		// invalid pulse
		rc5.flags.invalid = 1;
		return;
	}
}


static void
rc5_shift(void)
{
	uint16_t value;

	value = rc5.data & 0x1;
	if (!rc5.flags.same) value ^= 0x1;
	rc5.data = rc5.data << 1;
	rc5.data |= value;

	// check if we are done. rc5.data is used as bit counter
	if (rc5.data & 0x8000) {
		ir.type = IR_RC5;
		ir.command = (uint8_t)(rc5.data & 0x3f);
		ir.address = (rc5.data >> 6) & 0x1f;
		ir.extra = 0;
		if (rc5.data & 0x800) {
			ir.extra |= 0x80;
		} 
		rc5.flags.last = 1;
		if (ir.command & 0x1) rc5.flags.last2 = 1;
	}
}

#endif	// IR_RC5_SUPPORT


#if IR_RC6_SUPPORT == 1
static void
rc6_decode(void)
{
	if (ir_flags.first) {
		rc6.data = 0x1;
		rc6.bitcount = 0;
		rc6.flags.raw = 0;
		rc6.flags.x = 1;
		rc6.flags.first = 1;	// next : first pulse
		return;
	}

	if (rc6.flags.invalid)
		return;

	if (rc6.flags.last2) {
		rc6.flags.last2 = 0;
		return;
	}

	if (rc6.flags.last) {
		// we should see timeout or sleep time (2.6ms)
		if (ir_flags.timeout || CCPR1H > RC6_FIRST_MIN) {
			// valid data !
			ir_flags.decoded = 1;
		} else {
			rc6.flags.invalid = 1;
		}
		return;
	}

	if (rc6.flags.first) {
		if (CCPR1H < RC6_FIRST_MIN || CCPR1H > RC6_FIRST_MAX) {
			rc6.flags.invalid = 1;
		} else {
			rc6.flags.first = 0;
			rc6.flags.second = 1;
		}
		return;
	}

	if (rc6.flags.second) {
		if (CCPR1H < RC6_SECOND_MIN || CCPR1H > RC6_SECOND_MAX) {
			rc6.flags.invalid = 1;
		} else {
			rc6.flags.second = 0;
		}
		return;
	}

	// check that pulse is not too short
	if (CCPR1H < RC6_SHORT_MIN) {
		rc6.flags.invalid = 1;
		return;
	}

	// check that pulse is not too long
	temp0 = RC6_LONG_MAX;
	if (rc6.bitcount == 4 || rc6.bitcount == 5) {
		temp0 = temp0 << 1;
	}
	if (CCPR1H > temp0) {
		rc6.flags.invalid = 1;
		return;
	}

	// find pulse width : short or long
	temp0 = RC6_SHORT_MAX;
	if (rc6.bitcount == 4 || rc6.bitcount == 5) {
		temp0 = temp0 << 1;
	}

	if (CCPR1H <= temp0) {
		// short pulse or gap
		if (rc6.flags.x) {
			rc6.flags.same = 1;
			rc6_shift();
			rc6.flags.x = 0;
		} else {
			rc6.flags.x = 1;
		}

	} else if (CCPR1H >= RC6_LONG_MIN) {
		// double pulse or gap
		if (rc6.flags.x) {
			rc6.flags.invalid = 1;
			return;
		}
		rc6.flags.same = 0;
		rc6_shift();

	} else {
		// invalid pulse
		rc6.flags.invalid = 1;
		return;
	}
}


static void
rc6_shift(void)
{
	uint24_t value;

	value = rc6.data & 0x1;
	if (!rc6.flags.same) value ^= 0x1;
	rc6.data = rc6.data << 1;
	rc6.data |= value;

	rc6.bitcount++;
	if (rc6.bitcount == 21) {
		ir.type = IR_RC6;
		ir.command = (uint8_t)(rc6.data & 0xff);
		ir.address = (uint16_t)((rc6.data >> 8) & 0xff);
		ir.extra = (uint8_t)((rc6.data >> 17) & 0x7);
		if (rc6.data & 0x10000) {
			ir.extra |= 0x80;
		}
		rc6.flags.last = 1;
		if ((ir.command & 0x1) == 0) rc6.flags.last2 = 1;
	}
}

#endif	// IR_RC6_SUPPORT



#if IR_JVC_SUPPORT == 1

static void
jvc_decode(void)
{
	if (ir_flags.first) {
		jvc.flags.raw = 0;
		jvc.flags.first = 1;
		jvc.bitcount = 0;
		jvc.data = 0;
		return;
	}

	if (jvc.flags.invalid)
		return;

	if (jvc.flags.last) {
		if (ir_flags.timeout || CCPR1H > JVC_TIMEOUT) {
			// valid JVC !
			ir.type = IR_JVC;
			ir.address = jvc.data & 0xff;
			ir.command = (uint8_t)(jvc.data >> 8);
			ir.extra = 0;
			ir_flags.decoded = 1;
		} else {
			jvc.flags.invalid = 1;
		}
		return;
	}

	if (jvc.flags.first) {
		if (CCPR1H < JVC_FIRST_MIN || CCPR1H > JVC_FIRST_MAX) {
			jvc.flags.invalid = 1;
		} else {
			jvc.flags.first = 0;
			jvc.flags.second = 1;
		}
		return;
	}

	if (jvc.flags.second) {
		if (CCPR1H < JVC_SECOND_MIN || CCPR1H > JVC_SECOND_MAX) {
			jvc.flags.invalid = 1;
		} else {
			jvc.flags.second = 0;
			jvc.flags.pulse = 1;
		}
		return;
	}

	if (CCPR1H < JVC_SHORT_MIN) {
		jvc.flags.invalid = 1;
		return;
	}

	if (jvc.flags.pulse) {
		if (CCPR1H > JVC_SHORT_MAX) {
			jvc.flags.invalid = 1;
			return;
		}
		jvc.flags.pulse = 0;
		if (jvc.bitcount == 16) {
			jvc.flags.last = 1;
		}
		return;
	}

	jvc.flags.pulse = 1;

	if (CCPR1H > JVC_LONG_MAX) {
		jvc.flags.invalid = 1;
		return;
	}

	jvc.data = jvc.data >> 1;
	if (CCPR1H >= JVC_LONG_MIN) {
		jvc.data |= 0x8000;
	}
	jvc.bitcount++;
}

#endif	// JVC_SUPPORT


#if IR_NEC_SUPPORT == 1

static void
nec_decode(void)
{
	if (ir_flags.first) {
		nec.flags.raw = 0;
		nec.flags.first = 1;
		nec.bitcount = 0;
		nec.data = 0;
		return;
	}

	if (nec.flags.invalid)
		return;

	if (nec.flags.last) {
		if (ir_flags.timeout || CCPR1H > NEC_TIMEOUT) {
			// valid NEC !
			ir_flags.decoded = 1;
		} else {
			nec.flags.invalid = 1;
		}
		return;
	}

	if (nec.flags.first) {
		if (CCPR1H < NEC_FIRST_MIN || CCPR1H > NEC_FIRST_MAX) {
			nec.flags.invalid = 1;
		} else {
			nec.flags.first = 0;
			nec.flags.second = 1;
		}
		return;
	}

	if (nec.flags.second) {
		if (CCPR1H < NEC_SECOND_MIN || CCPR1H > NEC_SECOND_MAX) {
			nec.flags.invalid = 1;
		} else {
			nec.flags.second = 0;
			nec.flags.pulse = 1;
		}
		return;
	}

	if (CCPR1H < NEC_SHORT_MIN) {
		nec.flags.invalid = 1;
		return;
	}

	if (nec.flags.pulse) {
		if (CCPR1H > NEC_SHORT_MAX) {
			nec.flags.invalid = 1;
			return;
		}
		nec.flags.pulse = 0;
		if (nec.bitcount == 32) {
			ir.address = (uint16_t)(nec.data & 0xffff);
			if ((ir.address & 0xff) == (~ir.address >> 8)) {
				ir.type = IR_NEC;
				ir.address &= 0xff;
			} else {
				ir.type = IR_NEC_EXTENDED;
			}
			ir.command = (uint8_t)(nec.data >> 24);
			ir.extra = 0;
			nec.flags.last = 1;
		}
		return;
	}

	nec.flags.pulse = 1;

	if (CCPR1H > NEC_LONG_MAX) {
		nec.flags.invalid = 1;
		return;
	}

	nec.data = nec.data >> 1;
	if (CCPR1H >= NEC_LONG_MIN) {
		nec.data |= 0x80000000;
	}
	nec.bitcount++;
}

#endif	// NEC_SUPPORT


#if IR_SONY_SUPPORT == 1

static void
sony_decode(void)
{
	if (ir_flags.first) {
		sony.flags.raw = 0;
		sony.flags.first = 1;
		sony.bitcount = 0;
		sony.data = 0;
		return;
	}

	if (sony.flags.invalid)
		return;

	if (sony.flags.last) {
		if (ir_flags.timeout || CCPR1H > SONY_TIMEOUT) {
			// valid SONY !
			if (sony.bitcount == 12) {
				sony.data = sony.data >> 12;
				ir.type = IR_SONY_12BITS;
			} else if (sony.bitcount == 15) {
				sony.data = sony.data >> 9;
				ir.type = IR_SONY_15BITS;
			} else {
				sony.data = sony.data >> 4;
				ir.type = IR_SONY_20BITS;
			}
			ir.command = (uint8_t)(sony.data & 0x7f);
			ir.address = (uint16_t)(sony.data >> 7);
			ir.extra = 0;
			ir_flags.decoded = 1;
		} else if (sony.bitcount == 20) {
			sony.flags.invalid = 1;
		}
		return;
	}

	if (sony.flags.first) {
		if (CCPR1H < SONY_FIRST_MIN || CCPR1H > SONY_FIRST_MAX) {
			sony.flags.invalid = 1;
		} else {
			sony.flags.first = 0;
			sony.flags.gap = 1;
		}
		return;
	}

	if (CCPR1H < SONY_SHORT_MIN) {
		sony.flags.invalid = 1;
		return;
	}

	if (sony.flags.gap) {
		if (CCPR1H > SONY_SHORT_MAX) {
			sony.flags.invalid = 1;
			return;
		}
		sony.flags.gap = 0;
		return;
	}

	sony.flags.gap = 1;

	if (CCPR1H > SONY_LONG_MAX) {
		sony.flags.invalid = 1;
		return;
	}

	sony.data = sony.data >> 1;
	if (CCPR1H >= SONY_LONG_MIN) {
		sony.data |= 0x800000;
	}
	sony.bitcount++;

	if (sony.bitcount == 12 || sony.bitcount == 15 || sony.bitcount == 20) {
		sony.flags.last = 1;
	}
}

#endif	// SONY_SUPPORT


#if IR_SHARP_SUPPORT == 1

static void
sharp_decode(void)
{
	if (ir_flags.first) {
		sharp.flags.invalid = 0;
		sharp.flags.last = 0;
		sharp.flags.gap = 0;
		sharp.data = 0x8000;	// used as bit counter
		return;
	}
	
	if (sharp.flags.invalid)
		return;
		
	if (sharp.flags.last) {
		if (ir_flags.timeout) {
			sharp.data = sharp.data >> 1;
			if (sharp.flags.second) {
				// second packet is valid.
				sharp.flags.second = 0;
				
				// address is the same in both packets
				temp0 = sharp.prev & 0x1f;
				ir.address = sharp.data & 0x1f;
				if (ir.address != temp0) {
					sharp.flags.invalid = 1;
					return;
				}
				
				// extra bits are inverted
				temp0 = (uint8_t)(sharp.prev >> 13);
				temp0 = ~temp0 & 0x3;
				ir.extra = (uint8_t)(sharp.data >> 13);
				if (temp0 != ir.extra) {
					sharp.flags.invalid = 1;
					return;
				}
				
				// command is inverted
				temp0 = (uint8_t)(sharp.prev >> 5);
				ir.command = (uint8_t)(sharp.data >> 5);
				if (temp0 != ~ir.command) {
					sharp.flags.invalid = 1;
					return;
				}
				
				// need to pick the non-inverted results !
				if (ir.extra & 0x2) {
					ir.extra = ~ir.extra & 0x3;
					ir.command = temp0;
				}
				ir.type = IR_SHARP;	
				ir_flags.decoded = 1;
			
			} else {
				// first packed is valid
				sharp.prev = sharp.data;
				sharp.flags.second = 1;
			}
		} else {
			sharp.flags.invalid = 1;
			sharp.flags.second = 0;
		}
		return;
	}
	
	if (sharp.flags.gap) {
		if (CCPR1H < SHARP_ZERO_MIN || CCPR1H > SHARP_ONE_MAX) {
			sharp.flags.invalid = 1;
			sharp.flags.second = 0;
			return;
		}
		
		sharp.data = sharp.data >> 1;
		if (CCPR1H >= SHARP_ONE_MIN) {
			sharp.data |= 0x8000;
		} 
	
		sharp.flags.gap = 0;
		
		return;
	} 
	
	if (CCPR1H < SHARP_SHORT_MIN || CCPR1H > SHARP_SHORT_MAX) {
		sharp.flags.invalid = 1;
		sharp.flags.second = 0;
		return;
	}
	
	if (sharp.data & 0x1) {
		sharp.flags.last = 1;
	}
	
	sharp.flags.gap = 1;
}	

#endif	// SHARP_SUPPORT


#if IR_PACE_SUPPORT == 1

static void
pace_decode(void)
{
	if (ir_flags.first) {
		pace.flags.raw = 0;
		pace.flags.first = 1;
		pace.flags.pulse = 1;
		pace.data = 0x1;
		return;
	}

	if (pace.flags.invalid)
		return;

	if (pace.flags.last) {
		if (ir_flags.timeout) {
			// valid PACE !
			ir.type = IR_PACE;
			
			/* 
			 * The following is a guess, since I only have one PACE
			 * remote control in my possession. It looks like 5 bits
			 * are used for command, 4 bits for address, and the last
			 * bit is a toggle (like RC5 and RC6)
			 */ 
			ir.address = (pace.data >> 5) & 0xf;
			ir.command = pace.data & 0x1f;
			ir.extra = (pace.data >> 9) & 0x1;
			ir_flags.decoded = 1;
		} else {
			pace.flags.invalid = 1;
		}
		return;
	}

	if (CCPR1H < PACE_SHORT_MIN) {
		pace.flags.invalid = 1;
		return;
	}

	if (pace.flags.pulse) {
		if (CCPR1H > PACE_SHORT_MAX) {
			pace.flags.invalid = 1;
		} else {
			pace.flags.pulse = 0;
			if (pace.data & 0x400) {
				pace.flags.last = 1;
			}
		}
		return;
	}

	if (pace.flags.first || pace.flags.second) {
		if (CCPR1H < PACE_SECOND_MIN || CCPR1H > PACE_SECOND_MAX) {
			pace.flags.invalid = 1;
		} else {
			if (pace.flags.first) {
				pace.flags.first = 0;
				pace.flags.second = 1;
			} else {
				pace.flags.second = 0;
			}
			pace.flags.pulse = 1;
		}
		return;
	}
	
	if (CCPR1H < PACE_ZERO_MIN || CCPR1H > PACE_ONE_MAX) {
		pace.flags.invalid = 1;
		return;
	}
	
	pace.data = pace.data << 1;
	if (CCPR1H >= PACE_ONE_MIN) {
		pace.data |= 0x1;
	}
	
	pace.flags.pulse = 1;
}

#endif	// PACE_SUPPORT
