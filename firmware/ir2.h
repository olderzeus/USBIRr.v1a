/*
 * $Id: ir2.h,v 1.10 2009/01/16 13:08:48 fvecoven Exp $
 */
#ifndef _IR2_H
#define	_IR2_H

#include "stdint.h"

/*
 * IR is a library which can decode various IR protocols.
 *
 * The library is written to be flexible, so if you only need one protocol for
 * your project, you can still use it without overhead. The behavior is
 * controlled by various #define	below.
 *
 * All the code runs with a single interrupt. It uses the capture module and
 * its associated timer. (Note that it should be easy to rewrite the code to
 * only use a timer and an interrupt pin. Basically, we need a way to detect
 * edges on a pin, and a way to measure the time between such edges.
 */

/*
 * TIMER 1  runs at Fosc/4. With a 4MHz crystal and HSPLL, it increments
 * every 1/4.000.000 sec. It overlaps at 0xFFFF, which is every 16.384ms.
 */
#define	T1_RELOAD_H	0
#define	T1_RELOAD_L	0


/*
 * Library options
 */
#define	IR_RAW_SUPPORT		0
#define	IR_RC5_SUPPORT		0
#define	IR_RC6_SUPPORT		1
#define	IR_NEC_SUPPORT		0
#define	IR_JVC_SUPPORT		0
#define	IR_SONY_SUPPORT		1
#define IR_SHARP_SUPPORT	0
#define IR_PACE_SUPPORT		1


typedef enum {
	IR_NEC,
	IR_NEC_EXTENDED,
	IR_JVC,
	IR_RC5,
	IR_RC6,
	IR_SONY_12BITS,
	IR_SONY_15BITS,
	IR_SONY_20BITS,
	IR_SHARP,
	IR_PACE
} ir_type_t;


typedef union {
	struct {
		unsigned first :1;
		unsigned edge :1;
		unsigned timeout :1;
		unsigned decoded :1;
	};
	uint8_t raw;
} ir_flags_t;


typedef struct {
	ir_type_t type;
	uint8_t command;
	uint16_t address;
	uint8_t extra;
} ir_t;



#if IR_RAW_SUPPORT == 1

/* max number of transitions saved in memory */
#define	IR_RAW_SIZE	192

#endif


/*
 * RC5
 *
 * short pulse : 889us
 * long pulse  : 2x 889us
 */
#if IR_RC5_SUPPORT == 1

#define	RC5_SHORT_MIN	10
#define	RC5_SHORT_MAX	16
#define	RC5_LONG_MIN	24
#define	RC5_LONG_MAX	30

typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned last2 :1;
			unsigned x :1;
			unsigned same :1;
		};
		uint8_t raw;
	} flags;
	uint16_t data;
} rc5_t;

#endif

/*
 * RC6
 *
 * leader pulse : 2.666ms 889us
 * short pulse : 444us
 * long pulse  : 2x 444us
 *
 * Format is  LP 1 M2 M1 M0 T A7 ... A0 C7 ... C0 sleep
 *
 * Toggle bit uses pulses twice longer than others
 * Sleep is 6 periods, so 2.666ms
 */
#if IR_RC6_SUPPORT == 1

#define	RC6_FIRST_MIN	40
#define	RC6_FIRST_MAX	44
#define	RC6_SECOND_MIN	9
#define	RC6_SECOND_MAX	13
#define	RC6_SHORT_MIN	4
#define	RC6_SHORT_MAX	8
#define	RC6_LONG_MIN	10
#define	RC6_LONG_MAX	16

typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned last2 :1;
			unsigned x :1;
			unsigned same :1;
			unsigned first :1;
			unsigned second :1;
		};
		uint8_t raw;
	} flags;
	uint8_t bitcount;
	uint24_t data;
} rc6_t;

#endif


/*
 * JVC : pulse distance encoding
 *
 * first pulse : 8.4ms 4.2ms
 * 0 : 526us 526us
 * 1 : 526us 1574us
 *
 * Address (8 bits) is sent first, then Command (8 bits)
 * LSB is sent first !
 */
 #if IR_JVC_SUPPORT == 1

 #define	JVC_FIRST_MIN		128
 #define	JVC_FIRST_MAX		136
 #define	JVC_SECOND_MIN		60
 #define	JVC_SECOND_MAX		68
 #define	JVC_SHORT_MIN		6
 #define	JVC_SHORT_MAX		11
 #define	JVC_LONG_MIN		21
 #define	JVC_LONG_MAX		25
 #define	JVC_TIMEOUT		200

 typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned first :1;
			unsigned second :1;
			unsigned pulse :1;
			unsigned rbit :1;
		};
		uint8_t raw;
	} flags;
	uint8_t bitcount;
	uint16_t data;
} jvc_t;

 #endif


/*
 * NEC : pulse distance encoding
 *
 * first pulse : 9ms 4.5ms
 * 0 : 560us 560us
 * 1 : 560us 1690us
 *
 * Address (8 bits) is sent first, then ~Address
 * Command (8 bits) follows, then ~Command
 * LSB is sent first !
 *
 * NEC extended allows 16 bits addresses. In this case, the 8 bits following
 * Address are not ~Address, but the address extension.
 */
 #if IR_NEC_SUPPORT == 1

 #define	NEC_FIRST_MIN		136
 #define	NEC_FIRST_MAX		144
 #define	NEC_SECOND_MIN		64
 #define	NEC_SECOND_MAX		72
 #define	NEC_SHORT_MIN		6
 #define	NEC_SHORT_MAX		11
 #define	NEC_LONG_MIN		22
 #define	NEC_LONG_MAX		27
 #define	NEC_TIMEOUT		200

 typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned first :1;
			unsigned second :1;
			unsigned pulse :1;
		};
		uint8_t raw;
	} flags;
	uint8_t bitcount;
	uint32_t data;
} nec_t;

 #endif





/*
 * SONY SIRC
 *
 * 12bits, 15 bits or 20 bits
 *
 * lead pulse : 2.4ms 600us
 * 1 : 1.2ms 600us
 * 0 : 600us 600us
 *
 * Command repeated every 45ms
 */
#if IR_SONY_SUPPORT == 1

#define	SONY_FIRST_MIN		35
#define	SONY_FIRST_MAX		40
#define	SONY_SHORT_MIN		6
#define	SONY_SHORT_MAX		11
#define	SONY_LONG_MIN		17
#define	SONY_LONG_MAX		21
#define	SONY_TIMEOUT		150

typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned first :1;
			unsigned gap :1;
		};
		uint8_t raw;
	} flags;
	uint8_t bitcount;
	uint24_t data;
} sony_t;


#endif



/*
 * SHARP protocol
 *
 * Pulse distance encoding, no leading pulse
 *
 * 0 : 320us (1ms - 320us)
 * 1 : 320us (2ms - 320us)
 *
 * LSB first. Address (5 bits), Command (8 bits), Expansion (1bit)
 * and finally Check (1 bit)
 *
 * The same command is send 40ms, with bits reversed.
 */
#if IR_SHARP_SUPPORT == 1

#define SHARP_SHORT_MIN		3
#define SHARP_SHORT_MAX		6
#define	SHARP_ZERO_MIN		9
#define SHARP_ZERO_MAX		13
#define SHARP_ONE_MIN		25
#define SHARP_ONE_MAX		29

typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned gap :1;
			unsigned second :1;
		};
		uint8_t raw;
	} flags;
	uint16_t data;
	uint16_t prev;
} sharp_t;

#endif


/*
 * PACE 
 *
 * Leader pulse :  800us 3ms
 * 0 : 750us  6.8ms
 * 1 : 750us  4.3ms
 */
#if IR_PACE_SUPPORT == 1

#define PACE_SHORT_MIN		7
#define PACE_SHORT_MAX		12
#define PACE_SECOND_MIN		46
#define PACE_SECOND_MAX		50
#define PACE_ONE_MIN		104
#define PACE_ONE_MAX		110
#define PACE_ZERO_MIN		66
#define PACE_ZERO_MAX		70

typedef struct {
	union {
		struct {
			unsigned invalid :1;
			unsigned last :1;
			unsigned pulse :1;
			unsigned first :1;
			unsigned second :1;
		};
		uint8_t raw;
	} flags;
	uint16_t data;
} pace_t;


#endif
 
 
/* export functions */
extern void ir_init(void);
extern void ir_enable(void);
extern void ir_disable(void);
extern void ir_interrupt(void);
extern void ir_timeout(void);


/* exported variables */
extern ir_t ir;
extern ir_flags_t ir_flags;


#endif	// _IR2_H
