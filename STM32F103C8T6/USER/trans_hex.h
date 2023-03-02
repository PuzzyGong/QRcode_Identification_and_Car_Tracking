#ifndef _TRANS_HEX_H_
#define _TRANS_HEX_H_

#include "stm32f10x.h"

#include "uart.h"

void trans_hex_init(int i, u32 baud);

int trans_hex_R( int i, 
				int* i1,
				int* i2,
				int* i3,
				int* i4,
				int* i5,
				int* i6,
				int* i7,
				int* i8,
				int* i9,
				int* i10,
				int* i11,
				int* i12,
				int* i13,
				int* i14,
				int* i15,
				int* i16 );
				
#endif
