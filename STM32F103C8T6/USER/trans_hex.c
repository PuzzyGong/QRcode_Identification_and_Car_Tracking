#include "trans_hex.h"

#include "math.h"

static struct
{
	char first;    char second;
	
	char c1_high;  char c1_low;
	char c2_high;  char c2_low;
	char c3_high;  char c3_low;
	char c4_high;  char c4_low;
	char c5_high;  char c5_low;
	char c6_high;  char c6_low;
	char c7_high;  char c7_low;
	char c8_high;  char c8_low;
	char c9_high;  char c9_low;
	char c10_high; char c10_low;
	char c11_high; char c11_low;
	char c12_high; char c12_low;
	char c13_high; char c13_low;
	char c14_high; char c14_low;
	char c15_high; char c15_low;
	char c16_high; char c16_low;
	
	char check;
}
trans_hex_Rbuff[4];

static int trans_hex_Rflag[4] = {0,0,0,0};

void trans_hex_init(int i, u32 baud)
{
	uart_init(i, baud, 0,                       0, 
					   &trans_hex_Rbuff[i - 1], sizeof(trans_hex_Rbuff[i - 1]), 
					   &trans_hex_Rflag[i - 1]);
}


int trans_hex_R(int i, 
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
				int* i16 )
{
	if(trans_hex_Rflag[i - 1] != 0)
	{
		*i1  = trans_hex_Rbuff[i - 1].c1_low  + 256 * trans_hex_Rbuff[i - 1].c1_high;
        *i2  = trans_hex_Rbuff[i - 1].c2_low  + 256 * trans_hex_Rbuff[i - 1].c2_high;
		*i3  = trans_hex_Rbuff[i - 1].c3_low  + 256 * trans_hex_Rbuff[i - 1].c3_high;
		*i4  = trans_hex_Rbuff[i - 1].c4_low  + 256 * trans_hex_Rbuff[i - 1].c4_high;
		*i5  = trans_hex_Rbuff[i - 1].c5_low  + 256 * trans_hex_Rbuff[i - 1].c5_high;
		*i6  = trans_hex_Rbuff[i - 1].c6_low  + 256 * trans_hex_Rbuff[i - 1].c6_high;
		*i7  = trans_hex_Rbuff[i - 1].c7_low  + 256 * trans_hex_Rbuff[i - 1].c7_high;
		*i8  = trans_hex_Rbuff[i - 1].c8_low  + 256 * trans_hex_Rbuff[i - 1].c8_high;
		*i9  = trans_hex_Rbuff[i - 1].c9_low  + 256 * trans_hex_Rbuff[i - 1].c9_high;
		*i10 = trans_hex_Rbuff[i - 1].c10_low + 256 * trans_hex_Rbuff[i - 1].c10_high;
		*i11 = trans_hex_Rbuff[i - 1].c11_low + 256 * trans_hex_Rbuff[i - 1].c11_high;
		*i12 = trans_hex_Rbuff[i - 1].c12_low + 256 * trans_hex_Rbuff[i - 1].c12_high;
		*i13 = trans_hex_Rbuff[i - 1].c13_low + 256 * trans_hex_Rbuff[i - 1].c13_high;
		*i14 = trans_hex_Rbuff[i - 1].c14_low + 256 * trans_hex_Rbuff[i - 1].c14_high;
		*i15 = trans_hex_Rbuff[i - 1].c15_low + 256 * trans_hex_Rbuff[i - 1].c15_high;
		*i16 = trans_hex_Rbuff[i - 1].c16_low + 256 * trans_hex_Rbuff[i - 1].c16_high;
			
		trans_hex_Rflag[i - 1] = 0;
		return 1;
	}
	return 0;
}
