#include <stdio.h>
#include <stdint.h>
#define UI unsigned int
#define ZZ (4*257)
uint8_t b[ZZ];
uint32_t chk = 0;
uint64_t binchk = 0;

int main(void)
{
	int i;
	uint8_t a = 0x8e;
	for (i = 0; i < ZZ; i++)
	{
		b[i] = a;
		a = a*7+i;
		printf(" %02X",b[i]);
	}
	printf("\n");

	uint32_t* pp = (uint16_t*)&b[0];
	for (i = 0; i < ZZ/2; i ++)
	{
		chk += *pp++;
	}	
	printf("   chk 0x%08X %d\n",chk,chk);
	uint32_t tmp16 = chk>>16;
	chk &= 0xffff;
	chk += tmp16;
	tmp16 = chk>>16;
	chk &= 0xffff;
	chk += tmp16;
	printf ("   chk 0x%12X %d\n",chk, chk);

	uint32_t* p = (uint32_t*)&b[0];
	for (i = 0; i < ZZ/4; i++)
	{
		binchk += *p++;
	}	
	printf("binchk 0x%012X %d\n",binchk,binchk);

	uint32_t tmp;
	int ctr = 0;
	while ((binchk>>16) != 0)
	{
		tmp = binchk>>16;
		binchk &= 0xffff;
printf("binchk1 0x%012X %08X ctr: %d\n",binchk,tmp, ctr);		
		binchk += tmp;
		ctr += 1;
printf("binchk2 0x%012X %08X ctr: %d\n",binchk,tmp, ctr);
	}
	printf("binchk3 0x%012X %d ctr: %d\n",binchk,binchk, ctr);
	printf("\n");
	return 0;
}