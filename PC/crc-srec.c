/******************************************************************************
* File Name          : crc-srec.c
* Date First Issued  : 09/02/2022
* Board              : Linux PC
* Description        : Add CRC-32 to end of .srec file
*******************************************************************************/
/*
gcc -Wall crc-srec.c -o crc-srec 
gcc -Wall crc-srec.c -o crc-srec && ./crc-srec ../../ldrfixedL431/build/ldrfixed.srec

*/

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>

FILE* fpIn;

//char *canid_insert = "w";

/* ************************************************************************************************************ */
/*  Yes, this is where it starts.                                                                               */
/* ************************************************************************************************************ */
int main(int argc, char **argv)
{
	int i;
	int linect;
	int s3ct;
	int bct;

   /* **********************************************************************************  */
   /* Read .srec generated by compiler */
   /* **********************************************************************************  */

	if (fpIn = Xfopen(*argv[1] == NULL) )
	{
		printf ("\nInput file did not open: %s\n",*argv[1]);
		exit (-1);
	}

// S3150800000000C00020E54900086D0A0008710A0008CA	
printf("srec file opened: %s\n",*argv[1]);
	linect = 0; // Line counter
	while ( (fgets (&buf[0],LINESZ,fpIn)) != NULL)	// Get a line
	{
		linect += 1;
		if ((buf[0] = 'S') && (buf[2] == '3'))
		{ // Here an S3 line
			s3ct += 1;
			bct = sscanf(&buf[2],"%2X",&bct);
			if ((bct < 5) || (bct > 128))
			{
				printf("S3 byte ct bad: %bct on line %d\n\r",bct,linect);
			}
			
		}
	}
	printf ("Total line ct: %5d\n\r",linect);
	printf ("Total s3   ct: %5d\n\r",bct);
	fclose(fpIn);
}
