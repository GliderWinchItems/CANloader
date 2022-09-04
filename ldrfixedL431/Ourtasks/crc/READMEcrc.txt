READMEcrc.txt

08/29/2022

CRC-32 computation: 100,000 Bytes in flash
us = computation total in microseconds with 16 MHz sysclock
size = flash bytes
								  not xor	  xor	 cycles	  us	size			
crc-32_sw:  rosetta code:        B1077772  4EF8888D 1313632  82102  1068* 
crc-32_hw:  L432 hardware:       34C00EEA  CB3FF115  214659  13416    28
HAL CRC CALC: HAL routine:       34C00EEA  CB3FF115  278028  17376   340**  
crc-32_nib: Lookup nibble table: 34C00EEA  CB3FF115 1338739  83671   120 
crc-32_min: Shifting, no table:  34C00EEA  CB3FF115 8108771 506798    60

*  - includes 256 word lookup table; 
** - includes 132 initialization.