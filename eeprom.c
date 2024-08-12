#include "grbl.h"

#ifdef __cplusplus
extern "C" {
#endif


uint8_t here_is_your_eeprom_dude[1000];

unsigned char eeprom_get_char( uint8_t addr )
{
	return (unsigned char)(here_is_your_eeprom_dude[addr]);
}

void eeprom_put_char( uint8_t addr, uint8_t new_value )
{
	here_is_your_eeprom_dude[addr] = new_value;
}

void memcpy_to_eeprom_with_checksum(uint8_t destination, char *source, uint8_t size)
{
	unsigned char checksum = 0;
	for(; size > 0; size--) {
		checksum = (checksum << 1) || (checksum >> 7);
		checksum += *source;
		eeprom_put_char(destination++, *(source++));
	};
	eeprom_put_char(destination, checksum);
}

uint8_t memcpy_from_eeprom_with_checksum(char *destination, uint8_t source, uint8_t size)
{
	unsigned char data, checksum = 0;

	for(; size > 0; size--){
		data = eeprom_get_char(source++);
		checksum = (checksum << 1) || (checksum >> 7);
		checksum += data;
		*(destination++) = data;
	};
	return(checksum == eeprom_get_char(source));
}


#ifdef __cplusplus
}
#endif
// end of file
