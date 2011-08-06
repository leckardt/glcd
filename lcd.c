


#include "lcd.h"

void lcd_select_chip(uint8_t chip)
{
	switch(chip)
	{
	case CHIP1:
		HIGH(CS1_PORT, CS1_PIN);
		LOW(CS2_PORT, CS2_PIN);
		break;
	case CHIP2:
		LOW(CS1_PORT, CS1_PIN);
		HIGH(CS2_PORT, CS2_PIN);
		break;
	case (CHIP1|CHIP2):
		HIGH(CS1_PORT, CS1_PIN);
		HIGH(CS2_PORT, CS2_PIN);
		break;
	default:
		LOW(CS1_PORT, CS1_PIN);
		LOW(CS2_PORT, CS2_PIN);
		break;
	}
}

void lcd_write(uint8_t value, uint8_t chip)
{
	DATA_PORT = value;
	HIGH(EN_PORT, EN_PIN);
	_delay_us(2); //>1000ns
	LOW(EN_PORT,EN_PIN);
	DATA_PORT = 0x00;
}

uint8_t lcd_read()
{
	uint8_t data;
	DATA_PORT = 0xFF;
	DATA_DDR = 0x00;
	HIGH(EN_PORT, EN_PIN);
	_delay_us(2); //>320ns
	data = DATA_PIN;
	LOW(EN_PORT, EN_PIN);

	DATA_DDR = 0xFF;
	return data;
}

void lcd_cmd_write(uint8_t cmd, uint8_t chip)
{
	wait_while_busy(chip);
	LOW(RW_PORT, RW_PIN);//write
	LOW(RS_PORT, RS_PIN);//instruction
	lcd_write(cmd, chip);

}

void lcd_data_write(uint8_t data, uint8_t chip)
{
	wait_while_busy(chip);
	LOW(RW_PORT, RW_PIN);//write
	HIGH(RS_PORT, RS_PIN);//data
	lcd_write(data, chip);
}

void lcd_dummy_read(uint8_t chip)
{
	wait_while_busy(chip);
	HIGH(RW_PORT, RW_PIN);//read
	HIGH(RS_PORT, RS_PIN);//data
	HIGH(EN_PORT, EN_PIN);
	_delay_us(2);//>1000ns
	LOW(EN_PORT, EN_PIN);
}

uint8_t lcd_data_read(uint8_t chip)
{
	wait_while_busy(chip);
	uint8_t data;
	HIGH(RW_PORT, RW_PIN);//read
	HIGH(RS_PORT, RS_PIN);//data
	data = lcd_read();
	return data;
}

uint8_t lcd_state_read(uint8_t chip)
{
	wait_while_busy(chip);
	uint8_t temp_state;
	HIGH(RW_PORT, RW_PIN);
	LOW(RS_PORT, RS_PIN);
	temp_state = lcd_read();
	return temp_state;
}

void wait_while_busy(uint8_t chip)
{
	lcd_select_chip(chip);
	HIGH(RW_PORT, RW_PIN);
	LOW(RS_PORT, RS_PIN);
	DATA_PORT = 0x00;
	DATA_DDR = 0x00;
	HIGH(EN_PORT, EN_PIN);
	_delay_us(2);//>1000ns
	LOW(EN_PORT, EN_PIN);
	while (LCD_BUSY & DATA_PIN);
	DATA_DDR = 0xFF;
}

void lcd_set_pixel(uint8_t x, uint8_t y, char color)
{
	uint8_t chip, page;
	if ( x < 64 )
		chip = CHIP1;
	else
	{
		chip = CHIP2;
		x -= 64;
	}
	page = y/8;
	y = y%8;
	wait_while_busy(chip);
	lcd_cmd_write(CMD_LCD_SET_ADDRESS|x,chip);
	lcd_cmd_write(CMD_LCD_SET_PAGE|page,chip);
	lcd_dummy_read(chip);
	wait_while_busy(chip);
	uint8_t data;
	data = lcd_data_read(chip);
	if (color)
		data |= (1<<y);
	else
		data &= ~(1<<y);
	lcd_cmd_write(CMD_LCD_SET_ADDRESS|x,chip);
	lcd_data_write(data,chip);
}

void lcd_invert()
{
	for ( uint8_t pageloop = 0; pageloop < 8; pageloop++ )
	{
		wait_while_busy(CHIP1);
		wait_while_busy(CHIP2);
		lcd_cmd_write(CMD_LCD_SET_ADDRESS|ADDRESS0, CHIP1|CHIP2);
		lcd_cmd_write(CMD_LCD_SET_PAGE|pageloop, CHIP1|CHIP2);
		lcd_dummy_read(CHIP1);
		lcd_dummy_read(CHIP2);
		for ( uint8_t chip = CHIP1; chip <= CHIP2; chip++ )
		{
			uint8_t data[64];
			uint8_t loop;
			for ( loop = 0; loop < 64; loop++ )
			{
				wait_while_busy(chip);
				data[loop] = lcd_data_read(chip);
			}
			wait_while_busy(chip);
			lcd_cmd_write(CMD_LCD_SET_ADDRESS|ADDRESS0, chip);
			for ( loop = 0; loop < 64; loop++ )
			{
				wait_while_busy(chip);
				lcd_data_write(~data[loop], chip);
			}
		}
	}
}

void lcd_setup()
{
	RS_DDR |= (1<<RS_PIN);
	RW_DDR |= (1<<RW_PIN);
	EN_DDR |= (1<<EN_PIN);
	CS1_DDR |= (1<<CS1_PIN);
	CS2_DDR |= (1<<CS2_PIN);
	RST_DDR |= (1<<RST_PIN);
	DATA_DDR = 0xff;
}

void lcd_init()
{
	lcd_select_chip(CHIP1|CHIP2);

	LOW(RST_PORT,RST_PIN);//reset
	_delay_ms(30);
	HIGH(RST_PORT,RST_PIN);


	lcd_cmd_write(CMD_LCD_ON,CHIP1|CHIP2);
	lcd_cmd_write(CMD_LCD_DISP_START|LINE0,CHIP1|CHIP2);
	lcd_cmd_write(CMD_LCD_SET_ADDRESS|ADDRESS0,CHIP1|CHIP2);
	lcd_cmd_write(CMD_LCD_SET_PAGE|PAGE0,CHIP1|CHIP2);
}

void lcd_clear()
{
	for(uint8_t y = 0; y < 8; y++)
	{
		lcd_cmd_write(CMD_LCD_SET_PAGE|y,CHIP1|CHIP2);
		lcd_cmd_write(CMD_LCD_SET_ADDRESS|ADDRESS0,CHIP1|CHIP2);
		for(uint8_t x = 0; x < 64; x++)
		{
			lcd_data_write(0x00,CHIP1|CHIP2);
		}
		lcd_cmd_write(CMD_LCD_SET_PAGE|PAGE0,CHIP1|CHIP2);
		lcd_cmd_write(CMD_LCD_SET_ADDRESS|ADDRESS0,CHIP1|CHIP2);
	}
}

void draw_bitmap_fullscreen(const uint8_t *image)
{
	uint8_t chip;
	uint16_t byte = 0;
	for(uint8_t y = 0; y < 8; y++)
	{
		lcd_cmd_write(CMD_LCD_SET_PAGE|y,CHIP1|CHIP2);
		lcd_cmd_write(CMD_LCD_SET_ADDRESS|0,CHIP1|CHIP2);
		for(uint8_t x = 0; x < 128; x++)
		{
			if(x<64) chip=1; else chip=2;
			lcd_data_write(pgm_read_byte(image+(byte++)),chip);
		}
	}
}


int main()
{
	lcd_setup();
	lcd_init();
	lcd_clear();

	draw_bitmap_fullscreen(logo);

	while(1)
	{
		uint8_t t=0;
		t++;
	}

}
