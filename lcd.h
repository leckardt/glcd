


#include <avr/io.h>
#include <util/delay.h>

#include "logo_load.h"

#define DATA_PORT PORTB
#define DATA_DDR  DDRB
#define DATA_PIN  PINB

#define RS_PORT PORTD
#define RS_DDR  DDRD
#define RS_PIN PD0

#define RW_PORT PORTD
#define RW_DDR DDRD
#define RW_PIN PD1

#define EN_PORT PORTD
#define EN_DDR DDRD
#define EN_PIN PD2

#define CS1_PORT PORTD
#define CS1_DDR DDRD
#define CS1_PIN PD3

#define CS2_PORT PORTD
#define CS2_DDR DDRD
#define CS2_PIN PD4

#define RST_PORT PORTD
#define RST_DDR DDRD
#define RST_PIN PD5

#define HIGH(PORT,PIN) PORT |= (1<<PIN)
#define LOW(PORT,PIN) PORT &= ~(1<<PIN)
//#define HIGH(A) A##_PORT |= (1<<A##_PIN)
#define CHIP1 0x01
#define CHIP2 0x02


#define CMD_LCD_ON		0x3F
#define CMD_LCD_OFF		0x3E

#define CMD_LCD_SET_ADDRESS	0x40
#define CMD_LCD_SET_PAGE	0xB8
#define CMD_LCD_DISP_START	0xC0

#define PAGE0			0x00
#define ADDRESS0		0x00
#define LINE0			0x00

#define LCD_BUSY		0x80
#define LCD_ON			0x20
#define LCD_RESET		0x10

void lcd_select_chip(uint8_t chip);

void lcd_write(uint8_t value, uint8_t chip);

uint8_t lcd_read();

void lcd_cmd_write(uint8_t cmd, uint8_t chip);

void lcd_data_write(uint8_t data, uint8_t chip);

void lcd_dummy_read(uint8_t chip);

uint8_t lcd_data_read(uint8_t chip);

uint8_t lcd_state_read(uint8_t chip);

void wait_while_busy(uint8_t chip);

void lcd_setup();

void lcd_init();

void lcd_clear();

void draw_bitmap_fullscreen(const uint8_t *image);
