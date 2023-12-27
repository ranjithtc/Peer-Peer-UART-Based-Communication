/*
 * LCD_Header_file.h
 *
 *  Created on: Nov 16, 2022
 *      Author: ranji
 */

#ifndef INC_LCD_HEADER_FILE_H_
#define INC_LCD_HEADER_FILE_H_

#include "stm32f407xx.h"
#include <stdint.h>

void delayMs(uint32_t n);
void LCD_command(unsigned char command);
void LCD_Data(char data);
void LCD_Init(void);
void PORTS_Init(void);
void Char_to_Binary(char data);

#endif /* INC_LCD_HEADER_FILE_H_ */
