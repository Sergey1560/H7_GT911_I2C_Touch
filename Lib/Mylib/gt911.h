#ifndef GT911_H
#define GT911_H
#include "common_defs.h"
#include "i2c.h"

/*
TouchPad_ID:9,1,1
TouchPad_Config_Version:41
FirmwareVersion:1060
*/
#define CT_MAX_TOUCH   5

#define GT911_MAX_WIDTH			800    	//Touchscreen pad max width
#define GT911_MAX_HEIGHT		480			//Touchscreen pad max height

#define GT911_RST_LOW  GPIOB->BSRR = GPIO_BSRR_BR_0
#define GT911_RST_HI   GPIOB->BSRR = GPIO_BSRR_BS_0

#define GT911_INT_LOW  GPIOB->BSRR = GPIO_BSRR_BR_1
#define GT911_INT_HI   GPIOB->BSRR = GPIO_BSRR_BS_1

#define GT911_WAIT_200ms   for(uint32_t i=0;i<0xAAAAAA;i++){__NOP();}
#define GT911_WAIT_10ms    for(uint32_t i=0;i<0x1000;i++){__NOP();}

#define GT911_PRODUCT_ID_REG	    0x8140
#define GT911_CONFIG_REG		    0x8047
#define GT911_FIRMWARE_VERSION_REG  0x8144 
#define GT911_READ_XY_REG 			0x814E

/* В даташите указан адрес 0xBA/0xBB. Отличия в последнем
бите, он указывает запись это или чтение. При использовании
аппаратного I2C этот бит выставляется автоматически, поэтому 
только один адрес.
При записи в регистр адреса, используется 7 бит с 1 сдвигом влево,
т.е. реальный адрес 0xBA >> 1 = 0x5D
Чтобы не заниматься сдвигами и для наглядности, в регистр записываю
0xBA без сдвига. Бит 0 игнорируется в режиме 7бит адреса.
*/
#define GT911_I2C_ADDR  0xBA


typedef struct
{
	uint8_t TouchCount;
 
	uint8_t Touchkeytrackid[CT_MAX_TOUCH];
	uint16_t X[CT_MAX_TOUCH];
	uint16_t Y[CT_MAX_TOUCH];
	uint16_t S[CT_MAX_TOUCH];
}GT911_Dev;

void g911_gpio_init(void);
void g911_touch_init(void);
void GT911_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
void GT911_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
void gt911_enable_irq(void);
void GT911_Scan(void);


#endif