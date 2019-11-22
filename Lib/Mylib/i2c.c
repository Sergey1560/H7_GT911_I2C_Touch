#include "i2c.h"



void i2c_init(void){

    i2c_gpio_init();
    RCC->APB1LENR |= RCC_APB1LENR_I2C2EN;

    I2C2->CR1 = 0;
    //I2C2->CR1 = (1 << I2C_CR1_DNF_Pos);
    //I2C2->TIMINGR = 0x307075B1;  
	I2C2->TIMINGR = 0x20A09DEB;  
    I2C2->CR1 |= I2C_CR1_PE;
}


uint8_t i2c_write_cmd(uint8_t adr,uint16_t cmd){
   	//Отправка номера регистра
	//0xBA 8 бит, младший бит - направление чтение-запись, без сдвига на 1, чтобы не считать число
	I2C2->CR2 = (adr|I2C_CR2_AUTOEND|(2 << I2C_CR2_NBYTES_Pos));
	I2C2->CR2 |= I2C_CR2_START;

	for(uint8_t i=0; i<2; i++){
		while(!(I2C2->ISR & I2C_ISR_TXE)){__NOP();};
		I2C2->TXDR = (uint8_t)((cmd >> 8*(1-i)) & 0xFF) ;
	}
    //Условие STOP формируется автоматически при наличии I2C_CR2_AUTOEND
    return 0;
}

uint8_t i2c_read(uint8_t adr,uint8_t *data,uint8_t len){

	I2C2->CR2 = (adr)|I2C_CR2_AUTOEND|(len << I2C_CR2_NBYTES_Pos)|I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	for(uint8_t i=0; i<len; i++){
		while(!(I2C2->ISR & I2C_ISR_RXNE)){__NOP();};
		*data++ = I2C2->RXDR;
	};

    //Условие STOP формируется автоматически при наличии I2C_CR2_AUTOEND
    return 0;
}


uint8_t i2c_write(uint8_t adr,uint8_t *data,uint8_t len){
	I2C2->CR2 = (adr|I2C_CR2_AUTOEND|(len << I2C_CR2_NBYTES_Pos));
	I2C2->CR2 |= I2C_CR2_START;

	for(uint8_t i=0; i<len; i++){
		while(!(I2C2->ISR & I2C_ISR_TXE)){__NOP();};
		I2C2->TXDR = (uint8_t)*data++;
	}
   return 0;
}

/*
Запись в регистр происходит без состояния STOP,
два байта регистра и сразу нагрузка.
*/
uint8_t i2c_write_reg(uint8_t adr,uint16_t reg,uint8_t *data,uint8_t len){
	I2C2->CR2 = (adr|I2C_CR2_AUTOEND|((len+2) << I2C_CR2_NBYTES_Pos));
	I2C2->CR2 |= I2C_CR2_START;

	for(uint8_t i=0; i<2; i++){
		while(!(I2C2->ISR & I2C_ISR_TXE)){__NOP();};
		I2C2->TXDR = (uint8_t)((reg >> 8*(1-i)) & 0xFF) ;
	}

	for(uint8_t i=0; i<len; i++){
		while(!(I2C2->ISR & I2C_ISR_TXE)){__NOP();};
		I2C2->TXDR = (uint8_t)*data++;
	}
    return 0;
}



void i2c_gpio_init(void){
/*
I2C2
PH4 - SCL
PB11 - SDA
AF4
*/

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN|RCC_AHB4ENR_GPIOHEN; 
	/* PB11 */
    //MODE - 4 AF
	GPIOB->MODER &= ~(GPIO_MODER_MODER11);
    GPIOB->MODER |= (GPIO_MODER_MODER11_1);

	//Open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	//Very High speed 11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR11);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR11_Pos);

    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR11;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0;

	GPIOB->AFR[1] &= ~GPIO_AFRH_AFRH11;
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFRH11_Pos);
	
	/* PH4 */
    //MODE - 4 AF
	GPIOH->MODER &= ~(GPIO_MODER_MODER4);
    GPIOH->MODER |= (GPIO_MODER_MODER4_1);

	//Push-pull mode 0
	GPIOH->OTYPER |= GPIO_OTYPER_OT_4;
	//Very High speed 11
    GPIOH->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4);
    GPIOH->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR4_Pos);

    GPIOH->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOH->PUPDR |= GPIO_PUPDR_PUPDR4_0;


	GPIOH->AFR[0] &= ~GPIO_AFRL_AFRL4;
    GPIOH->AFR[0] |= (4 << GPIO_AFRL_AFRL4_Pos);




}