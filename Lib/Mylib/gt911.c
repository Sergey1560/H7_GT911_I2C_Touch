#include "gt911.h"

GT911_Dev Dev_Now,Dev_Backup;


void gt911_test(void){

	while(1){
		GT911_Scan();
	}

}

void g911_touch_init(void){
	uint8_t buf[4];
	
	g911_gpio_init();
	i2c_init();
	
	GT911_RD_Reg(GT911_PRODUCT_ID_REG, (uint8_t *)&buf, 3);
	GT911_RD_Reg(GT911_CONFIG_REG, (uint8_t *)&buf[3], 1);

	INFO("TouchPad_ID:%c,%c,%c TouchPad_Config_Version:%2x",buf[0],buf[1],buf[2],buf[3]);

	GT911_RD_Reg(GT911_FIRMWARE_VERSION_REG, (uint8_t *)&buf, 2);
	INFO("FirmwareVersion:%2x",(((uint16_t)buf[1] << 8) + buf[0]));

	gt911_enable_irq();
}


void GT911_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len){
	if(i2c_write_cmd(GT911_I2C_ADDR,reg)){
		ERROR("CMD Write %0X failed",reg);
	};
	if(i2c_read(GT911_I2C_ADDR,buf,len)){
		ERROR("Read failed");
	};
}


void GT911_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len){
	i2c_write_reg(GT911_I2C_ADDR,reg,buf,len);
}


void GT911_Scan(void){
	uint8_t buf[41]; // 5*8bit + 1 byte
  	uint8_t Clearbuf = 0;
	
		GT911_RD_Reg(GT911_READ_XY_REG, buf, 1);	
		if ((buf[0]&0x80) == 0x00){
			GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
		}else{
			Dev_Now.TouchpointFlag = buf[0];
			Dev_Now.TouchCount = buf[0]&0x0f;
			if( (Dev_Now.TouchCount > 5) || (Dev_Now.TouchCount == 0) )
			{
				GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
				return ;
			}		
			GT911_RD_Reg(GT911_READ_XY_REG+1, &buf[1], Dev_Now.TouchCount*8);
			GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);

			for (uint8_t i=0;i<Dev_Now.TouchCount;i++){
				Dev_Now.Touchkeytrackid[i] = buf[1+(8*i)];
				Dev_Now.X[i] = ((uint16_t)buf[3+(8*i)] << 8) + buf[2+(8*i)];  
				Dev_Now.Y[i] = ((uint16_t)buf[5+(8*i)] << 8) + buf[4+(8*i)];
				Dev_Now.S[i] = ((uint16_t)buf[7+(8*i)] << 8) + buf[6+(8*i)]; //Площадь 
			
			
				if(Dev_Now.Y[i]<20)Dev_Now.Y[i]=20;
				if(Dev_Now.Y[i]>GT911_MAX_HEIGHT-20)Dev_Now.Y[i]=GT911_MAX_HEIGHT-20;
				if(Dev_Now.X[i]<20)Dev_Now.X[i]=20;
				if(Dev_Now.X[i]>GT911_MAX_WIDTH-20)Dev_Now.X[i]=GT911_MAX_WIDTH-20;
				INFO("Now touch %d X:%d Y:%d S:%d",i,Dev_Now.X[i],Dev_Now.Y[i],Dev_Now.S[i]);			
			}

			}
			
	}	




void g911_gpio_init(void){
/*
PB0 - Touch Wake - Out PP  Pull UP
PB1 - Touch IRQ - Input Pull UP

Ноги I2C на время включения прижать к земле
PB11 - I2C 
PH4 -  I2C 
*/

	/* PB11 */
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN|RCC_AHB4ENR_GPIOHEN; 

	GPIOB->MODER &= ~(GPIO_MODER_MODER11);
    GPIOB->MODER |= (GPIO_MODER_MODER11_0);

	//Open drain
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_11;
	//Very High speed 11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR11);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR11_Pos);

    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR11;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_1;

	/********* PH4 Out PP Pull Down */
	GPIOH->MODER &= ~(GPIO_MODER_MODER4);
    GPIOH->MODER |= (GPIO_MODER_MODER4_0);

	//Push-pull mode 0
	GPIOH->OTYPER &= ~GPIO_OTYPER_OT_4;
	//Very High speed 11
    GPIOH->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4);
    GPIOH->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR4_Pos);

    GPIOH->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOH->PUPDR |= GPIO_PUPDR_PUPDR4_1;


	/****** RST PB0 Out PP Pull Down */
    //MODE - Out
	GPIOB->MODER &= ~(GPIO_MODER_MODER0);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0);

	//Push-pull mode 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);
	//Very High speed 11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR0_Pos);

	//Pull UP
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR0;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0;


    /****** PB1 In Exti на время старта как OUT */
    //MODE - Out
	GPIOB->MODER &= ~(GPIO_MODER_MODER1);
    GPIOB->MODER |= (GPIO_MODER_MODER1_0);

	//Push-pull mode 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_1);
	//Very High speed 11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR1_Pos);
	//Pull UP
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR1;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR1_0;

 
	/* Для установки адреса нужна последовательность включения 
	Рабочее состояние RST - Hi.
	*/
	GT911_RST_LOW;
	GT911_INT_LOW;
	GT911_WAIT_200ms;
	GT911_RST_HI;
	GT911_WAIT_200ms;
	GT911_INT_HI;


    /****** PB1 Int  */
    //MODE - In
	GPIOB->MODER &= ~(GPIO_MODER_MODER1);
	//Push-pull mode 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_1);
	//Very High speed 11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR1_Pos);
	//Pull UP
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR1;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR1_0;



}


void gt911_enable_irq(void){

    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;  
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; 
	EXTI->FTSR1 	|= EXTI_FTSR1_TR1;

	NVIC_EnableIRQ (EXTI1_IRQn);
	NVIC_SetPriority (EXTI1_IRQn, 0);
	//Разрешаем прерывания в периферии
	EXTI_D1->IMR1 |= EXTI_IMR1_IM1;

}


void EXTI1_IRQHandler(void){
	if(EXTI_D1->PR1 & EXTI_PR1_PR1){
			EXTI_D1->PR1 = EXTI_PR1_PR1;
	};

	GT911_Scan();
}
