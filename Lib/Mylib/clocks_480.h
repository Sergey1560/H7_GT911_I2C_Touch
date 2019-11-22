#ifndef CLOCKS480_H
#define CLOCKS480_H

#define RCC_HSE 16000000

/* Источник PLL */
#define PLL_HSE 2
#define PLL_HSI 0
#define PLL_SRC PLL_HSE

/* Основные настройки тактирования */
#define RCC_DIVM1 1
#define RCC_DIVM2 1
#define RCC_DIVM3 1

/* PLL1 */
#define PLL_DIVR1 2
#define PLL_DIVQ1 20
#define PLL_DIVP1 2
#define PLL_DIVN1 60
#define PLL_FRACN1 0
/* PLL2 */
#define PLL_DIVR2 1
#define PLL_DIVQ2 1
#define PLL_DIVP2 2
#define PLL_DIVN2 20
#define PLL_FRACN2 0
/* PLL3 */
#define PLL_DIVR3 6
#define PLL_DIVQ3 2
#define PLL_DIVP3 2
#define PLL_DIVN3 12
#define PLL_FRACN3 0

#define RCC_D1CPRE 0


/* Тактирование шин */
#define APB1_CLK 120000000UL
#define APB2_CLK 120000000UL

#endif