#include "main.h"

int main(void){
	INFO("Sysytem start");
	RCC_init();
	SEGGER_RTT_Init();

	g911_touch_init();

	while(1){};
}