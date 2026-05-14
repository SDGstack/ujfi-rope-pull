#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include "wifi_web_manager.h"

void app_main(void)
{
	while(1){
		start_server();
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
