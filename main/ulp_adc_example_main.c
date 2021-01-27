/* ULP Example
	author :  CarlosInacio
	data : January 27, 2021
*/

#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

#define BLINK_GPIO GPIO_NUM_33

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program(void);

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program(void);
/* This function is called every time before exit into deep sleep.
 * Start blink on LED.
 */
void blink_GPIO(void);


void app_main(void)
{

	/*Configurações do GPIO do LED*/
	gpio_pad_select_gpio(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

	/*Verificação da causa do wakeup*/
	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup\n");
        init_ulp_program();
    } else {
        printf("Deep sleep wakeup\n");
        ulp_last_result &= UINT16_MAX;
        printf("Value=%d \n", ulp_last_result);
    }

    /*Execução de tarefas*/
    blink_GPIO();

    /*Chamada de funções para iniciar o deep_sleep*/
    printf("Entering deep sleep\n\n");
    start_ulp_program();
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    esp_deep_sleep_start(); //Iniciando o deep_sleep
}
static void init_ulp_program(void)
{
	/*Carregando o binpario para memória */
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Configuração do ADC channel */
    /* Essas configurações dependem de adc.S */
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();

    /* Definindo valor de ADC para wakeup*/
    ulp_Tpig_detect = 2000;

    /* Definindo o periodo para wakeup */
   ulp_set_wakeup_period ( 0 , 20 * 1000 ); // 20 ms


    /*Wake up por GPIO*/
    gpio_num_t gpio_num = GPIO_NUM_32;
    int rtcio_num = rtc_io_number_get(gpio_num); // Transforma o numero GPIO em numero RTC_IO
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");
	/* Iniciando GPIO e configurando-o para pullup */
    rtc_gpio_init(gpio_num);
	rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pulldown_dis(gpio_num);
	rtc_gpio_pullup_en(gpio_num);

	ulp_io_number = rtcio_num; /* mapeando de GPIO# para RTC_IO# */

    /*Desabilitando pinos 12 e 15 */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suspender mensagens de boot
}

static void start_ulp_program(void)
{
    /* Chamada da função ulp_run para iniciar o processo no coprocessador */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void blink_GPIO(void)
{
	/* Blink off (output low) */
	printf("Turning off the LED\n");
	gpio_set_level(BLINK_GPIO, 0);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	/* Blink on (output high) */
	printf("Turning on the LED\n");
	gpio_set_level(BLINK_GPIO, 1);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}



