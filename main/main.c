/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_spiffs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wear_levelling.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "linenoise/linenoise.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_pm.h"
#include "esp32/clk.h"
#include "common_defines.h"
#include "cmd_wifi.h"
#include "cmd_system.h"
#include "utils.h"
#include "tcp_log.h"
#include "mqtt_ctrl.h"
#include "ntp_sync.h"
#include "esp_ota_ops.h"
#include "gateop.h"
#include "hal/adc_types.h"
#include "adc_op.h"
#include "pumpop.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define TAG "ctrl_dev"
#define PROMPT_STR "CTRLDEV"
#define CONFIG_STORE_HISTORY 0
#define CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH	1024

#include "wifi_credentials.h"

TaskHandle_t ntp_sync_task_handle;

int console_state;
int restart_in_progress;

esp_vfs_spiffs_conf_t conf_spiffs =
	{
	.base_path = BASE_PATH,
	.partition_label = PARTITION_LABEL,
	.max_files = 5,
	.format_if_mount_failed = true
	};


static void initialize_nvs(void)
	{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
		{
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
		}
	ESP_ERROR_CHECK(err);
	}

#if CONFIG_STORE_HISTORY
	#define MOUNT_PATH "/data"
	#define HISTORY_PATH MOUNT_PATH "/history.txt"
	static void initialize_filesystem(void)
		{
		static wl_handle_t wl_handle;
		const esp_vfs_fat_mount_config_t mount_config =
			{
			.max_files = 4,
			.format_if_mount_failed = true
			};
		esp_err_t err = esp_vfs_fat_spiflash_mount(MOUNT_PATH, "storage", &mount_config, &wl_handle);
		if (err != ESP_OK)
			{
			ESP_LOGE(TAG, "Failed to mount FATFS (%d/%s)", err, esp_err_to_name(err));
			return;
			}
		}
#endif // CONFIG_STORE_HISTORY

void app_main(void)
	{
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
	int bp_ctrl = PUMP_ONLINE_CMD;
#elif ACTIVE_CONTROLLER == AGATE_CONTROLLER
	int bp_ctrl = 6; //IO6 on J5 pin 8
#endif
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << bp_ctrl);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    /*
     * if BOOT_CTRL_PIN is 0 at boot restart in factory mode
     */
    if(gpio_get_level(bp_ctrl) == 0)
    	{
    	const esp_partition_t *sbp = NULL;;
    	esp_partition_iterator_t pit = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    	while(pit)
    		{
    		sbp = esp_partition_get(pit);
    		if(sbp)
    			{
   				if(!strcmp(sbp->label, FACTORY_PART_NAME))
    				break;
    			}
    		pit = esp_partition_next(pit);
    		}
    	if(sbp)
			{
			int err = esp_ota_set_boot_partition(sbp);
			if(err == ESP_OK)
				esp_restart();
    		}
    	}
	gpio_reset_pin(bp_ctrl);
	restart_in_progress = 0;
	console_state = CONSOLE_OFF;
	setenv("TZ","EET-2EEST,M3.4.0/03,M10.4.0/04",1);
	esp_err_t ret = esp_vfs_spiffs_register(&conf_spiffs);
	if (ret != ESP_OK)
		{
        if (ret == ESP_FAIL)
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        else
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        esp_restart();
		}
	initialize_nvs();
	rw_params(PARAM_READ, PARAM_CONSOLE, &console_state);
	wifi_join(DEFAULT_SSID, DEFAULT_PASS, JOIN_TIMEOUT_MS);
	esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
	if(mqtt_start() == ESP_OK)
		register_mqtt();
	tcp_log_init();
	esp_log_set_vprintf(my_log_vprintf);
	//esp_console_repl_t *repl = NULL;
    //esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    //repl_config.prompt = PROMPT_STR ">";
    //repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

/*
#if CONFIG_STORE_HISTORY
	initialize_filesystem();
	repl_config.history_save_path = HISTORY_PATH;
	ESP_LOGI(TAG, "Command history enabled");
#else
	ESP_LOGI(TAG, "Command history disabled");
#endif
*/

	// start task to sync local time with NTP server

	/* Register commands */
	//esp_console_register_help_command();
	register_system();
	register_wifi();

	xTaskCreate(ntp_sync, "NTP_sync_task", 4096, NULL, USER_TASK_PRIORITY, &ntp_sync_task_handle);

#if ACTIVE_CONTROLLER == AGATE_CONTROLLER
	register_gateop();
#elif ACTIVE_CONTROLLER == PUMP_CONTROLLER
	register_pumpop();
#endif

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    //esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    //repl_config.task_stack_size = 8192;
    //ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    //ESP_LOGI(TAG, "console stack: %d", repl_config.task_stack_size);

#else
	#error Unsupported console type
#endif

#if CONFIG_PM_ENABLE
#if CONFIG_IDF_TARGET_ESP32S3
	esp_pm_config_esp32s3_t pm_config;

	int mhz = 80;
	pm_config.max_freq_mhz = mhz;
	pm_config.min_freq_mhz = rtc_clk_xtal_freq_get();
	pm_config.light_sleep_enable = false;
	printf("\n XTAL freq = %d\n", pm_config.min_freq_mhz);
	printf("\n CPU  freq = %d\n", esp_clk_cpu_freq() / MHZ);

	/*
	ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	while (esp_clk_cpu_freq() / MHZ != mhz)
		{
        vTaskDelay(pdMS_TO_TICKS(200));
        printf("Frequency is %d MHz\n", esp_clk_cpu_freq() / MHZ);
        }
	*/
#endif
#endif

	//ESP_ERROR_CHECK(esp_console_start_repl(repl));
	//while(1)
	//	sleep(10);
	}
