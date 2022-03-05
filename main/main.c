/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_timer.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "PMU.h"

#define LED_ONBOARD_GPIO 2

#ifdef ENABLE_HARMONICS
extern unsigned char ucData[832];
extern float harmonics_R_mag[15];
extern float harmonics_R_phase[15];
extern float harmonics_S_mag[15];
extern float harmonics_S_phase[15];
extern float harmonics_T_mag[15];
extern float harmonics_T_phase[15];
#else
extern unsigned char ucData[128];
#endif
extern volatile float Mag_R_final,Mag_S_final,Mag_T_final,Fase_R_final,Fase_S_final,Fase_T_final;
extern float Freq_final;


QueueHandle_t xQueue1;
cmdMessage_t cmdMessage1;
TimerHandle_t xSendFrameTimer;


#define PORT                        CONFIG_EXAMPLE_PORT
#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "TCP server";
static const char *TAGTX = "TCP TX";

static int s_retry_num = 0;
uint8_t send_phasors_flag = 0;

int sock;

uint16_t PMUId = 100;
int64_t time_since_boot = 0;
esp_timer_handle_t periodic_timer_FPS;

static void obtain_time(void);
static void initialize_sntp(void);
static void periodic_timer_callback(void* arg);

/* Wifi and network event handler function */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    //ESP_ERROR_CHECK(esp_event_loop_create_default());
#ifdef USE_DHCP
    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
#else
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));
    esp_netif_ip_info_t ip_info;
    //ip_info.ip.addr = ipaddr_addr("192.168.15.81");
    ip_info.ip.addr = ipaddr_addr(WIFI_IP_FIXED);
    //ip_info.gw.addr = ipaddr_addr("192.168.15.1");
    ip_info.gw.addr = ipaddr_addr(WIFI_GATEWAY_IP);
    ip_info.netmask.addr = ipaddr_addr(WIFI_MASK);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = ipaddr_addr(WIFI_DNS_IP);
    ESP_ERROR_CHECK(esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info));
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

//static void do_retransmit(const int sock)
static void do_retransmit()
{
    int len;

    uint8_t rx_buffer[128];
    pmu_cmd_frame_struct_t cmd_frame;

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
			if (send_phasors_flag == 1){
				ESP_ERROR_CHECK(esp_timer_stop(periodic_timer_FPS));
				send_phasors_flag = 0;
				ESP_LOGW(TAG, "FPS timer stopped.");
			}
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
#if 0
            ESP_LOGI(TAG, "Received %d bytes: ", len);
            for(int i=0;i<len;i++){
            	printf("%0X ",rx_buffer[i]);
            }
            printf("\r\n");
#endif
            cmd_frame.sync = rx_buffer[1] | (rx_buffer[0] << 8);
            cmd_frame.cmd = rx_buffer[15] | (rx_buffer[14] << 8);
            cmd_frame.idcode = rx_buffer[5] | (rx_buffer[4] << 8);

            if(cmd_frame.sync == ((A_SYNC_AA << 8) | A_SYNC_CMD)){
            	//ESP_LOGI(TAG,"Received command: %X",cmd_frame.cmd);
            	cmdMessage1.command = cmd_frame.cmd;
            	cmdMessage1.pmuid = cmd_frame.idcode;

            	xQueueSend( xQueue1, ( void * ) &cmdMessage1, ( TickType_t ) 0 );
            }
        }
    } while (len > 0);
}

/* Task were the socket is created and keeps listening for command comming
 * from the PDC. */
static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit();

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void vSendFrameTimerCallback( TimerHandle_t pxTimer ){
	cmdMessage1.command = SEND_1_FRAME_CMD;
	//cmdMessage1.pmuid = cmd_frame.idcode;
	xQueueSend( xQueue1, ( void * ) &cmdMessage1, ( TickType_t ) 0 );
}

static void tcp_transmit_task(void *pvParameters)
{
	uint8_t led_state = 0;
	//uint8_t counter = 0;
	cmdMessage_t pxRxedMessage;

    time_t now;
    uint32_t SOC = 0;
    uint16_t ret = 0;
    // Faction of Second
    unsigned long FracSec = 0x00000000;  //

    int written = 0;
    int len_send = 0;
    int to_write = 0;

    //float my_random;

	gpio_reset_pin(LED_ONBOARD_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_ONBOARD_GPIO, GPIO_MODE_OUTPUT);

	while(1){
		if( xQueue1 != 0 ){
			// Received command from Queue:
			if( xQueueReceive( xQueue1, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
				// pcRxedMessage now points to the struct cmdMessage_t variable posted
				// by vATask.

				if(pxRxedMessage.command == 0x05){
					ESP_LOGI(TAGTX, "PMU %d Received command %d ", pxRxedMessage.pmuid, pxRxedMessage.command);
					ret = PMU_config_frame_init(PMUId, A_SYNC_CFG2, 1617793084, 0);
					ESP_LOGI(TAG,"Config frame has: %d bytes",ret);
		            len_send = ret;//sizeof(ucData);
		            to_write = len_send;
		            while (to_write > 0) {
		                written = send(sock, ucData + (len_send - to_write), to_write, 0);
		                if (written < 0) {
		                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
		                }
		                to_write -= written;
		            }
				}
				else if(pxRxedMessage.command == 0x01){ // Stop transmission.
					ESP_LOGI(TAGTX, "PMU %d Received command %d ", pxRxedMessage.pmuid, pxRxedMessage.command);
					//xTimerStop(xSendFrameTimer, 0); // Stop the report rate timer.
					if (send_phasors_flag == 1){
						ESP_ERROR_CHECK(esp_timer_stop(periodic_timer_FPS));
						send_phasors_flag = 0;
						gpio_set_level(LED_ONBOARD_GPIO, 0);
					}
				}
				else if(pxRxedMessage.command == 0x02){ // Turn on transmission.
					ESP_LOGI(TAGTX, "PMU %d Received command %d ", pxRxedMessage.pmuid, pxRxedMessage.command);
					if (send_phasors_flag == 0){
						ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_FPS, FRAC_SEC_DELTA));
						send_phasors_flag = 1;
					}

				}
				else if(pxRxedMessage.command == SEND_1_FRAME_CMD){
					//my_random = (float)(esp_random()& 0x000000FF);
					Mag_R_final = 220.0 + (esp_random()& 0x000000FF)/100.0 - 1.275;
					Mag_S_final = 220.0 + (esp_random()& 0x000000FF)/100.0 - 1.275;
					Mag_T_final = 220.0 + (esp_random()& 0x000000FF)/100.0 - 1.275;
					Freq_final = 60.0 + ((esp_random()& 0x000000FF)/1000.0) - 0.1275;
					time(&now);
					if((uint32_t)now > SOC){ // New SOC
						SOC = now;
						FracSec = 0;
					}else{
						FracSec = FracSec + FRAC_SEC_DELTA;
					}
					ret = PMU_data_frame_update(PMUId, A_SYNC_DATA, (uint32_t)SOC, FracSec);
		            len_send = ret;//sizeof(ucData);
		            to_write = len_send;
		            while (to_write > 0) {
		                written = send(sock, ucData + (len_send - to_write), to_write, 0);
		                if (len_send > written){
		                	ESP_LOGW(TAG, "Send less bytes than frame size!");
		                }
		                if (written < 0) {
		                    ESP_LOGE(TAG, "Error occurred during sending data frame: errno %d", errno);
		                    if (esp_timer_is_active(periodic_timer_FPS)){
		                    	ESP_LOGW(TAG, "FPS timer active.");
		                    }else{
		                    	ESP_LOGW(TAG, "FPS timer not active.");
		                    }
							if (send_phasors_flag == 1){
								ESP_ERROR_CHECK(esp_timer_stop(periodic_timer_FPS));
								send_phasors_flag = 0;
								ESP_LOGW(TAG, "FPS timer stopped.");
								gpio_set_level(LED_ONBOARD_GPIO, 0);
							}
		                }
		                to_write -= written;
		            }
					gpio_set_level(LED_ONBOARD_GPIO, led_state);
					led_state = !led_state;
				}
				else if(pxRxedMessage.command == TEMP_FRAME_CMD){
					ESP_LOGI(TAGTX, "Command temp");
				    //int64_t time_since_boot = esp_timer_get_time();
				    ESP_LOGI(TAG, "Time since boot: %lld us", time_since_boot);
					//gpio_set_level(BLINK_GPIO, led_state);
					//led_state = !led_state;
				}
				else{
					ESP_LOGI(TAGTX, "Command not recognized!");
				}
			}
		}
	    /* Set the GPIO level according to the state (LOW or HIGH) */
#if 0
	    gpio_set_level(BLINK_GPIO, led_state);
	    led_state = !led_state;

	    if (counter > 10){
	    	time(&now);
	    	localtime_r(&now, &timeinfo);
	    	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	    	ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
	    	counter = 0;
	    }
	    counter++;
		vTaskDelay(500 / portTICK_PERIOD_MS);
#endif
	} // while(1)

} // tcp_transmit_task

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
    gpio_set_level(LED_ONBOARD_GPIO, 1);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create a queue capable of containing 10 cmdMessage_t values.
    xQueue1 = xQueueCreate( 10, sizeof(cmdMessage_t) );
    if (xQueue1 == 0){
    	ESP_LOGE(TAG, "Failed to create the xQueue1!");
    }

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
			.dispatch_method = 1, //ESP_TIMER_ISR,
            .name = "FPStimer"
    };
    /* Create timer but not start it yet: */
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer_FPS));


    /* Start wifi connection */
    wifi_init_sta();

    time_t now;
    struct tm timeinfo;
    time(&now);
    char strftime_buf[64];

    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
    	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    	ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    // Set timezone to China Standard Time
    setenv("TZ", "UTC−03", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Brasilia is: %s", strftime_buf);

#if 0 // Call this periodically to adjust the internal clock:
    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        (long)outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
#endif

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET6, 4, NULL);
#endif
    xTaskCreate(tcp_transmit_task, "tcp_transmit", 4096, NULL, 5, NULL);

    /* Creating a timer to schedule the transmisson o data frames. In the timer callback,
     * the same command Queue is used. There is a custom command to signal the transmission
     * of a new frame.*/
    xSendFrameTimer = xTimerCreate("Timer",( SEND_DATA_FRAME_PERIOD_MS / portTICK_PERIOD_MS ),pdTRUE,( void * )1,vSendFrameTimerCallback);
    if( xSendFrameTimer == NULL ){
    	ESP_LOGE(TAG, "Failed to create the xSendFrameTimer!");
    }

}


static void obtain_time(void)
{

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 5;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    //ESP_ERROR_CHECK( example_disconnect() );
}


static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    //sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(0, "a.ntp.br");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

/* Periodic callback of the High Resolution Timer used to generate de FPS rate. */
static void periodic_timer_callback(void* arg)
{
	BaseType_t xHigherPriorityTaskWokenByPost;
	// We have not woken a task at the start of the ISR.
	xHigherPriorityTaskWokenByPost = pdFALSE;
	cmdMessage1.command = SEND_1_FRAME_CMD;
	xQueueSendFromISR(xQueue1,( void * ) &cmdMessage1, &xHigherPriorityTaskWokenByPost);
    time_since_boot = esp_timer_get_time();
}
