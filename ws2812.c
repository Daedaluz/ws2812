#include "wiced.h"
#include "platform.h"
#include "platform_peripheral.h"

#include "command_console.h"
#include "wifi/command_console_wifi.h"

#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#define RX_WAIT_TIMEOUT        (1*SECONDS)
#define PORTNUM                (50007)           /* UDP port */


#define BUFFER_SIZE (8*3*60*20)


static wiced_timed_event_t process_udp_rx_event;
static wiced_result_t process_received_udp_packet( );
static wiced_udp_socket_t  udp_socket;
//static wiced_result_t udp_cb(wiced_udp_socket_t* socket, void* arg);
static void link_cb_up(void);
static void link_cb_down(void);

uint16_t write_buffer[BUFFER_SIZE];

static DMA_InitTypeDef dma = {
	.DMA_Channel = DMA_Channel_2,
	.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR3,
	.DMA_Memory0BaseAddr = (uint32_t)(&write_buffer[0]),
	.DMA_DIR = DMA_DIR_MemoryToPeripheral,
	.DMA_BufferSize = BUFFER_SIZE,
	.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
	.DMA_MemoryInc = DMA_MemoryInc_Enable,
	.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
	.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
	.DMA_FIFOMode = DMA_FIFOMode_Disable,
	.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
	.DMA_Mode = DMA_Mode_Circular,
	.DMA_Priority = DMA_Priority_High,
	.DMA_MemoryBurst = DMA_MemoryBurst_Single,
	.DMA_PeripheralBurst = DMA_PeripheralBurst_Single
};

void ws2812_init() {
	wiced_pwm_init(WICED_PWM_2, 800000, 0);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_Init(DMA1_Stream7, &dma);
	TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);
	DMA_Cmd(DMA1_Stream7, ENABLE);
	wiced_pwm_start(WICED_PWM_2);
}


#define CMPH 90/2
#define CMPL 40/2
void write_ws2812(uint32_t offset, uint32_t size, char* data) {
	uint32_t x = 0;
	for(x = 0; x < size; x++) {
		write_buffer[((x+offset)*8)+0] = (data[x] & 0x80) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+1] = (data[x] & 0x40) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+2] = (data[x] & 0x20) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+3] = (data[x] & 0x10) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+4] = (data[x] & 0x08) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+5] = (data[x] & 0x04) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+6] = (data[x] & 0x02) ? CMPH:CMPL;
		write_buffer[((x+offset)*8)+7] = (data[x] & 0x01) ? CMPH:CMPL;
	}
}

static char init_data[] = {0x00, 0x00, 0x10};
//static char init_data[] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
static char cmdline[300];
static char history[300*10];
static command_t commands[] = {
	WIFI_COMMANDS
	CMD_TABLE_END
};

void application_start(void)
{
    wiced_interface_t interface;
    wiced_result_t result;
	int fail = 0;
    wiced_init( );
	command_console_init(STDIO_UART, 300, cmdline, 10, history, " ");
	console_add_cmd_table(commands);
	memset(write_buffer, 0, BUFFER_SIZE);
	ws2812_init();
	write_ws2812(0, 3, init_data);
	wiced_network_register_link_callback(link_cb_up, link_cb_down, WICED_STA_INTERFACE);
    result = wiced_network_up_default( &interface, NULL);
//	result = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

    if( result != WICED_SUCCESS ) {
        printf("Bringing up network interface failed !\r\n");
		fail = 1;
    }

    /* Create UDP socket */
    if (wiced_udp_create_socket(&udp_socket, PORTNUM, interface) != WICED_SUCCESS) {
        WPRINT_APP_INFO( ("UDP socket creation failed\n") );
		fail = 1;
    } else {
	//	wiced_udp_register_callbacks(&udp_socket, udp_cb, &udp_socket);
		wiced_rtos_register_timed_event( &process_udp_rx_event, WICED_NETWORKING_WORKER_THREAD, &process_received_udp_packet, (1*SECONDS)/50, 0 );
	}
	if(!fail){
		init_data[0] = 0x10;
		init_data[1] = 0x00;
		init_data[2] = 0x00;
	} else {
		init_data[0] = 0x00;
		init_data[1] = 0x10;
		init_data[2] = 0x00;
	}
	write_ws2812(0, 3, init_data);
}

static void link_cb_up(void) {
	WPRINT_APP_INFO( ("LINK UP\n") );
	return;
}

static void link_cb_down(void) {
	WPRINT_APP_INFO( ("LINK DOWN\n") );
	return;
}

//static wiced_result_t udp_cb(wiced_udp_socket_t* socket, void* arg) {
//    wiced_packet_t*           packet;
//    char*                     rx_data;
//    static uint16_t           rx_data_length;
//    uint16_t                  available_data_length;
//    static wiced_ip_address_t udp_src_ip_addr;
//    static uint16_t           udp_src_port;
//
//    wiced_result_t result = wiced_udp_receive( socket, &packet, RX_WAIT_TIMEOUT );
//    if ( ( result == WICED_ERROR ) || ( result == WICED_TIMEOUT ) ) {
//        return result;
//    }
//    wiced_udp_packet_get_info( packet, &udp_src_ip_addr, &udp_src_port );
//    wiced_packet_get_data( packet, 0, (uint8_t**) &rx_data, &rx_data_length, &available_data_length );
//	uint32_t data_len = MIN(rx_data_length, BUFFER_SIZE-(48));
//	write_ws2812(0, data_len, rx_data);
//    wiced_packet_delete( packet );
//    return WICED_SUCCESS;
//
//}

wiced_result_t process_received_udp_packet()
{
    wiced_packet_t*           packet;
    char*                     rx_data;
    static uint16_t           rx_data_length;
    uint16_t                  available_data_length;
    static wiced_ip_address_t udp_src_ip_addr;
    static uint16_t           udp_src_port;

    wiced_result_t result = wiced_udp_receive( &udp_socket, &packet, RX_WAIT_TIMEOUT );
    if ( ( result == WICED_ERROR ) || ( result == WICED_TIMEOUT ) ) {
        return result;
    }
    wiced_udp_packet_get_info( packet, &udp_src_ip_addr, &udp_src_port );
    wiced_packet_get_data( packet, 0, (uint8_t**) &rx_data, &rx_data_length, &available_data_length );
	uint32_t data_len = MIN(rx_data_length, BUFFER_SIZE-(48));
	write_ws2812(0, data_len, rx_data);
//    WPRINT_APP_INFO ( ("UDP Rx: \"%.2x\" from IP %u.%u.%u.%u:%d\n", *rx_data,
//                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 24 ) & 0xff ),
//                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 16 ) & 0xff ),
//                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  8 ) & 0xff ),
//                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  0 ) & 0xff ),
//                                                                  udp_src_port ) );
    wiced_packet_delete( packet );

    return WICED_SUCCESS;
}

