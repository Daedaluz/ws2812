#include "wiced.h"
#include "platform.h"
#include "platform_peripheral.h"

#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#define RX_WAIT_TIMEOUT        (1*SECONDS)
#define PORTNUM                (50007)           /* UDP port */

static wiced_result_t process_received_udp_packet( );

#define BUFFER_SIZE (8*3*60*20)

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
};

static wiced_timed_event_t process_udp_rx_event;
static wiced_udp_socket_t  udp_socket;

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
	.DMA_Priority = DMA_Priority_VeryHigh,
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

static char init_data[] = {0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00};

void application_start(void)
{
    wiced_interface_t interface;
    wiced_result_t result;

    wiced_init( );
	memset(write_buffer, 0, BUFFER_SIZE);
	ws2812_init();
	write_ws2812(0, 12, init_data);
    result = wiced_network_up_default( &interface, &device_init_ip_settings );

    if( result != WICED_SUCCESS ) {
        printf("Bringing up network interface failed !\r\n");
    }

    /* Create UDP socket */
    if (wiced_udp_create_socket(&udp_socket, PORTNUM, interface) != WICED_SUCCESS) {
        WPRINT_APP_INFO( ("UDP socket creation failed\n") );
    }
    wiced_rtos_register_timed_event( &process_udp_rx_event, WICED_NETWORKING_WORKER_THREAD, &process_received_udp_packet, (1*SECONDS)/50, 0 );
}


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
	uint32_t data_len = MIN(rx_data_length, BUFFER_SIZE-(3*8*3));
	write_ws2812(0, data_len, rx_data);
    WPRINT_APP_INFO ( ("UDP Rx: \"%.2x\" from IP %u.%u.%u.%u:%d\n", *rx_data,
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 24 ) & 0xff ),
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 16 ) & 0xff ),
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  8 ) & 0xff ),
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  0 ) & 0xff ),
                                                                  udp_src_port ) );
    wiced_packet_delete( packet );

    return WICED_SUCCESS;
}

