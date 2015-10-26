
GLOBAL_DEFINES += CONSOLE_ENABLE_WL

NAME := App_ws2812

$(NAME)_SOURCES := ws2812.c 

#$(NAME)_COMPONENTS := libraries/utilites/command_console
$(NAME)_COMPONENTS := utilities/command_console \
					  utilities/command_console/wifi

LWIP_NUM_PACKET_BUFFERS_IN_POOL := 2

WIFI_CONFIG_DCT_H := wifi_config_dct.h
