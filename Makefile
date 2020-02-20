main: main.c buffer.c buffer.h datatypes.h bldc_interface.c bldc_interface.h
	gcc -o bldc main.c buffer.c buffer.h datatypes.h bldc_interface.c bldc_interface.h packet.c packet.h  bldc_interface_uart.c bldc_interface_uart.h crc.c crc.h -lpthread



	# comm_uart2.c comm_uart.h
