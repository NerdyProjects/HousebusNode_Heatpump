## Panasonic Aquarea Heatpump monitoring solution

This project implements monitoring and basic controlling of a panasonic aquarea heatpump in an environment with a heatmeter and an electricity meter.
Especially the controlling is still to be done.

### Hardware / Libraries
This project is made to be run on a [HouseBus Node](https://github.com/NerdyProjects/HouseBusNode) but follows the Arduino / Platform IO project style and should be easily portable to other microcontrollers supported by Arduino/PlatformIO.
* Basically an STM32F072 board with CAN enabled.
* STM32Duino Arduino Core.
* uavcan v1 for communication via CAN bus, implemented using the [107 systems Arduino UAVCAN library](https://github.com/107-systems/107-Arduino-UAVCAN).
* Basic Arduino STM32F072 CAN implementation from [nopnop2002](https://github.com/nopnop2002/Arduino-STM32-CAN/tree/master/stm32f072)
* CAN bootloader from the original HouseBus Node project - use a standard linker file if the application is to be used standalone.

### Connections
* UART to Panasonic Aquarea heatpump (Thanks to [HeishaMon](https://github.com/Egyras/HeishaMon) for all the protocol reverse engineering!)
* UART to UART/MBUS Adapter to Engelmann Sensostar U heat meter, could with little software changes work with any other MBUS heatmeter or MBUS/Infrared
* 1x UART in Rx Only mode to read Easymeter Q3DA electricity meter
* (Planned: Second Q3DA electricity meter for electrical heater)
* CAN to house bus

### Usage
Connect everything together, see source code for connection details.
If you don't want to use the CAN bootloader, you have to remove the custom linker file in the custom board settings (`boards/`).
Be aware, that the STM32 uses 3.3 V I/O level, but has mostly 5V tolerant pins. I went safe and added a 360 ohms serial resistor and a protection diode to 3.3V on the heatpump lines (also as they go outdoor).
The node starts transmitting UAVCAN messages for all the connected meters regularly, listen to them using [Yakut](https://github.com/UAVCAN/yakut) or write them to influx db with [UAVCANv1 InfluxDB writer](https://github.com/NerdyProjects/uavcanv1-influxdb-writer).
Basic control is implemented with the `uavcan.node.ExecuteCommand.1.1` command. See source code for details as this is WIP.


