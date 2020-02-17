# Aceinna OpenRTK Navigation Project

## Introduction



## Hardware list

- Desktop/Laptop

- Aceinna OpenRTK330BA EVK (WiKi to see detail)

- ST-Link V2

  

## Software list

- OS: Win10, Mac OS, Linux

- Visual Studio Code (VS Code) with Aceinna Navigation Studio extension (ANS)

- ST-Link V2 driver

  

## Firmware structre

```
+--| OpenRTK
+----| Applications
+------| OpenRTK330LI
+--------| INS
+----------| Android
+----------| Bootloader
+----------| Web
+----| CPU
+----| Lib
+------| Algorithm
+------| Buffer
+------| Common
+------| GnssData
+------| Math
+------| UartCom
+----| Platform
+------| OpenRTK330LI
+----| RTOS
+----| Sensors
+------| OpenIMU330
+----| GnssRtk
+------| src
+----| README.md
```



## Build steps

1.  Install PlatformIO extension to Visual Studio Code, and copy the project files to PlatformIO directory

   ```
   cp Applications/OpenRTK330LI/Bootloader/board/openrtk.json .platformio/platforms/aceinna_imu/boards
   cp Applications/OpenRTK330LI/Bootloader/board/stm32f469.ld .platformio/platforms/aceinna_imu/ldscripts
   ```

   

2. Flash ST GNSS firmware into ST GNSS chipsets

   

3. Assuming ST GNSS firmware is flashed into ST GNSS chipsets, 

- Clone the openrtk repo (master branch) using git

  ```
  git clone https://github.com/Aceinna/openrtk.git
  ```

- Flash bootloader to STM32

  - Open "Bootloader" folder using VS Code

  ```
  cd openrtk/Applications/OpenRTK330BA/Bootloader
  code .
  ```

  - Click "Upload" from the PlatformIO extension (comes with ANS)

- Flash bluetooth firmware to Esp32 module

  - Connect PC to EVK with a USB-Serial converter (TODO: figure or USB - EVK jumper)
  - Run the following python (Python2.7) script to flash bluetooth firmware

  ```
  cd openrtk/Applications/OpenRTK330BA/INS/esp32_upload
  python ./upload_esp32.py
  ```

- Flash INS App to STM32 

  1. Android use case

     - Using OpenRTK GUI on Android to show positioning results via Bluetooth, 

        ```
        cd openrtk/Applications/OpenRTK330BA/INS/Android
        code .
        ```

     - Click "Upload" from the PlatformIO extension

  2. Web use case

     - Using OpenRTK GUI on web to show positioning results via Ethernet, 

        ```
        cd openrtk/Applications/OpenRTK330BA/INS/Web
        code .
        ```

     - Click "Upload" from the PlatformIO extension    
     
     - Connect the ethernet port to computer directly or other network equipment before turning on the power switch.
     
     - Vew the debugging information through the first serial port (460800,8,N,1).
     
     - The board will try dhcp function first. If the device don’t support dhcp, it will use static ip. The ip information will be printed to the DEBUG serial port.

			The default configuration :

				Ip:192.168.1.110
				Netmast:255.255.255.0
				Gateway:192.168.1.1
		  
     - The computer should be in the same gateway as the board. Then, you can visit the webserver interface to config the ntrip client through browser. The url is the ip of the board.
     
			Configuration:
		  
				Username : the username of your aceinna account.
				Apikey : the apikey of your account.
					
				Don’t modify the Ethnet Parameters if not necessary.
				When you can only use static ip, please remember your configuration.
					
				RTK Ntrip : Local Service (Aceinna) / Other Ntrip
				Url: 106.12.40.121
				Port: 2201(LocalRTK) / 2202(CloudRTK)
				Mount Point: /RTK (auto mount)
				The Userame and Password are only used for "Other Ntrip".

  

## LED Indication

1. boot mode

	- red and yellow leds are off
	
	- green led flash periodically 1s
	
2. app mode

	- yellow led toggle upon receiving pps
	
	- red led toggle upon receiving rtcm

	- green led flash quickly with rtk task running

## amendment record
20200211 yundong
   Solve the fly point problem of web version map
      send_packet.c 
      if(gPtrGnssSol->gpsFixType != 0)
      {
         // pS 0x7053
         type[0] = 0x70;
         type[1] = 0x53;
         continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
         SendUcbPacket(UART_USER, &continuousUcbPacket);
      }
	