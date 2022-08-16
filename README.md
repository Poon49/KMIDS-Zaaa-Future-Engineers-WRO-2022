# KMIDS-Zaaa-Future-Engineers-WRO-2022
## WRO-Future-Engineers 2022
This repository contains codes and working principle that are used to creating the autonomous vehicle for competition.

## Electromechanical parts:
Our car contains the follow parts:
1.	Raspberry Pi 400  – The brain of the robot, use to receive the frames from the camera, give an order to the medium motor and servo motor, and use for keeping the value from the Gyro sensor (BNO055).
 #### Specification 
  -  Processor: Broadcom BCM2711 quad-core Cortex-A72 (ARM v8) 64-bit SoC @ 1.8GHz 
  -  Memory: 4GB LPDDR4-3200 
  -  Connectivity: • Dual-band (2.4GHz and 5.0GHz) IEEE 802.11b/g/n/ac wireless LAN, Bluetooth 	    5.0, BLE • Gigabit Ethernet • 2 × USB 3.0 and 1 × USB 2.0 ports 
      GPIO: Horizontal 40-pin GPIO header 
  - Video & sound: 2 × micro HDMI ports (supports up to 4Kp60) 
      Multimedia: H.265 (4Kp60 decode); H.264 (1080p60 decode, 1080p30 encode); OpenGL ES 3.0 	       graphics
  - SD card support: MicroSD card slot for operating system and data storage  32 GB
      Power: 5V DC via USB connector 
  - Dimensions: 286 mm × 122 mm × 23 mm (maximum) 
      https://www.raspberrypi.com/products/raspberry-pi-400/
2.	Differential  Gear  - LEGO                
     ![image](https://user-images.githubusercontent.com/76239146/184805949-00d65f80-3247-4614-a780-27d97956860e.png)
     
3.	EV3 medium motor - LEGO
    The Medium Motor also includes a built-in Rotation Sensor (with 1-degree resolution), but it     is smaller and lighter than the Large Motor. That means it is able to respond more quickly       than the Large Motor. The Medium Motor can be programmed to turn on or off, control its           power level, or to run for a specified amount of time or rotations.
    
    ![image](https://user-images.githubusercontent.com/76239146/184805655-5e7650d8-ceb1-406e-ba12-6d49d288082d.png)
                                 

4.	Servo motor MG90S : MG90S servo, Metal gear with one bearing can rotate approximately
    180 degrees (90 in each direction), 
  - Specifications • Weight: 13.4 g • Dimension: 22.5 x 12 x 35.5
  - Stall torque: 1.8 kgf·cm (4.8V ) 
  - Operating speed: 0.1 s/60 degree •
  - Operating voltage: 4.8 V – 6V
  - Dead band width: 5 µs
  
    ![image](https://user-images.githubusercontent.com/76239146/184806310-27832e6a-8155-44fa-a336-4cb7cb3dec55.png)

5.	 BNO055 (Gyro sensor) : The BNO055 uses three triple-axis sensors to simultaneously measure tangential acceleration (via an accelerometer), rotational acceleration (via a gyroscope), and the strength of the local magnetic field (via a magnetometer). Data can then be either sent to an external microprocessor or analysed inside the sensor with an M0+ microprocessor running a proprietary fusion algorithm.
    ![image](https://user-images.githubusercontent.com/76239146/184806668-65537011-5b25-4a82-a5db-142302253826.png)
    ![image](https://user-images.githubusercontent.com/76239146/184806948-33e04e5a-727d-4141-a580-900cc4f91c1b.png)
    https://github.com/adafruit/Adafruit_BNO055                 
    https://cdn-shop.adafruit.comhttps://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdfdatasheets/BST_BNO055_DS000_12.pdf
6.	Step down 8-35V to 5V 8A 4 USB                                         
7.	USB  TTL CH340 : Support USB1.1 or USB2.0 communication                                       
8.	Open MV H7 is A Python programmable machine vision camera. The STM32H743VI ARM Cortex M7 processor running at 480 MHz with 1MB SRAM and 2MB of flash. All I/O pins output 3.3V and are 5V tolerant. The processor has the following I/O interfaces:
  ![image](https://user-images.githubusercontent.com/76239146/184808169-7c739857-0d77-401f-9831-51ee6e379104.png)                               
 https://cdn.shopify.com/s/files/1/0803/9211/files/OpenMV-H7.pdf

