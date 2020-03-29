# S32K Libuavcan V1
### Bare-metal media layer driver for the NXP S32K14x family of automotive-grade microcontrollers, featuring CAN-FD running at 4 Mb/s and 1 Mb/s in data and nominal phases, respectively.
An example project of it's usage for custom applications, and file dependencies used, is available in this **[Demo.](https://github.com/noxuz/libuavcan_demo)**
| Peripheral used by this driver | Resources utilized |
| ------------- | ------------- |
| LPIT  | Channels 0,1 and 2, 3rd channel is available |
| FlexCAN | All message buffers from each instance; the frame's reception interrupt handler is enabled and its priority is set to default, could be modified to meet application-specific requirements  |


| Clocks in Normal RUN | Frequency set |
| ------------- | ------------- |
| CORE_CLK  | 80Mhz  |
| SYS_CLK | 80Mhz  |
| BUS_CLK  | 40Mhz  |
| FLASH_CLK  | 26.67Mhz  |

| Asynchronous peripheral clock sources | Divider value |
| ------------- | ------------- |
| SPLLDIV2  | 1  |

*Asynchronous clock dividers not mentioned are left unset and SCG registers are locked after initial setup.*
<br/>
<br/>
| Peripheral's functional clock  | Source and frequency |
| ------------- | ------------- |
| LPIT  | SPLLDIV2 @ 80Mhz  |
| FlexCAN  | SYS_CLK @ 80Mhz  |

| Pin location | MUX |
| ------------- | ------------- |
| PTE4 | CAN0 RX | 
| PTE5 | CAN0 TX |
| PTA12 | CAN1 RX |
| PTA13 | CAN1 TX |
| PTB12 | CAN2 RX |
| PTB13 | CAN2 TX |
| PTE10 | CAN0 STB° (GPIO) |
| PTE11 | CAN1 STB° (GPIO) |

*° UAVCAN UCANS32K146 node board-only setup, which uses the TJA1044 transceiver featuring standby (STB) mode.*

*S32K146 and S32K148 although having multiple CAN-FD capable FlexCAN instances, their evaluation boards (EVB's) have
 only one transceiver, the other instances's  digital signals do are set, to the board's pin headers.*

 **For further details consult the S32K1xx reference manual [here.](https://www.nxp.com/webapp/Download?colCode=S32K1XXRM)**
 ![alt text](https://s3-prod-europe.autonews.com/s3fs-public/NXP_logo%20web.jpg) 
 ### Copyright© 2020, NXP. All rights reserved.


 

