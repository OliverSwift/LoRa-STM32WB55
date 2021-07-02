***WIP***

# LoRa SX1272 Shield and STM32WB55

***Project status:***
- [x] **Ping Pong SubGhzPhy application**
- [ ] add BLE stack
- [ ] LPM management
- [ ] LoRaWAN example

## Goal
The main idea is to be able to use Semtech LoRa SX1272MB2DAS shield on a STM32WB55 Nucleo board. Unfortunately code available by ST ([I-CUBE-LRWAN](https://www.st.com/en/embedded-software/i-cube-lrwan.html)) for this shield is only available for a few Nucleo boards (L053RG, L073RZ, L152RE and L476RG).

The derivative ideas are to marry LoRa and BLE, nowadays many IoT objects are BLE accessible for various reasons (settings, provisonning, FUOTA, ...) but mostly because it is compatible with smartphones. The STM32WL serie has no BLE and the STM32WB serie has no LoRa capabilities.

Well, I had the opportunity to create a pair with SX1272 and WB55 and allow a two chip combination for flexible IoT project. Keep in mind that the WB55 serie is not limited to BLE. The coprocessor firmware can handle 2.4GHz sideway protocols like ZigBee and OpenThread. So the combination of long range and short range RF communications is interesting.

This project is only a port of existing codes and libraries but might save setup time for some developers as there are some technical issues to solve.

With upcoming LoRa 2.4GHz this is to be considered as transitional work. The SX1280 will combine short and long range communication in a single chip. You might want to take a [closer look](https://www.semtech.com/products/wireless-rf/lora-24ghz).

## Hardware
### Semtech
That's an easy one to come by. The [Semtech SX1272MB2xAS LoRa Mbed shield](https://www.mouser.fr/new/semtech/semtech-sx1272-mbed-shield/) is arduino compatible shield and Nucleo boards have the corresponding connector.
Technical brief:
- SX1272 Low Power Long Range Transceiver
- 868MHz to 915MHz
- 14dBm output
- LoRa modulation as well as FSK/OOK

### STMicro
As mentioned the STM32WB55 serie is the target here and the [P-Nucleo-WB55RG](https://www.st.com/en/evaluation-tools/p-nucleo-wb55.html)  is a nice board to play with.
Technical brief:
- STM32 family with M4+M0 cores (dual core)
- 2.4GHz capable with 802.15.4, BLE4/5, ZigBee, OpenThread protocol stack firmware
- On-board ST-LINK
- USB connectors, 3 LEDs, buttons
- Battery socket

### The combo
Once hooked up (it's a no brainer) here is the resulting object.
![](Docs/STM32WB55-SX1272-1.jpg) ![](Docs/STM32WB55-SX1272-2.jpg) 

Never forget to plug the antenna prior to powering up the boards. You may damage the RF amplifier output (although it's a rather low power one).
Of course, buying two of each is a bit necessary for Ping Pong.

## Software
### Licenses
99% of the code are covered by ST and Semtech sublicensing. Check out respective agreements for proper use:
- [ST License agreement SLA0044](https://www.st.com/content/ccc/resource/legal/legal_agreement/license_agreement/group0/39/50/32/6c/e0/a8/45/2d/DM00218346/files/DM00218346.pdf/jcr:content/translations/en.DM00218346.pdf)
- [BSD Revised Licensed for Semtech parts](/Middlewares/SubGHz_Phy/LICENSE.txt)

### Development requirements
My favorite development environment is Linux but ST tools are available for Mac and Windows as well. The first two are easier to setup, usually no issues or driver problems. Well, hardnuts know that already.

Although I'm not a great fan of IDEs, STM32CudeIDE works ok and ST plugins helped (MX, Software expansions download,...).
STMicro provides a software expansion package for some LoRa shields. As linked above it's the I-CUBE-LRWAN and contains example projects and libraries like low level SX1272 drivers (through the SPI) and the LoRaWAN stack (1.0.3 compatible).

### Technical achievements
#### Ping Pong SubGhzPhy example application
That was the very first setp. Making things work and I chose the low level RF transmission example for that.
Porting the ST/Semtech code was a matter of dealing with reconfiguring the WB55 to:
- match the shield PIN out (DIOx and SPI)
- properly set clocks
- setup the RTC
- deal with IRQs

Pin mapping between the [Nucleo WB55 board](https://os.mbed.com/platforms/ST-Nucleo-WB55RG/) and the [SX1272MB2DAS](https://www.mouser.fr/images/marketingid/2017/microsites/185566741/Semtech_SX1272MB2DAS_Pinout.jpg) shield:

SX1272MB2DAS | P-NUCLEO-STM32WB55 | Arduino connector pin name
-------------|--------------------|-----------------
DIO0|PC6|D2
DIO1|PA10|D3
DIO2|PC10|D4
DIO3|PA15|D5
NSS|PA4|D10
SCLK|PA5|D13
MISO|PA6|D12
MOSI|PA7|D11
RESET|PC0|A0

So nothing incompatible at first look. SPI1 matches, NSS and Reset pins are ok too. But here comes the first hickup with the IRQ lines.
PC6 uses the EXTI6 IRQ line (EXTI9_5_IRQn), that's fine, but PA10,PC10 and PA15 share the same EXTI IRQ line, that is EXTI15_10_IRQn.

_to be continued_
