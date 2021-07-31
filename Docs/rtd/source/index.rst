.. LoRaWAN on STM32WB55 documentation master file, created by
   sphinx-quickstart on Fri Jul 30 08:30:27 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

LoRaWAN on STM32WB55 project
============================

The main idea is to be able to use Semtech LoRa SX1272MB2DAS shield on a STM32WB55 Nucleo board. Unfortunately code available by ST (I-CUBE-LRWAN_) for this shield is only available for a few Nucleo boards (L053RG, L073RZ, L152RE and L476RG).

.. _I-CUBE-LRWAN: https://www.st.com/en/embedded-software/i-cube-lrwan.html

The derivative ideas are to marry LoRa and BLE, nowadays many IoT objects are BLE accessible for various reasons (settings, provisonning, FUOTA, ...) but mostly because it is compatible with smartphones. The STM32WL serie has no BLE and the STM32WB serie has no LoRa capabilities.

Well, I had the opportunity to create a pair with SX1272 and WB55 and allow a two chip combination for flexible IoT project. Keep in mind that the WB55 serie is not limited to BLE. The coprocessor firmware can handle 2.4GHz sideway protocols like ZigBee and OpenThread. So the combination of long range and short range RF communications is interesting.

This project is only a port of existing codes and libraries but might save setup time for some developers as there are some technical issues to solve.

With upcoming LoRa 2.4GHz this is to be considered as transitional work. The SX1280 will combine short and long range communication in a single chip. You might want to take a `closer look`_.

.. _closer look: https://www.semtech.com/products/wireless-rf/lora-24ghz

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   hardware
   software
   achievements
   customization


Credits
=======

- STMicroelectronics_
- Semtech_
- NordicSemiconductor_

.. _STMicroelectronics: https://www.st.com
.. _Semtech: https://www.semtech.com
.. _NordicSemiconductor: https://github.com/NordicSemiconductor

