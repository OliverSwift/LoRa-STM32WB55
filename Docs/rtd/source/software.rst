Software
========

.. _I-CUBE-LRWAN: https://www.st.com/en/embedded-software/i-cube-lrwan.html

Software packages and Licenses
------------------------------

+--------------+------------+
| Package      | Version    |
+==============+============+
| STM32CubeIDE | 1.7        |
+--------------+------------+
| STM32CubeMX  | 6.3.0      |
+--------------+------------+
| FW_WB        | 1.12.1     |
+--------------+------------+
| I-CUBE-LRWAN | 2.0        |
+--------------+------------+

Major part of the code is covered by ST and Semtech sublicensing. Check out respective agreements for proper use:

+-----------------------------+--------------------+-------------------------------------------+
| Component                   | Copyright          | License                                   |
+=============================+====================+===========================================+
| Original application source | STMicroelectronics | `ST License agreement`_                   |
+-----------------------------+--------------------+-------------------------------------------+
| LoRaWAN® stacks             | Semtech            | `BSD Revised Licensed for Semtech parts`_ |
+-----------------------------+--------------------+-------------------------------------------+
| Cortex®-M CMSIS             | ARM Ltd            | `BSD-3-Clause`_ or `Apache License 2`_    |
+-----------------------------+--------------------+-------------------------------------------+

.. _ST License agreement: https://www.st.com/content/ccc/resource/legal/legal_agreement/license_agreement/group0/39/50/32/6c/e0/a8/45/2d/DM00218346/files/DM00218346.pdf/jcr:content/translations/en.DM00218346.pdf
.. _BSD Revised Licensed for Semtech parts: https://opensource.org/licenses/BSD-3-Clause
.. _BSD-3-Clause: https://opensource.org/licenses/BSD-3-Clause
.. _Apache License 2: https://opensource.org/licenses/Apache-2.0

Development requirements
------------------------

My favorite development environment is Linux but ST tools are available for Mac and Windows as well. The first two are easier to setup, usually no issues or driver problems. Well, real tech savvy's know that already.

Although I'm not a great fan of IDEs, STM32CudeIDE works ok and ST plugins helped (MX, Software expansions download,...).
STMicro provides a software expansion package for some LoRa shields. As linked above it's the I-CUBE-LRWAN_ and contains example projects and libraries like low level SX1272 drivers (through the SPI) and the LoRaWAN stack (`1.0.3 compatible`_).

.. _1.0.3 compatible: https://lora-alliance.org/resource_hub/lorawan-specification-v1-0-3/

The project can easily be open in STM32CudeIDE without dependencies. All code is there no need to install WB or LoRaWAN firmare from ST. Open  the project and build it (tested on Linux and MacOS).
However, you need the BLE Stack firmware installed for CPU2. Here, **stm32wb5x_BLE_Stack_full_fw.bin** has been flashed. Every stacks firmware are provided in **Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x** directories of STM32CubeWB firmware package as well as how to program the boards for this. It needs to be done **once**.

The whole experiment needed some extra tools. I used the mighty `nRF52840 USB Dongle`_ with `NRF Connect For Desktop 3.7.0`_ (available on Linux, Mac and Windows) from Nordic Semiconductor to test BLE connection. I know that smartphones can do that, I use `LightBlue`_ on iOS or Android, but when you mess about UUIDs or Device names the platforms tend to become lost whith their cached data, so I like to figure out what's going on with development tools like Nordic ones.

For the LoRaWAN part, `TTN`_ is just great and I chose to buy an inexpensive LoRaWAN gateway, a TTIG (cheaper than RAK hats for Raspberry Pi) for local tests. The gateway is hardcoded to TTN services, but there must be a way to connect to your own LNS.

.. _nRF52840 USB Dongle: https://www.nordicsemi.com/Products/Development-hardware/nRF52840-Dongle
.. _NRF Connect For Desktop 3.7.0: https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-desktop
.. _LightBlue: https://punchthrough.com/lightblue/
.. _TTN: https://www.thethingsnetwork.org/


