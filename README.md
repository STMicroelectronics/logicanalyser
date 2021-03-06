# logicanalyser

This project contains the source code of the STM32MP157 Cortex-M4 firmware for the logic analyzer demonstration (see details in https://wiki.st.com/stm32mpu/wiki/How_to_exchange_large_data_buffers_with_the_coprocessor_-_example) on top of STM32MP1 MMDV-3.0.0.

This demonstration is planned to be executed on the STM32MP157 Discovery kits boards.

This project is linked with the https://github.com/STMicroelectronics/meta-st-stm32mpu-app-logicanalyser meta layer (logic analyzer demonstration based on GTK GUI solution). In particular, the STM32P157 Cortex-M4 firmware (how2elbd04140.elf), generated thanks to this project, must be manualy copied in the meta layer.

This project requires STM32CubeMX and STM32CubeIDE.

## Installation of the project
Get the logicanalyser project:
 > cd [your STM32CubeIDE workspace]<br />
 > git clone https://github.com/STMicroelectronics/logicanalyser.git -b master

Please note the following files:
* LA_M4_FW.ioc: STM32CubeMX project
* CM4/.cproject: STM32CubeIDE project
* CM4/Debug/how2elbd04140.elf: firmware

The source code contains all the files needed to build the firmware: there is no dependency on any other package or repository.<br />
The source code added on top of the code generated by STM32CubeMX is in user sections.

## Known issues
* STM32CubeMX: problem with the the OpenSTLinux "DeviceTree Root Location" setting<br />
The absolute path to the OpenSTLinux "DeviceTree Root Location" is hard-coded in the Project Manager. Consequently, to modify this path, please edit the "LA_M4_FW.ioc" file and set the "ProjectManager.DeviceTreeLocation" setting with your own path.
