# PSoC&trade; 6 MCU: Dual-CPU Shared memory application

![This project using **CY8CPROTO-062-04343W** board ( PSoC&trade; 6 MCU devices )](https://www.infineon.com/export/sites/default/_images/product/evaluation-boards/cypress-boards/CY8CPROTO-062-4343W_0.jpg_1361197165.jpg)

## Project Summary

- Target board : *CY8CPROTO-062-04343W*
- Dual Core project :

	-	CM-0+ will read adc for thermistor
	-	CM-4 will print the value ( temperature ) on terminal every 2 seconds
- Debuging ( print ) message will be sent by each core

## Requirements

- [ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- Board support package (BSP) minimum required version: 4.0.0
- Programming language: C
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu/) parts


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm® embedded compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; compiler v6.16 (`ARM`)
- IAR C/C++ compiler v9.30.1 (`IAR`)


## Location of Linker 

- CM-0+ :  [.\bsps\TARGET_APP_CY8CPROTO-062-4343W\COMPONENT_CM0P\TOOLCHAIN_GCC_ARM\linker.ld](.\bsps\TARGET_APP_CY8CPROTO-062-4343W\COMPONENT_CM0P\TOOLCHAIN_GCC_ARM\linker.ld)


- CM-4  :  [.\bsps\TARGET_APP_CY8CPROTO-062-4343W\COMPONENT_CM4\TOOLCHAIN_GCC_ARM\linker.ld](.\bsps\TARGET_APP_CY8CPROTO-062-4343W\COMPONENT_CM4\TOOLCHAIN_GCC_ARM\linker.ld)

## Tutorial & Setup

- Modify & remap memory for both linker script
- Create section of memory
- Declare a variable ( shared ) as a part of section using attribute

## Modify & remap memory for both linker

- **Modify** linker.ld of CM-0+ <br>
	**Change line 66 - 67**
	
	66	ram               (rwx)   : ORIGIN = 0x08000000, LENGTH = 0x2000
    67	flash             (rx)    : ORIGIN = 0x10000000, LENGTH = 0x4400 

 **become** 	


    66	ram               (rwx)   : ORIGIN = 0x08000000, LENGTH = 0x4000
    67	flash             (rx)    : ORIGIN = 0x10000000, LENGTH = 0x10000
    68	shared_ram        (rwx)   : ORIGIN = 0x08004000, LENGTH = 0x2000

since default flash-size of CM-0+ is 0x4400, that's to small for our project 
so we need to enlarge it
- Change the FLASH_BASE on **Makefile** of [CM-0+](.\..\Dual-CPU_IPC\proj_cm0p\Makefile) line **76**
	
	DEFINES+=CY_CORTEX_M4_APPL_ADDR=CY_FLASH_BASE+0x4400U

** to **

	DEFINES+=CY_CORTEX_M4_APPL_ADDR=CY_FLASH_BASE+0x10000U
	
- Create a new memory sections<br>
	**.mysection**, just add this code to the linker. 
	
	.mysection(NOLOAD):
	{
		*(.mysection)
	}> shared_ram
**I put that script on line 319**

- **Modify** linker.ld of CM-4<br>
	Change the value of line 51 : **FLASH_CM0P_SIZE  = 0x4400;** to **0x10000**<br>
	*Value* must be same with **LENGTH** of **flash** in CM-0+ linker script
	
	FLASH_CM0P_SIZE  = 0x10000;

Change line number **73 - 75**
    
    73 	ram               (rwx)   : ORIGIN = 0x08002000, LENGTH = 0xFD800
    74 	flash             (rx)    : ORIGIN = 0x10000000, LENGTH = 0x200000
    75

Value of ram's LENGTH from **0xFD800** becomes 0xF9800 because we just modified ram size of CM-0+<br> 
and we took **0x2000** size for **shared_ram**. <br>
**0xF9800** = **0xFD800** - ( **0x2000** + **0x2000** ) <br>
**For memory adjusting or remap, please refer to memory map in datasheet **

	73	shared_ram        (rwx)    : ORIGIN = 0x08004000, LENGTH = 0x2000
    74	ram               (rwx)    : ORIGIN = 0x08006000, LENGTH = 0xF9800
    75	flash             (rx)     : ORIGIN = 0x10000000, LENGTH = 0x200000
    
-	Create same section of shared memory in CM-4 linker script, so the *section* can be recognized by our **main.c** of CM-4<br>
	I put the declaration of section on line **323** - **326** 

	323	.mysection(NOLOAD):
	324	{
	325	    *(.mysection)
	326	}> shared_ram
	
- 	Finally, we can declare one or more same variable using **attribute-section** inside **both main.c** of CM-4 and CM-0+<br>
	in this case, i want use a float - variable and some buffer char
	
	>char buf_txt[32] __attribute__ ((section(".mysection")));<br>
	>float temperature __attribute__ ((section(".mysection")));
