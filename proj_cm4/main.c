/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM4 in the the Dual CPU Empty 
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "string.h"
#include "stdio.h"
#include "cycfg_peripherals.h"
#include "ipc_def.h"

char buf_txt[32] __attribute__ ((section(".mysection")));
float temperature __attribute__ ((section(".mysection")));
uint32_t timeTick __attribute__ ((section(".mysection")));

uint32_t ticking = 0;
void count_tick (){
	ticking++;
}
uint32_t getTick(){return ticking;}


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();
    init_cycfg_peripherals();
    /* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 115200);

	/* Unlock the semaphore and wake-up the CM0+ */
	Cy_IPC_Sema_Clear(SEMA_NUM, false);

	__SEV();

	printf("\x1b[2J\x1b[;H");
	printf("****************** "
		    "Sample of Dual Core Shared Memory"
		    "****************** \r\n");

	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, (8000000/1000)-1);
	Cy_SysTick_SetCallback(0,count_tick);

	uint32_t timeMillis = 0;
	timeTick = 0;
    for (;;)
    {
    	if( getTick() - timeMillis >= 1000 )
    	{
    		if (Cy_IPC_Sema_Set(SEMA_NUM, false) == CY_IPC_SEMA_SUCCESS)
			{
				printf("Message sent from CM4\r\n");
				// copy SysTick Value to shared-memory
				timeTick =  getTick();
				// check if there is data buffer-txt in shared memory, tries to print it
				if( strlen(buf_txt)> 0)
				{
					printf(buf_txt);
					memset(buf_txt,'\0', 32);
				}
				while (CY_IPC_SEMA_SUCCESS != Cy_IPC_Sema_Clear(SEMA_NUM, false));

			}

			printf(" CM-4 time tick = %d\r\n", getTick());
			timeMillis = getTick();
    	}
    }
}

/* [] END OF FILE */
