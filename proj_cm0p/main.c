/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM0+ in the the Dual CPU Empty 
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
#include "cycfg.h"
#include "cybsp.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

#include "ipc_def.h"

char buf_txt[32] __attribute__ ((section(".mysection")));
float temperature __attribute__ ((section(".mysection")));
uint32_t timeTick __attribute__ ((section(".mysection")));

#define NTC_A 	1028444e-9
#define NTC_B 	239243e-9
#define NTC_C 	156e-9
#define R0 		10000.000f
float R2Temp(float R)
{
	float ret = 0;
	ret = log(R);
	float tempInC = (1/ (NTC_A + (NTC_B * ret) + (NTC_C * ret * ret * ret))) - 273.15;
	ret = tempInC;
	return ret;
}

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

	/* Enable CM4. CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
	Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

	/* Lock the sempahore to wait for CM4 to be init */
	Cy_IPC_Sema_Set(SEMA_NUM, false);

	/* Wait till CM4 unlocks the semaphore */
	do
	{
		__WFE();
	}
	while ( Cy_IPC_Sema_Status(SEMA_NUM) == CY_IPC_SEMA_STATUS_LOCKED );


    /* Update clock settings */
	SystemCoreClockUpdate();

	// enabling vref
	Cy_SysAnalog_Init(&AREF_config);
	Cy_SysAnalog_Enable();
	// setup adc
	Cy_SAR_Init(HW_SAR_HW,&HW_SAR_config);
	Cy_SAR_Enable(HW_SAR_HW);

	int32_t value1, value2;
	uint32_t timeMillis = 0;
	uint32_t timeTicks = 0;
    for (;;)
    {
    	Cy_SysLib_Delay(10);

    	if (Cy_IPC_Sema_Set(SEMA_NUM, false) == CY_IPC_SEMA_SUCCESS)
		{
    		timeTicks = timeTick;
    		sprintf(buf_txt, "temperature = %0.3f%cC \t %d\r\n", temperature, (char)176, timeTicks);
			while (CY_IPC_SEMA_SUCCESS != Cy_IPC_Sema_Clear(SEMA_NUM, false));
		}

    	if( timeTicks - timeMillis >= 500 )
    	{
    		// read the thermistor
    		Cy_GPIO_Clr(P10_3_PORT, P10_3_NUM);
			Cy_GPIO_Set(P10_0_PORT, P10_0_NUM);
			Cy_SysLib_Delay(10);
			// read reference
			Cy_SAR_StartConvert(HW_SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
			if(Cy_SAR_IsEndConversion(HW_SAR_HW, CY_SAR_WAIT_FOR_RESULT) == CY_SAR_SUCCESS )
				value1 = Cy_SAR_GetResult16(SAR, 0);
			else
				value1 = 0;

			Cy_GPIO_Set(P10_3_PORT, P10_3_NUM);
			Cy_GPIO_Clr(P10_0_PORT, P10_0_NUM);
			Cy_SysLib_Delay(10);

			// read divided-voltage
			Cy_SAR_StartConvert(HW_SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
			if(Cy_SAR_IsEndConversion(HW_SAR_HW, CY_SAR_WAIT_FOR_RESULT) == CY_SAR_SUCCESS )
				value2 = Cy_SAR_GetResult16(SAR, 0);
			else
				value2 = 0;
			Cy_GPIO_Clr(P10_3_PORT, P10_3_NUM);
			Cy_GPIO_Clr(P10_0_PORT, P10_0_NUM);

			float thermistor = ( (float)value2*10000.f ) / (float)value1;
			temperature = R2Temp(thermistor);
			timeMillis = timeTicks; // set timeMillis from shared memory
    	}


    }
}

/* [] END OF FILE */
