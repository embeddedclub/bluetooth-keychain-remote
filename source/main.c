/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Bluetooth Server example
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "cyhal.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "wiced_bt_stack.h"
#include "cybsp_bt_config.h"
#include "cycfg_bt_settings.h"
#include "cybt_platform_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bt_app.h"
#include "timers.h"
#include "board.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable which holds the button pressed status */


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/* handler for general errors */
void handle_error(uint32_t status);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It initializes the CAN-FD channel and interrupt.
* User button and User LED are also initialized. The main loop checks for the
* button pressed interrupt flag and when it is set, a CAN-FD frame is sent.
* Whenever a CAN-FD frame is received from other nodes, the user LED toggles and
* the received data is logged over serial terminal.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t rtos_result;


    /* Initialize the board support package */
    result = cybsp_init();

    if(CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    board_init();

    printf("===========================================================\r\n");
    printf("Welcome to Bluetooth GoPro Remote\r\n");
    printf("===========================================================\r\n\n");

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(bt_app_management_cb, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if(WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth stack initialization successful!\r\n");
    }
    else
    {
        printf("Bluetooth stack initialization failed!\r\n");
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();


}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
*
* Summary:
* User defined error handling function. This function processes unrecoverable
* errors such as any initialization errors etc. In case of such error the system
* will enter into assert.
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

/* [] END OF FILE */
