/*******************************************************************************
* File Name: bt_app.h
*
* Description: This file is the public interface of bt_app.c source file
*
* Related Document: README.md
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
 * Include guard
 ******************************************************************************/
#ifndef BT_APP_H
#define BT_APP_H

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "cybt_platform_config.h"
#include "cycfg_gatt_db.h"


#define BT_TASK_PRIORITY             (2u)
#define BT_TASK_STACK_SIZE           (1024u)

#define DOOR_STATUS_CCCD_HANDLE          (3)
#define DOOR_STATUS_VALUE_HANDLE         (2)

/*******************************************************************************
* Global constants
*******************************************************************************/
/* Notification parameters */
enum
{
    NOTIFIY_OFF,
    NOTIFIY_ON,
    INDICATION_ON
};


enum
{
    CMD_MODE=2,
    CMD_TRIGGER= 1,
};


enum
{
    PHOTO_MODE=0,
    VIDEO_MODE= 1,
    TWRAP_MODE= 2,
};

/*******************************************************************************
 * Extern Variables
 ******************************************************************************/
extern TaskHandle_t  bt_task_handle;
extern volatile uint16_t bt_connection_id;
extern bool bt_connected;
/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void bt_task(void* param);
wiced_result_t bt_app_management_cb(wiced_bt_management_evt_t event,
                                    wiced_bt_management_evt_data_t *p_event_data);
wiced_bt_gatt_status_t bt_app_gatt_write_data(uint8_t handle , uint8_t *value, uint8_t len);
#endif /* BT_APP_H */
