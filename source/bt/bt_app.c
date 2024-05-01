/*******************************************************************************
* File Name: bt_app.c
*
* Description: This file contains the task that handles bluetooth events and
* notifications.
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
 * Header file includes
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "cycfg_gap.h"
#include "cybsp_bt_config.h"
#include "cycfg_gatt_db.h"
#include "cycfg_bt_settings.h"
#include "wiced_bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "bt_app.h"
#include "board.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GOPRO3 	0xfbu, 0x34u, 0x9bu, 0x5fu, 0x80u, 0x00u, 0x00u, 0x80u, 0x00u, 0x10u, 0x00u, 0x00u, 0xa6u, 0xfeu, 0x00u, 0x00u
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void bt_app_init(void);
wiced_bt_gatt_status_t bt_app_gatt_event_cb(wiced_bt_gatt_evt_t event,
                                            wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t bt_app_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t 
                                                                    *p_conn_status);
wiced_bt_gatt_status_t bt_app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req);
wiced_bt_gatt_status_t bt_app_gatt_req_write_value(uint16_t attr_handle,
                                                    uint8_t *p_val, uint16_t len);
wiced_bt_gatt_status_t bt_app_gatt_req_write_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *p_write_req,
                                                uint16_t len_req);
wiced_bt_gatt_status_t bt_app_gatt_req_read_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_t *p_read_req,
                                                uint16_t len_req);
wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler (uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_by_type_t *p_read_req,
                                                uint16_t len_requested);
wiced_bt_gatt_status_t bt_app_gatt_enable_notification(uint8_t handle ,
												uint8_t value);
wiced_result_t bt_app_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,
												uint8_t *p_adv_data);
static gatt_db_lookup_table_t *bt_app_find_by_handle(uint16_t handle);
static void* bt_app_alloc_buffer(int len);
static void  bt_app_free_buffer(uint8_t *p_event_data);
static void  bt_print_bd_address(wiced_bt_device_address_t bdadr);
wiced_bt_gatt_status_t bt_app_gatt_write_goprodLED( uint8_t onoff);
wiced_bt_gatt_status_t bt_app_gatt_write_goprodata( uint8_t cmd, uint8_t value);
/*******************************************************************************
 * Structures
 ******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

static uint8_t door_service_uuid[16u] ={GOPRO3};

uint8_t gopro_cam_uuid[2] = {0xA6 , 0xFE}; //0x2A00
static uint16_t door_service_handle = 0;
TaskHandle_t  bt_task_handle;
uint8_t notification_data[8];
volatile uint8_t bt_active;
/* Holds the connection ID */
volatile uint16_t bt_connection_id = 0;
volatile wiced_bt_device_address_t bt_peer_addr;
volatile uint16_t bt_peer_addrtype = 0;
uint16_t temperature;
uint8_t door_central_status, door_fr_status,door_fl_status,door_rr_status,door_rl_status = 0xff;
volatile uint8_t notification_enabled=0;

static bool bt_peer_device_found = FALSE;
/* Flag to denote status of BLE connection. */
bool bt_connected = FALSE;
bool bt_pairing_done = FALSE;
/**
 * Typdef for function used to free allocated buffer to stack
 */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************************
* Function Name: bt_task
********************************************************************************
* Summary:
*  Task that handles Bluetooth initialization and updates GATT notification data.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void bt_task(void* param)
{
    static uint32_t button_id;

    /* Suppress warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block till a notification is received. */
        xTaskNotifyWait(0, 0, &button_id, portMAX_DELAY);

        if(bt_connected && bt_pairing_done)
        {

        	if(button_id == 2)
        	{

        	}
        	else if(button_id == 3)
        	{

        	}
        }
        else
        {

        	if(button_id == 1)
        	{
        		bt_app_init();
        	}
        }


        /* do nothing */

    }
}

/*******************************************************************************
 * Function Name: bt_app_init
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is 
 *   called from the BT management callback once the LE stack enabled event 
 *   (BTM_ENABLED_EVT) is triggered This function is executed in the
 *    BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void bt_app_init(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(bt_app_gatt_event_cb);
    printf("GATT event handler registration status: %d \r\n",status);

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %d \r\n",status);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(TRUE, FALSE);


    result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_LOW_DUTY,TRUE,
                                    bt_app_scan_result_cback);

    /* Failed to start advertisement. Stop program execution */
    if ((WICED_BT_PENDING == result) || (WICED_BT_BUSY == result))
    {
        printf("Bluetooth scanning started.....\r\n");
    }
    else
    {
        printf("Failed to start scanning with error code: 0x%x\r\n", result);
        CY_ASSERT(0);
    }

    board_led_set_blink(USER_LED1, BLINK_SLOW);

}


/*******************************************************************************
* Function Name: bt_app_management_cb
********************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events
*   from the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event 
*                                                 structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t bt_app_management_cb(wiced_bt_management_evt_t event,
                                   wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t local_bda = {0};

    printf("Bluetooth app management callback: 0x%x\r\n", event);

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr(cy_bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(local_bda);
                printf("Bluetooth local device address: ");
                bt_print_bd_address(local_bda);

                /* Enter deep sleep mode */
                cyhal_syspm_deepsleep();

            }
            else
            {
                printf("Bluetooth enable failed, status = %d \r\n",
                                                p_event_data->enabled.status);
            }
            break;

        case BTM_DISABLED_EVT:
            break;
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_MASK;//BTM_LE_AUTH_REQ_MASK
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = 0xff;//BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;//BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = 0xff;//BTM_LE_KEY_PENC |BTM_LE_KEY_PID;//|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
       	    //memcpy(p_event_data->pairing_io_capabilities_ble_request.bd_addr, gopro_cam.peer_addr, sizeof(gopro_cam.peer_addr));
            break;
        case BTM_PAIRING_COMPLETE_EVT:
			{

				wiced_bt_dev_ble_pairing_info_t * p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

				printf( "Pairing Complete: %d\n", p_info->reason );

				if(p_info->reason == 0)
				{
					bt_pairing_done = 1;
				}
				else
				{
					bt_pairing_done = 0;
				}

			}
			break;

		case BTM_ENCRYPTION_STATUS_EVT:
			{
				wiced_bt_dev_encryption_status_t * p_status =
					&p_event_data->encryption_status;

				printf( "encryption status: bd ( %B ) res %d\n",
						p_status->bd_addr,
						p_status->result);
			}
			break;
	    case BTM_SECURITY_REQUEST_EVT:
	        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
	        break;

	    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:

			result = wiced_bt_dev_sec_bond( bt_peer_addr, bt_peer_addrtype,BT_TRANSPORT_LE,0, NULL );
			printf( "Start pairing status  0x%x \r\n", result );

	    	break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
            /* (sample app does not store keys to NVRAM).  New local identity keys will be generated */
            result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
             printf("Bluetooth scan %s \r\n", (BTM_BLE_SCAN_TYPE_NONE !=
                               p_event_data->ble_scan_state_changed) ?
                                                 "started":"stopped");
             if(bt_peer_device_found)
             {
                 printf("Peer Device address is: ");
                 bt_print_bd_address(bt_peer_addr);
                 printf("\r\n");

                 /* Initiate the connection */
                 if(wiced_bt_gatt_le_connect(bt_peer_addr,
                		 	 	 	 	 	 bt_peer_addrtype,
                                            BLE_CONN_MODE_HIGH_DUTY,
                                             TRUE)!= TRUE)
                 {
                     printf("Bluetooth connection failed!\r\n");
                 }
             }
             break;
        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Bluetooth connection parameter update status:%d\n \
                    parameter interval: %d ms\n \
                    parameter latency: %d ms\n \
                    parameter timeout: %d ms\r\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    p_event_data->ble_connection_param_update.supervision_timeout);
            result = WICED_SUCCESS;

            if(bt_pairing_done)
            {
                board_led_set_state(USER_LED1, LED_OFF);
              //  cyhal_system_delay_ms(500);
			//	bt_app_gatt_write_goprodata(CMD_MODE, PHOTO_MODE);
            	//cyhal_system_delay_ms(500);
				//bt_app_gatt_write_goprodata(CMD_MODE, PHOTO_MODE);
                bt_app_gatt_write_goprodLED(2);
               // vTaskDelay(600);
              //  bt_app_gatt_write_goprodLED(0);

            }
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            /* Print the updated BLE physical link*/
            printf("Bluetooth phy update selected TX - %dM\r\n \
                    Bluetooth phy update selected RX - %dM\r\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

        case BTM_PIN_REQUEST_EVT:
        case BTM_PASSKEY_REQUEST_EVT:
             result = WICED_BT_ERROR;
             break;

        default:
            printf("Bluetooth unhandled event: 0x%x \r\n", event);
            break;
    }
    return result;
}

/*******************************************************************************
* Function Name: bt_app_gatt_event_cb
********************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                : LE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_event_cb(wiced_bt_gatt_evt_t event,
                                        wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    /* Call the appropriate callback function based on the GATT event type, and 
     * pass the relevant event parameters to the callback function */
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            status = bt_app_gatt_conn_status_cb(&p_event_data->connection_status );
            if(WICED_BT_GATT_SUCCESS != status)
            {
               printf("GATT connection status failed: 0x%x\r\n", status);
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            status = bt_app_gatt_req_cb(p_attr_req);
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            bt_app_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)bt_app_free_buffer;
            status = WICED_BT_GATT_SUCCESS;
            break;

            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free =
                (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            status = WICED_BT_GATT_SUCCESS;
        }
            break;


        case GATT_DISCOVERY_RESULT_EVT:
            if (!memcmp(p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128,
                    door_service_uuid, sizeof(door_service_uuid)))
            {
            /* Copy the handle of sensorhub service */
            door_service_handle = p_event_data->discovery_result.discovery_data.group_value.s_handle;
            printf("GATT service handle: 0x%x\r\n",door_service_handle);
            bt_connected = TRUE;

            }
            break;

		case GATT_DISCOVERY_CPLT_EVT:
			printf("GATT attributes discovery complete\r\n");

			break;
        case GATT_OPERATION_CPLT_EVT:

            switch (p_event_data->operation_complete.op)
            {
            case GATTC_OPTYPE_READ_HANDLE:

                printf("GATT operation read status 0x%x \r\n", p_event_data->operation_complete.status);
                printf("GATT operation read handle  0x%x \r\n", p_event_data->operation_complete.response_data.handle);
                break;
            case GATTC_OPTYPE_WRITE_NO_RSP:

                printf("GATT operation write status 0x%x \r\n", p_event_data->operation_complete.status);
                printf("GATT operation write handle  0x%x \r\n", p_event_data->operation_complete.response_data.handle);
                break;
            case GATTC_OPTYPE_NOTIFICATION:

                /* Copy data to a local array variable */
                memcpy(notification_data, p_event_data->operation_complete.response_data.att_value.p_data,
                						  p_event_data->operation_complete.response_data.att_value.len);

                break;

            case GATTC_OPTYPE_CONFIG_MTU:
                printf( "GATT operation config mtu:%d\r\n", p_event_data->operation_complete.response_data.mtu);
                break;
                }

            break;

        default:
            status = WICED_BT_GATT_SUCCESS;
            break;
    }

    return status;
}

/*******************************************************************************
* Function Name: bt_app_gatt_req_cb
********************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t p_attr_req : Pointer to GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            status = bt_app_gatt_req_read_handler(p_attr_req->conn_id,
                                                  p_attr_req->opcode,
                                                  &p_attr_req->data.read_req,
                                                  p_attr_req->len_requested);
             break;

        case GATT_REQ_READ_BY_TYPE:
            status = bt_app_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                            p_attr_req->opcode,
                                                          &p_attr_req->data.read_by_type,
                                                          p_attr_req->len_requested);
            break;

        case GATT_REQ_READ_MULTI:
            break;

        case GATT_REQ_MTU:
            status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                       p_attr_req->data.remote_mtu,
                                                       CY_BT_MTU_SIZE);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             status = bt_app_gatt_req_write_handler(p_attr_req->conn_id,
                                                    p_attr_req->opcode,
                                                    &p_attr_req->data.write_req,
                                                    p_attr_req->len_requested);

             if ((GATT_REQ_WRITE == p_attr_req->opcode) &&
                 (WICED_BT_GATT_SUCCESS == status ))
             {
                 wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->data.write_req;
                 wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                                                     p_write_request->handle);
             }
             break;
        case GATT_HANDLE_VALUE_CONF:
        case GATT_HANDLE_VALUE_NOTIF:
             break;

        case GATT_HANDLE_VALUE_IND:
            printf("bt_app_gatt:ind\r\n");
            break;
        default:
            printf("bt_app_gatt: unhandled GATT request: %d\r\n", p_attr_req->opcode);
            break;
    }

    return status;
}

/***************************************************************************************
* Function Name: ble_app_scan_result_cback
****************************************************************************************
* Summary:
*   This function is registered as a callback to handle the scan results.
*   When the desired device is found, it will try to establish connection with
*   that device.
*
* Parameters:
*   wiced_bt_ble_scan_results_t *p_scan_result: Details of the new device found.
*   uint8_t                     *p_adv_data      : Advertisement data.
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
****************************************************************************************/
wiced_result_t bt_app_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,
												uint8_t *p_adv_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    uint8_t length = 0u;
    uint8_t *p_data = NULL;

    if(NULL != p_scan_result)
    {
        p_data = wiced_bt_ble_check_advertising_data(p_adv_data,
        											BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL,
                                                    &length);

        /* Check if the peer device's UUID and length */
        if ((p_scan_result->remote_bd_addr == NULL) ||
            (length != (sizeof(gopro_cam_uuid)/sizeof(uint8_t))))
        {
        	printf("Bluetooth Peer device uuid is :");
        	for(int i=0;i<16;i++)
        	{
        		printf(" %x",p_data[i]);
        	}
        	printf("\r\n");
            return WICED_BT_SUCCESS; //Skip - This is not the device we are looking for.
        }

        //printf("\r\nFound Peer Device : %B \n", p_scan_result->remote_bd_addr);
        memcpy(bt_peer_addr, p_scan_result->remote_bd_addr, BD_ADDR_LEN);
        bt_peer_addrtype =  p_scan_result->ble_addr_type;
        /* Device found. Stop scan. */
        bt_peer_device_found = TRUE;
        if((status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE,
                                               TRUE,
                                               bt_app_scan_result_cback))!= WICED_BT_SUCCESS)
        {
            printf("Bluetooth scanning status %d\r\n", status);
        }

    }

    return WICED_BT_ERROR;
}

/*******************************************************************************
 * Function Name : bt_app_gatt_req_read_by_type_handler
 * *****************************************************************************
 * Summary :
 *    Process read-by-type request from peer device
 *
 * Parameters:
 *  uint16_t                      conn_id       : Connection ID
 *  wiced_bt_gatt_opcode_t        opcode        : LE GATT request type opcode
 *  wiced_bt_gatt_read_by_type_t  p_read_req    : Pointer to read request 
 *                                                containing the handle to read
 *  uint16_t                      len_req        : Length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t  : LE GATT status
 ******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_by_type_t *p_read_req,
                                                uint16_t len_req)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = bt_app_alloc_buffer(len_req);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("bt_app_gatt:no memory found, len_req: %d!!\r\n",len_req);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, 
     * between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle)
            break;

        if ( NULL == (puAttribute = bt_app_find_by_handle(attr_handle)))
        {
            printf("bt_app_gatt:found type but no attribute for %d \r\n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            bt_app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                                len_req - used_len,
                                                                &pair_len,
                                                                attr_handle,
                                                                puAttribute->cur_len,
                                                                puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        printf("bt_app_gatt:attr not found start_handle: 0x%04x  end_handle: 0x%04x \
                                                        type: 0x%04x\r\n",
                                                        p_read_req->s_handle,
                                                        p_read_req->e_handle,
                                                        p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        bt_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)bt_app_free_buffer);
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_write_value
********************************************************************************
* Summary:
* This function handles writing to the attribute handle in the GATT database
* using the data passed from the BT stack. The value to write is stored in a
* buffer whose starting address is passed as one of the function parameters
*
* Parameters:
*  uint16_t attr_handle      : GATT attribute handle
*  uint8_t p_val            : Pointer to BLE GATT write request value
*  uint16_t len              : length of GATT write request
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_write_value(uint16_t attr_handle,
                                                    uint8_t *p_val, uint16_t len)
{
    wiced_bt_gatt_status_t gatt_status  = WICED_BT_GATT_INVALID_HANDLE;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;

    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {

        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {

                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when this attribute is written.
                 * In this case, we Initialize the characteristic value */

            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_HANDLE;
                printf("GATT write request to invalid handle: 0x%x\r\n", attr_handle);
            }
            break;
        }
    }

    if (!isHandleInTable)
    {

        switch(attr_handle)
        {
            default:
                /* The write operation was not performed for the
                    * indicated handle */
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                printf("GATT write request to invalid handle: 0x%x\n", attr_handle);
                break;
        }
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: bt_app_gatt_req_write_handler
********************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  uint16_t conn_id       : Connection ID
*  wiced_bt_gatt_opcode_t opcode        : LE GATT request type opcode
*  wiced_bt_gatt_write_req_t p_write_req   : Pointer to LE GATT write request
*  uint16_t len_req       : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_write_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *p_write_req,
                                                uint16_t len_req)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    printf("bt_app_gatt_write_handler: conn_id:%d handle:0x%x offset:%d len:%d\r\n",
                                                            conn_id, 
                                                            p_write_req->handle, 
                                                            p_write_req->offset, 
                                                            p_write_req->val_len );

    /* Attempt to perform the Write Request */
    status = bt_app_gatt_req_write_value(p_write_req->handle,
                                         p_write_req->p_val,
                                         p_write_req->val_len);

    if(WICED_BT_GATT_SUCCESS != status)
    {
        printf("bt_app_gatt:GATT set attr status : 0x%x\n", status);
    }

    return (status);
}

/*******************************************************************************
* Function Name: bt_app_gatt_req_read_handler
********************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* conn_id       : Connection ID
* opcode        : LE GATT request type opcode
* p_read_req    : Pointer to read request containing the handle to read
* len_req       : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_read_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req)
{
    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

    puAttribute = bt_app_find_by_handle(p_read_req->handle);
    if (NULL == puAttribute)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->cur_len;

    printf("bt_app_gatt_read_handler: conn_id:%d handle:0x%x offset:%d len:%d\r\n",
                                                    conn_id, p_read_req->handle,
                                                    p_read_req->offset,
                                                    attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;


    /* No need for context, as buff not allocated */
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send,
                                                                    from, NULL);

}

/*******************************************************************************
* Function Name: bt_app_gatt_conn_status_cb
********************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that 
*                                                       has connection details
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t 
                                                                    *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_BT_ERROR;

    if ( NULL != p_conn_status )
    {
        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf("Bluetooth connected with device address:" );
            bt_print_bd_address(p_conn_status->bd_addr);
            printf("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id);
            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* Send GATT service discovery request */
			wiced_bt_gatt_discovery_param_t gatt_discovery_setup;
			memset(&gatt_discovery_setup, 0, sizeof(gatt_discovery_setup));
			gatt_discovery_setup.s_handle = 1;
			gatt_discovery_setup.e_handle = 0xFFFF;
			gatt_discovery_setup.uuid.len = LEN_UUID_128;
			memcpy(gatt_discovery_setup.uuid.uu.uuid128, door_service_uuid, sizeof(door_service_uuid));

			if(WICED_BT_GATT_SUCCESS != (status = wiced_bt_gatt_client_send_discover(bt_connection_id,
											   GATT_DISCOVER_SERVICES_BY_UUID,
											   &gatt_discovery_setup)))
			{
				printf("GATT discovery request failed with error code: 0x%x \r\n",status);
			}
			else
			{
				printf("Discovering GATT...\r\n");
			}

       	    board_led_set_blink(USER_LED1, BLINK_FAST);
        }
        else
        {
            /* Device has disconnected */
            printf("Bluetooth disconnected with device address:" );
            bt_print_bd_address(p_conn_status->bd_addr);
            printf("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id);
            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;
            bt_connected = 0;
			bt_pairing_done = 0;

       	   // board_led_set_blink(USER_LED1, BLINK_SLOW);
            /* Enter deep sleep mode */
            cyhal_syspm_deepsleep();

        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*******************************************************************************
* Function Name: bt_app_gatt_enable_notification
********************************************************************************
* Summary:
* This function enables GATT notifictions.
*
* Parameters:
*  uint8_t handle     : BLE GATT event code
*  uint8_t uint8_t    : Pointer to BLE GATT event data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
********************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_enable_notification(uint8_t handle , uint8_t value)
{

	wiced_bt_gatt_write_hdr_t p_write = {0};
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;
    uint8_t cccd_val[2];
    uint8_t *notify_val = NULL;
    /* Allocate meory for data to be written on server DB and pass it to stack*/
    notify_val = bt_app_alloc_buffer(sizeof(uint16_t)); //CCCD is two bytes

    if (notify_val)
    {
    	cccd_val[0] = value;
    	memcpy(notify_val, cccd_val, sizeof(uint16_t));
    	p_write.handle   = door_service_handle + handle;
    	p_write.auth_req = GATT_AUTH_REQ_NONE;
    	p_write.len      = 2u;
    	p_write.offset = 0;
    	status = wiced_bt_gatt_client_send_write(bt_connection_id,
                                                    GATT_REQ_WRITE,
                                                    &p_write,
													notify_val,(void *)bt_app_free_buffer);
    }
    return status;
}


/*******************************************************************************
* Function Name: bt_app_gatt_enable_notification
********************************************************************************
* Summary:
* This function enables GATT notifictions.
*
* Parameters:
*  uint8_t handle     : BLE GATT event code
*  uint8_t uint8_t    : Pointer to BLE GATT event data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
********************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_write_data(uint8_t handle , uint8_t *value, uint8_t len)
{

	wiced_bt_gatt_write_hdr_t p_write = {0};
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;
    uint8_t *write_val = NULL;
    /* Allocate meory for data to be written on server DB and pass it to stack*/
    write_val = bt_app_alloc_buffer(len); //8 bytes

    if (write_val)
    {
    	memcpy(write_val, value, len);
    	p_write.handle   = door_service_handle + handle;
    	p_write.auth_req = GATT_AUTH_REQ_NONE;
    	p_write.len      = len;
    	p_write.offset = 0;
    	status = wiced_bt_gatt_client_send_write(bt_connection_id,
                                                    GATT_REQ_WRITE,
                                                    &p_write,
													write_val,(void *)bt_app_free_buffer);
    }
    return status;
}



wiced_bt_gatt_status_t bt_app_gatt_write_goprodLED( uint8_t onoff)
{

	wiced_bt_gatt_write_hdr_t p_write = {0};
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;
    uint8_t *write_val = NULL;
    uint8_t wvalue[4] ={0x03,0x5B,0x01,0x00};
    uint8_t wlen = 4;
    /* Allocate meory for data to be written on server DB and pass it to stack*/
    write_val = bt_app_alloc_buffer(4); //4 bytes

    if(!bt_pairing_done)
    	return WICED_BT_GATT_NOT_ALLOWED;

    wvalue[3] = onoff;

	printf("GoPro write LED VALUE: %d \r\n", onoff);
    if (write_val)
    {
    	memcpy(write_val, wvalue, wlen);
    	p_write.handle   = door_service_handle + 7;
    	p_write.auth_req = GATT_AUTH_REQ_NONE;
    	p_write.len      = wlen;
    	p_write.offset = 0;
    	status = wiced_bt_gatt_client_send_write(bt_connection_id,
    												GATT_REQ_WRITE,//GATT_REQ_WRITE,
                                                    &p_write,
													write_val,(void *)bt_app_free_buffer);
		if(WICED_BT_GATT_SUCCESS == status){

		//	printf("GATT Write successful!\r\n");
            /* Enter deep sleep mode */
          //  cyhal_syspm_deepsleep();
		}
		else
		{
			printf("GATT Write failed! status = 0x%x \r\n",status);
		}
    }
    return status;
}

wiced_bt_gatt_status_t bt_app_gatt_write_goprodata( uint8_t cmd, uint8_t value)
{

	wiced_bt_gatt_write_hdr_t p_write = {0};
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;
    uint8_t *write_val = NULL;
    uint8_t wvalue[4] ={0x03,0x00,0x01,0x00};
    uint8_t wlen = 4;
    /* Allocate meory for data to be written on server DB and pass it to stack*/
    write_val = bt_app_alloc_buffer(4); //4 bytes

    if(!bt_pairing_done)
    	return WICED_BT_GATT_NOT_ALLOWED;

    if(cmd == CMD_MODE)
    {
    	wlen = 4;
    	wvalue[0] =  0x03;
    	wvalue[1] = CMD_MODE;
    	wvalue[2] = 0x01;
    	wvalue[3] = value;
    }
    else
    {
    	wlen = 4;
    	wvalue[0] =  0x03;
    	wvalue[1] = CMD_TRIGGER;
    	wvalue[2] = 0x01;
    	wvalue[3] = value;
    }


	printf("GoPro write CMD: %d  VALUE: %d \r\n", cmd, value);
    if (write_val)
    {
    	memcpy(write_val, wvalue, wlen);
    	p_write.handle   = door_service_handle + 2;
    	p_write.auth_req = GATT_AUTH_REQ_NONE;
    	p_write.len      = wlen;
    	p_write.offset = 0;
    	status = wiced_bt_gatt_client_send_write(bt_connection_id,
    												GATT_REQ_WRITE,//GATT_REQ_WRITE,
                                                    &p_write,
													write_val,(void *)bt_app_free_buffer);
		if(WICED_BT_GATT_SUCCESS == status){

		//	printf("GATT Write successful!\r\n");
            /* Enter deep sleep mode */
          //  cyhal_syspm_deepsleep();
		}
		else
		{
			printf("GATT Write failed! status = 0x%x \r\n",status);
		}
    }
    return status;
}

/*******************************************************************************
 * Function Name: bt_app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 * 
 * Return:
 *  None
 *
 ******************************************************************************/
void bt_app_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/*******************************************************************************
 * Function Name: bt_app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 * 
 * Return:
 *  None
 *
 ******************************************************************************/
void* bt_app_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}

/*******************************************************************************
 * Function Name : bt_app_find_by_handle
 * *****************************************************************************
 * Summary :
 *    Find attribute description by handle
 *
 * Parameters:
 *  uint16_t handle    handle to look up
 *
 * Return:
 *  gatt_db_lookup_table_t   pointer containing handle data
 * 
 ******************************************************************************/
gatt_db_lookup_table_t  *bt_app_find_by_handle(uint16_t handle)
{
    for (uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/*******************************************************************************
* Function Name: bt_print_bd_address
********************************************************************************
* Summary: This is the utility function that prints the address of the 
*          Bluetooth device
*
* Parameters:
*  wiced_bt_device_address_t bdaddr : Bluetooth address
*
* Return:
*  None
*
*******************************************************************************/
void bt_print_bd_address(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=0;i<BD_ADDR_LEN-1;i++)
    {
        printf("%02X:",bdadr[i]);
    }
    printf("%02X\n",bdadr[BD_ADDR_LEN-1]);
}

/* END OF FILE [] */
