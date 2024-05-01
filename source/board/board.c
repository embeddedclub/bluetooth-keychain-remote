/*******************************************************************************
* File Name: board.c
*
* Description: This file contains board supported API's.
*
* Related Document: See README.md
*
********************************************************************************
* $ Copyright 2023-YEAR Cypress Semiconductor $
*******************************************************************************/

/*******************************************************************************
* Header file includes
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "bt_app.h"
#include "board.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define PWM_FREQUENCY_2KHZ              (2000u)

#define PWM_DUTY_CYCLE_0                (0.0)
#define PWM_DUTY_CYCLE_50               (50.0)
#define PWM_DUTY_CYCLE_100              (100.0)

#define BOARD_TASK_PRIORITY             (configMAX_PRIORITIES - 1u)
#define BOARD_TASK_STACK_SIZE           (512u)
#define GPIO_INTERRUPT_PRIORITY (7u)
/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void board_led_init(void);
/*******************************************************************************
* Extern Variables
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* FreeRTOS task handle for board task. Button task is used to handle button
 * events */
TaskHandle_t  board_task_handle;

/* Queue handle used for LED data */
QueueHandle_t led_command_data_q;

/* PWM object */
cyhal_pwm_t pwm_obj;

extern volatile uint8_t bt_active;
volatile uint8_t gopro_active_mode=0;
volatile uint8_t gopro_active_trigger=0;
volatile bool gpio_intr_flag1 = false;
volatile bool gpio_intr_flag2 = false;
volatile bool gpio_intr_flag3 = false;
/* Variables to keep the button timings. */
uint64_t button_pushed_time = 0u;
uint32_t button1_previous_value = 1u;
uint32_t button2_previous_value = 1u;
uint32_t button3_previous_value = 1u;
uint32_t button_pushed_duration = 0u;

cyhal_gpio_callback_data_t gpio_btn_callback_data1,gpio_btn_callback_data2,gpio_btn_callback_data3;

/* button press interrupt handler */
static void gpio_interrupt_handler1(void *handler_arg, cyhal_gpio_event_t event);
static void gpio_interrupt_handler2(void *handler_arg, cyhal_gpio_event_t event);
static void gpio_interrupt_handler3(void *handler_arg, cyhal_gpio_event_t event);

/*******************************************************************************
* Function Name: board_init
********************************************************************************
*
* Summary:
*   Initialize the board with LED's and Buttons
*
* Parameters:
*   None
*
* Return:
*   cy_rslt_t  Result status
*
*******************************************************************************/
cy_rslt_t board_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    BaseType_t rtos_result;

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    board_led_init();


    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYBSP_USER_BTN_DRIVE, CYBSP_BTN_OFF);
    /* Initialize the user button */
    result = cyhal_gpio_init(P1_0, CYHAL_GPIO_DIR_INPUT,
                    CYBSP_USER_BTN_DRIVE, CYBSP_BTN_OFF);
    /* Initialize the user button */
    result = cyhal_gpio_init(P1_1, CYHAL_GPIO_DIR_INPUT,
                    CYBSP_USER_BTN_DRIVE, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    gpio_btn_callback_data1.callback = gpio_interrupt_handler1;
    gpio_btn_callback_data2.callback = gpio_interrupt_handler2;
    gpio_btn_callback_data3.callback = gpio_interrupt_handler3;
    cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                 &gpio_btn_callback_data1);
    cyhal_gpio_register_callback(P1_0,
                                 &gpio_btn_callback_data2);
    cyhal_gpio_register_callback(P1_1,
                                 &gpio_btn_callback_data3);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_BOTH,
                                 GPIO_INTERRUPT_PRIORITY, true);
    cyhal_gpio_enable_event(P1_0, CYHAL_GPIO_IRQ_BOTH,
                                 GPIO_INTERRUPT_PRIORITY, true);
    cyhal_gpio_enable_event(P1_1, CYHAL_GPIO_IRQ_BOTH,
                                 GPIO_INTERRUPT_PRIORITY, true);


    /* Create Button Task for processing board events */
    rtos_result = xTaskCreate(board_task,"Board Task", BOARD_TASK_STACK_SIZE,
                            NULL, BOARD_TASK_PRIORITY, &board_task_handle);
    if( pdPASS != rtos_result)
    {
        printf("Failed to create board task.\r\n");
        CY_ASSERT(0u);
    }

    return result;
}

/*******************************************************************************
* Function Name: board_led_init
********************************************************************************
*
* Summary:
*   Initialize the leds with PWM
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void board_led_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the PWM for USER_LED1 */
    result = cyhal_pwm_init_adv(&pwm_obj, CYBSP_USER_LED2, NC,
                                     CYHAL_PWM_RIGHT_ALIGN, true, 0u, true, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM init failed with error: %lu\r\n", (unsigned long) result);
    }

    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_obj);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM start failed with error: %lu\r\n", (unsigned long) result);
    }

}


/*******************************************************************************
* Function Name: board_led_set_brightness
********************************************************************************
*
* Summary:
*   Set the led brightness over PWM
*
* Parameters:
*   index: index of LED
*   value: PWM duty cycle value
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_brightness(uint8_t index, uint8_t value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj, value, PWM_FREQUENCY_2KHZ);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: board_led_set_state
********************************************************************************
*
* Summary:
*   Set the led state over PWM
*
* Parameters:
*   index: index of LED
*   value: ON/OFF state
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_state(uint8_t index, bool value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj,
                                value?PWM_DUTY_CYCLE_0:PWM_DUTY_CYCLE_100, PWM_FREQUENCY_2KHZ);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: board_led_set_blink
********************************************************************************
*
* Summary:
*   Set the led frequency for PWM
*
* Parameters:
*   index: index of LED
*   value: value of PWM frequency
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_blink(uint8_t index, uint8_t value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj, PWM_DUTY_CYCLE_50, value);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}


/*******************************************************************************
* Function Name: board_task
********************************************************************************
*
* Summary:
*   This task initialize the board and button events
*
* Parameters:
*   void *param: Not used
*
* Return:
*   None
*
*******************************************************************************/
void board_task(void *param)
{
    uint32_t buttonid;

    /* Suppress warning for unused parameter */
    (void)param;

    for(;;)
    {
        /* Block until a command has been received over queue */
    	 xTaskNotifyWait(0, 0, &buttonid, portMAX_DELAY);
        switch(buttonid)
        {

        case 1:
        	board_led_set_state(USER_LED1, LED_ON);
        	gopro_active_trigger = 1 - gopro_active_trigger;
            bt_app_gatt_write_goprodata(CMD_TRIGGER, gopro_active_trigger);
            cyhal_system_delay_ms(100);
        	board_led_set_state(USER_LED1, LED_OFF);
            break;
        case 2:

        	if(!bt_active)
        	{
                /* Perform application-specific initialization */
                bt_app_init();

                bt_active = 1;
        	}

            break;
        case 3:
        	board_led_set_state(USER_LED1, LED_ON);
            bt_app_gatt_write_goprodata(CMD_MODE, gopro_active_mode);
        	gopro_active_mode++;
        	if(gopro_active_mode>2) gopro_active_mode =0;
            cyhal_system_delay_ms(100);
        	board_led_set_state(USER_LED1, LED_OFF);
            break;

        }
    }
}



/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_event_t (unused)
*
*******************************************************************************/
static void gpio_interrupt_handler1(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t value =  cyhal_gpio_read(CYBSP_USER_BTN);

    if (value == button1_previous_value)
    {
        return;
    }
    button1_previous_value = value;

    if (value == CYBSP_BTN_PRESSED)
    {
        return;

    }
    xTaskNotifyFromISR(board_task_handle, (uint32_t) 1, eSetValueWithoutOverwrite, xHigherPriorityTaskWoken);
    // Button is released

}


static void gpio_interrupt_handler2(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t value =  cyhal_gpio_read(P1_0);

    if (value == button2_previous_value)
    {
        return;
    }
    button2_previous_value = value;

    if (value == CYBSP_BTN_PRESSED)
    {

        return;
    }
    xTaskNotifyFromISR(board_task_handle, (uint32_t) 2, eSetValueWithoutOverwrite, xHigherPriorityTaskWoken);
    // Button is released
}


static void gpio_interrupt_handler3(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t value =  cyhal_gpio_read(P1_1);


    if (value == button3_previous_value)
    {
        return;
    }
    button3_previous_value = value;

    if (value == CYBSP_BTN_PRESSED)
    {
        return;
    }
    // Button is released
    xTaskNotifyFromISR(board_task_handle, (uint32_t) 3, eSetValueWithoutOverwrite, xHigherPriorityTaskWoken);
}



/* [] END OF FILE */
