/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Tuning CapSense
*              over BLE - Client Example for ModusToolbox.
*
* Related Document: See Readme.md
*
*
********************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_ble.h"
#include "cycfg.h"
#include "tuner_ble_client.h"
#include "stdlib.h"
#include "main.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define ENABLE                       (1u)
#define DISABLE                      (0u)
#define DEBUG_BLE_ENABLE             (DISABLE)
#define INIT_SUCCESS                 (1u)
#define INIT_FAILED                  (0u)

#if DEBUG_BLE_ENABLE
#define DEBUG_BLE                    (printf)
#else
#define DEBUG_BLE(...)
#endif

/* EZI2C macros */
#define EZI2C_INTR_PRIORITY          (7u)

/* BLE macros */
#define NOTIFICATION_PKT_SIZE        (492u)
#define TUNER_COMMAND_CHAR_HANDLE    (0x000C)

/* Tuner command packet */
#define TUNER_COMMAND_SIZE_0_IDX     (0u)
#define TUNER_COMMAND_OFFS_0_IDX     (1u)
#define TUNER_COMMAND_OFFS_1_IDX     (2u)
#define TUNER_COMMAND_DATA_0_IDX     (3u)
#define TUNER_COMMAND_DATA_1_IDX     (4u)
#define TUNER_COMMAND_DATA_2_IDX     (5u)
#define TUNER_COMMAND_DATA_3_IDX     (6u)
#define TUNER_COMMAND_SIZE           (7u)
#define MAX_DATA_LEN                 (4u)


/*******************************************************************************
 * Global variables
 ******************************************************************************/
/* Pointer to the buffer to be exposed to I2C master (CapSense Tuner) */
uint8_t * ezi2c_buffer = NULL;

/* Buffer to hold Tuner commands to be sent to GATT Server */
uint8_t ble_write_buffer[TUNER_COMMAND_SIZE];

/* Variable to hold the result of BLE apis */
cy_en_ble_api_result_t api_result;

/* Variable to hold the write request parameter to be sent to GATT Server */
cy_stc_ble_gattc_write_req_t write_req;

/* Variable to hold EZI2C slave context structure */
cy_stc_scb_ezi2c_context_t ezi2c_context;

/* Variable to hold Initialization configuration structure of EZI2C */
const cy_stc_sysint_t ezi2c_intr_config = {
        .intrSrc = EZI2C_IRQ ,
        .intrPriority = EZI2C_INTR_PRIORITY
    };


/*******************************************************************************
 * Extern variables
 ******************************************************************************/
/* No. of notifications to send the complete CapSense data structure */
extern uint8_t max_notificaion_count;

/* Variable to store hold BLE connection handle */
extern cy_stc_ble_conn_handle_t conn_handle;


/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static cy_en_scb_ezi2c_status_t tuner_ezi2c_init(void);
static void ezi2c_isr(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize ble
*  - initialize ezi2c for tuner communication
*  - load the ezi2c buffer if notification is received from GATT server
*  - frame tuner command packet when a tuner write modifies CapSense structure
*  - load ble write buffer with tuner command packet and send it to GATT server
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_en_scb_ezi2c_status_t ezi2c_status = CY_SCB_EZI2C_SUCCESS;

    uint32_t offset_address = 0;
    uint32_t index_position = 0;
    uint32_t byte_count = 0;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, \
                                    CY_RETARGET_IO_BAUDRATE);

    /* Initialize user LED */
    result |= cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* retarget-io init or LED init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

       /*Enable interrupts */
    __enable_irq();

    /* Initialize ezi2c to establish Tuner communication */
    ezi2c_status = tuner_ezi2c_init();

    /* ezi2c init failed. Stop program execution */
    if(ezi2c_status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize BLESS block */
    ble_init();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****************** "\
           "Tuning CapSense over BLE - Client"\
           " ******************\r\n\n");

    for(;;)
    {
        /* Cy_BLE_ProcessEvents() allows BLE stack to process pending events */
        ble_process_events();

        /* I2C Master - Tuner has written into the i2c buffer, frame the Tuner
         * Command packet and send it to GATT server */
        if((Cy_SCB_EZI2C_GetActivity(EZI2C_HW, &ezi2c_context) == CY_SCB_EZI2C_STATUS_WRITE1))
        {
            /* Disable I2C interrupts */
            NVIC_DisableIRQ(ezi2c_intr_config.intrSrc);

            /* OffSet Address written by Tuner */
            offset_address = ezi2c_context.baseAddr1;

            /* Calculate the index of the last written byte by Tuner */
            index_position = (uint32_t)(ezi2c_context.curBuf - ezi2c_buffer);

            /* Enable I2C interrupts */
            NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

            /* No of bytes written by Tuner */
            byte_count = index_position - offset_address;

            if((byte_count > 0u) && (byte_count <= MAX_DATA_LEN))
            {
                for(uint8_t i = 0, j = MAX_DATA_LEN - 1; i < byte_count; i++, j--)
                {
                    ble_write_buffer[TUNER_COMMAND_DATA_0_IDX + j] =\
                                               ezi2c_buffer[offset_address + i];
                }

                ble_write_buffer[TUNER_COMMAND_SIZE_0_IDX] = byte_count;
                ble_write_buffer[TUNER_COMMAND_OFFS_0_IDX] =\
                                                (offset_address & 0xFF00) >> 8;
                ble_write_buffer[TUNER_COMMAND_OFFS_1_IDX] =\
                                                (offset_address & 0x00FF);

                /* Send Write command to GATT Server to update the changes made
                 * by Tuner Write */
                write_req.handleValPair.attrHandle = TUNER_COMMAND_CHAR_HANDLE;
                write_req.handleValPair.value.val = ble_write_buffer;
                write_req.handleValPair.value.len = TUNER_COMMAND_SIZE;
                write_req.connHandle = conn_handle;

                if(Cy_BLE_GATT_GetBusyStatus(conn_handle.attId) ==\
                                                    CY_BLE_STACK_STATE_FREE)
                {
                    Cy_BLE_GATTC_WriteWithoutResponse (&write_req);
                }
            }
        }
    }
}


/*******************************************************************************
* Function Name: tuner_ezi2c_init
********************************************************************************
* Summary:
*  Initializes ezi2c interface between Tuner GUI and PSoC 6 MCU.
*
*  return
*  cy_en_scb_ezi2c_status_t: See cy_en_scb_ezi2c_status_t for details
*
*******************************************************************************/
static cy_en_scb_ezi2c_status_t tuner_ezi2c_init(void)
{
    /* Variable to store EZI2C slave status codes */
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* Initialize EZI2C */
    status = Cy_SCB_EZI2C_Init(EZI2C_HW, &EZI2C_config, &ezi2c_context);

    /* Initialize and enable EZI2C interrupts */
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Enable EZI2C block */
    Cy_SCB_EZI2C_Enable(EZI2C_HW);

    return (status);
}


/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
*
* Summary:
*   - EZI2C interrupt handler.
*
*******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(EZI2C_HW, &ezi2c_context);
}


/*******************************************************************************
* Function Name: set_tuner_buffer
********************************************************************************
*
* Summary:
*   - initialize ezi2c buffer to be read by the Tuner GUI
*
* Param:
* uint16_t size: Size of the CapSense data structure as received from GATT server
*
* Return:
* uint8_t: INIT_SUCCESS - Bridge initialization successful
*          INIT_FAILED  - Memory allocation failed
*
*******************************************************************************/
uint8_t set_tuner_buffer(uint16_t size)
{
    /* Dynamically allocate memory for ezi2c buffer based on the size of CapSense
     * data structure */
    ezi2c_buffer = (uint8_t *)malloc(size * sizeof(uint8_t));

    if((ezi2c_buffer) != NULL)
    {
        /* Set up communication and initialize data buffer to CapSense data
         * structure to transfer data structure to BLE server */
        Cy_SCB_EZI2C_SetBuffer1(EZI2C_HW, ezi2c_buffer, size, size, &ezi2c_context);
        return (INIT_SUCCESS);
    }
    else
    {
        /* Memory allocation failed */
        return (INIT_FAILED);
    }
}


/*******************************************************************************
* Function Name: clear_tuner_buffer
********************************************************************************
*
* Summary:
*   - free the ezi2c buffer memory after a ble disconnection event
*
*******************************************************************************/
void clear_tuner_buffer()
{
    /* Free the heap memory after BLE is disconnected */
    free(ezi2c_buffer);
}


/*******************************************************************************
* Function Name: update_ezi2c_buffer
********************************************************************************
*
* Summary:
*   - Updates the ezi2c buffer with CapSense structure
*   - Called by CY_BLE_EVT_GATTC_HANDLE_VALUE_NTF event when a notification is
*     received
*
* Param:
* uint16_t data: Pointer to the data received
* uint16_t size: Size of data received
*
* Return:
* void
*
*******************************************************************************/
void update_ezi2c_buffer(uint8_t * data, uint16_t size)
{
    static uint8_t notification_count = 0;

    if(notification_count == 0)
    {
        /* I2C interrupts are disabled when buffer is updated */
        NVIC_DisableIRQ(ezi2c_intr_config.intrSrc);
    }

    if(size <= NOTIFICATION_PKT_SIZE)
    {
        /* Update ezi2c_buffer with data from the notification packet
         * received from CapSense GATT server */
        memcpy(ezi2c_buffer + (notification_count * NOTIFICATION_PKT_SIZE),\
                                                                   data, size);
    }
    else
    {
        /* Size cannot be greater than NOTIFICATION_PKT_SIZE, handle error. */
        CY_ASSERT(0);
    }

    notification_count++;

    if(notification_count == max_notificaion_count)
    {
        /* Buffer update complete. Re-enable I2C interrupts */
        notification_count = 0;
        NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);
    }
}


/* [] END OF FILE */
