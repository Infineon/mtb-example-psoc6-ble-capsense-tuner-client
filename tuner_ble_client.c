/*******************************************************************************
* File Name: tuner_ble_client.c
*
* Description: This file contains BLE related functions.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Include header files
 ******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg_ble.h"
#include "cy_retarget_io.h"
#include "cy_pdl.h"
#include "tuner_ble_client.h"
#include "main.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define ENABLE                      (1u)
#define DISABLE                     (0u)
#define DEBUG_BLE_ENABLE            (DISABLE)
#define CY_ASSERT_FAILED            (0u)

#if DEBUG_BLE_ENABLE
#define DEBUG_BLE                   (printf)
#else
#define DEBUG_BLE(...)
#endif

#define BLESS_INTR_PRIORITY         (1u)
#define GATT_MTU_MAX                (512u)
#define CY_BLE_BD_ADDR_SIZE         (0x06u)
#define GATT_CCCD_LEN               (0x02u)
#define GATT_CCCD_NTF_ENABLE        (ENABLE)
#define AD_TYPE_COMPLETE_LOCAL_NAME (9u)

/* Handle of CapSense_DS characteristic */
#define CAPSENSE_DS_CHAR_HANDLE     (0x0009u)

/* Handle of Client Characteristic Configuration descriptor */
#define CAPSENSE_DS_CCCD_HANDLE     (0x000Au)

/* Length of notification packet received from GATT Server to initialize EZI2C bridge
 * Size of CapSense data structure:(2 bytes)
 * No. of notification packets to receive complete CapSense data: (1 byte) */
#define CAPSENSE_DS_SIZE_LSB_IDX    (0u)
#define CAPSENSE_DS_SIZE_MSB_IDX    (1u)
#define NOTIFICATION_COUNT_IDX      (2u)
#define TUNER_BRIDGE_INIT_NTF_SIZE  (3u)


/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static void stack_event_handler(uint32_t event, void* eventParam);
static void ble_assert(void);
static void bless_interrupt_handler(void);
static uint8_t* adv_parser(uint16_t ad_type, cy_stc_ble_gapc_adv_report_param_t *scan_report, uint8_t *adv_type_length);


/*******************************************************************************
 * Global variables
 ******************************************************************************/
/* Variable is set when a notification is received from GATT Server */
bool notify = false;

/* Variable to store no. of notification packets to receive complete CapSense data */
uint8_t max_notificaion_count = 1;

/* Variable to check if target gap peripheral device is found or not */
static bool target_found = false;

/* Variable to check if ezi2c bridge is initialized or not */
static bool bridge_initialised = false;

/* Variable to enable or disable notifications in GATT Server */
static uint8_t cccd_val[CY_BLE_CCCD_LEN] = {DISABLE, DISABLE};

/* Variable to store connection handle */
cy_stc_ble_conn_handle_t conn_handle;

/* Variable to store notification data received from server */
cy_stc_ble_gattc_handle_value_ntf_param_t notification_data;

/* Constant to store server name and use during scanning */
static const char target_name[] = "CapSense_GATT_Server";

/* Variable to hold the address of the peer device */
static cy_stc_ble_gap_bd_addr_t peer_addr;

/* Variable to hold advertisement report parameter */
static cy_stc_ble_gapc_adv_report_param_t adv_report;


/*******************************************************************************
* Function Name: bless_interrupt_handler
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from BLESS.
*
*******************************************************************************/
static void bless_interrupt_handler(void)
{
    Cy_BLE_BlessIsrHandler();
}


/*******************************************************************************
 * Function Name: ble_init()
 *******************************************************************************
 *
 * Summary:
 *   This function initializes the BLE for use in the code example.
 *
 ******************************************************************************/
void ble_init(void)
{
    /* Variable to store common error codes received as API result */
    cy_en_ble_api_result_t api_result = CY_BLE_SUCCESS;
    cy_rslt_t sysint_status = CY_RSLT_SUCCESS;

    /* BLESS interrupt configuration structure */
    const cy_stc_sysint_t  blessIsrCfg =
    {
        /* The BLESS interrupt */
           .intrSrc       = bless_interrupt_IRQn,

           /* The interrupt priority number */
           .intrPriority  = BLESS_INTR_PRIORITY
    };

    /* Initialize and enable BLESS interrupt */
    sysint_status = cyhal_system_set_isr(bless_interrupt_IRQn, bless_interrupt_IRQn,
            BLESS_INTR_PRIORITY, &bless_interrupt_handler);

    CY_ASSERT(sysint_status == CY_RSLT_SUCCESS);

    /* Initialize the BLESS interrupt */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;

    /* Register the generic event handler */
    Cy_BLE_RegisterEventCallback(stack_event_handler);

    /* Initialize the BLE host */
    api_result = Cy_BLE_Init(&cy_ble_config);

    if (api_result != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration,
         * notify error and halt CPU in debug mode */
        DEBUG_BLE("Cy_BLE_Init API Error: %x \r\n", api_result);
        ble_assert();
    }
    else
    {
        DEBUG_BLE("Cy_BLE_Init API Success: %x \r\n", api_result);
    }

    /* Enable BLE */
    api_result = Cy_BLE_Enable();

    /* To avoid compiler warning*/
    (void) sysint_status;

    if (api_result != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration,
         * notify error and halt CPU in debug mode */
        DEBUG_BLE("Cy_BLE_Enable API Error: %x \r\n", api_result);
        ble_assert();
    }
    else
    {
        DEBUG_BLE("Cy_BLE_Enable API Success: %x \r\n", api_result);
    }
}


/*******************************************************************************
* Function Name: stack_event_handler
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE stack.
*
* Parameters:
*  uint32_t event:      event from the BLE stack
*  void* eventParam:  parameters related to the event
*
*******************************************************************************/
static void stack_event_handler(uint32_t event, void* eventParam)
{
    cy_en_ble_api_result_t api_result = CY_BLE_SUCCESS;

    switch(event)
    {
        /***********************************************************************
         *                       General Events
         **********************************************************************/
        case CY_BLE_EVT_STACK_ON:
        {
            /* Stack initialized; ready for scan */
            DEBUG_BLE("CY_BLE_EVT_STACK_ON \r\n");
            printf("Scanning for GAP Peripheral with device name %s\n\r",\
                                                               target_name);
            /* Start scan */
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,\
                                  CY_BLE_CENTRAL_CONFIGURATION_0_INDEX);
            break;
        }

        /* This event is received when there is a timeout. */
        case CY_BLE_EVT_TIMEOUT:
        {
            DEBUG_BLE("CY_BLE_EVT_TIMEOUT \r\n");
            break;
        }

        /***********************************************************************
         *                     GAP Central Events
         **********************************************************************/

        /* This event is triggered every time a device is discovered */
        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
        {
            /* A new device listed in the scan report */
            adv_report = *(cy_stc_ble_gapc_adv_report_param_t *)eventParam;

            /* Pointer to store return value from advertisement parser */
            char* peer_name = NULL;
            uint8_t name_length = 0u;

            /* Process only for Advertisement packets, not on scan response
             * packets */
            if(adv_report.eventType != CY_BLE_GAPC_SCAN_RSP)
            {
                peer_name = (char*) adv_parser(AD_TYPE_COMPLETE_LOCAL_NAME,\
                                                     &adv_report, &name_length);

                if(peer_name  == NULL)
                {
                    target_found = false;
                    break;
                }
                /* Compare peer name with "CapSense_GATT_Server" */
                else
                {
                    peer_name[name_length]= '\0';
                    target_found = ((strcmp(peer_name, target_name)) ? false : true);
                }
                if (target_found)
                {
                    /* Stop Scan */
                    Cy_BLE_GAPC_StopScan();
                }
            }
            break;
        }

        /* The central device has started/stopped scanning */
        case CY_BLE_EVT_GAPC_SCAN_START_STOP:
        {
            if(Cy_BLE_GetScanState() == CY_BLE_SCAN_STATE_STOPPED)
            {
                if(target_found == true)
                {
                    /* Scan stopped manually; do not restart scan */
                    target_found = false;

                    /* Get address and address type of the peer device to
                     * initiate connection */
                    for(uint8_t i = 0u; i < CY_BLE_BD_ADDR_SIZE; i++)
                    {
                        peer_addr.bdAddr[i] = adv_report.peerBdAddr[i];
                    }

                    printf("Found Peer Device with address:");

                    /* Print the peer bd address on UART terminal */
                    for(uint8_t i = (CY_BLE_BD_ADDR_SIZE); i > 0u; i--)
                    {
                        printf(" %X", peer_addr.bdAddr[i - 1u]);
                    }

                    /* Get the peer address type */
                    peer_addr.type = adv_report.peerAddrType;

                    /* Initiate connection with discovered peer device */
                    api_result = Cy_BLE_GAPC_ConnectDevice(&peer_addr,\
                                          CY_BLE_CENTRAL_CONFIGURATION_0_INDEX);
                    if(api_result == CY_BLE_SUCCESS)
                    {
                         DEBUG_BLE("Initiated GAP connection..\r\n");
                    }
                    else
                    {
                         printf("Failed to initiate connection %u \r\n",\
                                                                    api_result);
                         ble_assert();
                    }
                }
            }
            break;
        }

        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
        {
            /* BLE link is established */
            DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GAP_DEVICE_CONNECTED \r\n");

            /* Variable to store values to update PHY to 2M */
            cy_stc_ble_set_phy_info_t phy_param;
            phy_param.allPhyMask = CY_BLE_PHY_NO_PREF_MASK_NONE;
            phy_param.bdHandle = conn_handle.bdHandle;
            phy_param.rxPhyMask = CY_BLE_PHY_MASK_LE_2M;
            phy_param.txPhyMask = CY_BLE_PHY_MASK_LE_2M;

            /* Function call to set PHY to 2M */
            api_result = Cy_BLE_SetPhy(&phy_param);
            if(api_result == CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Set PHY to 2M successfull");
                DEBUG_BLE("Request sent to switch PHY to 2M\r\n");
            }
            else
            {
                DEBUG_BLE("Set PHY to 2M API failure, errorcode = 0x%X", api_result);
            }
            break;
        }

        /* This event is triggered when there is a change to either the maximum
         * Payload length or the maximum transmission time of Data Channel PDUs
         * in either direction
         */
        case CY_BLE_EVT_DATA_LENGTH_CHANGE:
        {
            DEBUG_BLE("CY_BLE_EVT_DATA_LENGTH_CHANGE\r\n");
            break;
        }

        /* This event indicates that the controller has changed the transmitter
         * PHY or receiver PHY in use */
        case CY_BLE_EVT_PHY_UPDATE_COMPLETE:
        {
            /* Initiate an MTU exchange request */
            cy_stc_ble_gatt_xchg_mtu_param_t mtuParam = {conn_handle,\
                    GATT_MTU_MAX};

            /* To remove unused parameter warning when UART debug is disabled*/
#if (DEBUG_BLE_ENABLE == ENABLE)
            DEBUG_BLE("UPDATE PHY parameters\r\n");
            cy_stc_ble_events_param_generic_t *param =\
                                (cy_stc_ble_events_param_generic_t *)eventParam;
            cy_stc_ble_phy_param_t *phyparam = NULL;
            if(param->status == CY_BLE_SUCCESS)
            {
                phyparam = (cy_stc_ble_phy_param_t *)param->eventParams;
                DEBUG_BLE("RxPhy Mask : 0x%02X\r\nTxPhy Mask : 0x%02X\r\n",\
                        phyparam->rxPhyMask, phyparam->txPhyMask);
            }
#endif
            api_result = Cy_BLE_GATTC_ExchangeMtuReq(&mtuParam);

            if(api_result != CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Cy_BLE_GATTC_ExchangeMtuReq API Error: %xd \r\n",\
                        api_result);
            }
            break;
        }

        /***********************************************************************
         *                       GATT Events
         **********************************************************************/

        /* This event is generated at the GAP Peripheral end after a connection
         * is completed with a peer Central device. For a GAP Central device,
         * this event is generated as in acknowledgment of receiving this event
         * successfully by the BLE Controller.
         */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            /* Connected as Central (master role) */
            conn_handle = *(cy_stc_ble_conn_handle_t *)eventParam;

            /* Turn the connected LED on */
            cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_ON);

            DEBUG_BLE("CY_BLE_EVT_GATT_CONNECT_IND: attId=%X, bdHandle=%X\r\n",\
                       conn_handle.attId, conn_handle.bdHandle);
            printf("\n\rConnected to Device\r\n\n");

            break;
        }

        /* GATT MTU Exchange response from the server device. */
        case CY_BLE_EVT_GATTC_XCHNG_MTU_RSP:
        {
            cy_stc_ble_gattc_write_req_t write_req;
            /* To remove unused parameter warning when UART debug is disabled */
#if (DEBUG_BLE_ENABLE == ENABLE)
            cy_stc_ble_gatt_xchg_mtu_param_t *resp =\
                                 (cy_stc_ble_gatt_xchg_mtu_param_t *)eventParam;
            DEBUG_BLE("CY_BLE_EVT_GATTC_XCHNG_MTU_RSP"\
                      "[bdHandle 0x%02X MTU %hu]\r\n",\
                       resp->connHandle.bdHandle, resp->mtu);
#endif
            cccd_val[0] = ENABLE;
            write_req.connHandle = conn_handle;
            write_req.handleValPair.attrHandle = CAPSENSE_DS_CCCD_HANDLE;
            write_req.handleValPair.value.val = cccd_val;
            write_req.handleValPair.value.len = GATT_CCCD_LEN;

            /* Enable notification to receive data */
            api_result = Cy_BLE_GATTC_WriteCharacteristicDescriptors(&write_req);
            if(api_result != CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Failed to enable notification"\
                          " [bdhandle 0x%02X]: \r\n", write_req.connHandle.bdHandle);
                ble_assert();
            }
            else
            {
                DEBUG_BLE("Enabling notification [bdhandle 0x%02X]\r\n",\
                           write_req.connHandle.bdHandle);
            }
            break;
        }

        /* Disconnection event */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            DEBUG_BLE("CY_BLE_EVT_GATT_DISCONNECT_IND \r\n");
            if(conn_handle.bdHandle ==\
                             (*(cy_stc_ble_conn_handle_t *)eventParam).bdHandle)
            {
                printf("Disconnected.\r\n\n");
                conn_handle.bdHandle = 0xFF;
                conn_handle.attId    = 0xFF;
            }
            /* Free the tuner buffer memory */
            clear_tuner_buffer();

            /* BLE connection disconnected. Turn OFF the LED */
            cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1,\
                              CYBSP_LED_STATE_OFF);
            bridge_initialised = false;
            printf("Scanning for GAP Peripheral with device name %s\n\r",\
                                                               target_name);

            /* Start scan */
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,\
                                  CY_BLE_CENTRAL_CONFIGURATION_0_INDEX);
            break;
        }

        case CY_BLE_EVT_GATTC_WRITE_RSP:
        {
            /* Write response for the CCCD write; this means that the
             * notifications are now enabled. */
            printf("Notifications enabled successfully. Receiving data..\r\n");
            break;
        }

        case CY_BLE_EVT_GATTC_HANDLE_VALUE_NTF:
        {
            uint16_t buffer_size = 0;
            uint8_t status = 0;
            notification_data = *(cy_stc_ble_gattc_handle_value_ntf_param_t *)eventParam;

            if(notification_data.handleValPair.attrHandle == CAPSENSE_DS_CHAR_HANDLE)
            {
                if((notification_data.handleValPair.value.len == TUNER_BRIDGE_INIT_NTF_SIZE) &&\
                        bridge_initialised == false)
                {
                    printf("Received Tuner Bridge initialization parameters: \n\r");
                    /* Bridge Parameters */
                    buffer_size =\
                        (notification_data.handleValPair.value.val[CAPSENSE_DS_SIZE_MSB_IDX ] << 8) |\
                        (notification_data.handleValPair.value.val[CAPSENSE_DS_SIZE_LSB_IDX]);
                    max_notificaion_count =\
                            notification_data.handleValPair.value.val[NOTIFICATION_COUNT_IDX];

                    printf("CapSense data structure size: %u\n\r", buffer_size);
                    printf("No of notifications to completely receive CapSense"\
                           " data structure: %u\n\r", max_notificaion_count);

                    /* Set the buffer based on CapSense data structure size  */
                    status = set_tuner_buffer(buffer_size);
                    if(status == 1u)
                    {
                        printf("\n\rBridge initialized...\n\r");
                        bridge_initialised = true;
                    }
                }
                /* CapSense Data structure received */
                else if(bridge_initialised == true)
                {
                    /* Update the ezi2c tuner buffer with received data */
                    update_ezi2c_buffer(notification_data.handleValPair.value.val,\
                                        notification_data.handleValPair.value.len);
                }
            }
            break;
        }

        /* The event is received by the Client when the Server cannot perform
         * the requested operation and sends out an error response. */
        case CY_BLE_EVT_GATTC_ERROR_RSP:
        {
            DEBUG_BLE("CY_BLE_EVT_GATTC_ERROR_RSP \r\n");
            break;
        }

        /***********************************************************************
         *                       Other Events
         **********************************************************************/
        default:
            DEBUG_BLE("Unhandled BLE event: %x\r\n", event);
            break;
        }
}


/*******************************************************************************
 * Function Name: ble_assert()
 *******************************************************************************
 * Summary: ble initialization failed, halt CPU
 *
 ******************************************************************************/
static void ble_assert(void)
{
    CY_ASSERT(CY_ASSERT_FAILED);
}


/*******************************************************************************
* Function Name: ble_capsense_process
********************************************************************************
*
* Summary:
*   -  allows the BLE stack to process pending events
*
*******************************************************************************/
void ble_process_events(void)
{
    /* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
    Cy_BLE_ProcessEvents();
}


/*******************************************************************************
* Function Name: uint8_t* adv_parser(uint16_t AD_type,
*       cy_stc_ble_gapc_adv_report_param_t* scan_report, uint8_t* adv_type_length)
********************************************************************************
*
* Summary: This function searches adv packets for the given type.
*
* Parameters:
*  uint16_t adv_type : the type of value to be discovered
*  cy_stc_ble_gapc_adv_report_param_t* scan_report : advertisement report
*                                                    parameter
*  uint8_t* adv_type_length : length of discovered value
*
* Return:
*  uint8_t* : Pointer to the value discovered in ADV packet
*
*******************************************************************************/
static uint8_t* adv_parser(uint16_t adv_type, cy_stc_ble_gapc_adv_report_param_t*
                         scan_report, uint8_t* adv_type_length)
{
    uint8_t length = 0u;
    uint8_t* pName = NULL;

    for(uint8_t i = 0u; i < scan_report->dataLen; i += (length+1))
    {
        length = scan_report->data[i];
        if(scan_report->data[i+1] == adv_type)
        {
            pName = &scan_report->data[i+2];
            *adv_type_length = length - 1;
            return(pName);
        }
    }
    return((uint8_t*)NULL);
}


/* [] END OF FILE */
