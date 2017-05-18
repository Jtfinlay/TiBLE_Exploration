/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

// Stack headers
#include <hci_tl.h>
#include <gap.h>
#include <gatt.h>
#include <gapgattserver.h>
#include <gattservapp.h>
#include <osal_snv.h>
#include <gapbondmgr.h>
#include <peripheral.h>
#include <icall_apimsg.h>

#include <devinfoservice.h>

#include "util.h"

#include "ble.h"
#include "Board.h"
#include "project_zero.h"

// Bluetooth Developer Studio services
#include "led_service.h"
#include "button_service.h"
#include "data_service.h"


/*********************************************************************
 * CONSTANTS
 */


// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   800
#endif


/*********************************************************************
 * TYPEDEFS
 */


// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];

/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;

// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;

// Global display handle
Display_Handle dispHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void buttonDebounceSwiFxn(UArg buttonId);
static void user_handleButtonPress(button_state_t *pState);

// Generic callback handlers for value changes in services.
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

// Task context handlers for generated services.
static void user_LedService_ValueChangeHandler(char_data_t *pCharData);
static void user_ButtonService_CfgChangeHandler(char_data_t *pCharData);
static void user_DataService_ValueChangeHandler(char_data_t *pCharData);
static void user_DataService_CfgChangeHandler(char_data_t *pCharData);

// Task handler for sending notifications.
static void user_updateCharVal(char_data_t *pCharData);

// Utility functions
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );
static void user_enqueueCharDataMsg(app_msg_types_t appMsgType, uint16_t connHandle,
                                    uint16_t serviceUUID, uint8_t paramID,
                                    uint8_t *pValue, uint16_t len);
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);
static char *Util_getLocalNameStr(const uint8_t *data);

/*********************************************************************
 * PROFILE CALLBACKS
 */


/*
 * Callbacks in the user application for events originating from BLE services.
 */
// LED Service callback handler.
// The type LED_ServiceCBs_t is defined in led_service.h
static LedServiceCBs_t user_LED_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
};

// Button Service callback handler.
// The type Button_ServiceCBs_t is defined in button_service.h
static ButtonServiceCBs_t user_Button_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in Button Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

// Data Service callback handler.
// The type Data_ServiceCBs_t is defined in data_service.h
static DataServiceCBs_t user_Data_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &sem);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

  // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
    Log_error0("Error initializing board LED pins");
    Task_exit();
  }

  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
  if(!buttonPinHandle) {
    Log_error0("Error initializing button pins");
    Task_exit();
  }

  // Setup callback for button pins
  if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
    Log_error0("Error registering button callback function");
    Task_exit();
  }

  // Create the debounce clock objects for Button 0 and Button 1
  Clock_Params clockParams;
  Clock_Params_init(&clockParams);

  // Both clock objects use the same callback, so differentiate on argument
  // given to the callback in Swi context
  clockParams.arg = Board_BUTTON0;

  // Initialize to 50 ms timeout when Clock_start is called.
  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
  Clock_construct(&button0DebounceClock, buttonDebounceSwiFxn,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);

  // Second button
  clockParams.arg = Board_BUTTON1;
  Clock_construct(&button1DebounceClock, buttonDebounceSwiFxn,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);

  // Initialize BLE
  bleInit(selfEntity, hApplicationMsgQ, &sem);

  // Add services to GATT server and give ID of this task for Indication acks.
  LedService_AddService( selfEntity );
  ButtonService_AddService( selfEntity );
  DataService_AddService( selfEntity );

  // Register callbacks with the generated services that
  // can generate events (writes received) to the application
  LedService_RegisterAppCBs( &user_LED_ServiceCBs );
  ButtonService_RegisterAppCBs( &user_Button_ServiceCBs );
  DataService_RegisterAppCBs( &user_Data_ServiceCBs );

  // Placeholder variable for characteristic intialization
  uint8_t initVal[40] = {0};
  uint8_t initString[] = "This is a pretty long string, isn't it!";

  // Initalization of characteristics in LED_Service that can provide data.
  LedService_SetParameter(LS_LED0_ID, LS_LED0_LEN, initVal);
  LedService_SetParameter(LS_LED1_ID, LS_LED1_LEN, initVal);

  // Initalization of characteristics in Button_Service that can provide data.
  ButtonService_SetParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
  ButtonService_SetParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);

  // Initalization of characteristics in Data_Service that can provide data.
  DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
  DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ProjectZero_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
    	ProjectZero_processStackMsg();

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }
    }
  }
}


/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void user_processApplicationMessage(app_msg_t *pMsg)
{
  char_data_t *pCharData = (char_data_t *)pMsg->pdu;

  switch (pMsg->type)
  {
    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case LED_SERVICE_SERV_UUID:
          user_LedService_ValueChangeHandler(pCharData);
          break;
        case DATA_SERVICE_SERV_UUID:
          user_DataService_ValueChangeHandler(pCharData);
          break;

      }
      break;

    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case BUTTON_SERVICE_SERV_UUID:
          user_ButtonService_CfgChangeHandler(pCharData);
          break;
        case DATA_SERVICE_SERV_UUID:
          user_DataService_CfgChangeHandler(pCharData);
          break;
      }
      break;

    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
      user_updateCharVal(pCharData);
      break;

    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
      break;

    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
      {
        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
                  DEFAULT_PASSCODE);
        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
      }
      break;

    case APP_MSG_BUTTON_DEBOUNCED: /* Message from swi about pin change */
      {
        button_state_t *pButtonState = (button_state_t *)pMsg->pdu;
        user_handleButtonPress(pButtonState);
      }
      break;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/




/*
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     buttonCallbackFxn
 *
 * @param   pState  pointer to button_state_t message sent from debounce Swi.
 *
 * @return  None.
 */
static void user_handleButtonPress(button_state_t *pState)
{
  Log_info2("%s %s",
    (IArg)(pState->pinId == Board_BUTTON0?"Button 0":"Button 1"),
    (IArg)(pState->state?"\x1b[32mpressed\x1b[0m":
                         "\x1b[33mreleased\x1b[0m"));

  // Update the service with the new value.
  // Will automatically send notification/indication if enabled.
  switch (pState->pinId)
  {
    case Board_BUTTON0:
      ButtonService_SetParameter(BS_BUTTON0_ID,
                                 sizeof(pState->state),
                                 &pState->state);
      break;
    case Board_BUTTON1:
      ButtonService_SetParameter(BS_BUTTON1_ID,
                                 sizeof(pState->state),
                                 &pState->state);
      break;
  }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_LedService_ValueChangeHandler(char_data_t *pCharData)
{
  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
  Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                               pretty_data_holder, sizeof(pretty_data_holder));

  switch (pCharData->paramID)
  {
    case LS_LED0_ID:
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"LED Service",
                (IArg)"LED0",
                (IArg)pretty_data_holder);

      // Do something useful with pCharData->data here
      // -------------------------
      // Set the output value equal to the received value. 0 is off, not 0 is on
      PIN_setOutputValue(ledPinHandle, Board_LED0, pCharData->data[0]);
      Log_info2("Turning %s %s",
                (IArg)"\x1b[31mLED0\x1b[0m",
                (IArg)(pCharData->data[0]?"on":"off"));
      break;

    case LS_LED1_ID:
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"LED Service",
                (IArg)"LED1",
                (IArg)pretty_data_holder);

      // Do something useful with pCharData->data here
      // -------------------------
      // Set the output value equal to the received value. 0 is off, not 0 is on
      PIN_setOutputValue(ledPinHandle, Board_LED1, pCharData->data[0]);
      Log_info2("Turning %s %s",
                (IArg)"\x1b[32mLED1\x1b[0m",
                (IArg)(pCharData->data[0]?"on":"off"));
      break;

  default:
    return;
  }
}


/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_ButtonService_CfgChangeHandler(char_data_t *pCharData)
{
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
  {
  case GATT_CFG_NO_OPERATION:
    configValString = "Noti/Ind disabled";
    break;
  case GATT_CLIENT_CFG_NOTIFY:
    configValString = "Notifications enabled";
    break;
  case GATT_CLIENT_CFG_INDICATE:
    configValString = "Indications enabled";
    break;
  }

  switch (pCharData->paramID)
  {
    case BS_BUTTON0_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Button Service",
                (IArg)"BUTTON0",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;

    case BS_BUTTON1_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Button Service",
                (IArg)"BUTTON1",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;
  }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_DataService_ValueChangeHandler(char_data_t *pCharData)
{
  // Value to hold the received string for printing via Log, as Log printouts
  // happen in the Idle task, and so need to refer to a global/static variable.
  static uint8_t received_string[DS_STRING_LEN] = {0};

  switch (pCharData->paramID)
  {
    case DS_STRING_ID:
      // Do something useful with pCharData->data here
      // -------------------------
      // Copy received data to holder array, ensuring NULL termination.
      memset(received_string, 0, DS_STRING_LEN);
      memcpy(received_string, pCharData->data, DS_STRING_LEN-1);
      // Needed to copy before log statement, as the holder array remains after
      // the pCharData message has been freed and reused for something else.
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"Data Service",
                (IArg)"String",
                (IArg)received_string);
      break;

    case DS_STREAM_ID:
      Log_info3("Value Change msg: Data Service Stream: %02x:%02x:%02x...",
                (IArg)pCharData->data[0],
                (IArg)pCharData->data[1],
                (IArg)pCharData->data[2]);
      // -------------------------
      // Do something useful with pCharData->data here
      break;

  default:
    return;
  }
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_DataService_CfgChangeHandler(char_data_t *pCharData)
{
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
  {
  case GATT_CFG_NO_OPERATION:
    configValString = "Noti/Ind disabled";
    break;
  case GATT_CLIENT_CFG_NOTIFY:
    configValString = "Notifications enabled";
    break;
  case GATT_CLIENT_CFG_INDICATE:
    configValString = "Indications enabled";
    break;
  }

  switch (pCharData->paramID)
  {
    case DS_STREAM_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Data Service",
                (IArg)"Stream",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;
  }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/



/**
 * Callback handler for characteristic value changes in services.
 */
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
  // See the service header file to compare paramID with characteristic.
  Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                          pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
  Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid,
                          paramID, pValue, len);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

/*
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void buttonDebounceSwiFxn(UArg buttonId)
{
  // Used to send message to app
  button_state_t buttonMsg = { .pinId = buttonId };
  uint8_t        sendMsg   = FALSE;

  // Get current value of the button pin after the clock timeout
  uint8_t buttonPinVal = PIN_getInputValue(buttonId);

  // Set interrupt direction to opposite of debounced state
  // If button is now released (button is active low, so release is high)
  if (buttonPinVal)
  {
    // Enable negative edge interrupts to wait for press
    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
  }
  else
  {
    // Enable positive edge interrupts to wait for relesae
    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
  }

  switch(buttonId)
  {
    case Board_BUTTON0:
      // If button is now released (buttonPinVal is active low, so release is 1)
      // and button state was pressed (buttonstate is active high so press is 1)
      if (buttonPinVal && button0State)
      {
        // Button was released
        buttonMsg.state = button0State = 0;
        sendMsg = TRUE;
      }
      else if (!buttonPinVal && !button0State)
      {
        // Button was pressed
        buttonMsg.state = button0State = 1;
        sendMsg = TRUE;
      }
      break;

    case Board_BUTTON1:
      // If button is now released (buttonPinVal is active low, so release is 1)
      // and button state was pressed (buttonstate is active high so press is 1)
      if (buttonPinVal && button1State)
      {
        // Button was released
        buttonMsg.state = button1State = 0;
        sendMsg = TRUE;
      }
      else if (!buttonPinVal && !button1State)
      {
        // Button was pressed
        buttonMsg.state = button1State = 1;
        sendMsg = TRUE;
      }
      break;
  }

  if (sendMsg == TRUE)
  {
    user_enqueueRawAppMsg(APP_MSG_BUTTON_DEBOUNCED,
                      (uint8_t *)&buttonMsg, sizeof(buttonMsg));
  }
}

/*
 *  Callbacks from Hwi-context
 *****************************************************************************/

/*
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
  Log_info1("Button interrupt: %s",
            (IArg)((pinId == Board_BUTTON0)?"Button 0":"Button 1"));

  // Disable interrupt on that pin for now. Re-enabled after debounce.
  PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

  // Start debounce timer
  switch (pinId)
  {
    case Board_BUTTON0:
      Clock_start(Clock_handle(&button0DebounceClock));
      break;
    case Board_BUTTON1:
      Clock_start(Clock_handle(&button1DebounceClock));
      break;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke user_updateCharVal function for them.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID, uint8_t paramID,
                                     uint8_t *pValue, uint16_t len )
{
  // Called in Stack's Task context, so can't do processing here.
  // Send message to application message queue about received data.
  uint16_t readLen = len; // How much data was written to the attribute

  // Allocate memory for the message.
  // Note: The pCharData message doesn't have to contain the data itself, as
  //       that's stored in a variable in the service implementation.
  //
  //       However, to prevent data loss if a new value is received before the
  //       service's container is read out via the GetParameter API is called,
  //       we copy the characteristic's data now.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + sizeof(char_data_t) +
                                  readLen );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
    pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
    pCharData->paramID = paramID;
    // Copy data from service now.
    memcpy(pCharData->data, pValue, readLen);
    // Update pCharData with how much data we received.
    pCharData->dataLen = readLen;
    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}


/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void user_updateCharVal(char_data_t *pCharData)
{
  switch(pCharData->svcUUID) {
    case LED_SERVICE_SERV_UUID:
      LedService_SetParameter(pCharData->paramID, pCharData->dataLen,
                              pCharData->data);
    break;

    case BUTTON_SERVICE_SERV_UUID:
      ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                 pCharData->data);
    break;

  }
}

/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len)
{
  char        hex[] = "0123456789ABCDEF";
  uint8_t     *pStr = dst;
  uint8_t     avail = dst_len-1;

  memset(dst, 0, avail);

  while (src_len && avail > 3)
  {
    if (avail < dst_len-1) { *pStr++ = ':'; avail -= 1; };
    *pStr++ = hex[*src >> 4];
    *pStr++ = hex[*src++ & 0x0F];
    avail -= 2;
    src_len--;
  }

  if (src_len && avail)
    *pStr++ = ':'; // Indicate not all data fit on line.

  return (char *)dst;
}

/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}

/*********************************************************************
*********************************************************************/
