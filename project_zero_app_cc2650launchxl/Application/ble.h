/*
 * ble.h
 *
 *  Created on: Mar 15, 2017
 * 	    Author: James Finlay
 */

#ifndef APPLICATION_BLE_H_
#define APPLICATION_BLE_H_

/*********************************************************************
 * CONSTANTS
 */

#include "osal_snv.h"
#include <gap.h>
#include <gatt.h>
#include <gapgattserver.h>
#include <gattservapp.h>
#include <gapbondmgr.h>
#include <peripheral.h>
#include <devinfoservice.h>
#include <icall_apimsg.h>

#include <ti/sysbios/knl/Queue.h>

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      000000

// Internal Events for RTOS application
#define PRZ_STATE_CHANGE_EVT                  0x0001
#define PRZ_CHAR_CHANGE_EVT                   0x0002
#define PRZ_PERIODIC_EVT                      0x0004
#define PRZ_CONN_EVT_END_EVT                  0x0008

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value  */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
} app_msg_types_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32   numComparison;
} passcode_req_t;

void bleInit(ICall_EntityID  applicationEntity,
		Queue_Handle appMsgQ, ICall_Semaphore* sem);
static uint8_t ProjectZero_processStackMsg();

#endif /* APPLICATION_BLE_H_ */
