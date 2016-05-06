/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2015-07-13 11:43:11 -0700 (Mon, 13 Jul 2015) $
  Revision:       $Revision: 44336 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2650 Bluetooth Low Energy Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
//#include "devinfoservice.h"
#include "CLIMBProfile.h"
#include "bsp_i2c.h"

#if defined(SENSORTAG_HW)
#include "bsp_spi.h"
#endif // SENSORTAG_HW

#include "peripheralObserver.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
//#include "board_lcd.h"
//#include "board_key_ST.h"
#include "Board.h"
#include "sensor.h"
#include "CLIMBPerOb.h"
#include "Keys_Task.h"

#include <ti/drivers/lcd/LCDDogm1286.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>

#include <driverlib/aon_rtc.h>
#include <driverlib/aon_batmon.h>

#include "sensor_bmp280.h"
#include "sensor_hdc1000.h"
#include "sensor_mpu9250.h"
#include "sensor_opt3001.h"
#include "sensor_tmp007.h"

#ifdef FEATURE_LCD
#include "devpk_lcd.h"
#include <stdio.h>
#endif
/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#ifdef WORKAROUND
#define DEFAULT_ADVERTISING_INTERVAL          3200
#else
#define DEFAULT_ADVERTISING_INTERVAL          1600
#endif
#define EPOCH_PERIOD						  1000

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL   	 400

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          210

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

// Scan interval value in 0.625ms ticks
#define SCAN_INTERVAL 						  320

// scan window value in 0.625ms ticks
#define SCAN_WINDOW							  320

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1250//10000 //Tempo di durata di una scansione,

#define PRE_ADV_TIMEOUT				  DEFAULT_ADVERTISING_INTERVAL*0.625-5

// Whether to report all contacts or only the first for each device
#define FILTER_ADV_REPORTS					  FALSE

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL //non è ancora chiaro cosa cambi, con le altre due opzioni non vede

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// How often to perform periodic event (in msec)
#define PERIODIC_EVT_PERIOD_MSEC               2000

#define NODE_TIMEOUT_OS_TICKS				500000

#define LED_TIMEOUT_MSEC						  	  10

#warning CHECK TIMEOUTS!!!
#define CONNECTABLE_TIMEOUT_MSEC					1000*60
#define WAKEUP_DEFAULT_TIMEOUT_SEC				60*60*24
#define GOTOSLEEP_DEFAULT_TIMEOUT_SEC			1*60*60
#define GOING_TO_SLEEP_DELAY_AFTER_REQUEST_MSEC 1000*60

#define NODE_ID								  { 0x02  }

#define CHILD_NODE_ID_LENGTH				  1
#define MASTER_NODE_ID_LENGTH				  6

#define MAX_SUPPORTED_CHILD_NODES			  60
#define MAX_SUPPORTED_MASTER_NODES			  30
#if MAX_SUPPORTED_CHILD_NODES < 90
#warning MAX_SUPPORTED_CHILD_NODES is low because of debugging pourposes, it can be set to 90
#endif

// Maximum number of scan responses to be reported to application
#define DEFAULT_MAX_SCAN_RES                  MAX_SUPPORTED_CHILD_NODES+MAX_SUPPORTED_MASTER_NODES

#define ADV_PKT_ID_OFFSET					  12
#define ADV_PKT_STATE_OFFSET				  ADV_PKT_ID_OFFSET + CHILD_NODE_ID_LENGTH

#define GATT_PKT_SET_ID_CMD					  0x01

#define BROADCAST_MSG_TYPE_STATE_UPDATE_CMD	  0x01
#define BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD 0x02

#define BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD	  0x02
#define BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD  0x04

#define SNV_BASE_ID							  0x80
#define TIMERS_SNV_OFFSET					  0x02

#define MAX_ALLOWED_TIMER_DURATION_SEC	      42000 //actual max timer duration 42949.67sec

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define P_STATE_CHANGE_EVT                  0x0001
#define P_CHAR_CHANGE_EVT                   0x0002
#define PERIODIC_EVT                        0x0004
#define CONN_EVT_END_EVT                    0x0008
#define O_STATE_CHANGE_EVT                  0x0010
#define ADVERTISE_EVT					    0x0020
#define KEY_CHANGE_EVT					  	0x0040
#define LED_TIMEOUT_EVT						0x0080
#define CONNECTABLE_TIMEOUT_EVT				0x0100
#define EPOCH_EVT							0x0200
#define WAKEUP_TIMEOUT_EVT					0x0400
#define GOTOSLEEP_TIMEOUT_EVT				0x0800
#define PRE_ADV_EVT					0x1000
/*********************************************************************
 * TYPEDEFS
 */

typedef enum ChildClimbNodeStateType_t {
	BY_MYSELF = 0,
	CHECKING = 1,
	ON_BOARD = 2,
	ALERT = 3,
	GOING_TO_SLEEP = 4,
	ERROR = 5,
	INVALID_STATE = 6
} ChildClimbNodeStateType_t;

typedef enum MasterClimbNodeStateType_t {
	NOT_CONNECTED,
	CONNECTED,
	INVALID_MASTER_STATE
} MasterClimbNodeStateType_t;

typedef enum ClimbNodeType_t {
	CLIMB_CHILD_NODE,
	CLIMB_MASTER_NODE,
	NOT_CLIMB_NODE,
	NAME_NOT_PRESENT,
	WRONG_PARKET_TYPE
} ClimbNodeType_t;

// App event passed from profiles.
typedef struct {
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} sbpEvt_t;

typedef struct {
	//gapDevRec_t devRec;
	//uint8 addr[B_ADDR_LEN]; //!< Device's Address
	//uint8 id[NODE_ID_LENGTH];
	uint8 id[6];
	ChildClimbNodeStateType_t state;
	//uint8 advDataLen;
	//uint8 advData[31];
	//uint8 scnDataLen;
	//uint8 scnData[31];
	uint32 lastContactTicks;
	uint8 rssi;
	//ChildClimbNodeStateType_t stateToImpose;
} myGapDevRec_t;

typedef struct listNode{
    myGapDevRec_t device;
    struct listNode *next;
}listNode_t;

typedef struct {
	Clock_Struct wakeUpClock_Struct;
	Clock_Struct goToSleepClock_Struct;

	//uint32 wakeUpRemainingTimeout;
	//uint32 goToSleepRamainingTimeout;

	uint8 wakeUpTimerActive;
	uint8 goToSleepTimerActive;

	uint32 wakeUpTimeout_sec_global_value;

	uint8 validData;
} timerSNVDataStore_t;


/*********************************************************************
 * LOCAL VARIABLES
 */


// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct ledTimeoutClock;
static Clock_Struct connectableTimeoutClock;
static Clock_Struct wakeUpClock;
static Clock_Struct goToSleepClock;

static Clock_Struct preAdvClock;
#ifdef WORKAROUND
static Clock_Struct epochClock;
#endif
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration
//Task_Struct sbpTask;
//Char sbpTaskStack[SBP_TASK_STACK_SIZE];

static ChildClimbNodeStateType_t nodeState = BY_MYSELF;

static uint8 Climb_masterNodeName[] = {'C','L','I','M','B','M'};

static uint8 beaconActive = 0;

static uint8 myAddr[B_ADDR_LEN];
static uint8 myMasterId[MASTER_NODE_ID_LENGTH];

static uint8 advertData[31] = {

		0x07,// length of this data
		GAP_ADTYPE_LOCAL_NAME_COMPLETE,
		'C',
		'L',
		'I',
		'M',
		'B',
		'C',
		0x05,// length of this data
		GAP_ADTYPE_MANUFACTURER_SPECIFIC,
		0x0D,
		0x00,
		0, //ID
		(uint8)BY_MYSELF,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0

};

 // GAP - SCAN RSP data (max size = 31 bytes)
 static uint8 defScanRspData[] =
 {

 0x03,   // length of this data including the data type byte
 GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
 0x0D, // Company ID - Fixed
 0x00 // Company ID - Fixed

 };

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CLIMB Node";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Pins that are actively used by the application
static PIN_Config SensortagAppPinTable[] =
{
    Board_LED1       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_LED2       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    //Board_KEY_LEFT   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
    //Board_KEY_RIGHT  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
    //Board_RELAY      | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,      /* Relay is active high          */
    Board_BUZZER     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Buzzer initially off          */
	//Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;
#ifdef CLIMB_DEBUG
static uint8 adv_counter = 0;
#endif

static uint8 myIDArray[] = {0x00};//NODE_ID;
static uint8 broadcastID[] = { 0xFF };

//static uint8 readyToGoSleeping = 0;

static uint32 batteryLev = 0;

static uint32 wakeUpTimeout_sec_global;

static uint8 scanning = FALSE;

static uint8 devicesHeardDuringLastScan = 0;

static uint8 adv_startNodeIndex = 0;

static myGapDevRec_t* childListArray = NULL;
static myGapDevRec_t* masterListArray = NULL;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

////TASK FUNCTIONS
static void SimpleBLEPeripheral_init( void );
static void nodeListInit(void);
//static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

////GENERIC BLE STACK SUPPORT FUNCTIONS - EVENTS,MESSAGES ecc
static uint8_t BLE_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void BLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void Climb_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);
static void BLE_ConnectionEventHandler(void);
static void BLE_AdvertiseEventHandler(void);

////BLE STACK CBs
static uint8_t BLEObserver_eventCB(gapObserverRoleEvent_t *pEvent);
static void BLEPeripheral_stateChangeCB(gaprole_States_t newState);
static void BLEPeripheral_charValueChangeCB(uint8_t paramID);
static void Keys_EventCB(keys_Notifications_t notificationType);

////CLIMB IN/OUT FUNCTIONS
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent);
//static void Climb_updateRssiDataInScnRsp(void);

////CLIMB MANAGEMENT
static ClimbNodeType_t isClimbNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static void Climb_addNodeDeviceInfo( gapDeviceInfoEvent_t *gapDeviceInfoEvent , ClimbNodeType_t nodeType);
//static listNode_t* Climb_findNodeByDevice(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
static uint8 Climb_findNodeById(uint8 *nodeID, ClimbNodeType_t nodeType);
static uint8 Climb_addNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
static void Climb_updateNodeMetadata(gapDeviceInfoEvent_t *gapDeviceInfoEvent, uint8 index, ClimbNodeType_t nodeType);
static uint8 Climb_checkNodeState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static ChildClimbNodeStateType_t Climb_findMyBroadcastedState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static uint8 Climb_decodeBroadcastedMessage(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static void Climb_updateMyBroadcastedState(ChildClimbNodeStateType_t newState);
static void Climb_nodeTimeoutCheck();
static void Climb_removeNode(uint8 indexToRemove, ClimbNodeType_t nodeType) ;
static void Climb_periodicTask();
#ifdef WORKAROUND
static void Climb_epochStartHandler();
#endif
static void Climb_preAdvEvtHandler();
static void Climb_goToSleepHandler();
static void Climb_wakeUpHandler();
static void Climb_setWakeUpClock(uint32 wakeUpTimeout_sec_local);

////HARDWARE RELATED FUNCTIONS
static void CLIMB_FlashLed(PIN_Id pinId);
static void CLIMB_handleKeys(uint8 keys);
static void startNode();
static void stopNode();
static void destroyChildNodeList();
static void destroyMasterNodeList();
static void Climb_setAsConnectable();
static void Climb_setAsNonConnectable();
//static void Key_callback(PIN_Handle handle, PIN_Id pinId);
#ifdef FEATURE_LCD
static void displayInit(void);
#endif

////GENERIC SUPPORT FUNCTIONS
static uint8_t SimpleBLEPeripheral_enqueueMsg (uint8_t event, uint8_t state, uint8_t *pData);
static void Climb_clockHandler(UArg arg);
static uint8 memcomp(uint8 * str1, uint8 * str2, uint8 len);
static uint8 isNonZero(uint8 * str1, uint8 len);

static uint8 restoreTimersConf(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  BLEPeripheral_stateChangeCB,     // Profile State Change Callbacks
  BLEObserver_eventCB	// GAP Event callback
};

// Keys Callbacks
static keysEventCBs_t Keys_EventCBs = { Keys_EventCB, // Profile State Change Callbacks
		};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static climbProfileCBs_t SimpleBLEPeripheral_climbProfileCBs =
{
  BLEPeripheral_charValueChangeCB, // Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
//void SimpleBLEPeripheral_createTask(void)
//{
//  Task_Params taskParams;
//
//  // Configure task
//  Task_Params_init(&taskParams);
//  taskParams.stack = sbpTaskStack;
//  taskParams.stackSize = SBP_TASK_STACK_SIZE;
//  taskParams.priority = SBP_TASK_PRIORITY;
//
//  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
//}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void) {

	// Setup I2C for sensors
	bspI2cInit();

	// Handling of buttons, LED, relay
	hGpioPin = PIN_open(&pinGpioState, SensortagAppPinTable);
	//PIN_registerIntCb(hGpioPin, Key_callback);

	//initialize keys
	Keys_Init(&Keys_EventCBs);

	// ******************************************************************
	// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &sem);

	// Hard code the BD Address till CC2650 board gets its own IEEE address
	//uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
	//HCI_EXT_SetBDADDRCmd(bdAddress);

	// Set device's Sleep Clock Accuracy
	//HCI_EXT_SetSCACmd(40);

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueue = Util_constructQueue(&appMsg);

	//Board_initKeys(SimpleBLEObserver_keyChangeHandler);

	// Create one-shot clocks for internal periodic events.
	Util_constructClock(&periodicClock, Climb_clockHandler,
	PERIODIC_EVT_PERIOD_MSEC, 0, false, PERIODIC_EVT);

	Util_constructClock(&ledTimeoutClock, Climb_clockHandler,
	LED_TIMEOUT_MSEC, 0, false, LED_TIMEOUT_EVT);

	Util_constructClock(&connectableTimeoutClock, Climb_clockHandler,
	CONNECTABLE_TIMEOUT_MSEC, 0, false, CONNECTABLE_TIMEOUT_EVT);

	Util_constructClock(&wakeUpClock, Climb_clockHandler,
	WAKEUP_DEFAULT_TIMEOUT_SEC*1000, 0, false, WAKEUP_TIMEOUT_EVT);

	Util_constructClock(&goToSleepClock, Climb_clockHandler,
	GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000, 0, false, GOTOSLEEP_TIMEOUT_EVT);

	Util_constructClock(&preAdvClock, Climb_clockHandler,
	PRE_ADV_TIMEOUT, 0, false, PRE_ADV_EVT);

#ifdef WORKAROUND
	Util_constructClock(&epochClock, Climb_clockHandler,
			EPOCH_PERIOD, 0, false, EPOCH_EVT);
#endif
#ifndef SENSORTAG_HW
	Board_openLCD();
#endif //SENSORTAG_HW

#if SENSORTAG_HW
	// Setup SPI bus for serial flash and Devpack interface
	bspSpiOpen();
#endif //SENSORTAG_HW

	//read ID
	osal_snv_read(SNV_BASE_ID, CHILD_NODE_ID_LENGTH, myIDArray);
	memcpy(&advertData[ADV_PKT_ID_OFFSET], myIDArray, CHILD_NODE_ID_LENGTH);

	// Setup the GAP
	GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

	// Setup the GAP Peripheral Role Profile
	{
		// For all hardware platforms, device starts advertising upon initialization
		uint8_t initialAdvertEnable = FALSE;

		// By setting this to zero, the device will go into the waiting state after
		// being discoverable for 30.72 second, and will not being advertising again
		// until the enabler is set back to TRUE
		uint16_t advertOffTime = 0;

		uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
		uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
		uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
		uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
		uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

		// Set the GAP Role Parameters
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initialAdvertEnable);
		GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t), &advertOffTime);

		GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(defScanRspData), defScanRspData);
		GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

		GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t), &enableUpdateRequest);
		GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desiredMinInterval);
		GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desiredMaxInterval);
		GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t), &desiredSlaveLatency);
		GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t), &desiredConnTimeout);
	}

	// Set the GAP Characteristics
	GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

	// Set advertising interval
	{
		uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

		GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
		GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
		GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
		GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
		GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
		GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);
	}

	// Setup the GAP Bond Manager
	{
		uint32_t passkey = 0; // passkey "000000"
		uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
		uint8_t mitm = TRUE;
		uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
		uint8_t bonding = TRUE;

		GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
		GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
		GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
		GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
		GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
	}

	// Power on self-test for sensors, flash and DevPack
	sensorBmp280Init();
	sensorBmp280Enable(FALSE);
	sensorHdc1000Init();
	//PIN_setOutputValue(hGpioPin, Board_MPU_POWER, 0);
	//sensorMpu9250Init(); //ho gia scollegato l'alimentazione all'interno del file Board.c
	//sensorMpu9250Reset();
	sensorOpt3001Init();
	sensorTmp007Init();

#ifdef FEATURE_LCD
	displayInit();
#endif
	// Initialize GATT attributes
	GGS_AddService(GATT_ALL_SERVICES);           // GAP
	GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
	//DevInfo_AddService();

	ClimbProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

	// Setup the ClimbProfile Characteristic Values
	{
		//uint8_t charValue1 = 1;

		//ClimbProfile_SetParameter(CLIMBPROFILE_CHAR1, sizeof(uint8_t), &charValue1);
	}

	// Setup Observer Profile

	uint8 childScanRes = DEFAULT_MAX_SCAN_RES;
	GAPRole_SetParameter(GAPOBSERVERROLE_MAX_SCAN_RES, sizeof(uint8_t), &childScanRes);

	GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
	GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
	GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, SCAN_INTERVAL);
	GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, SCAN_INTERVAL);
	GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, SCAN_WINDOW);
	GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, SCAN_WINDOW);
	GAP_SetParamValue(TGAP_FILTER_ADV_REPORTS, FILTER_ADV_REPORTS);

	// Register callback with SimpleGATTprofile
	ClimbProfile_RegisterAppCBs(&SimpleBLEPeripheral_climbProfileCBs);

	// Start the Device
	bStatus_t status = GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
#ifdef PRINTF_ENABLED
	System_printf("\nBLE Peripheral+Broadcaster started (advertise started). Status: %d\n\n", status);
#endif
	// Start Bond Manager
	VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

	// Register with GAP for HCI/Host messages
	GAP_RegisterForMsgs(selfEntity);

	// Register for GATT local events and ATT Responses pending for transmission
	GATT_RegisterForMsgs(selfEntity);

	HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);

	nodeListInit();

	if ( restoreTimersConf() ) { // the flash contained a valid timers configuration
		if (Util_isActive(&goToSleepClock)) { //if this timer is active it means that this boot follows a fault, and before the fault the node was active, then start it.
			startNode(); //start the node
		} else {
			//do nothing
		}
	} else {  // the flash did not contain a valid configuration. Proceed with normal boot
//automatically start-up the node
//#warning the node is configured to start up automatically upon power up
//		events |= WAKEUP_TIMEOUT_EVT;
//		Semaphore_post(sem);
	}
}

static void nodeListInit(void){

	uint16 arraySize = MAX_SUPPORTED_CHILD_NODES*sizeof(myGapDevRec_t);
	uint16 i = 0;
	childListArray = (myGapDevRec_t*) ICall_malloc( arraySize );

	//resetta la memoria
	for(i = 0; i < arraySize; i++ ){
		((uint8*)childListArray)[i] = 0;
	}


	arraySize = MAX_SUPPORTED_MASTER_NODES*sizeof(myGapDevRec_t);
	masterListArray = (myGapDevRec_t*) ICall_malloc( arraySize );

	//resetta la memoria
	for(i = 0; i < arraySize; i++ ){
		((uint8*)masterListArray)[i] = 0;
	}

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
void CLIMB_taskFxn(uint32* a0, uint32* a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();

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
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest, (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              BLE_ConnectionEventHandler();
            }
            if (pEvt->event_flag & ADVERTISE_EVT) {
              BLE_AdvertiseEventHandler();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = BLE_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg)
        {
          if(safeToDealloc){
        	  ICall_freeMsg(pMsg);
          }
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & PERIODIC_EVT)
    {
      events &= ~PERIODIC_EVT;

      if(beaconActive != 0){
    	  Util_startClock(&periodicClock);
    	  // Perform periodic application task
          Climb_periodicTask();
      }

    }
    if (events & LED_TIMEOUT_EVT){
    	events &= ~LED_TIMEOUT_EVT;
    	//only turn off leds
    	PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_OFF);
    	PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
    }

    if (events & CONNECTABLE_TIMEOUT_EVT){
    	events &= ~CONNECTABLE_TIMEOUT_EVT;

    	Climb_setAsNonConnectable();
    }

	if (events & GOTOSLEEP_TIMEOUT_EVT) {
		events &= ~GOTOSLEEP_TIMEOUT_EVT;

		Climb_goToSleepHandler();
	}

	if (events & WAKEUP_TIMEOUT_EVT) {
		events &= ~WAKEUP_TIMEOUT_EVT;

		Climb_wakeUpHandler();
	}

	if (events & PRE_ADV_EVT) {
		events &= ~PRE_ADV_EVT;

		Climb_preAdvEvtHandler();


	}

#ifdef WORKAROUND
	if (events & EPOCH_EVT) {
		events &= ~EPOCH_EVT;

		if (beaconActive == 1) {

			Climb_epochStartHandler();

			float randDelay = 10 * ((float) Util_GetTRNG()) / 4294967296;
			Util_restartClock(&epochClock, EPOCH_PERIOD + randDelay);

		}

	}
#endif
  }
}

/*********************************************************************
 * @fn      BLE_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t BLE_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;
            
          default:
            break;
        }
      }
      break;

    case GAP_MSG_EVENT:
      SimpleBLEObserver_processRoleEvent((gapObserverRoleEvent_t *) pMsg);
      break;

    default:
      // do nothing
      break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case P_STATE_CHANGE_EVT:
      BLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->hdr.state);
      break;

    case P_CHAR_CHANGE_EVT:
      Climb_processCharValueChangeEvt(pMsg->hdr.state);
      break;

	case O_STATE_CHANGE_EVT:
	  BLE_processStackMsg((ICall_Hdr *)pMsg->pData);

	  // Free the stack message
	  ICall_freeMsg(pMsg->pData);
	  break;

	case KEY_CHANGE_EVT:
	  CLIMB_handleKeys(pMsg->hdr.state);
	  break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      BLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void BLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
//        uint8_t ownAddress[B_ADDR_LEN];
//        //uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
//
//        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
//        systemId[0] = ownAddress[0];
//        systemId[1] = ownAddress[1];
//        systemId[2] = ownAddress[2];
//
//        // set middle bytes to zero
//        systemId[4] = 0x00;
//        systemId[3] = 0x00;
//
//        // shift three bytes up
//        systemId[7] = ownAddress[5];
//        systemId[6] = ownAddress[4];
//        systemId[5] = ownAddress[3];

        //DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

      }
      break;

    case GAPROLE_ADVERTISING:
      //LCD_WRITE_STRING("Advertising", LCD_PAGE2);
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
        advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Reset flag for next connection.
        firstConnFlag = false;
        
        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER   

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        //Util_startClock(&periodicClock);

        //LCD_WRITE_STRING("Connected", LCD_PAGE2);
        //LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      //LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      Climb_setAsNonConnectable();
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      Climb_setAsNonConnectable();
      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
       break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      Climb_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void Climb_processCharValueChangeEvt(uint8_t paramID)
{

	uint8_t newValue[20];

	switch (paramID) {
	case CLIMBPROFILE_CHAR1: {
		ClimbProfile_GetParameter(CLIMBPROFILE_CHAR1, newValue);

		break;
	}
	case CLIMBPROFILE_CHAR2: {
		ClimbProfile_GetParameter(CLIMBPROFILE_CHAR2, newValue);

		if(newValue[0] == GATT_PKT_SET_ID_CMD){
			memcpy(myIDArray, &newValue[1], CHILD_NODE_ID_LENGTH);
			osal_snv_write(SNV_BASE_ID, CHILD_NODE_ID_LENGTH, &myIDArray);
		}

		break;
	}
	default:
		// should not reach here!
		break;
	}
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      BLE_ConnectionEventHandler
 *
 * @brief   Called after every connection event
 *
 * @param   none
 *
 * @return  none
 */
static void BLE_ConnectionEventHandler(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
    }
  }
}

/*********************************************************************
 * @fn      BLE_AdvertiseEventHandler
 *
 * @brief   Called after every advertise event
 *
 * @param
 *
 * @return  none
 */
static void BLE_AdvertiseEventHandler(void) {
	//temp_tick_3 = Clock_getTicks();
	Util_startClock(&preAdvClock);

#ifdef WORKAROUND
	uint8 adv_active = 0;
	uint8 status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),&adv_active);
	status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&adv_active);
#endif
#ifdef PRINTF_ENABLED
		System_printf("\nAdvertise event\n");
#endif

}






/*********************************************************************
 * @fn      BLEObserver_eventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t BLEObserver_eventCB(gapObserverRoleEvent_t *pEvent) {
	// Forward the role event to the application
	if ( SimpleBLEPeripheral_enqueueMsg( O_STATE_CHANGE_EVT , SUCCESS , (uint8_t *) pEvent) ) {
		// App will process and free the event
		return FALSE;
	}

	// Caller should free the event
	return TRUE;
}

/*********************************************************************
 * @fn      BLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void BLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(P_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      BLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void BLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(P_CHAR_CHANGE_EVT, paramID, NULL);
}
/*********************************************************************
 * @fn      Keys_EventCB
 *
 * @brief   Callback from Keys task indicating a role state change.
 *
 * @param   notificationType - type of button press
 *
 * @return  None.
 */
static void Keys_EventCB(keys_Notifications_t notificationType) {

	SimpleBLEPeripheral_enqueueMsg(KEY_CHANGE_EVT, (uint8) notificationType,
			NULL);

}

/*********************************************************************
 * @fn      SimpleBLEObserver_processRoleEvent
 *
 * @brief   Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent) {

	switch (pEvent->gap.opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		memcpy(myAddr, pEvent->initDone.devAddr,B_ADDR_LEN); //salva l'indirizzo del nodo

		//myIDArray[0] = myAddr[0];

#ifdef PRINTF_ENABLED
		System_printf(Util_convertBdAddr2Str(pEvent->initDone.devAddr));
		System_printf("\nGAP Initialized\n");
#endif
#ifdef FEATURE_LCD
		char buf[10];
		sprintf(buf,"Me: ");
		devpkLcdText(buf, 1, 0);
		sprintf(buf,Util_convertBdAddr2Str(childDevList[0].devRec.addr));
		devpkLcdText(buf, 1, 5);
#endif
		// enable advertise event notification

}
		break;

	case GAP_DEVICE_INFO_EVENT: {

		devicesHeardDuringLastScan++;

		if (pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_SCAN_IND | //adv data event (Scannable undirected)
			pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND      |
			pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_NONCONN_IND) { //adv data event (Connectable undirected)

#ifdef PRINTF_ENABLED
			System_printf(
					"\nGAP_DEVICE_INFO_EVENT-GAP_ADRPT_ADV_SCAN_IND, ADV_DATA. address: ");
			System_printf(Util_convertBdAddr2Str(pEvent->deviceInfo.addr));
			System_printf("\nChecking if it is a CLIMB node.\n");
#endif
			ClimbNodeType_t nodeType = isClimbNode((gapDeviceInfoEvent_t*) &pEvent->deviceInfo);
			if (nodeType == CLIMB_CHILD_NODE || nodeType == CLIMB_MASTER_NODE) {
				Climb_addNodeDeviceInfo(&pEvent->deviceInfo, nodeType);
				if (nodeType == CLIMB_MASTER_NODE){
					Climb_checkNodeState(&pEvent->deviceInfo);
				}
			}else {

#ifdef PRINTF_ENABLED
				System_printf("\nIt isn't a CLIMB node, device discarded!\n");
#endif
			}
		} else if(pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP) {  //scan response data event
#ifdef PRINTF_ENABLED
			System_printf("\nScan response received!\n");
#endif
		}

		if (devicesHeardDuringLastScan >= DEFAULT_MAX_SCAN_RES) {
			if (beaconActive) {
				GAPObserverRole_CancelDiscovery();
				scanning = FALSE;
				if (!scanning) {
					uint8 status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
					scanning = TRUE;
				}
			}
		}

	}
		break;

	case GAP_DEVICE_DISCOVERY_EVENT:

		devicesHeardDuringLastScan = 0;
#ifndef WORKAROUND

		scanning = FALSE;



			//GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
			//Util_startClock(&preAdvClock);
			//CLIMB_FlashLed(Board_LED2);

#endif
		break;
	case GAP_ADV_DATA_UPDATE_DONE_EVENT:
		//temp_tick_2 = Clock_getTicks();
		break;
	case GAP_MAKE_DISCOVERABLE_DONE_EVENT:

		break;
	case GAP_END_DISCOVERABLE_DONE_EVENT:

		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      Climb_updateRssiDataInScnRsp
 *
 * @brief   Updates scn rsp data accordingly to stored data

 *
 * @return  none
 */
/*
static void Climb_updateRssiDataInScnRsp(void) {

	uint8 i = 4;
	uint8 observedDeviceData[31]; //per ora limito a 20, verificare quanto è realmente

	observedDeviceData[1] = GAP_ADTYPE_MANUFACTURER_SPECIFIC; // manufacturer specific adv data type
	observedDeviceData[2] = 0x0D; // Company ID - Fixed //VERIFICARE SE QUESTA REGOLA VALE ANCHE PER I TAG NON COMPATIBILI CON IBEACON
	observedDeviceData[3] = 0x00; // Company ID - Fixed

	listNode_t* node = childListRootPtr;

	while (node != NULL && i < 18) {
		observedDeviceData[i++] = node->device.devRec.addr[0];
		observedDeviceData[i++] = node->device.rssi;
		node = node->next; //passa al nodo sucessivo
	}

	observedDeviceData[0] = (i-1);

	GAP_UpdateAdvertisingData(selfEntity, true, i,	observedDeviceData);

}
*/








/*********************************************************************
 * @fn      isClimbNode
 *
 * @brief	checks if the gap event is related to a Climb node or not
 *
 * @param	pointer to gap info event
 *
 * @return  Type of node, see ClimbNodeType_t
 */

static ClimbNodeType_t isClimbNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	uint8 index = 0;

	if (gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND |
		gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND      |
		gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //il nome è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1]
					== GAP_ADTYPE_LOCAL_NAME_COMPLETE) { //ho trovato il nome del dispositivo

				if (memcomp(&(gapDeviceInfoEvent_a->pEvtData[index + 2]),&(advertData[2]),(gapDeviceInfoEvent_a->pEvtData[index]) - 1) == 0) { //il nome è compatibile con CLIMB

					return CLIMB_CHILD_NODE;

				}else if(memcomp(&(gapDeviceInfoEvent_a->pEvtData[index + 2]),&Climb_masterNodeName[0],(gapDeviceInfoEvent_a->pEvtData[index]) - 1) == 0 ){ //TODO: verificare...
					return CLIMB_MASTER_NODE;
				}
				else {
					return NOT_CLIMB_NODE; //con questo return blocco la ricerca appena trovo un nome, quindi se dentro il pachetto sono definiti due nomi la funzione riconoscerà solo il primo
				}

			} else { // ricerca il nome nella parte successiva del pacchetto
				index = index + gapDeviceInfoEvent_a->pEvtData[index] + 1;
			}

		}
		return NAME_NOT_PRESENT; //

	} else { //sto cercando il nome dentro un scan response
		return WRONG_PARKET_TYPE; //
	}
}

/*********************************************************************
 * @fn      Climb_addNodeDeviceInfo
 *
 * @brief	adds information related to a gap info event, if the node isn't in the list is added, otherwise if the node is already inside the list its data is updated
 *
 * @return  none
 */
static void Climb_addNodeDeviceInfo( gapDeviceInfoEvent_t *gapDeviceInfoEvent , ClimbNodeType_t nodeType) {

	uint8 node_position = 255;

	if(nodeType == CLIMB_CHILD_NODE){
		node_position = Climb_findNodeById(&gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], nodeType);
	}else if(CLIMB_MASTER_NODE){
		node_position = Climb_findNodeById(gapDeviceInfoEvent->addr, nodeType);
		//return; //for now a master don't need to store other master nodes data, this saves some memory....
	}

	if(node_position == 255){	//dispositivo nuovo, aggiungilo!
		Climb_addNode(gapDeviceInfoEvent,nodeType);
	}else{
		Climb_updateNodeMetadata(gapDeviceInfoEvent,node_position,nodeType);
	}

	return;
}

/*********************************************************************
 * @fn      Climb_findNodeByDevice
 *
 * @brief	Find the node inside child or master lists and returns the node pointer. If the node cannot be found this function returns null. If the list is empty it returns null.
 *
 * @return  Pointer to node instance that can be used to modify stored data
 */
//static listNode_t* Climb_findNodeByDevice(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType){
//
//	listNode_t* node = NULL;
//		if (nodeType == CLIMB_CHILD_NODE ) {
//			if (childListRootPtr == NULL) {
//				return NULL;
//			}
//			node = childListRootPtr;
//		} else if (nodeType == CLIMB_MASTER_NODE ) {
//			if (masterListRootPtr == NULL) {
//				return NULL;
//			}
//			node = masterListRootPtr;
//		}
//
//		while (node != NULL) {
//			if (memcomp(gapDeviceInfoEvent->addr, node->device.addr,	B_ADDR_LEN) == 0) {
//				return node; //se trovi il nodo esci e passane il puntatore
//			}
//			node = node->next; //passa al nodo sucessivo
//		}
//
//		return NULL;
//}

/*********************************************************************
 * @fn      Climb_findNodeById
 *
 * @brief	Find the node inside child lists (it uses only the ID information) and returns the node pointer. If the node cannot be found this function returns null. If the list is empty it returns null.
 *
 * @return  none
 */
static uint8 Climb_findNodeById(uint8 *nodeID, ClimbNodeType_t nodeType) {

	uint8 i;
	if (nodeType == CLIMB_CHILD_NODE) {

		for (i = 0; i < MAX_SUPPORTED_CHILD_NODES; i++) {
			if (memcomp(nodeID, childListArray[i].id, CHILD_NODE_ID_LENGTH) == 0) {
				return i;
			}
		}

	} else if (nodeType == CLIMB_MASTER_NODE) {
		for (i = 0; i < MAX_SUPPORTED_MASTER_NODES; i++) {
			if (memcomp(nodeID, masterListArray[i].id, MASTER_NODE_ID_LENGTH) == 0) {
				return i;
			}
		}

	}
	return 255;

}

/*********************************************************************
 * @fn      Climb_addNode
 *
 * @brief	Adds a node at the end of the child or master list with informations contained in the gap info event. It automatically manages the adding of first node which is critical because it is referenced by childListRootPtr or masterListRootPtr
 *
 * @return  The pointer to the node istance inside the list
 */
static uint8 Climb_addNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType) {

	uint8 freePos = 0;

	if (nodeType == CLIMB_CHILD_NODE) {

		while( isNonZero(childListArray[freePos].id, CHILD_NODE_ID_LENGTH) ){
			freePos++;

			if(freePos >= MAX_SUPPORTED_CHILD_NODES){
				return 255;
			}

		}


		childListArray[freePos].rssi = gapDeviceInfoEvent->rssi;
		childListArray[freePos].lastContactTicks = Clock_getTicks();

		memcpy(childListArray[freePos].id, &gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], CHILD_NODE_ID_LENGTH);
		childListArray[freePos].state = (ChildClimbNodeStateType_t) gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET];

		return 1;
	} else if (nodeType == CLIMB_MASTER_NODE) {

		while (isNonZero(masterListArray[freePos].id, MASTER_NODE_ID_LENGTH)) {
			freePos++;

			if (freePos >= MAX_SUPPORTED_MASTER_NODES) {
				return 255;
			}

		}

		masterListArray[freePos].rssi = gapDeviceInfoEvent->rssi;
		masterListArray[freePos].lastContactTicks = Clock_getTicks();

		memcpy(masterListArray[freePos].id, &gapDeviceInfoEvent->addr, MASTER_NODE_ID_LENGTH);
		masterListArray[freePos].state = INVALID_STATE;

		return 1;
	}
	return 0;

}


/*********************************************************************
 * @fn      Climb_updateNodeMetadata
 *
 * @brief	updates targetNode metadata with infomation contained in gapDeviceInfoEvent
 *
 * @return  none
 */
static void Climb_updateNodeMetadata(gapDeviceInfoEvent_t *gapDeviceInfoEvent, uint8 index, ClimbNodeType_t nodeType) {

	if (gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_SCAN_IND | gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_IND | gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_NONCONN_IND) {//adv data

		if (nodeType == CLIMB_CHILD_NODE) {

			childListArray[index].rssi = gapDeviceInfoEvent->rssi;
			childListArray[index].lastContactTicks = Clock_getTicks();

			memcpy(childListArray[index].id, &gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], CHILD_NODE_ID_LENGTH);
			childListArray[index].state = (ChildClimbNodeStateType_t) gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET];

			return;

		} else if (nodeType == CLIMB_MASTER_NODE) {

			masterListArray[index].rssi = gapDeviceInfoEvent->rssi;
			masterListArray[index].lastContactTicks = Clock_getTicks();

			memcpy(masterListArray[index].id, &gapDeviceInfoEvent->addr, MASTER_NODE_ID_LENGTH);
			masterListArray[index].state = INVALID_STATE;

			return;

		}

	} else if (gapDeviceInfoEvent->eventType == GAP_ADRPT_SCAN_RSP) {	//scan response data

	}

	return;

}

/*********************************************************************
 * @fn      Climb_checkNodeState
 *
 * @brief   Checks if the state of this node is the same as the one broadcasted by the master, if not it call Climb_updateMyBroadcastedState. Moreover it saves myMasterId!
 *
 * @return  1 if the state has been updated, 0 if not.
 */

static uint8 Climb_checkNodeState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	ChildClimbNodeStateType_t broadcastedState;

	//OBTAIN broadcastedState and check where the message is from (myMaster or not)
	if (nodeState == CHECKING || nodeState == ON_BOARD || nodeState == ALERT) {
		if (memcomp(gapDeviceInfoEvent_a->addr, myMasterId, B_ADDR_LEN) == 0) { //sto analizzando un adv del MIO master
			broadcastedState = Climb_findMyBroadcastedState(gapDeviceInfoEvent_a);
		} else {	//sto analizzando l'adv di un altro master,
			return 0;
		}
	} else { //se sono in by myself cerca in tutti i master visibili (il primo master che mi farà fare il checking diventerà il MIO master)
		broadcastedState = Climb_findMyBroadcastedState(gapDeviceInfoEvent_a);
	}

	//VERIFY STATE TRANSITIONS
	//if( broadcastedState != nodeState || broadcastedState == INVALID_STATE){
	uint8 i;
	switch (broadcastedState) {
	case BY_MYSELF: //CHECK OUT
		//if (nodeState == ON_BOARD || nodeState == CHECKING) { //il checkout è possibile da qualsiasi stato
		for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del nodo master
			myMasterId[i] = 0;
		}
		if(nodeState == GOING_TO_SLEEP){
			Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000);
		}
		//if (nodeState == ON_BOARD || nodeState == ALERT) { //il passaggio da ON_BOARD o (ALERT) a BY_MYSELF triggera lo spegnimento del nodo
			//Util_restartClock(&goToSleepClock, 20000); //schedula lo spegnimento automatico
			//readyToGoSleeping = 1;
		//}
		//}
		break;
	case CHECKING:
		if (nodeState == BY_MYSELF) {
			//if (readyToGoSleeping == 0) {
			memcpy(myMasterId, gapDeviceInfoEvent_a->addr, MASTER_NODE_ID_LENGTH); //salva l'id del nodo master

		} else { //se il master sta cercando di mettere il nodo in checking ma questo non era in by myself scarta la richiesta
				 //broadcastedState = nodeState;
			return 0;
		}
		break;

	case ON_BOARD:
		if (nodeState == CHECKING || nodeState == ALERT) { //il passaggio in ON BOARD è permesso solo dagli stati checking e alert

		} else { //se il master sta cercando di mettere il nodo in checking ma questo non era in by myself scarta la richiesta
				 //broadcastedState = nodeState;
			return 0;
		}
		break;

	case ALERT:
		return 0;
		//break;

	case GOING_TO_SLEEP:
		if (nodeState == ON_BOARD || nodeState == CHECKING) { //il passaggio da ON_BOARD o (ALERT) a GOING_TO_SLEEP triggera lo spegnimento del nodo
			Util_restartClock(&goToSleepClock, GOING_TO_SLEEP_DELAY_AFTER_REQUEST_MSEC); //schedula lo spegnimento automatico
			//readyToGoSleeping = 1;
		} else {
			//broadcastedState = nodeState;
			return 0;
		}
		break;

	case INVALID_STATE:				//check to see if there is a broadcast message in this packet
		if (nodeState == BY_MYSELF) { //broadcast message can be received only if I have a myMaster Node
			//broadcastedState = nodeState;
			return 0;
		} else {
			Climb_decodeBroadcastedMessage(gapDeviceInfoEvent_a);
			return 0;
		}
		//break;

	default:
		return 0;
		//break;
	}

//		if(nodeState == BY_MYSELF && broadcastedState == CHECKING){ //ho trovato il master (pedibus driver) di oggi. se readyToGoSleeping è diverso da 0 significa che ho gia fatto un checkout, quindi non devo tornare ON_BOARD
//			if(readyToGoSleeping == 0){
//				memcpy(myMasterId, gapDeviceInfoEvent_a->addr,B_ADDR_LEN); //salva l'indirizzo del nodo master
//			}else{ //se il master sta cercando di fare un check in subito dopo il check out non permetterlo!
//				broadcastedState = nodeState;
//			}
//		}

//		if((nodeState == ON_BOARD || nodeState == CHECKING )  && broadcastedState == BY_MYSELF){ //CHECKOUT!!!!! si può fare anche se il child era in checking
//			uint8 i;
//			for(i = 0; i < B_ADDR_LEN; i++){ //resetta l'indirizzo del nodo master
//				myMasterId[i] = 0;
//			}
//
//			Util_restartClock(&goToSleepClock,20000); //schedula lo spegnimento automatico tra 60 secondi
//			readyToGoSleeping = 1;
//		}

#ifndef CLIMB_DEBUG
	Climb_updateMyBroadcastedState(broadcastedState);
#else	//quando il sistema è in modalità debug si aggiorna solo la variabile nodeState, la funzione Climb_updateMyBroadcastedState che aggiorna anche l'adv verrà chiamata dalla adv_event_handler
	nodeState = broadcastedState;
#endif

	return 1;
	//}
	//return 0;
}

/*********************************************************************
 * @fn      Climb_findMyBroadcastedState
 *
 * @brief   Search within broadcasted data to find the state master wants for this node.
 * 			It also find broadcast state (check in all, check out all).
 *
 * @param   none.
 *
 * @return  The found state, INVALID_STATE if not found
 */
static ChildClimbNodeStateType_t Climb_findMyBroadcastedState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	uint8 index = 0;

	if (  gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND
		| gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND
		| gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //lo stato è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1] == GAP_ADTYPE_MANUFACTURER_SPECIFIC) { //ho trovato il campo GAP_ADTYPE_MANUFACTURER_SPECIFIC

				uint8 manufacter_specific_field_end = index + gapDeviceInfoEvent_a->pEvtData[index]; //=15
				index = index + 4; //salto i campi length, adtype_flag e manufacter ID
				//uint8 temp_ID[]= myIDArray;//{myAddr[0]};//{myAddr[1] , myAddr[0]}; //address bytes must be flipped (for little/big endian stuf)
				while (index + 1 < manufacter_specific_field_end) {

					if (memcomp(myIDArray, &gapDeviceInfoEvent_a->pEvtData[index], CHILD_NODE_ID_LENGTH) == 0) { //MESSAGE ADDRESSED TO THIS NODE
						return (ChildClimbNodeStateType_t) (gapDeviceInfoEvent_a->pEvtData[index + 1]);

					} else if(memcomp(broadcastID, &gapDeviceInfoEvent_a->pEvtData[index], CHILD_NODE_ID_LENGTH) == 0) { // BROADCAST MESSAGE FOUND

						if(gapDeviceInfoEvent_a->pEvtData[index + 1] == BROADCAST_MSG_TYPE_STATE_UPDATE_CMD){
							return (ChildClimbNodeStateType_t) (gapDeviceInfoEvent_a->pEvtData[index + 2]); //return broadcasted state
						}else{
							return INVALID_STATE; //a broadcast message has been found but it is not BROADCAST_MSG_TYPE_STATE_UPDATE_CMD, then return since no other message is contained in this adv packet
						}

					}else{

						index = index + CHILD_NODE_ID_LENGTH + 1;
					}

				}

				return INVALID_STATE; //questo blocca la ricerca una volta trovato il flag GAP_ADTYPE_MANUFACTURER_SPECIFIC, quindi se ce ne fossero due il sistema vede solo il primo
			} else { // ricerca il nome nella parte successiva del pacchetto
				index = index + gapDeviceInfoEvent_a->pEvtData[index] + 1;
			}

		}
		return INVALID_STATE; //

	} else { //sto cercando il nome dentro un scan response
		return INVALID_STATE; //
	}
}
/*********************************************************************
 * @fn      Climb_decodeBroadcastedMessage
 *
 * @brief   Search within broadcasted data to find message sent to broadcast ID, if it is found the relative command is launced
 *
 * @param   none.
 *
 * @return  TRUE if some broadcast message have been found, FALSE otherwise
 */
static uint8 Climb_decodeBroadcastedMessage(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	uint8 index = 0;

	if (gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND | gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND
			| gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //lo stato è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1] == GAP_ADTYPE_MANUFACTURER_SPECIFIC) { //ho trovato il campo GAP_ADTYPE_MANUFACTURER_SPECIFIC

				uint8 manufacter_specific_field_end = index + gapDeviceInfoEvent_a->pEvtData[index];
				index = index + 4; //salto i campi length, adtype_flag e manufacter ID
				while (index < manufacter_specific_field_end && index < 29) {

					if (memcomp(broadcastID, &gapDeviceInfoEvent_a->pEvtData[index], CHILD_NODE_ID_LENGTH) == 0) { // BROADCAST MESSAGE FOUND

						uint8 broadcastMsgType = gapDeviceInfoEvent_a->pEvtData[index + 1];
						uint32 wakeUpTimeout_sec_local;
						//float randDelay_msec;

						switch(broadcastMsgType) {
						case BROADCAST_MSG_TYPE_STATE_UPDATE_CMD:
							//THIS MESSAGE IS DECODED IN Climb_findMyBroadcastedState(...);
							break;

						case BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD:
							//wakeUpTimeout_sec_global = ((gapDeviceInfoEvent_a->pEvtData[index + 2])<<16) + ((gapDeviceInfoEvent_a->pEvtData[index + 3])<<8) + (gapDeviceInfoEvent_a->pEvtData[index + 4]); //here the unit is seconds
							wakeUpTimeout_sec_local = ((gapDeviceInfoEvent_a->pEvtData[index + 2])<<16) + ((gapDeviceInfoEvent_a->pEvtData[index + 3])<<8) + (gapDeviceInfoEvent_a->pEvtData[index + 4]);
							Climb_setWakeUpClock( wakeUpTimeout_sec_local );
//							if(wakeUpTimeout_sec_global > MAX_ALLOWED_TIMER_DURATION_SEC){ //max timer duration 42949.67sec
//								Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC*1000);
//								wakeUpTimeout_sec_global = wakeUpTimeout_sec_global - MAX_ALLOWED_TIMER_DURATION_SEC;
//							}else{
//								//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
//								randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
//								Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_global*1000 - 1000 + randDelay_msec);
//								wakeUpTimeout_sec_global = 0; //reset this so that when the device wakes up, it knows that there is no need to restart timer but it is the actual time to wake up the device
//							}
							break;

						default: //should not reach here
							break;
						}
						return TRUE;

					} else {
						return FALSE; //the broadcast message cannot be placed at random position
						//index = index + NODE_ID_LENGTH + 1;
					}

				}

				return FALSE; //questo blocca la ricerca una volta trovato il flag GAP_ADTYPE_MANUFACTURER_SPECIFIC, quindi se ce ne fossero due il sistema vede solo il primo
			} else { // ricerca il nome nella parte successiva del pacchetto
				index = index + gapDeviceInfoEvent_a->pEvtData[index] + 1;
			}

		}
		return FALSE; //

	} else { //sto cercando il nome dentro un scan response
		return FALSE; //
	}
}
/*********************************************************************
 * @fn      Climb_updateMyBroadcastedState
 *
 * @brief   Updated the node state and adv data
 *
 * @param   new state
 *
 * @return  none
 */

static void Climb_updateMyBroadcastedState(ChildClimbNodeStateType_t newState) {

	uint8 newAdvertData[31];

	nodeState = newState;

	newAdvertData[0] = 0x07; // length of this data
	newAdvertData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
	newAdvertData[2] = advertData[2];
	newAdvertData[3] = advertData[3];
	newAdvertData[4] = advertData[4];
	newAdvertData[5] = advertData[5];
	newAdvertData[6] = advertData[6];
	newAdvertData[7] = advertData[7];
	//newAdvertData[8] = i-9; assigned later
	newAdvertData[9] = GAP_ADTYPE_MANUFACTURER_SPECIFIC;
	newAdvertData[10] = 0x0D;
	newAdvertData[11] = 0x00;
	memcpy(&newAdvertData[ADV_PKT_ID_OFFSET], myIDArray, CHILD_NODE_ID_LENGTH);
	newAdvertData[ADV_PKT_STATE_OFFSET] = (uint8)nodeState;

//TODO: rssi can be added also when CLIMB_DEBUG is not defined
#ifdef CLIMB_DEBUG
	uint8 i = ADV_PKT_STATE_OFFSET + 1;
	uint8 nodeArrayIndex = adv_startNodeIndex;
	uint8 roundCompleted = FALSE;

	while (i < 27 && roundCompleted == FALSE) {

		if (isNonZero(childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

			memcpy(&newAdvertData[i], childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH);
			i += CHILD_NODE_ID_LENGTH;
			newAdvertData[i++] = childListArray[nodeArrayIndex].rssi;

		}

		nodeArrayIndex++;

		if(nodeArrayIndex >= MAX_SUPPORTED_CHILD_NODES){
			nodeArrayIndex = 0;
		}

		if(nodeArrayIndex == adv_startNodeIndex){
			roundCompleted = TRUE;
		}

	}
	adv_startNodeIndex = nodeArrayIndex;

//	while(i < 30){
//		if(node != NULL && i < 29){
//			newAdvertData[i++] = node->device.devRec.addr[0];
//			newAdvertData[i++] = node->device.rssi;
//			node = node->next; //passa al nodo sucessivo
//		}else{
//			newAdvertData[i++] = 0;
//		}
//	}
#ifdef CLIMB_DEBUG
//	while (i < 28) {
//		newAdvertData[i++] = 0;
//	}
	newAdvertData[i++] = (uint8) (batteryLev >> 8);
	newAdvertData[i++] = (uint8) (batteryLev);
	newAdvertData[i++] = adv_counter; //the counter is always in the last position
#endif
	newAdvertData[8] = i-9;

	//temp_tick_1 = Clock_getTicks();
	GAP_UpdateAdvertisingData(selfEntity, true, i,	&newAdvertData[0]);

#else
	newAdvertData[8] = 0x04;// length of this data
	GAP_UpdateAdvertisingData(selfEntity, true, 12+ID_LENGTH+1,	&newAdvertData[0]);
#endif

	//GAP_UpdateAdvertisingData(selfEntity, false, 13,	&advertData[0]);

	return;
}
/*********************************************************************
 * @fn      Climb_nodeTimeoutCheck
 *
 * @brief	Check the timeout for every node contained in child and master lists. If timeout is expired the node is removed.
 *
 *
 * @return  none
 */
static void Climb_nodeTimeoutCheck() {
#ifdef PRINTF_ENABLED
	System_printf("\nRunning timeout check!\n");
#endif
	uint32 nowTicks = Clock_getTicks();
	uint16 i = 0;

	//controlla la lista dei child
	while (i < MAX_SUPPORTED_CHILD_NODES) { //NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		if (isNonZero(childListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

			if (nowTicks - childListArray[i].lastContactTicks > NODE_TIMEOUT_OS_TICKS) {

				Climb_removeNode(i,CLIMB_CHILD_NODE); //rimuovi il nodo

			} else { //il nodo non è andato in timeout,

			}
			//NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP

		}
		i++;
	}

	//controlla la lista dei master
	i = 0;
	while (i < MAX_SUPPORTED_MASTER_NODES) { //NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		if (isNonZero(masterListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces
			if (nowTicks - masterListArray[i].lastContactTicks > NODE_TIMEOUT_OS_TICKS) {

				//RIMUOVI SOLO I MASTER CHE NON SONO MY_MASTER
				if (memcomp(masterListArray[i].id, myMasterId, MASTER_NODE_ID_LENGTH) != 0) {
					Climb_removeNode(i,CLIMB_MASTER_NODE); //rimuovi il nodo
				} else {
					switch (nodeState) {
					case BY_MYSELF:
						//should not reach here since when this node is in BY_MYSELF the myMasterId should be non-valid, for safety reason I add the next line
						Climb_removeNode(i,CLIMB_MASTER_NODE); //rimuovi il nodo
						break;

					case CHECKING:
						nodeState = BY_MYSELF; //se dopo essere stato passato a checking non vedo più il master torna in BY_MYSELF
						uint8 i;
						for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del MY_MASTER
							myMasterId[i] = 0;
						}
						Climb_removeNode(i,CLIMB_MASTER_NODE); //rimuovi il nodo
						break;

					case ON_BOARD:
						//nodeState = ALERT; //se non ho visto il mio master vai in alert (solo se ero nello stato ON_BOARD)
						Climb_updateMyBroadcastedState(ALERT);
						break;

					case ALERT:
						//no nothing
						break;

					default:
						break;
					}

				}
			} else { //il nodo non è andato in timeout,

			}
		}

		i++;
		//NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
	}
}

/*********************************************************************
 * @fn      Climb_removeNode
 *
 * @brief	Removes nodeToRemove node form the list which contains it
 *
 * @return  pointer to the next node
 */
static void Climb_removeNode(uint8 indexToRemove, ClimbNodeType_t nodeType) {

	uint8 i = 0;

	for (i = 0; i < sizeof(myGapDevRec_t); i++) {

		if (nodeType == CLIMB_CHILD_NODE) {

			((uint8*) (&childListArray[indexToRemove]))[i] = 0;

		} else if (nodeType == CLIMB_MASTER_NODE) {

			((uint8*) (&masterListArray[indexToRemove]))[i] = 0;

		}

	}

	return;

}

/*********************************************************************
 * @fn      Climb_periodicTask
 *
 * @brief	Handler associated with Periodic Clock instance.
 *
 * @return  none
 */
static void Climb_periodicTask(){
	Climb_nodeTimeoutCheck();


//	uint32 nowSec;
//
//	nowSec = AONRTCSecGet();

#ifdef PRINTF_ENABLED
#ifdef CLIMB_DEBUG
#ifdef HEAPMGR_METRICS

	uint16 *pBlkMax;
	uint16 *pBlkCnt;
	uint16 *pBlkFree;
	uint16	*pMemAlo;
	uint16 *pMemMax;
	uint16 *pMemUb;

	ICall_heapGetMetrics(pBlkMax, pBlkCnt, pBlkFree, pMemAlo, pMemMax, pMemUb);

	System_printf("\nMax allocated(blocks): %d\n"
			      "Current allocated (blocks): %d\n"
			      "Current free (blocks):%d\n"
		      	  "Current allocated (bytes): %d\n"
	      	  	  "Max allocated (bytes): %d\n"
				  "Furthest allocated: %d\n",
				  *pBlkMax, *pBlkCnt, *pBlkFree, *pMemAlo, *pMemMax, pMemUb);
#endif
#endif
#endif
}
#ifdef WORKAROUND
/*********************************************************************
 * @fn      Climb_epochStartHandler
 *
 * @brief	Handler associated with epoch clock instance.
 *
 * @return  none
 */
static void Climb_epochStartHandler(){
	GAPObserverRole_CancelDiscovery();
	if(beaconActive){
		uint8 adv_active = 1;
		uint8 status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),&adv_active);
		GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
	}
}
#endif

static void Climb_preAdvEvtHandler(){

#ifdef CLIMB_DEBUG
	adv_counter++;

	if ((adv_counter - 1) % 60 == 0) {
		batteryLev = AONBatMonBatteryVoltageGet();
		batteryLev = (batteryLev * 125) >> 5;
	}

	Climb_updateMyBroadcastedState(nodeState); //update adv data every adv event to update adv_counter value. Since the argument is nodeState this function call doesn't modify the actual state of this node
#endif

	if(!scanning){
		if(beaconActive){
			GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
			scanning = TRUE;
		}
	}
}
/*********************************************************************
 * @fn      Climb_goToSleepHandler
 *
 * @brief	Handler associated with goToSleep Clock instance.
 *
 * @return  none
 */
static void Climb_goToSleepHandler(){

	stopNode();

	destroyChildNodeList();
	destroyMasterNodeList();

	uint8 i;
	for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del MY_MASTER
		myMasterId[i] = 0;
	}

	Climb_updateMyBroadcastedState(BY_MYSELF);

}

/*********************************************************************
 * @fn      Climb_wakeUpHandler
 *
 * @brief	Handler associated with wakeUp Clock instance.
 *
 * @return  none
 */
static void Climb_wakeUpHandler(){

	if(wakeUpTimeout_sec_global == 0){
		startNode();
		Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000);
		Climb_setWakeUpClock(WAKEUP_DEFAULT_TIMEOUT_SEC);

		return;
	}

	if(wakeUpTimeout_sec_global > MAX_ALLOWED_TIMER_DURATION_SEC){
		Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC*1000);
		wakeUpTimeout_sec_global = wakeUpTimeout_sec_global - MAX_ALLOWED_TIMER_DURATION_SEC;
	}else{
		//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
		float randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
		Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_global*1000 - 1000 + randDelay_msec);
		wakeUpTimeout_sec_global = 0; //reset this so that when the device wakes up, it knows that there is no need to restart timer but it is the actual time to wake up the device
	}

}
/*********************************************************************
 * @fn      Climb_setWakeUpClock
 *
 * @brief	Helper function that sets the wake up clock, it manages the clock overflow for intervals larger than MAX_ALLOWED_TIMER_DURATION_SEC
 *
 * @return  none
 */
static void Climb_setWakeUpClock(uint32 wakeUpTimeout_sec_local){

	if(wakeUpTimeout_sec_local > MAX_ALLOWED_TIMER_DURATION_SEC){ //max timer duration 42949.67sec
		Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC*1000);
		wakeUpTimeout_sec_global = wakeUpTimeout_sec_local - MAX_ALLOWED_TIMER_DURATION_SEC;
	}else{
		//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
		float randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
		Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_local*1000 - 1000 + randDelay_msec);
		wakeUpTimeout_sec_global = 0; //reset this so that when the device wakes up, it knows that there is no need to restart timer but it is the actual time to wake up the device
	}

}


/*********************************************************************
 * @fn      CLIMB_FlashLed
 *
 * @brief   Turn on a led and start the timer that switch it off;
 *
 * @param   pinId - the led id to turn on

 * @return  none
 */
static void CLIMB_FlashLed(PIN_Id pinId){

	PIN_setOutputValue(hGpioPin, pinId, Board_LED_ON);
	Util_startClock(&ledTimeoutClock); //start the clock that switch the led off

}
/*********************************************************************
 * @fn      CLIMB_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                HAL_KEY_SW_1
 *
 * @return  none
 */
static void CLIMB_handleKeys(uint8 keys) {

	switch ((keys_Notifications_t) keys) {
	case LEFT_SHORT:
		adv_counter = 0;
		break;

	case RIGHT_SHORT:
		adv_counter = 128;
		break;

	case LEFT_LONG:

		break;

	case RIGHT_LONG:
		if (beaconActive != 1){
			startNode();

			Climb_setWakeUpClock(WAKEUP_DEFAULT_TIMEOUT_SEC);
			Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000);

		}else{ //if manually switched off, no automatic wakeup is setted
			stopNode();

			destroyChildNodeList();

			if(nodeState != ON_BOARD){
				destroyMasterNodeList();

				uint8 i;
				for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del MY_MASTER
					myMasterId[i] = 0;
				}

				Climb_updateMyBroadcastedState(BY_MYSELF);
			}else{
				Climb_updateMyBroadcastedState(nodeState);
			}

			Util_stopClock(&wakeUpClock);
			Util_stopClock(&goToSleepClock);
		}
		break;

	case BOTH:
		CLIMB_FlashLed(Board_LED1);
		Climb_setAsConnectable();
		break;

	default:
		break;
	}
}
/*********************************************************************
 * @fn      startNode
 *
 * @brief   Function to call to start the node.
 *

 * @return  none
 */
static void startNode() {

	if (beaconActive != 1) {
		//readyToGoSleeping = 0;

		uint8 adv_active = 1;
		uint8 status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active);
		HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);
		Util_startClock(&periodicClock);
		CLIMB_FlashLed(Board_LED2);
		GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);

		beaconActive = 1;
#ifdef WORKAROUND
		Util_startClock(&epochClock);
#endif
	}

}

/*********************************************************************
 * @fn      startNode
 *
 * @brief   Function to call to stop the node.
 *

 * @return  none
 */
static void stopNode() {
	beaconActive = 0;
	GAPObserverRole_CancelDiscovery();
	scanning = FALSE;
	uint8 adv_active = 0;
	uint8 status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active);

	Util_stopClock(&periodicClock);


	CLIMB_FlashLed(Board_LED1);

#ifdef WORKAROUND
	Util_stopClock(&epochClock);
#endif
}
/*********************************************************************
 * @fn      destroyChildNodeList
 *
 * @brief   Destroy child list
 *

 * @return  none
 */
static void destroyChildNodeList() {

	uint16 arraySize = MAX_SUPPORTED_CHILD_NODES*sizeof(myGapDevRec_t);
	uint16 i = 0;
	//resetta la memoria
	for(i = 0; i < arraySize; i++ ){
		((uint8*)childListArray)[i] = 0;
	}
}
/*********************************************************************
 * @fn      destroyMasterNodeList
 *
 * @brief   Destroy master list
 *

 * @return  none
 */
static void destroyMasterNodeList() {

	uint16 arraySize = MAX_SUPPORTED_MASTER_NODES*sizeof(myGapDevRec_t);
	uint16 i = 0;
	//resetta la memoria
	for(i = 0; i < arraySize; i++ ){
		((uint8*)masterListArray)[i] = 0;
	}

}

/*********************************************************************
 * @fn      Climb_setAsConnectable
 *
 * @brief
 *

 * @return  none
 */
static void Climb_setAsConnectable(){
	uint8 adv_active;
	if(beaconActive){
		adv_active = 0;
		GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active);
	}
	Util_startClock(&connectableTimeoutClock);
	adv_active = 1;
	GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active);
}

/*********************************************************************
 * @fn      Climb_setAsNonConnectable
 *
 * @brief
 *

 * @return  none
 */
static void Climb_setAsNonConnectable(){
	uint8 adv_active = 0;
	uint8 status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active);
	Util_stopClock(&connectableTimeoutClock);
	if(beaconActive){
		adv_active = 1;
		status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active);
	}

}

/*!*****************************************************************************
 *  @fn         Key_callback
 *
 *  Interrupt service routine for buttons, relay and MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
//static void Key_callback(PIN_Handle handle, PIN_Id pinId)
//{
//
//  SimpleBLEPeripheral_enqueueMsg(KEY_CHANGE_EVT, pinId, NULL);
//
//}

#ifdef FEATURE_LCD
/*********************************************************************
 * @fn      displayInit
 *
 * @brief
 *
 * @param
 *
 * @return  none
 */
static void displayInit(void){

  // Initialize LCD
  devpkLcdOpen();
}
#endif


/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,uint8_t *pData) {
	sbpEvt_t *pMsg;

	// Create dynamic pointer to message.
	if (pMsg = ICall_malloc(sizeof(sbpEvt_t))) {
		pMsg->hdr.event = event;
		pMsg->hdr.state = state;
		pMsg->pData = pData;

		// Enqueue the message.
		return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *) pMsg);
	}

	return FALSE;
}

/*********************************************************************
 * @fn      Climb_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void Climb_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      memcomp
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 *
 * @return  none
 */

static uint8 memcomp(uint8 * str1, uint8 * str2, uint8 len) { //come memcmp (ma ritorna 0 se è uguale e 1 se è diversa, non dice se minore o maggiore)
	uint8 index;
	for (index = 0; index < len; index++) {
		if (str1[index] != str2[index]) {
			return 1;
		}
	}
	return 0;
}

static uint8 isNonZero(uint8 * str1, uint8 len) { //ritorna 0 se il vettore è zero, 1 altrimenti
	uint8 index;
	for (index = 0; index < len; index++) {
		if (str1[index] != 0) {
			return 1;
		}
	}
	return 0;
}


void exceptionHandler(struct exceptionFrame *e, unsigned int execlR){

	timerSNVDataStore_t timerConfToStore;


	//timerConfToStore.wakeUpRemainingTimeout = Util_getClockTimeout(&wakeUpClock);
	timerConfToStore.wakeUpClock_Struct = wakeUpClock;
	timerConfToStore.wakeUpTimerActive = Util_isActive(&wakeUpClock);
	//timerConfToStore.goToSleepRamainingTimeout = Util_getClockTimeout(&goToSleepClock);
	timerConfToStore.goToSleepClock_Struct = goToSleepClock;
	timerConfToStore.goToSleepTimerActive = Util_isActive(&goToSleepClock);
	timerConfToStore.wakeUpTimeout_sec_global_value = wakeUpTimeout_sec_global;
	timerConfToStore.validData = TRUE;

	//todo: verifica che la scrittura sia stata fatta realmente e non sia bufferizata
	osal_snv_write(SNV_BASE_ID+TIMERS_SNV_OFFSET, sizeof(timerSNVDataStore_t), &timerConfToStore);


	while(1){
		HAL_SYSTEM_RESET();
	}
}

static uint8 restoreTimersConf(){

	timerSNVDataStore_t timerConfToRestore;

	uint8 ret = osal_snv_read(SNV_BASE_ID + TIMERS_SNV_OFFSET, sizeof(timerSNVDataStore_t), &timerConfToRestore);

	if (ret == SUCCESS) {

		if (timerConfToRestore.validData) {
			Util_stopClock(&wakeUpClock);
			Util_stopClock(&goToSleepClock);

			//Reload wakeUp timer value
			wakeUpTimeout_sec_global = timerConfToRestore.wakeUpTimeout_sec_global_value;
			wakeUpClock = timerConfToRestore.wakeUpClock_Struct;
			Util_startClock(&wakeUpClock);
			if (!timerConfToRestore.wakeUpTimerActive) { //if the clock was inactive stop it immediatelly
				Util_stopClock(&wakeUpClock);
			}

			//Reload goToSleep timer value
			goToSleepClock = timerConfToRestore.goToSleepClock_Struct;
			Util_startClock(&goToSleepClock);
			if (!timerConfToRestore.goToSleepTimerActive) {
				Util_stopClock(&goToSleepClock);
			}

			//unvalidate data so that when no fauls occour it doesn't load an old timers configuration
			timerConfToRestore.validData = FALSE;
			osal_snv_write(SNV_BASE_ID + TIMERS_SNV_OFFSET, sizeof(timerSNVDataStore_t), &timerConfToRestore);

			return TRUE;
		}
	}
	return FALSE;

}
/*********************************************************************
*********************************************************************/
