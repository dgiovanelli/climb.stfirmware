/******************************************************************************
 * @file  simple_ble_topology.c
 *
 * @description Application task for the Simple Topology example
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright   
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

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
//#include "DevInfoservice.h"
#include "CLIMBProfile.h"

#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "linkdb.h"
#include "CLIMBPerOb.h"
#include "Keys_Task.h"

#include "battservice.h"

#include "bsp_I2c.h"
#include "bsp_spi.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>

#include "sensor_bmp280.h"
#include "sensor_hdc1000.h"
#include "sensor_mpu9250.h"
#include "sensor_opt3001.h"
#include "sensor_tmp007.h"

#include <xdc/runtime/Types.h>
#include <ti/sysbios/BIOS.h>

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#ifdef HIGH_PERFORMANCE
#define DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL          240
#define DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL          240
#else
#define DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL          1600
#define DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL          1000
#endif

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL   	 162

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     176

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          150

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         4

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 2000//1300//250//1000

#define PRE_ADV_TIMEOUT				  (DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL*625)/1000-5
#define PRE_CE_TIMEOUT				  (DEFAULT_DESIRED_MIN_CONN_INTERVAL*1250)/1000-5

// Scan interval value in 0.625ms ticks
#define SCAN_INTERVAL 						  80

// scan window value in 0.625ms ticks
#define SCAN_WINDOW							  80

// Whether to report all contacts or only the first for each device
#define FILTER_ADV_REPORTS					  FALSE

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL //non è ancora chiaro cosa cambi, con le altre due opzioni non vede

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// How often to perform periodic event (in msec)
#define PERIODIC_EVT_PERIOD              	  2000

#define NODE_TIMEOUT_OS_TICKS				  500000

#define LED_TIMEOUT						  	  5
#define RESET_BROADCAST_CMD_TIMEOUT			  7000

#warning CHECK TIMEOUTS
#define WAKEUP_DEFAULT_TIMEOUT_SEC				60*60*24
#define GOTOSLEEP_DEFAULT_TIMEOUT_SEC			1*60*60
#define GOTOSLEEP_POSTPONE_INTERVAL_SEC			5*60

#define MAX_ALLOWED_TIMER_DURATION_SEC	      42000 //actual max timer duration 42949.67sec
//#define NODE_ID								  0x01 //per ora non è applicabile ai nodi master

#define CHILD_NODE_ID_LENGTH				  1
#define MASTER_NODE_ID_LENGTH				  6

#define MAX_SUPPORTED_CHILD_NODES			  60
#define MAX_SUPPORTED_MASTER_NODES			  30
#if MAX_SUPPORTED_CHILD_NODES < 90
#warning MAX_SUPPORTED_CHILD_NODES is low because of debugging pourposes, it can be set to 90
#endif
// Maximum number of scan responses to be reported to application
#define DEFAULT_MAX_SCAN_RES                  MAX_SUPPORTED_CHILD_NODES + MAX_SUPPORTED_MASTER_NODES
//NB: 0 = unlimited
#define ADV_PKT_ID_OFFSET					  12
#define ADV_PKT_STATE_OFFSET				  ADV_PKT_ID_OFFSET + CHILD_NODE_ID_LENGTH

#define BROADCAST_MSG_TYPE_STATE_UPDATE_CMD	  0x01
#define BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD 0x02

#define BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD	  0x02
#define BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD  0x04

#define BROADCAST_RESET_CMD_FLAG_CMD		  0xFF
#define BROADCAST_LENGTH_RESET_CMD_FLAG_CMD   0x01

#define	DEFAULT_MTU_LENGTH						23

#define MIN_BATT_MEAS_INTERVAL_TICKS		  3000000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD


#define SNV_BASE_ID							  0x80
#define TIMERS_SNV_OFFSET					  0x02
// Task configuration
#define SBP_TASK_PRIORITY                     1

//#ifndef SBP_TASK_STACK_SIZE
//#define SBP_TASK_STACK_SIZE                   644
//#endif

// Internal Events for RTOS application
//#define SBT_STATE_CHANGE_EVT                  0x0001
//#define SBT_CHAR_CHANGE_EVT                   0x0002
//#define CONN_EVT_END_EVT                  0x0004
//#define SBT_START_DISCOVERY_EVT               0x0008
//#define SBT_KEY_CHANGE_EVT                    0x0010
// Internal Events for RTOS application
#define SBT_STATE_CHANGE_EVT                  0x0001
#define SBT_CHAR_CHANGE_EVT                   0x0002
#define PERIODIC_EVT                        0x0004
#define CONN_EVT_END_EVT                    0x0008
#define O_STATE_CHANGE_EVT                  0x0010
#define ADVERTISE_EVT					    0x0020
#define KEY_CHANGE_EVT					  	0x0040
#define LED_TIMEOUT_EVT						0x0080
#define RESET_BROADCAST_CMD_EVT				0x0100
#define EPOCH_EVT							0x0400
#define PRE_ADV_EVT							0x1000
#define WATCHDOG_EVT						0x2000
#define WAKEUP_TIMEOUT_EVT					0x4000
#define GOTOSLEEP_TIMEOUT_EVT				0x8000
#define PRE_CE_EVT							0x0800

// Discovery states
enum {
	BLE_DISC_STATE_IDLE,                // Idle
	BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
	BLE_DISC_STATE_SVC,                 // Service discovery
	BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

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

typedef enum ClimbNodeType_t {
	CLIMB_CHILD_NODE = 0, CLIMB_MASTER_NODE, NOT_CLIMB_NODE, NAME_NOT_PRESENT, WRONG_PARKET_TYPE
} ClimbNodeType_t;

typedef struct {
	uint8 id[6];
	ChildClimbNodeStateType_t state;
	uint32 lastContactTicks;
	uint8 rssi;
	uint8 contactSentThoughGATT;
	ChildClimbNodeStateType_t stateToImpose;
} myGapDevRec_t;

typedef struct listNode {
	myGapDevRec_t device;
	struct listNode *next;
} listNode_t;

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
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct {
	uint16_t event;  // event type
	uint8_t status; // event status
	uint8_t *pData; // event data pointer
} sbtEvt_t;

/*********************************************************************
 * LOCAL VARIABLES
 */
// structure to store link attributes 
//extern gapRoleInfo_t multiConnInfo[MAX_NUM_BLE_CONNS];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration
//Task_Struct sbmTask;
//Char sbmTaskStack[SBT_TASK_STACK_SIZE];

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CLIMB Node";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Connection handle of current connection 
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct ledTimeoutClock;
static Clock_Struct resetBroadcastCmdClock;
static Clock_Struct preAdvClock;
static Clock_Struct preCEClock;
static Clock_Struct wakeUpClock;
static Clock_Struct goToSleepClock;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 defAdvertData[] = { 0x07,	// length of this data
								GAP_ADTYPE_LOCAL_NAME_COMPLETE,
								'C', 'L', 'I', 'M', 'B', 'M',
								0x02,   // length of this data
								GAP_ADTYPE_FLAGS,
								GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED// | DEFAULT_DISCOVERABLE_MODE, //

};
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 defScanRspData[] = { 0x03,
GAP_ADTYPE_APPEARANCE, (uint8) (GAP_APPEARE_GENERIC_TAG & 0xFF), (uint8) ((GAP_APPEARE_GENERIC_TAG >> 8) & 0xFF) };

// Global pin resources
static PIN_Handle hGpioPin;
static PIN_State pinGpioState;

static PIN_Config SensortagAppPinTable[] = {
Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off             */
Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off             */
//Board_KEY_LEFT   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
//Board_KEY_RIGHT  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
//Board_RELAY      | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,      /* Relay is active high          */
		Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Buzzer initially off          */
		//Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

		PIN_TERMINATE };

static uint8 Climb_childNodeName[] = { 'C', 'L', 'I', 'M', 'B', 'C' };

static uint8 advUpdateReq = FALSE;

static uint8 isBroadcastMessageValid = FALSE;
static uint8 broadcastMessage[10];

static uint8 nodeTurnedOn = FALSE;
static uint8 mtu_size = DEFAULT_MTU_LENGTH;
static uint8 connectionConfigured = FALSE;

static uint8 myAddr[B_ADDR_LEN];

static uint8 adv_counter = 0;

static uint8 broadcastID[] = { 0xFF };
static uint8 zeroID[] = { 0x00 };

//static listNode_t* childListRootPtr = NULL;
//static listNode_t* masterListRootPtr = NULL;
static uint8 adv_startNodeIndex = 0;
static uint8 gatt_startNodeIndex = 0;

static myGapDevRec_t* childListArray = NULL;
static myGapDevRec_t* masterListArray = NULL;

#ifdef WATCHDOGTIMER_EN
GPTimerCC26XX_Handle hTimer;
static uint8 unclearedWatchdogEvents = 0;
#endif

static uint16 connHanlde = 0;

static uint32 wakeUpTimeout_sec_global = 0;

static uint8 onBoardChildren = 0; //not to be used in critical context, updated on Climb_contactsCheckSendThroughGATT

static uint8 devicesHeardDuringLastScan = 0;

static uint32 lastBattMeas = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleTopology_init(void);
static void nodeListInit(void);
//void simpleTopology_taskFxn(uint32* a0, uint32* a1);
static uint8_t simpleTopology_processStackMsg(ICall_Hdr *pMsg);
static uint8_t simpleTopology_processGATTMsg(gattMsgEvent_t *pMsg);
static void simpleTopology_processAppMsg(sbtEvt_t *pMsg);
static void Climb_processCharValueChangeEvt(uint8_t paramID);
static void simpleTopology_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void simpleTopology_freeAttRsp(uint8_t status);
static void simpleTopology_charValueChangeCB(uint8_t paramID);
static void Keys_EventCB(keys_Notifications_t notificationType);
static uint8_t simpleTopology_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData);
static uint8_t simpleTopology_eventCB(gapMultiRoleEvent_t *pEvent);
static void BLE_ConnectionEventHandler_sendAttRsp(void);
static void simpleTopology_freeAttRsp(uint8_t status);

//static void updateConnParam_CB(uint16_t connInterval, uint16_t connSlaveLatency, uint16_t connTimeout);
static void Climb_clockHandler(UArg arg);
//void simpleTopology_startDiscHandler(UArg a0);
//void simpleTopology_keyChangeHandler(uint8 keysPressed);

////CLIMB MANAGEMENT
static ClimbNodeType_t isClimbNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static void Climb_addNodeDeviceInfo(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
//static listNode_t* Climb_findNodeByDevice(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
static uint8 Climb_findNodeById(uint8 *nodeID, ClimbNodeType_t nodeType);
static uint8 Climb_addNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
static void Climb_updateNodeMetadata(gapDeviceInfoEvent_t *gapDeviceInfoEvent, uint8 index, ClimbNodeType_t nodeType);
static void Climb_advertisedStatesCheck(void);
//static uint8 Climb_isMyChild(uint8 nodeID);
static void Climb_nodeTimeoutCheck();
static void Climb_removeNode(uint8 indexToRemove, ClimbNodeType_t nodeType);
static void Climb_periodicTask();
#ifdef PRINTF_ENABLED
#ifdef PRINT_NODE_INFO_ENABLED
static void Climb_printfNodeInfo(gapDeviceInfoEvent_t *gapDeviceInfoEvent );
#endif
#endif

static void Climb_preAdvEvtHandler();
static void Climb_preCEEvtHandler();
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
//static void Key_callback(PIN_Handle handle, PIN_Id pinId);
#ifdef FEATURE_LCD
static void displayInit(void);
#endif
#ifdef WATCHDOGTIMER_EN
static void watchdogTimerInit();
static void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
#endif
////CLIMB IN/OUT FUNCTIONS
static void Climb_contactsCheckSendThroughGATT(void);
static void Climb_advertisedStatesUpdate(void);

static uint8 memcomp(uint8 * str1, uint8 * str2, uint8 len);
static uint8 isNonZero(uint8 * str1, uint8 len);
#ifdef HEAPMGR_METRICS
static void plotHeapMetrics();
#endif
#ifdef STACK_METRICS
static void checkStackMetrics();
#endif

static void BLE_AdvertiseEventHandler(void);

static uint8 restoreTimersConf(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleTopology_gapRoleCBs = { simpleTopology_eventCB,        // events to be handled by the app are passed through the GAP Role here
		};

// Simple GATT Profile Callbacks
static climbProfileCBs_t simpleTopology_simpleProfileCBs = { simpleTopology_charValueChangeCB // Characteristic value change callback
		};

// Keys Callbacks
static keysEventCBs_t Keys_EventCBs = { Keys_EventCB, // Profile State Change Callbacks
		};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs = {
NULL, // Passcode callback (not used by application)
		NULL  // Pairing / Bonding state Callback (not used by application)
		};

//static gapRolesParamUpdateCB_t gapRoleUpdateConnParam_CB = updateConnParam_CB;
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simpleTopology_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
//void SimpleTopology_createTask(void)
//{
//  Task_Params taskParams;
//
//  // Configure task
//  Task_Params_init(&taskParams);
//  taskParams.stack = sbmTaskStack;
//  taskParams.stackSize = SBT_TASK_STACK_SIZE;
//  taskParams.priority = SBT_TASK_PRIORITY;
//
//  Task_construct(&sbmTask, simpleTopology_taskFxn, &taskParams, NULL);
//}
/*********************************************************************
 * @fn      simpleTopology_init
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
static void simpleTopology_init(void) {
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

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueue = Util_constructQueue(&appMsg);

	// Create one-shot clocks for internal periodic events.
	Util_constructClock(&periodicClock, Climb_clockHandler,
	PERIODIC_EVT_PERIOD, 0, false, PERIODIC_EVT);

	Util_constructClock(&ledTimeoutClock, Climb_clockHandler,
	LED_TIMEOUT, 0, false, LED_TIMEOUT_EVT);

	Util_constructClock(&resetBroadcastCmdClock, Climb_clockHandler,
	RESET_BROADCAST_CMD_TIMEOUT, 0, false, RESET_BROADCAST_CMD_EVT);

	Util_constructClock(&preAdvClock, Climb_clockHandler,
	PRE_ADV_TIMEOUT, 0, false, PRE_ADV_EVT);

	Util_constructClock(&preCEClock, Climb_clockHandler,
	PRE_CE_TIMEOUT, 0, false, PRE_CE_EVT);

	Util_constructClock(&wakeUpClock, Climb_clockHandler,
	WAKEUP_DEFAULT_TIMEOUT_SEC * 1000, 0, false, WAKEUP_TIMEOUT_EVT);

	Util_constructClock(&goToSleepClock, Climb_clockHandler,
	GOTOSLEEP_DEFAULT_TIMEOUT_SEC * 1000, 0, false, GOTOSLEEP_TIMEOUT_EVT);

//#warning REMOVE THE ADDRESS UPDATE COMMAND BELOW
//	uint8 tempAddr[] = {
//			0x83,
//			0xcd,
//			0xb6,
//			0xf8,
//			0xe6,
//			0xa0,
//	};
//	HCI_EXT_SetBDADDRCmd(tempAddr);
//	GAP_ConfigDeviceAddr(ADDRTYPE_STATIC, tempAddr);


#if SENSORTAG_HW
	// Setup SPI bus for serial flash and Devpack interface
	bspSpiOpen();
#endif //SENSORTAG_HW

//	// Setup discovery delay as a one-shot timer
//	Util_constructClock(&startDiscClock, simpleTopology_startDiscHandler,
//	DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

	//init keys and LCD
//  Board_initKeys(simpleTopology_keyChangeHandler);
//  Board_openLCD();
	uint32_t uint32_arg;
	// Setup the GAP
	{

//		uint32_arg = GAP_PROFILE_PERIPHERAL | GAP_PROFILE_OBSERVER;
//		GAPRole_SetParameter(GAPROLE_PROFILEROLE, sizeof(uint8_t), &uint32_arg,
//		NULL);

		/*-------------------PERIPHERAL-------------------*/
		// Setup the GAP Peripheral Role Profile
		// Setup the GAP
		GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

		// For all hardware platforms, device starts advertising upon initialization
		uint32_arg = FALSE;
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &uint32_arg, NULL);

		// Set the GAP Role Parameters
		// By setting this to zero, the device will go into the waiting state after
		// being discoverable for 30.72 second, and will not being advertising again
		// until the enabler is set back to TRUE
		uint32_arg = 0;
		GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_ENABLE_UPDATE_REQUEST;
		GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
		GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
		GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_SLAVE_LATENCY;
		GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_CONN_TIMEOUT;
		GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t), &uint32_arg, NULL);

		//set up advertise and scan response data
		GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(defScanRspData), defScanRspData, NULL);
		GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(defAdvertData), defAdvertData, NULL);

		uint32_arg = DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL;

		GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, uint32_arg);
		GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, uint32_arg);

		uint32_arg = DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL;

		GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, uint32_arg);
		GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, uint32_arg);

		GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, uint32_arg);
		GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, uint32_arg);

		/*-------------------CENTRAL-------------------*/
//    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
//    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
//    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
//    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
//    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
//    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
//    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
//    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
//    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
//    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
//    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);
//    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_CONN_INT);
//    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
//    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
//    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);
		uint32_arg = DEFAULT_MAX_SCAN_RES;
		GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t), &uint32_arg, NULL);

		GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);

		GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, SCAN_INTERVAL);
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, SCAN_INTERVAL);
		GAP_SetParamValue(TGAP_CONN_SCAN_INT, SCAN_INTERVAL);
		GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, SCAN_WINDOW);
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, SCAN_WINDOW);
		GAP_SetParamValue(TGAP_CONN_SCAN_WIND, SCAN_WINDOW);
	}

	// Setup the GAP Role Profile
	{
		/*--------PERIPHERAL-------------*/
		// For all hardware platforms, device starts advertising upon initialization
//    uint8_t initialAdvertEnable = TRUE;
//    // By setting this to zero, the device will go into the waiting state after
//    // being discoverable for 30.72 second, and will not being advertising again
//    // until the enabler is set back to TRUE
//    uint16_t advertOffTime = 0;
//    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
//    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
//    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
//    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
//
//    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
//                         &initialAdvertEnable, NULL);
//    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
//                         &advertOffTime, NULL);
//    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
//                         scanRspData, NULL);
//    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(defAdvertData), defAdvertData, NULL);
//    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
//                         &desiredMinInterval, NULL);
//    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
//                         &desiredMaxInterval, NULL);
//    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
//                         &desiredSlaveLatency, NULL);
//    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
//                         &desiredConnTimeout, NULL);
		/*--------------CENTRAL-----------------*/
//    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
//    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
//                                &scanRes, NULL);
		// Register with GAP for HCI/Host messages
		GAP_RegisterForMsgs(selfEntity);
	}

	//GATT
	{
		/*---------------------SERVER------------------------*/
		// Set the GAP Characteristics
		GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

		// Initialize GATT Server Services
		GGS_AddService(GATT_ALL_SERVICES);           // GAP
		GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
		//DevInfo_AddService();                        // Device Information Service
		ClimbProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

		// Add battery service.
		Batt_AddService();

		// Setup Profile Characteristic Values
//    {
//      uint8_t charValue1 = 1;
//      uint8_t charValue2 = 2;
//      uint8_t charValue3 = 3;
//      uint8_t charValue4 = 4;
//      uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
//
//      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
//                                 &charValue1);
//      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
//                                 &charValue2);
//      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
//                                 &charValue3);
//      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
//                                 &charValue4);
//      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
//                                 charValue5);
//    }

		// Register callback with SimpleGATTprofile
		ClimbProfile_RegisterAppCBs(&simpleTopology_simpleProfileCBs);

		/*-----------------CLIENT------------------*/
		// Initialize GATT Client
		//VOID GATT_InitClient();
		// Register for GATT local events and ATT Responses pending for transmission
		GATT_RegisterForMsgs(selfEntity);

		// Register to receive incoming ATT Indications/Notifications
		//GATT_RegisterForInd(selfEntity);

	}
	// Setup the GAP Bond Manager
	{
		uint32_arg = 0; // passkey "000000"
		GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), &uint32_arg);

		uint32_arg = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ; //GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
		GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &uint32_arg);

		uint32_arg = FALSE;
		GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &uint32_arg);

		uint32_arg = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
		GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &uint32_arg);

		uint32_arg = TRUE;
		GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &uint32_arg);
	}

	//sensorTestExecute(ST_TEST_MAP);

	sensorBmp280Init();
	sensorBmp280Enable(FALSE);
	sensorHdc1000Init();
	//PIN_setOutputValue(hGpioPin, Board_MPU_POWER, 0);
	//sensorMpu9250Init(); //ho gia scollegato l'alimentazione all'interno del file Board.c
	//sensorMpu9250Reset();
	sensorOpt3001Init();
	sensorTmp007Init();

	// Start Bond Manager
	VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

	//SETTING POWER
	HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);

#ifdef WATCHDOGTIMER_EN
	watchdogTimerInit();
#endif
	nodeListInit();

	// Start the Device
	VOID GAPRole_StartDevice(&simpleTopology_gapRoleCBs);

	if ( restoreTimersConf() ) { // the flash contained a valid timers configuration
		if (Util_isActive(&goToSleepClock)) { //if this timer is active it means that this boot follows a fault, and before the fault the node was active, then start it.
			startNode(); //start the node
		} else {
			//do nothing
		}
	} else {  // the flash did not contain a valid configuration. Proceed with normal boot
//automatically start-up the node
#warning the node is configured to start up automatically upon power up
		events |= WAKEUP_TIMEOUT_EVT;
		Semaphore_post(sem);
	}

}

static void nodeListInit(void) {

	//init children list
	uint16 arraySize = MAX_SUPPORTED_CHILD_NODES * sizeof(myGapDevRec_t);
	childListArray = (myGapDevRec_t*) ICall_malloc(arraySize);
	memset(childListArray, 0, arraySize); //reset the memory

	//init masters list
	arraySize = MAX_SUPPORTED_MASTER_NODES * sizeof(myGapDevRec_t);
	masterListArray = (myGapDevRec_t*) ICall_malloc(arraySize);
	memset(masterListArray, 0, arraySize); //reset the memory


	//resetta la memoria
//	uint16 i = 0;
//	for (i = 0; i < arraySize; i++) {
//		((uint8*) childListArray)[i] = 0;
//	}
	//resetta la memoria
//	for (i = 0; i < arraySize; i++) {
//		((uint8*) masterListArray)[i] = 0;
//	}

}
/*********************************************************************
 * @fn      simpleTopology_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Multi.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
void simpleTopology_taskFxn(uint32* a0, uint32* a1) {
	// Initialize application
	simpleTopology_init();

	// Application main loop
	for (;;) {
		// Waits for a signal to the semaphore associated with the calling thread.
		// Note that the semaphore associated with a thread is signaled when a
		// message is queued to the message receive queue of the thread or when
		// ICall_signal() function is called onto the semaphore.
		ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

		if (errno == ICALL_ERRNO_SUCCESS) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			if (ICall_fetchServiceMsg(&src, &dest, (void **) &pMsg) == ICALL_ERRNO_SUCCESS) {
				uint8 safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Event *pEvt = (ICall_Event *) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature == 0xffff) {
						if (pEvt->event_flag & CONN_EVT_END_EVT) {
							// Try to retransmit pending ATT Response (if any)
							BLE_ConnectionEventHandler_sendAttRsp();
						}

						if (pEvt->event_flag & ADVERTISE_EVT) {
							BLE_AdvertiseEventHandler();
						}
					} else {
						// Process inter-task message
						safeToDealloc = simpleTopology_processStackMsg((ICall_Hdr *) pMsg);
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}

			// If RTOS queue is not empty, process app message.
			while (!Queue_empty(appMsgQueue)) {
				sbtEvt_t *pMsg = (sbtEvt_t *) Util_dequeueMsg(appMsgQueue);
				if (pMsg) {
					// Process message.
					simpleTopology_processAppMsg(pMsg);

					// Free the space from the message.
					ICall_free(pMsg);
				}
			}
		}
		if (events & PERIODIC_EVT) {
			events &= ~PERIODIC_EVT;
			if (nodeTurnedOn == TRUE) {
				Util_startClock(&periodicClock);
			}
			// Perform periodic application task
			if (connectionConfigured == TRUE) {
				Climb_periodicTask();
			}
		}

		if (events & LED_TIMEOUT_EVT) {

			events &= ~LED_TIMEOUT_EVT;
			//only turn off leds
			PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_OFF);
			PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
		}
		if (events & PRE_ADV_EVT) {
			events &= ~PRE_ADV_EVT;

			Climb_preAdvEvtHandler();

		}
		if (events & PRE_CE_EVT) {
			events &= ~PRE_CE_EVT;

			Climb_preCEEvtHandler();

		}
		if (events & RESET_BROADCAST_CMD_EVT) {

			events &= ~RESET_BROADCAST_CMD_EVT;

			isBroadcastMessageValid = FALSE;
		}
#ifdef WATCHDOGTIMER_EN
		if (events & WATCHDOG_EVT) {
			events &= ~WATCHDOG_EVT;

			unclearedWatchdogEvents = 0;

		}
#endif
		if (events & GOTOSLEEP_TIMEOUT_EVT) {
			events &= ~GOTOSLEEP_TIMEOUT_EVT;

			Climb_goToSleepHandler();
		}

		if (events & WAKEUP_TIMEOUT_EVT) {
			events &= ~WAKEUP_TIMEOUT_EVT;

			Climb_wakeUpHandler();
		}

	}
}

/*********************************************************************
 * @fn      simpleTopology_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t simpleTopology_processStackMsg(ICall_Hdr *pMsg) {
	uint8_t safeToDealloc = TRUE;

	switch (pMsg->event) {
	case GATT_MSG_EVENT:
		// Process GATT message
		safeToDealloc = simpleTopology_processGATTMsg((gattMsgEvent_t *) pMsg);
		break;

	case HCI_GAP_EVENT_EVENT: {
		// Process HCI message
		switch (pMsg->status) {
		case HCI_COMMAND_COMPLETE_EVENT_CODE:
			// Process HCI Command Complete Event
			break;

		default:
			break;
		}
	}
		break;

	case GAP_MSG_EVENT:
		simpleTopology_processRoleEvent((gapMultiRoleEvent_t *) pMsg);
		break;

	default:
		// do nothing
		break;
	}

	return (safeToDealloc);
}

/*********************************************************************
 * @fn      simpleTopology_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t simpleTopology_processGATTMsg(gattMsgEvent_t *pMsg) {
	// See if GATT server was unable to transmit an ATT response
	if (pMsg->hdr.status == blePending) {
		// No HCI buffer was available. Let's try to retransmit the response
		// on the next connection event.
		if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
		CONN_EVT_END_EVT) == SUCCESS) {
			// First free any pending response
			simpleTopology_freeAttRsp(FAILURE);

			// Hold on to the response message for retransmission
			pAttRsp = pMsg;

			// Don't free the response message yet
			return (FALSE);
		}
	} else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
		// ATT request-response or indication-confirmation flow control is
		// violated. All subsequent ATT requests or indications will be dropped.
		// The app is informed in case it wants to drop the connection.

		// Display the opcode of the message that caused the violation.
		//LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode, 10, LCD_PAGE6);
	} else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
		mtu_size = pMsg->msg.mtuEvt.MTU;
		//PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_ON);
		// MTU size updated
		//LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE6);
	}

	//messages from GATT server
	if ((gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) > 0)) {
		if ((pMsg->method == ATT_READ_RSP) || ((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ))) {
			if (pMsg->method == ATT_ERROR_RSP) {
				//LCD_WRITE_STRING_VALUE("Read Error", pMsg->msg.errorRsp.errCode, 10,  LCD_PAGE6);
			} else {
				// After a successful read, display the read value
				//LCD_WRITE_STRING_VALUE("Read rsp:", pMsg->msg.readRsp.pValue[0], 10, LCD_PAGE6);
			}

		} else if ((pMsg->method == ATT_WRITE_RSP) || ((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ))) {

			if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP) {
				//LCD_WRITE_STRING_VALUE("Write Error", pMsg->msg.errorRsp.errCode, 10, LCD_PAGE6);
			} else {
				// After a succesful write, display the value that was written and
				// increment value
				//LCD_WRITE_STRING_VALUE("Write sent:", charVal++, 10, LCD_PAGE6);
			}
		} else if (discState != BLE_DISC_STATE_IDLE) {
			//simpleTopology_processGATTDiscEvent(pMsg);
		}
	} // else - in case a GATT message came after a connection has dropped, ignore it.

	// Free message payload. Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);

	// It's safe to free the incoming message
	return (TRUE);
}

/*********************************************************************
 * @fn      BLE_ConnectionEventHandler_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void BLE_ConnectionEventHandler_sendAttRsp(void) {


	Util_startClock(&preCEClock);

	// See if there's a pending ATT Response to be transmitted
	if (pAttRsp != NULL) {
		uint8_t status;

		// Increment retransmission count
		rspTxRetry++;

		// Try to retransmit ATT response till either we're successful or
		// the ATT Client times out (after 30s) and drops the connection.
		status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
		if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL)) {
			// Disable connection event end notice
			//HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

			// We're done with the response message
			simpleTopology_freeAttRsp(status);
		} else {
			// Continue retrying
			//LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE6);
		}
	}
}

/*********************************************************************
 * @fn      simpleTopology_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void simpleTopology_freeAttRsp(uint8_t status) {
	// See if there's a pending ATT response message
	if (pAttRsp != NULL) {
		// See if the response was sent out successfully
		if (status == SUCCESS) {
			//LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE6);
		} else {
			// Free response payload
			GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

			//LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE6);
		}

		// Free response message
		ICall_freeMsg(pAttRsp);

		// Reset our globals
		pAttRsp = NULL;
		rspTxRetry = 0;
	}
}

/*********************************************************************
 * @fn      simpleTopology_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void simpleTopology_processAppMsg(sbtEvt_t *pMsg) {
	switch (pMsg->event) {
	case SBT_STATE_CHANGE_EVT:
		simpleTopology_processStackMsg((ICall_Hdr *) pMsg->pData);
		// Free the stack message, the SBT_STATE_CHANGE_EVT is the only event that carries a pointer to a valid pData, the others call to simpleTopology_enqueueMsg set pData to NULL
		ICall_freeMsg(pMsg->pData);
		break;

	case SBT_CHAR_CHANGE_EVT:
		Climb_processCharValueChangeEvt(pMsg->status);
		break;

	case KEY_CHANGE_EVT:
		CLIMB_handleKeys(pMsg->status);
		break;

	default:
		// Do nothing.
		break;
	}
}

/*********************************************************************
 * @fn      simpleTopology_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t simpleTopology_eventCB(gapMultiRoleEvent_t *pEvent) {
	// Forward the role event to the application
	if (simpleTopology_enqueueMsg(SBT_STATE_CHANGE_EVT, SUCCESS, (uint8_t *) pEvent)) {
		// App will process and free the event
		return FALSE;
	}

	// Caller should free the event
	return TRUE;
}

/*********************************************************************
 * @fn      simpleTopology_processRoleEvent
 *
 * @brief   Multi role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleTopology_processRoleEvent(gapMultiRoleEvent_t *pEvent) {
	switch (pEvent->gap.opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		//maxPduSize = pEvent->initDone.dataPktLen;

		memcpy(myAddr, pEvent->initDone.devAddr, B_ADDR_LEN); //salva l'indirizzo del nodo

#ifdef FEATURE_LCD
		char buf[10];
		sprintf(buf,"Me: ");
		devpkLcdText(buf, 1, 0);
		sprintf(buf,Util_convertBdAddr2Str(childDevList[0].devRec.addr));
		devpkLcdText(buf, 1, 5);
#endif

		//LCD_WRITE_STRING("Connected to 0", LCD_PAGE0);
		//LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->initDone.devAddr), LCD_PAGE1);
		//LCD_WRITE_STRING("Initialized", LCD_PAGE2);

		//DevInfo_SetParameter(//DevInfo_SYSTEM_ID, //DevInfo_SYSTEM_ID_LEN, pEvent->initDone.devAddr);
	}
		break;

	case GAP_MAKE_DISCOVERABLE_DONE_EVENT: {

		HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);
	}
		break;

	case GAP_END_DISCOVERABLE_DONE_EVENT: {

	}
		break;

	case GAP_DEVICE_INFO_EVENT: {
		devicesHeardDuringLastScan++;

		if (pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_SCAN_IND | //adv data event (Scannable undirected)
				pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND |
				pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_NONCONN_IND) { //adv data event (Connectable undirected)

			ClimbNodeType_t nodeType = isClimbNode((gapDeviceInfoEvent_t*) &pEvent->deviceInfo);
			if (nodeType == CLIMB_CHILD_NODE || nodeType == CLIMB_MASTER_NODE) {
				Climb_addNodeDeviceInfo(&pEvent->deviceInfo, nodeType);
				if (nodeType == CLIMB_CHILD_NODE) {
					//Climb_contactsCheckSendThroughGATT();
#ifdef PRINTF_ENABLED
#ifdef PRINT_NODE_INFO_ENABLED
					Climb_printfNodeInfo(&pEvent->deviceInfo);
#endif
#endif
				}

			} else {

			}
		} else if (pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP) { //scan response data event

		}

//		if (devicesHeardDuringLastScan >= DEFAULT_MAX_SCAN_RES) {
//			if (connectionConfigured) {
//				GAPRole_CancelDiscovery();
//				//scanningActive = FALSE;
//				//if (!scanningActive) {
//				//	uint8 status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
//				//	scanningActive = TRUE;
//				//}
//			}
//		}
		// if filtering device discovery results based on service UUID
//        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
//        {
//          if (simpleTopology_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
//                                           pEvent->deviceInfo.pEvtData,
//                                           pEvent->deviceInfo.dataLen))
//          {
//            simpleTopology_addDeviceInfo(pEvent->deviceInfo.addr,
//                                           pEvent->deviceInfo.addrType);
//          }
//        }
	}
		break;

	case GAP_DEVICE_DISCOVERY_EVENT: {
		devicesHeardDuringLastScan = 0;

		if (connectionConfigured) {
			uint8 status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
			//CLIMB_FlashLed(Board_LED2);
			PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_ON);
		}

		// discovery complete
//        scanningStarted = FALSE;
//
//        // if not filtering device discovery results based on service UUID
//        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
//        {
//          // Copy results
//          scanRes = pEvent->discCmpl.numDevs;
//          memcpy(devList, pEvent->discCmpl.pDevList,
//                 (sizeof(gapDevRec_t) * scanRes));
//        }
//
//        //LCD_WRITE_STRING_VALUE("Devices Found", scanRes, 10, LCD_PAGE3);
//
//        if (scanRes > 0)
//        {
//          //LCD_WRITE_STRING("<- To Select", LCD_PAGE4);
//        }
//
//        // initialize scan index to last device
//        scanIdx = scanRes;
	}
		break;

	case GAP_LINK_ESTABLISHED_EVENT: {
		if (pEvent->gap.hdr.status == SUCCESS) {
			//NOTE: connectable/non-connctable adv switch is automatically done in multi.c
			//store connection handle
			connHandle = pEvent->linkCmpl.connectionHandle;

			HCI_EXT_ConnEventNoticeCmd(connHanlde, selfEntity, CONN_EVT_END_EVT);


			HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);

			if ((pEvent->linkCmpl.connInterval >= DEFAULT_DESIRED_MIN_CONN_INTERVAL && pEvent->linkCmpl.connInterval <= DEFAULT_DESIRED_MAX_CONN_INTERVAL)	&& pEvent->linkCmpl.connLatency == DEFAULT_DESIRED_SLAVE_LATENCY && pEvent->linkCmpl.connTimeout == DEFAULT_DESIRED_CONN_TIMEOUT) {
				connectionConfigured = TRUE;
				GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST); //trigger the first discovery, the subsequent will be triggered by GAP_DEVICE_DISCOVERY_EVENT
				Util_rescheduleClock(&preCEClock, (pEvent->linkCmpl.connInterval * 1250) / 1000 - 10);

			}


			if(nodeTurnedOn){
				uint8 adv_active = FALSE;
				GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);
				adv_active = TRUE;
				GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
			}

		} else {
			connHandle = GAP_CONNHANDLE_INIT;
			discState = BLE_DISC_STATE_IDLE;
		}
	}
		break;

	case GAP_LINK_TERMINATED_EVENT: {
		connHandle = GAP_CONNHANDLE_INIT;

		connectionConfigured = FALSE;
		mtu_size = DEFAULT_MTU_LENGTH;
		GAPRole_CancelDiscovery();

		GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(defAdvertData), defAdvertData, NULL);

		if(nodeTurnedOn){
			uint8 adv_active = FALSE;
			GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
			adv_active = TRUE;
			GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);
		}else{
			uint8 adv_active = FALSE;
			GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);
			GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
		}
		// Disable connection event end notice
		HCI_EXT_ConnEventNoticeCmd(pEvent->linkTerminate.connectionHandle, selfEntity, 0);

		HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);

		simpleTopology_freeAttRsp(bleNotConnected);
		discState = BLE_DISC_STATE_IDLE;

	}
		break;

	case GAP_LINK_PARAM_UPDATE_EVENT: {

		if(!connectionConfigured){
//			if ((pEvent->linkUpdate.connInterval >= DEFAULT_DESIRED_MIN_CONN_INTERVAL && pEvent->linkUpdate.connInterval <= DEFAULT_DESIRED_MAX_CONN_INTERVAL)
//					&& pEvent->linkUpdate.connLatency == DEFAULT_DESIRED_SLAVE_LATENCY && pEvent->linkUpdate.connTimeout == DEFAULT_DESIRED_CONN_TIMEOUT) {
				connectionConfigured = TRUE;
				GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST); //trigger the first discovery, the subsequent will be triggered by GAP_DEVICE_DISCOVERY_EVENT
				Util_rescheduleClock(&preCEClock, (pEvent->linkUpdate.connInterval * 1250) / 1000 - 10);

			//}
		}else{
			Util_rescheduleClock(&preCEClock, (pEvent->linkUpdate.connInterval * 1250) / 1000 - 10);
		}
		//moved outside the if for safety...
//		connectionConfigured = TRUE;
//		GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,	DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST); //trigger the first discovery, the subsequent will be triggered by GAP_DEVICE_DISCOVERY_EVENT
//		Util_rescheduleClock(&preCEClock,	(pEvent->linkUpdate.connInterval * 1250) / 1000 - 10);

	}
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      simpleTopology_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void simpleTopology_charValueChangeCB(uint8_t paramID) {
	simpleTopology_enqueueMsg(SBT_CHAR_CHANGE_EVT, paramID, NULL);
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

	simpleTopology_enqueueMsg(KEY_CHANGE_EVT, (uint8) notificationType,	NULL);

}

/*********************************************************************
 * @fn      simpleTopology_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void Climb_processCharValueChangeEvt(uint8_t paramID) {
	uint8_t newValue[CLIMBPROFILE_CHAR2_LEN];

	switch (paramID) {
	case CLIMBPROFILE_CHAR1: {
		//ClimbProfile_GetParameter(CLIMBPROFILE_CHAR1, newValue);

		break;
	}
	case CLIMBPROFILE_CHAR2: {
		ClimbProfile_GetParameter(CLIMBPROFILE_CHAR2, newValue);

		uint8 i = 0;
		while (i < CLIMBPROFILE_CHAR2_LEN - 1) { //TODO: cambiare CLIMBPROFILE_CHAR2_LEN con la lunghezza effettiva dell'operazione di scrittura, per ora risolto con il break nelle righe successive
			uint8 nodeID[CHILD_NODE_ID_LENGTH];
			uint32 wakeUpTimeout_sec_local;

			memcpy(nodeID, &newValue[i], CHILD_NODE_ID_LENGTH);

			if (memcomp(nodeID, zeroID, CHILD_NODE_ID_LENGTH) == 0) { //if it find a zero ID stop checking!
				break;
			}

			if (memcomp(nodeID, broadcastID, CHILD_NODE_ID_LENGTH) == 0) { 	//broadcastID found! ONLY ONE BROADCAST MSG PER NOTIFICATION (PER GATT PACKET)

				isBroadcastMessageValid = TRUE;
				uint8 broadcastMsgType = newValue[i + CHILD_NODE_ID_LENGTH];

				Util_restartClock(&resetBroadcastCmdClock, RESET_BROADCAST_CMD_TIMEOUT);

				switch (broadcastMsgType) {
				case BROADCAST_MSG_TYPE_STATE_UPDATE_CMD:
					memcpy(broadcastMessage, &newValue[i + CHILD_NODE_ID_LENGTH], BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD);
					//i = i + NODE_ID_LENGTH + BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD;
					break;

				case BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD:
					memcpy(broadcastMessage, &newValue[i + CHILD_NODE_ID_LENGTH], BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD);
					wakeUpTimeout_sec_local = ((newValue[i + CHILD_NODE_ID_LENGTH + 1]) << 16) + ((newValue[i + CHILD_NODE_ID_LENGTH + 2]) << 8)
							+ (newValue[i + CHILD_NODE_ID_LENGTH + 3]);
					Climb_setWakeUpClock(wakeUpTimeout_sec_local);
					//i = i + NODE_ID_LENGTH + BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD;
					break;

				case BROADCAST_RESET_CMD_FLAG_CMD:
					//no message to be copied or to be send
					isBroadcastMessageValid = FALSE;
					//i = i + NODE_ID_LENGTH + BROADCAST_LENGTH_RESET_CMD_FLAG_CMD;
					break;

				default: //should not reach here
					//i = i + NODE_ID_LENGTH; //this is to avoid infinite loops in case of wrong packet formatting
					break;
				}

				return; //ONLY ONE BROADCAST MSG PER NOTIFICATION (PER GATT PACKET)

			} else { //broadcastID not found

				uint8 index = Climb_findNodeById(nodeID, CLIMB_CHILD_NODE);
				if (index != 255) {
					if (newValue[i + CHILD_NODE_ID_LENGTH] != childListArray[index].state) {
						if (newValue[i + CHILD_NODE_ID_LENGTH] != INVALID_STATE) {
							childListArray[index].stateToImpose = (ChildClimbNodeStateType_t) newValue[i + CHILD_NODE_ID_LENGTH]; //the correctness of this will be checked in Climb_advertisedStatesCheck
						}
					}
				} else {
					uint8 fakeNodeData[13 + CHILD_NODE_ID_LENGTH];
					uint8 j = 0;
					gapDeviceInfoEvent_t fakeEvt;

					fakeEvt.dataLen = 13 + CHILD_NODE_ID_LENGTH;
					fakeEvt.pEvtData = fakeNodeData;
					fakeEvt.eventType = GAP_ADRPT_ADV_NONCONN_IND;
					fakeEvt.rssi = 0;

					fakeNodeData[j++] = 0x07, // length of this data
					fakeNodeData[j++] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
					fakeNodeData[j++] = 'C';
					fakeNodeData[j++] = 'L';
					fakeNodeData[j++] = 'I';
					fakeNodeData[j++] = 'M';
					fakeNodeData[j++] = 'B';
					fakeNodeData[j++] = 'C';
					fakeNodeData[j++] = 4+CHILD_NODE_ID_LENGTH;
					fakeNodeData[j++] = GAP_ADTYPE_MANUFACTURER_SPECIFIC; // manufacturer specific adv data type
					fakeNodeData[j++] = 0x0D; // Company ID - Fixed
					fakeNodeData[j++] = 0x00; // Company ID - Fixed
					memcpy(&fakeNodeData[12], nodeID, CHILD_NODE_ID_LENGTH);
					j += CHILD_NODE_ID_LENGTH;
					fakeNodeData[j++] = newValue[i + CHILD_NODE_ID_LENGTH];

					Climb_addNodeDeviceInfo(&fakeEvt, CLIMB_CHILD_NODE);
			}

			}
			i = i + CHILD_NODE_ID_LENGTH + 1;
		}
		break;
	}
	default:
		// should not reach here!
		break;
	}
}

/*********************************************************************
 * @fn      simpleTopology_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
//void simpleTopology_startDiscHandler(UArg a0)
//{
//  events |= SBT_START_DISCOVERY_EVT;
//
//  // Wake up the application thread when it waits for clock event
//  Semaphore_post(sem);
//}
/*********************************************************************
 * @fn      simpleTopology_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t simpleTopology_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData) {
	sbtEvt_t *pMsg = ICall_malloc(sizeof(sbtEvt_t));

	// Create dynamic pointer to message.
	if (pMsg) {
		pMsg->event = event;
		pMsg->status = status;
		pMsg->pData = pData;

		// Enqueue the message.
		return Util_enqueueMsg(appMsgQueue, sem, (uint8*) pMsg);
	}else{
		//CLIMB_FlashLed(Board_LED1);
	}

	return FALSE;
}

/*********************************************************************
 * @fn      simpleTopology_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
//void simpleTopology_keyChangeHandler(uint8 keys)
//{
//  simpleTopology_enqueueMsg(SBT_KEY_CHANGE_EVT, keys, NULL);
//}
/*********************************************************************
 * @fn      simpleTopology_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
/*
static void simpleTopology_startDiscovery(void) {
	attExchangeMTUReq_t req;

	// Initialize cached handles
	svcStartHdl = svcEndHdl = 0;

	discState = BLE_DISC_STATE_MTU;

	// Discover GATT Server's Rx MTU size
	req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

	// ATT MTU size should be set to the minimum of the Client Rx MTU
	// and Server Rx MTU values
	VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}
*/
/*********************************************************************
 * @fn      simpleTopology_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
/*
static void simpleTopology_processGATTDiscEvent(gattMsgEvent_t *pMsg) {
	if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
		// MTU size updated
		//LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE4);
	} else if (discState == BLE_DISC_STATE_MTU) {
//    // MTU size response received, discover simple BLE service
//    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
//    {
//      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
//                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
//      discState = BLE_DISC_STATE_SVC;
//
//      // Discovery simple BLE service
//      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
//                                         selfEntity);
//    }
	} else if (discState == BLE_DISC_STATE_SVC) {
//    // Service found, store handles
//    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
//        pMsg->msg.findByTypeValueRsp.numInfo > 0)
//    {
//      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
//      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
//    }
//
//    // If procedure complete
//    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
//         (pMsg->hdr.status == bleProcedureComplete))  ||
//        (pMsg->method == ATT_ERROR_RSP))
//    {
//      if (svcStartHdl != 0)
//      {
//        attReadByTypeReq_t req;
//
//        // Discover characteristic
//        discState = BLE_DISC_STATE_CHAR;
//
//        req.startHandle = svcStartHdl;
//        req.endHandle = svcEndHdl;
//        req.type.len = ATT_BT_UUID_SIZE;
//        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
//        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
//
//        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
//      }
//    }
	} else if (discState == BLE_DISC_STATE_CHAR) {
		// Characteristic found, store handle
		if ((pMsg->method == ATT_READ_BY_TYPE_RSP) && (pMsg->msg.readByTypeRsp.numPairs > 0)) {
			//find index to store handle
			uint8_t connIndex = gapRoleInfo_Find(connHandle);
			charHdl[connIndex] = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1]);

			//LCD_WRITE_STRING("Simple Svc Found", LCD_PAGE6);
		}

		discState = BLE_DISC_STATE_IDLE;
	}
}
*/
/*********************************************************************
 * @fn      simpleTopology_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
/*static bool simpleTopology_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen) {
	uint8_t adLen;
	uint8_t adType;
	uint8_t *pEnd;

	pEnd = pData + dataLen - 1;

	// While end of data not reached
	while (pData < pEnd) {
		// Get length of next AD item
		adLen = *pData++;
		if (adLen > 0) {
			adType = *pData;

			// If AD type is for 16-bit service UUID
			if ((adType == GAP_ADTYPE_16BIT_MORE) || (adType == GAP_ADTYPE_16BIT_COMPLETE)) {
				pData++;
				adLen--;

				// For each UUID in list
				while (adLen >= 2 && pData < pEnd) {
					// Check for match
					if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid))) {
						// Match found
						return TRUE;
					}

					// Go to next
					pData += 2;
					adLen -= 2;
				}

				// Handle possible erroneous extra byte in UUID list
				if (adLen == 1) {
					pData++;
				}
			} else {
				// Go to next item
				pData += adLen;
			}
		}
	}

	// Match not found
	return FALSE;
}
*/
/*********************************************************************
 * @fn      simpleTopology_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
//static void simpleTopology_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
//{
//  uint8_t i;
//
//  // If result count not at max
//  if (scanRes < DEFAULT_MAX_SCAN_RES)
//  {
//    // Check if device is already in scan results
//    for (i = 0; i < scanRes; i++)
//    {
//      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
//      {
//        return;
//      }
//    }
//
//    // Add addr to scan result list
//    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
//    devList[scanRes].addrType = addrType;
//
//    // Increment scan result count
//    scanRes++;
//  }
//}

/*********************************************************************
 * @fn      Keys_EventCB
 *
 * @brief   Callback from peripheralObserver task indicating a connection parameter change.
 *
 * @param   notificationType - type of button press
 *
 * @return  None.
 */
//static void updateConnParam_CB(uint16_t connInterval, uint16_t connSlaveLatency, uint16_t connTimeout){
//
//	if( (connInterval >=  DEFAULT_DESIRED_MIN_CONN_INTERVAL &&  connInterval <=  DEFAULT_DESIRED_MAX_CONN_INTERVAL) && connSlaveLatency==DEFAULT_DESIRED_SLAVE_LATENCY && connTimeout==DEFAULT_DESIRED_CONN_TIMEOUT){
//
//	}
//
//	//move outside the if for safety...
//	connectionConfigured = TRUE;
//	GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST); //trigger the first discovery, the subsequent will be triggered by GAP_DEVICE_DISCOVERY_EVENT
//	Util_rescheduleClock(&preCEClock, (connInterval * 1250)/1000-10 );
//
//}
/*********************************************************************
 * @fn      Climb_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void Climb_clockHandler(UArg arg) {
	// Store the event.
	events |= arg;

	// Wake up the application.
	Semaphore_post(sem);
}

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

	if (gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND | gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND
			| gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //il nome è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1] == GAP_ADTYPE_LOCAL_NAME_COMPLETE) { //ho trovato il nome del dispositivo

				if (memcomp(&(gapDeviceInfoEvent_a->pEvtData[index + 2]), &Climb_childNodeName[0], (gapDeviceInfoEvent_a->pEvtData[index]) - 1) == 0) { //il nome è compatibile con CLIMB

					return CLIMB_CHILD_NODE;

				} else if (memcomp(&(gapDeviceInfoEvent_a->pEvtData[index + 2]), &(defAdvertData[2]), (gapDeviceInfoEvent_a->pEvtData[index]) - 1) == 0) { //TODO: verificare...
					return CLIMB_MASTER_NODE;
				} else {
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
static void Climb_addNodeDeviceInfo(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType) {

	uint8 node_position = 255;

	if (nodeType == CLIMB_CHILD_NODE) {
		node_position = Climb_findNodeById(&gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], nodeType);
	} else if (CLIMB_MASTER_NODE) {
		node_position = Climb_findNodeById(gapDeviceInfoEvent->addr, nodeType);
		return; //for now a master don't need to store other master nodes data, this saves some memory....
	}

	if (node_position == 255) {	//dispositivo nuovo, aggiungilo!
		Climb_addNode(gapDeviceInfoEvent, nodeType);
	} else {
		Climb_updateNodeMetadata(gapDeviceInfoEvent, node_position, nodeType);
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

//static listNode_t* Climb_findNodeByDevice(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType) {
//
//	listNode_t* node = NULL;
//	if (nodeType == CLIMB_CHILD_NODE) {
//		if (childListRootPtr == NULL) {
//			return NULL;
//		}
//		node = childListRootPtr;
//	} else if (nodeType == CLIMB_MASTER_NODE) {
//		if (masterListRootPtr == NULL) {
//			return NULL;
//		}
//		node = masterListRootPtr;
//	}
//
//	while (node != NULL) {
//		if (memcomp(gapDeviceInfoEvent->addr, node->device.addr, B_ADDR_LEN) == 0) {
//			return node; //se trovi il nodo esci e passane il puntatore
//		}
//		node = node->next; //passa al nodo sucessivo
//	}
//
//	return NULL;
//}
/*********************************************************************
 * @fn      Climb_findChildNodeById
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

		while (isNonZero(childListArray[freePos].id, CHILD_NODE_ID_LENGTH)) {
			freePos++;

			if (freePos >= MAX_SUPPORTED_CHILD_NODES) {
				return 255;
			}

			if(gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET] > 5){
				return 255;
			}

		}

		childListArray[freePos].rssi = gapDeviceInfoEvent->rssi;
		childListArray[freePos].lastContactTicks = Clock_getTicks();

		memcpy(childListArray[freePos].id, &gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], CHILD_NODE_ID_LENGTH);
		childListArray[freePos].state = (ChildClimbNodeStateType_t) gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET];
		childListArray[freePos].stateToImpose = childListArray[freePos].state;
		childListArray[freePos].contactSentThoughGATT = FALSE;

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
		masterListArray[freePos].stateToImpose = masterListArray[freePos].state;
		masterListArray[freePos].contactSentThoughGATT = FALSE;

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

			if(gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET] > 5){
				return;
			}

//#warning NOW THE MASTER SEND CHILD PACKET COUNTER INSTEAD OF RSSI
			childListArray[index].rssi = gapDeviceInfoEvent->rssi;
			//childListArray[index].rssi = gapDeviceInfoEvent->pEvtData[gapDeviceInfoEvent->dataLen - 1];
			childListArray[index].lastContactTicks = Clock_getTicks();

			memcpy(childListArray[index].id, &gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], CHILD_NODE_ID_LENGTH);
			childListArray[index].state = (ChildClimbNodeStateType_t) gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET];
			childListArray[index].contactSentThoughGATT = FALSE;

			return;

		} else if (nodeType == CLIMB_MASTER_NODE) {

			masterListArray[index].rssi = gapDeviceInfoEvent->rssi;
			masterListArray[index].lastContactTicks = Clock_getTicks();

			memcpy(masterListArray[index].id, &gapDeviceInfoEvent->addr, MASTER_NODE_ID_LENGTH);
			masterListArray[index].state = INVALID_STATE;
			masterListArray[index].contactSentThoughGATT = FALSE;

			return;

		}

	} else if (gapDeviceInfoEvent->eventType == GAP_ADRPT_SCAN_RSP) {	//scan response data

	}

	return;
}

/*********************************************************************
 * @fn      Climb_advertisedStatesCheck
 *
 * @brief	Checks if the states advertised by nodes are the same as stateToImpose, if not it requests the update of advertised data to inform nodes that they have to change their states
 *
 * @return  none
 */
static void Climb_advertisedStatesCheck(void) {
	//FA IL CONTROLLO DELLO STATO ATTUALE RISPETTO A QUELLO VOLUTO
	//listNode_t* node = childListRootPtr;
	onBoardChildren = 0;
	uint8 i = 0;
	while (i < MAX_SUPPORTED_CHILD_NODES) {

		if (isNonZero(childListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

			if (isBroadcastMessageValid == TRUE && broadcastMessage[0] == BROADCAST_MSG_TYPE_STATE_UPDATE_CMD) { //SE IL MESSAGGIO DI BROADCAST E' VALIDO SOVRASCRIVI IL CAMPO stateToImpose su tutti i nodi

				if ((ChildClimbNodeStateType_t) broadcastMessage[1] != INVALID_STATE) {
					childListArray[i].stateToImpose = (ChildClimbNodeStateType_t) broadcastMessage[1];
				}

			}

//		// se il nodo è BY_MYSELF non ha senso cercare di imporre uno stato inviato come broadcast, quindi si può sovrascrivere
//		if (node->device.advData[ADV_PKT_STATE_OFFSET] == BY_MYSELF && Climb_isMyChild(node->device.advData[ADV_PKT_ID_OFFSET])) {
//
//			node->device.stateToImpose = CHECKING;
//
//		}

			{ //CHECK IF THE REQUESTED STATE CHANGE IS VALID OR NOT
				ChildClimbNodeStateType_t actualNodeState = childListArray[i].state; //(ChildClimbNodeStateType_t)node->device.advData[ADV_PKT_STATE_OFFSET];
				switch (childListArray[i].stateToImpose) {
				case BY_MYSELF:
					if (actualNodeState == CHECKING || actualNodeState == ON_BOARD || actualNodeState == ALERT || actualNodeState == GOING_TO_SLEEP) { // && Climb_isMyChild(node->device.advData[ADV_PKT_ID_OFFSET])) {
					//allowed transition
					} else {
						childListArray[i].stateToImpose = actualNodeState; //mantieni lo stato precedente
					}
					break;

				case CHECKING:
					if (actualNodeState == BY_MYSELF) { // && Climb_isMyChild(node->device.advData[ADV_PKT_ID_OFFSET])) {
						//allowed transition
					} else {
						childListArray[i].stateToImpose = actualNodeState; //mantieni lo stato precedente
					}
					break;

				case ON_BOARD:
					if ((actualNodeState == CHECKING || actualNodeState == ALERT)) { // && Climb_isMyChild(node->device.advData[ADV_PKT_ID_OFFSET])) {
						//allowed transition
					} else {
						childListArray[i].stateToImpose = actualNodeState; //mantieni lo stato precedente
					}
					break;

				case ALERT:
					childListArray[i].stateToImpose = ON_BOARD; //don't broadcast ALERT state!
					break;

				case GOING_TO_SLEEP:
					if (actualNodeState == CHECKING || actualNodeState == ON_BOARD) { // && Climb_isMyChild(node->device.advData[ADV_PKT_ID_OFFSET])) {
					//allowed transition
					} else {
						childListArray[i].stateToImpose = actualNodeState; //mantieni lo stato precedente
					}
					break;

				case INVALID_STATE:
					break;

				default:
					break;
				}

				//if (actualNodeState != childListArray[i].stateToImpose) {
				advUpdateReq = TRUE;
				//}

				if (actualNodeState == ON_BOARD) {
					onBoardChildren++;
				}

			}

		}
		i++; //passa al nodo sucessivo
	}

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

	uint32 nowTicks = Clock_getTicks();

	uint8 i = 0;
	//controlla la lista dei child
	//listNode_t* targetNode = childListRootPtr;
	//listNode_t* previousNode = NULL;
	while (i < MAX_SUPPORTED_CHILD_NODES) { //NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		if (isNonZero(childListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

			if (nowTicks - childListArray[i].lastContactTicks > NODE_TIMEOUT_OS_TICKS) {
				switch ((ChildClimbNodeStateType_t) childListArray[i].stateToImpose) {
				case BY_MYSELF:
				case CHECKING:
					Climb_removeNode(i, CLIMB_CHILD_NODE); //rimuovi il nodo
					break;

				case ON_BOARD:
					//do nothing, app will trigger the alert!!
				case ALERT:
					//do nothing
					childListArray[i].state = ALERT;
					break;
				case GOING_TO_SLEEP:
					//do nothing
					Climb_removeNode(i, CLIMB_CHILD_NODE); //rimuovi il nodo
					break;

				default:
					//should not reach here
					break;
				}
			} else { //il nodo non è andato in timeout,

			}
			//NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP

		}
		i++;
	}

	//controlla la lista dei master
	//targetNode = masterListRootPtr;
	//previousNode = NULL;
	i = 0;
	while (i < MAX_SUPPORTED_MASTER_NODES) { //NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		if (isNonZero(masterListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces
			if (nowTicks - masterListArray[i].lastContactTicks > NODE_TIMEOUT_OS_TICKS) { //se il timeout è scaduto
				Climb_removeNode(i, CLIMB_MASTER_NODE); //rimuovi il nodo
				advUpdateReq = TRUE;
			} else { //il nodo non è andato in timeout,

			}
			//NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		}

		i++;
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
static void Climb_periodicTask() {
	Climb_nodeTimeoutCheck();

#ifdef  HEAPMGR_METRICS
	plotHeapMetrics();
#endif

#ifdef STACK_METRICS
	checkStackMetrics();
#endif

}

#ifdef PRINTF_ENABLED
#ifdef PRINT_NODE_INFO_ENABLED
static void Climb_printfNodeInfo(gapDeviceInfoEvent_t *gapDeviceInfoEvent ) {
	static uint8 usbPktsCounter = 0;
	uint32 nowTicks = Clock_getTicks();
	System_printf("%d ", nowTicks);
	//System_printf(Util_convertBdAddr2Str(myAddr));
	System_printf(Util_convertBdAddr2Str(gapDeviceInfoEvent->addr));
	System_printf(" CLIMBC ADV %02x %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",usbPktsCounter++, gapDeviceInfoEvent->pEvtData[12],gapDeviceInfoEvent->pEvtData[13],gapDeviceInfoEvent->pEvtData[14],gapDeviceInfoEvent->pEvtData[15],gapDeviceInfoEvent->pEvtData[16],gapDeviceInfoEvent->pEvtData[17],gapDeviceInfoEvent->pEvtData[18],gapDeviceInfoEvent->pEvtData[19],gapDeviceInfoEvent->pEvtData[20],gapDeviceInfoEvent->pEvtData[21],gapDeviceInfoEvent->pEvtData[22],gapDeviceInfoEvent->pEvtData[23],gapDeviceInfoEvent->pEvtData[24],gapDeviceInfoEvent->pEvtData[25],gapDeviceInfoEvent->pEvtData[26],gapDeviceInfoEvent->pEvtData[27],gapDeviceInfoEvent->pEvtData[28],gapDeviceInfoEvent->pEvtData[29],gapDeviceInfoEvent->pEvtData[30] );
	//System_printf(" CLIMBD ADV %02x %02x%02x%02x\n",usbPktsCounter++, gapDeviceInfoEvent->pEvtData[12] ,gapDeviceInfoEvent->pEvtData[30] );

}
#endif
#endif


static void Climb_preAdvEvtHandler() {

	if (connectionConfigured) {
		advUpdateReq = true;
		adv_counter++;
	}

	if (advUpdateReq && connectionConfigured) {
		Climb_advertisedStatesCheck(); //aggiorna l'adv
		Climb_advertisedStatesUpdate();
		advUpdateReq = false;
	}

	GAPRole_CancelDiscovery();
//	if (connectionConfigured) {
//		if(scanningActive == TRUE){
//			GAPObserverRole_CancelDiscovery(); //the scan will be restarted on GAP_DEVICE_DISCOVERY_EVENT
//			scanningActive = FALSE;
//		}else{
//			GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
//			scanningActive = TRUE;
//		}
//	}
}

static void Climb_preCEEvtHandler() {
	if (connectionConfigured) {
		Climb_contactsCheckSendThroughGATT();
	}
}
/*********************************************************************
 * @fn      Climb_goToSleepHandler
 *
 * @brief	Handler associated with goToSleep Clock instance.
 *
 * @return  none
 */
static void Climb_goToSleepHandler() {

	if (onBoardChildren == 0 || !connectionConfigured) {
		stopNode();
	} else {
		Util_restartClock(&goToSleepClock, GOTOSLEEP_POSTPONE_INTERVAL_SEC * 1000);
	}

}

/*********************************************************************
 * @fn      Climb_wakeUpHandler
 *
 * @brief	Handler associated with wakeUp Clock instance.
 *
 * @return  none
 */
static void Climb_wakeUpHandler() {

	if (wakeUpTimeout_sec_global == 0) {
		startNode();
		Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC * 1000);
		Climb_setWakeUpClock(WAKEUP_DEFAULT_TIMEOUT_SEC);

		return;
	}

	if (wakeUpTimeout_sec_global > MAX_ALLOWED_TIMER_DURATION_SEC) {
		Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC * 1000);
		wakeUpTimeout_sec_global = wakeUpTimeout_sec_global - MAX_ALLOWED_TIMER_DURATION_SEC;
	} else {
		//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
		//float randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
		Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_global * 1000);
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
static void Climb_setWakeUpClock(uint32 wakeUpTimeout_sec_local) {

	if (wakeUpTimeout_sec_local > MAX_ALLOWED_TIMER_DURATION_SEC) { //max timer duration 42949.67sec
		Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC * 1000);
		wakeUpTimeout_sec_global = wakeUpTimeout_sec_local - MAX_ALLOWED_TIMER_DURATION_SEC;
	} else {
		//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
		float randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
		Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_local * 1000 - 1000 + randDelay_msec);
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
static void CLIMB_FlashLed(PIN_Id pinId) {

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
//	timerSNVDataStore_t timerConfToStore;
//	timerSNVDataStore_t timerConfToRestore;

	switch ((keys_Notifications_t) keys) {
	case LEFT_SHORT:

		adv_counter = 0;


//		stopNode();
//		Util_stopClock(&goToSleepClock);
//
//		//timerConfToStore.wakeUpRemainingTimeout = Util_getClockTimeout(&wakeUpClock);
//		timerConfToStore.wakeUpClock_Struct = wakeUpClock;
//		timerConfToStore.wakeUpTimerActive = Util_isActive(&wakeUpClock);
//		//timerConfToStore.goToSleepRamainingTimeout = Util_getClockTimeout(&goToSleepClock);
//		timerConfToStore.goToSleepClock_Struct = goToSleepClock;
//		timerConfToStore.goToSleepTimerActive = Util_isActive(&goToSleepClock);
//		timerConfToStore.wakeUpTimeout_sec_global_value = wakeUpTimeout_sec_global;
//		timerConfToStore.validData = TRUE;
//
//		osal_snv_write(SNV_BASE_ID + TIMERS_SNV_OFFSET, sizeof(timerSNVDataStore_t), &timerConfToStore);
//
//
//		uint8 ret = osal_snv_read(SNV_BASE_ID + TIMERS_SNV_OFFSET, sizeof(timerSNVDataStore_t), &timerConfToRestore);
//
//		if (ret == SUCCESS) {
//
//			if (timerConfToRestore.validData) {
//
//				//Reload wakeUp timer value
//				wakeUpTimeout_sec_global = timerConfToRestore.wakeUpTimeout_sec_global_value;
//				wakeUpClock = timerConfToRestore.wakeUpClock_Struct;
//				Util_startClock(&wakeUpClock);
//				if (!timerConfToRestore.wakeUpTimerActive) { //if the clock was inactive stop it immediatelly
//					Util_stopClock(&wakeUpClock);
//				}
//
//				//Reload goToSleep timer value
//				goToSleepClock = timerConfToRestore.goToSleepClock_Struct;
//				Util_startClock(&goToSleepClock);
//				if (!timerConfToRestore.goToSleepTimerActive) {
//					Util_stopClock(&goToSleepClock);
//				}
//				HAL_SYSTEM_RESET();
//			}
//		}



		break;

	case RIGHT_SHORT:

		break;

	case LEFT_LONG:

		break;

	case RIGHT_LONG:
		if (nodeTurnedOn != TRUE) {
			HAL_SYSTEM_RESET();
//			startNode();
//			Climb_setWakeUpClock(WAKEUP_DEFAULT_TIMEOUT_SEC);
//			Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC * 1000);
		}else{

			stopNode();
		}

		break;

	case BOTH:
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
#ifdef WATCHDOGTIMER_EN
	GPTimerCC26XX_start(hTimer);
#endif

//#warning REMOVE COMMENT TO ENSURE CORRECT BEHAVIOUR
	GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(defAdvertData), defAdvertData, NULL);

	uint8 adv_active = 1;

	uint8 status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);

	Util_startClock(&periodicClock);

	CLIMB_FlashLed(Board_LED2);
	nodeTurnedOn = TRUE;

}

/*********************************************************************
 * @fn      startNode
 *
 * @brief   Function to call to stop the node.
 *

 * @return  none
 */
static void stopNode() {
#ifdef WATCHDOGTIMER_EN
	GPTimerCC26XX_stop(hTimer);
#endif

	uint8 status = HCI_EXT_AdvEventNoticeCmd(selfEntity, 0);

	connectionConfigured = FALSE;
	nodeTurnedOn = FALSE;

	Util_stopClock(&periodicClock);

	uint16 size = CLIMBPROFILE_CHAR1_LEN;
	uint8* zeroArray = (uint8 *) ICall_malloc( sizeof(uint8) * size);
	if (zeroArray != NULL) {
		memset(zeroArray, 0, size);

		ClimbProfile_SetParameter(CLIMBPROFILE_CHAR1, size, zeroArray); //NB: zeroArray is freed in any case by ClimbProfile_SetParameter
	}

	uint8 adv_active = 0;
	status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);
	status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);

	if(connHandle != GAP_CONNHANDLE_INIT){
		GAPRole_TerminateConnection(connHandle);
	}

	GAPRole_CancelDiscovery();

	destroyChildNodeList();
	destroyMasterNodeList();

	Climb_advertisedStatesUpdate();

	CLIMB_FlashLed(Board_LED1);

}
/*********************************************************************
 * @fn      destroyChildNodeList
 *
 * @brief   Destroy child list
 *

 * @return  none
 */
static void destroyChildNodeList() {

	uint8 i;
	for (i = 0; i < MAX_SUPPORTED_CHILD_NODES; i++) {
		if (isNonZero(childListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

			switch ((ChildClimbNodeStateType_t) childListArray[i].stateToImpose) {
			case BY_MYSELF:
			case CHECKING:
				Climb_removeNode(i, CLIMB_CHILD_NODE); //rimuovi il nodo
				break;

			case ON_BOARD:
#warning maybe it is better to destroy the list regardless of the stat
			case ALERT:
				if(!Util_isActive(&goToSleepClock)){ //remove the node only if it is an automatic switch off
					Climb_removeNode(i, CLIMB_CHILD_NODE); //rimuovi il nodo
				}
				break;
			case GOING_TO_SLEEP:
				Climb_removeNode(i, CLIMB_CHILD_NODE); //rimuovi il nodo
				break;

			default:
				//should not reach here
				break;
			}
		}
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

	uint8 i;
	for (i = 0; i < MAX_SUPPORTED_MASTER_NODES; i++) {
		Climb_removeNode(i, CLIMB_MASTER_NODE);
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
static void displayInit(void) {

	// Initialize LCD
	devpkLcdOpen();
}
#endif
#ifdef WATCHDOGTIMER_EN
static void watchdogTimerInit() {
	GPTimerCC26XX_Params params;
	GPTimerCC26XX_Params_init(&params);
	params.width = GPT_CONFIG_32BIT;
	params.mode = GPT_MODE_PERIODIC_UP;
	params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
	hTimer = GPTimerCC26XX_open(CC2650_GPTIMER0A, &params);
	if (hTimer == NULL) {
		//Log_error0("Failed to open GPTimer");
		//Task_exit();
	}

	Types_FreqHz freq;
	BIOS_getCpuFreq(&freq);
	GPTimerCC26XX_Value loadVal = freq.lo / 1 - 1; //47999999
	GPTimerCC26XX_setLoadValue(hTimer, loadVal);
	GPTimerCC26XX_registerInterrupt(hTimer, timerCallback, GPT_INT_TIMEOUT);

}
static void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {

	switch (unclearedWatchdogEvents) {
	case 0:
	case 1:
		events |= WATCHDOG_EVT;
		Semaphore_post(sem);
		break;
	case 2:
		unclearedWatchdogEvents--;
		//resetApplication?
		break;
	default:
		//HAL_SYSTEM_RESET();
		//or
		//HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);

		break;
	}

	unclearedWatchdogEvents++;
}
#endif
/*********************************************************************
 * @fn      Climb_contactsCheckSendThroughGATT
 *
 * @brief	Checks all nodes in child list and send, through gatt, contacts recorded since last call of this function
 *
 * @param   None.
 *
 * @return  None.
 */
static void Climb_contactsCheckSendThroughGATT(void) {
	//INVIA I CONTATTI RADIO AVVENUTI NELL'ULTIMO PERIODO TRAMITE GAT
	//NB: SE ANDROID LIMITA A 4 PPCE LA LUNGHEZZA MASSIMA PER UNA CARATTERISTICA E' 3*27+20 byte = 101 byte = 33 nodi
	// ATTENZIONE NON USARE L'MTU IMPOSTATO NEI PREPROCESSOR DEFINES MA QUELLO OTTENUTO TRAMITE LE API O GLI EVENTI

	//INVIA I CONTATTI RADIO AVVENUTI NELL'ULTIMO PERIODO TRAMITE GATT
	uint16 size = mtu_size - 3;
	//uint8* childrenNodesData = (uint8 *) GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, mtu_size - 3, &size);
	uint8* childrenNodesData = (uint8 *) ICall_malloc( size );
	uint8 i = 0;
	uint8 roundCompleted = FALSE;

	uint8 gattUpdateReq = FALSE;
	uint8 nodeArrayIndex = gatt_startNodeIndex;

//	attHandleValueNoti_t noti;
//	noti.pValue = childrenNodesData;

	if (childrenNodesData != NULL) { //if allocated
		while (i < size - 2 && roundCompleted == FALSE) {

			if (isNonZero(childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

				if (childListArray[nodeArrayIndex].contactSentThoughGATT == FALSE) {				// //invia solo i nodi visti dopo l'ultimo check, invia tutti i nodi CLIMBC
					memcpy(&childrenNodesData[i], childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH); //salva l'id del nodo
					i += CHILD_NODE_ID_LENGTH;
					childrenNodesData[i++] = childListArray[nodeArrayIndex].state;
#ifdef INCLUDE_RSSI_IN_GATT_DATA
					childrenNodesData[i++] = childListArray[nodeArrayIndex].rssi;
#endif
					childListArray[nodeArrayIndex].contactSentThoughGATT = TRUE;
					gattUpdateReq = TRUE;
				}
			}

			nodeArrayIndex++; //passa al nodo sucessivo

			if (nodeArrayIndex >= MAX_SUPPORTED_CHILD_NODES) {
				nodeArrayIndex = 0;
			}

			if (nodeArrayIndex == gatt_startNodeIndex) {
				roundCompleted = TRUE;
			}
		}
		gatt_startNodeIndex = nodeArrayIndex;

		//noti.len = i;

		if (gattUpdateReq) {
			//azzero i byte successivi
			//			while (i < 20) {
			//				childrenNodesData[i++] = 0;
			//			}

			if (ClimbProfile_SetParameter(CLIMBPROFILE_CHAR1, i, childrenNodesData) != SUCCESS) {
				PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_ON);
				PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_ON);

				//sending the notification in the right way the &childrenNodesData memory is freed in any case
				//GATT_bm_free((gattMsg_t *) &noti, ATT_HANDLE_VALUE_NOTI);
				//ICall_free(childrenNodesData);
			}
			gattUpdateReq = FALSE;
		} else {
			//GATT_bm_free((gattMsg_t *) &noti, ATT_HANDLE_VALUE_NOTI);
			ICall_free(childrenNodesData);
		}
	}else{ //an error occured during allocation
		PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_ON);
		PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_ON);
	}
}

/*********************************************************************
 * @fn      Climb_advertisedStatesUpdate
 *
 * @brief	Changes advertised data accordingly with states stored in stateToImpose
 *
 * @param   None.
 *
 * @return  None.
 */
static void Climb_advertisedStatesUpdate(void) {

	uint8 i = 12;
	uint8 newChildrenStatesData[31];

	newChildrenStatesData[0] = 0x07, // length of this data
	newChildrenStatesData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
	newChildrenStatesData[2] = defAdvertData[2];
	newChildrenStatesData[3] = defAdvertData[3];
	newChildrenStatesData[4] = defAdvertData[4];
	newChildrenStatesData[5] = defAdvertData[5];
	newChildrenStatesData[6] = defAdvertData[6];
	newChildrenStatesData[7] = defAdvertData[7];
	//newChildrenStatesData[8] = i-9; //assigned later!!!
	newChildrenStatesData[9] = GAP_ADTYPE_MANUFACTURER_SPECIFIC; // manufacturer specific adv data type
	newChildrenStatesData[10] = 0x0D; // Company ID - Fixed //VERIFICARE SE QUESTA REGOLA VALE ANCHE PER I TAG NON COMPATIBILI CON IBEACON
	newChildrenStatesData[11] = 0x00; // Company ID - Fixed

	if (isBroadcastMessageValid == FALSE) {
		uint8 nodeArrayIndex = adv_startNodeIndex;
		uint8 roundCompleted = FALSE;

		while (i < 29 && roundCompleted == FALSE) {

			if (isNonZero(childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

				memcpy(&newChildrenStatesData[i], childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH);
				i += CHILD_NODE_ID_LENGTH;
				newChildrenStatesData[i++] = childListArray[nodeArrayIndex].stateToImpose;

			}

			nodeArrayIndex++;

			if (nodeArrayIndex >= MAX_SUPPORTED_CHILD_NODES) {
				nodeArrayIndex = 0;
			}

			if (nodeArrayIndex == adv_startNodeIndex) {
				roundCompleted = TRUE;
			}

		}
		adv_startNodeIndex = nodeArrayIndex;

	} else { //send broadcast msg
		memcpy(&newChildrenStatesData[i], broadcastID, CHILD_NODE_ID_LENGTH);
		i += CHILD_NODE_ID_LENGTH;

		uint8 broadcastMsgType = broadcastMessage[0];
		switch (broadcastMsgType) {
		case BROADCAST_MSG_TYPE_STATE_UPDATE_CMD:
			memcpy(&newChildrenStatesData[i], &broadcastMessage[0], BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD);
			i = i + BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD;
			break;

		case BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD:
			memcpy(&newChildrenStatesData[i], &broadcastMessage[0], BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD);
			i = i + BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD;
			break;

		default: //should not reach here
			break;
		}

	}

//	while (i < 30) {
//		newChildrenStatesData[i++] = 0;
//	}
	newChildrenStatesData[i++] = adv_counter; //the counter is always in the last position


	newChildrenStatesData[8] = i - 9;
	GAP_UpdateAdvertisingData(selfEntity, true, i, &newChildrenStatesData[0]);   //adv data
	//GAP_UpdateAdvertisingData(selfEntity, false, i,	&newChildrenStatesData[0]); //scn data

}

/*********************************************************************
 * @fn      memcomp
 *
 * @brief   Compares memory locations, same as memcmp, but it returns 0 if two memory sections are identical, 1 otherwise.
 *
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

#ifdef  HEAPMGR_METRICS
static void plotHeapMetrics() {

#ifdef PRINTF_ENABLED
	System_printf("\nHEAP METRICS\n");
#endif
	uint16_t pBlkMax[1];
	uint16_t pBlkCnt[1];
	uint16_t pBlkFree[1];
	uint16_t pMemAlo[1];
	uint16_t pMemMax[1];
	uint16_t pMemUb[1];
	uint16_t pFailAm[1];
	ICall_heapGetMetrics(pBlkMax, pBlkCnt,pBlkFree,pMemAlo,pMemMax,pMemUb,pFailAm);

#ifdef PRINTF_ENABLED
	System_printf("max cnt of all blocks ever seen at once: %d\ncurrent cnt of all blocks: %d\ncurrent cnt of free blocks: %d\ncurrent total memory allocated: %d\nmax total memory ever allocated at once: %d\nupper bound of memory usage: %d\namount of failed malloc calls: %d\n\n\n", pBlkMax[0], pBlkCnt[0], pBlkFree[0],pMemAlo[0], pMemMax[0], pMemUb[0],pFailAm[0] );
#endif

}
#endif
static uint8 isNonZero(uint8 * str1, uint8 len) { //ritorna 0 se il vettore è zero, 1 altrimenti
	uint8 index;
	for (index = 0; index < len; index++) {
		if (str1[index] != 0) {
			return 1;
		}
	}
	return 0;
}

#ifdef STACK_METRICS
static void checkStackMetrics() {
	static uint16 maxStackUse = 0;

	Task_Stat statbuf; /* declare buffer */
	Task_stat(Task_self(), &statbuf); /* call func to get status */
	System_printf("\nSTACK METRICS\n");

	if(statbuf.used > maxStackUse) {
		maxStackUse = statbuf.used;
	}
#ifdef PRINTF_ENABLED
	System_printf("stack size: %d, stack pointer: 0x%x, stack base: 0x%x, used: %d, absolute max used: %d\n", statbuf.stackSize ,statbuf.sp, statbuf.stack , statbuf.used, maxStackUse);
	if (statbuf.used > (statbuf.stackSize * 9 / 10)) {
		System_printf("Over 90% of task's stack is in use.\n");
	}
#endif

	//RESET UNUSED STACK
	uint32 i;

	for(i = 0; i < (statbuf.sp - statbuf.stack)-1; i++) { //-1 for safety, don't care warning
		sbpTaskStack[i] = 0xBE;
	}

}
#endif

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

	if(nodeTurnedOn){
		if (!connectionConfigured) {

			CLIMB_FlashLed(Board_LED2);
			lastBattMeas = 0;

		} else {
			PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);

			uint32 ticksNow = Clock_getTicks();
			if(ticksNow - lastBattMeas > MIN_BATT_MEAS_INTERVAL_TICKS){
				Batt_MeasLevel();

				lastBattMeas = ticksNow;
			}

		}

		Util_startClock(&preAdvClock);
	} else {
		uint8 adv_active = 0;

		uint8 status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);
		status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
	}
}

/*********************************************************************
 * @fn      exceptionHandler
 *
 * @brief   Faults handler
 *
 * @param
 *
 * @return  none
 */
/**
 * Exception handler
 */
struct exceptionFrame
{
	unsigned int _r4;
	unsigned int _r5;
	unsigned int _r6;
	unsigned int _r7;
	unsigned int _r8;
	unsigned int _r9;
	unsigned int _r10;
	unsigned int _r11;
	unsigned int _r0;
	unsigned int _r1;
	unsigned int _r2;
	unsigned int _r3;
	unsigned int _r12;
	unsigned int _lr;
	unsigned int _pc;
	unsigned int _xpsr;
};

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
