/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab04.c
//! \brief  Using InstaSPIN锟紽OC only as a torque controller
//!
//! (C) Copyright 2011, Texas Instruments, Inc.
//! \defgroup PROJ_LAB04 PROJ_LAB04
//@{
//! \defgroup PROJ_LAB04_OVERVIEW Project Overview
//!
//! Running InstaSPIN锟紽OC only as a Torque controller
//!
// **************************************************************************
// the includes

#include <math.h>

// modules

#include "cpu_usage.h"
#include "memCopy.h"

// platforms

#include "NUG_user.h"
#include "NUG_hal.h"
#include "NUG_ctrl.h"


#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5

#define BIT_UDCOL			(1<<0)
#define BIT_UDCUL			(1<<1)
#define BIT_IAOL			(1<<2)
#define BIT_IBOL			(1<<3)
#define BIT_ICOL			(1<<4)
#define BIT_UAOL			(1<<5)
#define BIT_UBOL			(1<<6)
#define BIT_UCOL			(1<<7)
#define BIT_CTRL			(1<<8)

//! \brief Defines the number of main iterations before global variables are updated
//!
#define NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE  1

//! \brief Defines the speed acceleration scale factor.
//!
#define MAX_ACCEL_KRPMPS_SF  _IQ(USER_MOTOR_NUM_POLE_PAIRS*1000.0/USER_TRAJ_FREQ_Hz/USER_IQ_FULL_SCALE_FREQ_Hz/60.0)

//! \brief Initialization values of global variables
//!
#define MOTOR_Vars_INIT {false, \
                         false, \
                         false, \
                         true, \
                         false, \
                         false, \
                         true, \
                         true, \
                         false, \
                         false, \
						 \
						 false, \
						 false, \
						 false, \
						 \
                         CTRL_State_Idle, \
                         EST_State_Idle, \
                         USER_ErrorCode_NoError, \
                         {0,CTRL_TargetProc_Unknown,0,0}, \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.1), \
                         _IQ(0.0), \
                         _IQ(0.2), \
                         _IQ(0.0), \
                         _IQ(USER_MAX_VS_MAG_PU), \
                         _IQ(0.1 * USER_MOTOR_MAX_CURRENT), \
                         400, \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         0.0, \
                         0.0, \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.8 * USER_MAX_VS_MAG_PU), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         {0,0,0}, \
                         {0,0,0}, \
						 \
						 _IQ(0.6), \
						 _IQ(0.0), \
                         _IQ(0.0), \
						 _IQ(0.0), \
						 _IQ(0.0), \
						 _IQ(0.0), \
						 _IQ(0.0), \
						 \
						 _IQ(0.0), \
						 _IQ(0.0), \
						 _IQ(0.0), \
						 _IQ(0.0)  \
}

#define SYSTEM_Vars_INIT {false, \
                         false, \
                         false, \
                         false, \
                         false, \
                         \
                         false, \
                         false, \
}

// **************************************************************************
// the typedefs

typedef struct _SYSTEM_Vars_t_
{
	bool Flag_openVavleLeft;
	bool Flag_openVavleRight;
	bool Flag_openFanSource;
	bool Flag_openFan;
	bool Flag_startForce;

	bool Ack_Fan0;
	bool Ack_Fan1;
}SYSTEM_Vars_t;

typedef struct _MOTOR_Vars_t_
{
  bool Flag_enableSys;
  bool Flag_Run_Identify;
  bool Flag_MotorIdentified;
  bool Flag_enableForceAngle;
  bool Flag_enableFieldWeakening;
  bool Flag_enableRsRecalc;
  bool Flag_enableUserParams;
  bool Flag_enableOffsetcalc;
  bool Flag_enablePowerWarp;
  bool Flag_enableSpeedCtrl;

  bool Flag_enableRun;
  bool Flag_RunState;
  bool Flag_enableFlyingStart;

  CTRL_State_e CtrlState;
  EST_State_e EstState;

  USER_ErrorCode_e UserErrorCode;

  CTRL_Version CtrlVersion;

  _iq IdRef_A;
  _iq IqRef_A;
  _iq SpeedRef_pu;
  _iq SpeedRef_krpm;
  _iq SpeedTraj_krpm;
  _iq MaxAccel_krpmps;
  _iq Speed_krpm;
  _iq OverModulation;
  _iq RsOnLineCurrent_A;
  _iq SvgenMaxModulation_ticks;
  _iq Flux_Wb;
  _iq Torque_Nm;

  float_t MagnCurr_A;
  float_t Rr_Ohm;
  float_t Rs_Ohm;
  float_t RsOnLine_Ohm;
  float_t Lsd_H;
  float_t Lsq_H;
  float_t Flux_VpHz;

  float_t ipd_excFreq_Hz;
  _iq     ipd_Kspd;
  _iq     ipd_excMag_coarse_pu;
  _iq     ipd_excMag_fine_pu;
  float   ipd_waitTime_coarse_sec;
  float   ipd_waitTime_fine_sec;

  _iq Kp_spd;
  _iq Ki_spd;

  _iq Kp_Idq;
  _iq Ki_Idq;

  _iq Vd;
  _iq Vq;
  _iq Vs;
  _iq VsRef;
  _iq VdcBus_kV;

  _iq Id_A;
  _iq Iq_A;
  _iq Is_A;

  MATH_vec3 I_bias;
  MATH_vec3 V_bias;

  _iq SpeedSet_krpm;

  _iq angle_sen_pu;
  _iq angle_est_pu;
  _iq speed_sen_pu;
  _iq speed_est_pu;

  _iq speedHigh_hall2fast_pu;
  _iq speedLow_hall2fast_pu;
  _iq IdSet_A;
  _iq IqSet_A;
  _iq IdRef_pu;
  _iq IqRef_pu;
}MOTOR_Vars_t;

typedef struct _HALL_Obj_
{
	// input
	uint16_t hallSta;
	uint32_t capCounter;
	uint32_t tsCounter;
	bool	 flag_cap1;

	// output
	float_t elec_angle_degree;
	float_t elec_angle_degree_prev;
	float_t elec_speed_radps;
	float_t elec_speed_rpm;
	uint16_t dir;

	// parameter
	float_t sample_period;

	// static
	uint16_t sector;
	uint16_t prevSector;
	bool Flag_secChanged;

}HALL_Obj,*HALL_Handle;

typedef struct _ENC_Obj_
{
	// input
	uint32_t qposlat;	// 单位时间位置计数
	uint16_t qcprdlat;	// 单位事件事件计数
	uint16_t dirFlag;
	uint16_t UTOFlag; // 测周
	uint16_t UPEVNTFlag; // 测频
	uint16_t COEFFlag; // 测频

	// output
//	float_t mech_angle_rad;
	float_t mech_speed_radps;
	float_t mech_speed_rpm;
	uint16_t dir;

	// parameter
	uint16_t num_enc_slots;		//!< number of encoder slots
	float_t sample_period;     //
	uint16_t ccps;				// cap clock prescale,sysclkout div
	uint16_t upps;				// qclk div

	// static
	uint32_t prev_enc;				//!< previous encoder reading
	int32_t delta_enc;				//!< encoder count delta
//	uint32_t enc_zero_offset;     //!< encoder zero offset in counts
	float_t rpm_cycle_way;
	float_t rpm_freq_way;
} ENC_Obj,*ENC_Handle;

// **************************************************************************
// the globals



uint16_t gLEDcnt = 0;

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

/*************************************************************/
HAL_Handle halHandle;

CTRL_Handle ctrlHandle;
CTRL_Obj *controller_obj;

USER_Params gUserParams;

HAL_AdcData_t gAdcData;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
//HAL_PwmData_t gPwmDataSim = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
/*******************************************************************/

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;
volatile SYSTEM_Vars_t gSysVars = SYSTEM_Vars_INIT;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
		extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif

// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

//
uint16_t flag1 = 0;
uint16_t flag2 = 0;
uint16_t flag3 = 0;
uint16_t rxPos = 0;
uint16_t rxSta = 0;
uint16_t cnt_Rx = 0;
uint16_t RX[11];

uint16_t txPos = 0;
uint16_t txSta = 0;
uint16_t TX[10];

float RtTorque = 0.0;
float LiftTorque = 0.0;
float RollbackTorque = 0.0;
uint16_t gStatus = 0;

uint16_t gXor = 0;

uint16_t flagTxRx = 0;
uint16_t cnt_mainIsr = 0;

_iq VDcTrs = 0;
_iq VDcFlt = 0;

uint16_t gHallA = 0;
uint16_t gHallB = 0;
uint16_t gHallC = 0;
uint16_t gHall_GpioData = 0;

uint32_t gCntTimer0 = 0;
uint32_t gCntTimer1 = 0;
uint32_t gCntTimer2 = 0;

float_t estAngle = 0.0;
float_t prevEstAngle = 0.0;
float_t syncAngle = 0.0;
float_t gDeltaAngle = 0.0;
float_t gDeltaAngle1 = 0.0;
float_t gDeltaAngle2 = 0.0;
float_t fm = 0.0;
float_t fe = 0.0;

float_t estSpeed = 0.0;

bool isrFlag = 0;
uint32_t isrCnt1 = 0;
uint32_t isrCnt2 = 0;
uint32_t isrCntDelta = 0;


uint32_t HallACap1 = 0xFFFFFFFF;
uint32_t HallACap2 = 0;
uint32_t HallACap3 = 0;
uint32_t HallACap4 = 0;
uint32_t HallBCap1 = 0;
uint32_t HallBCap2 = 0;
uint32_t HallCCap1 = 0;
uint32_t HallCCap2 = 0;

uint32_t halfPrdPos = 0;
uint32_t halfPrdNeg = 0;

uint16_t gFault = 0;
uint16_t cnt_UDCOL = 0;
uint16_t cnt_UDCUL = 0;
uint16_t cnt_IAOL = 0;
uint16_t cnt_IBOL = 0;
uint16_t cnt_ICOL = 0;
uint16_t cnt_UAOL = 0;
uint16_t cnt_UBOL = 0;
uint16_t cnt_UCOL = 0;

//
CPU_USAGE_Handle cpu_usageHandle;
CPU_USAGE_Obj    cpu_usage;
float_t          gCpuUsagePercentageMin = 0.0;
float_t          gCpuUsagePercentageAvg = 0.0;
float_t          gCpuUsagePercentageMax = 0.0;

MATH_vec2		Udq;
MATH_vec2		Uab;
float_t			gUd; // V

IPARK_Handle 	iparkHandle;
IPARK_Obj 		ipark;
float_t 		gAngle; // degree

SVGEN_Handle	svgenHandle;
SVGEN_Obj		svgen;

ENC_Handle encHandle1,encHandle2;
ENC_Obj enc1,enc2;

HALL_Handle hallHandle;
HALL_Obj hall;
//! brief Define relationships between hallStatus and hallSector
//! hallStatus:HallA MSB ,HallC LSB
//! hallStatus->sector 1->3 2->1 3->2 4->5 5->4 6->6
//! sector forward 1->30 backward 1->90 etc
const uint16_t gHallSectorIndex[6] = {3,1,2,5,4,6};


// **************************************************************************
// the function prototypes

// the interrupt function
interrupt void mainISR(void);

void counter(void);
void protect(void);
void sciTTL(void);

//
void updateGlobalVariables_motor(CTRL_Handle handle);

void updateIqRef(CTRL_Handle handle);

void updateCPUusage(void);

// the uart
void SCIB_TX_PRE(void);
void SCIB_TX(void);

void SCIB_RX(void);
void SCIB_RX_RSL(void);

// the chopper
void chopper(void);

// the test
void test(void);

// the enc
ENC_Handle ENC_init(void *pMemory, const size_t numBytes);
void ENC_setParams(ENC_Handle handle,uint16_t numSlots,float_t smpPrd,uint16_t ccps,uint16_t upps);
void ENC_run(ENC_Handle handle);

// the hall
HALL_Handle HALL_init(void *pMemory, const size_t numBytes);
void HALL_setParams(HALL_Handle handle);
void HALL_run(HALL_Handle handle);

// **************************************************************************
// the functions

void main(void) {

	// Only used if running from FLASH
	// Note that the v+ariable FLASH is defined by the project
#ifdef FLASH
	// Copy time critical code and Flash setup code to RAM
	// The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
#endif

	// initialize the hardware abstraction layer
	halHandle = HAL_init(&hal, sizeof(hal));

	// initialize the user parameters
	USER_setParams(&gUserParams);

	// set the hardware abstraction layer parameters
	HAL_setParams(halHandle, &gUserParams);

	// check for errors in user parameters
	USER_checkForErrors(&gUserParams);

	// store user parameter error in global variable
	gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

	// do not allow code execution if there is a user parameter error
	if (gMotorVars.UserErrorCode != USER_ErrorCode_NoError) {
		for (;;) {
			gMotorVars.Flag_enableSys = false;
		}
	}

	RtTorque = 0.0;

	uint16_t i;
	for(i=0;i<11;i++)
		RX[i] = 0;

	// initialize the controller
	ctrlHandle = CTRL_initCtrl(0, 0);  	//v1p6 format (06xF and 06xM devices)

	controller_obj = (CTRL_Obj *) ctrlHandle;

	CTRL_Version version;

	// get the version number
	CTRL_getVersion(ctrlHandle, &version);

	gMotorVars.CtrlVersion = version;

	// set the default controller parameters
	CTRL_setParams(ctrlHandle, &gUserParams);

	// initialize the ipark handle
	iparkHandle = IPARK_init(&ipark,sizeof(IPARK_Obj));

	// initialize the svgen handle
	svgenHandle = SVGEN_init(&svgen,sizeof(SVGEN_Obj));

	// initialize the cpu usage handle
	cpu_usageHandle = CPU_USAGE_init(&cpu_usage, sizeof(CPU_USAGE_Obj));
	CPU_USAGE_setParams(cpu_usageHandle, HAL_getTimerPeriod(halHandle, 1),
			(uint32_t) USER_ISR_FREQ_Hz);

	// initialize the encoder handle
	encHandle1 = ENC_init(&enc1,sizeof(ENC_Obj));
	ENC_setParams(encHandle1,600,100.0,64,16);

	// initialize the encoder handle
	encHandle2 = ENC_init(&enc2,sizeof(ENC_Obj));
	ENC_setParams(encHandle2,64,10.0,128,2);

	// initialize the hall handle
	hallHandle = HALL_init(&hall,sizeof(HALL_Obj));
	HALL_setParams(hallHandle);

	// setup faults
	HAL_setupFaults(halHandle);

	// initialize the interrupt vector table
	HAL_initIntVectorTable(halHandle);

	// enable the ADC interrupts
	HAL_enableAdcInts(halHandle);

	// enable global interrupts
	HAL_enableGlobalInts(halHandle);

	// enable debug interrupts
	HAL_enableDebugInt(halHandle);

	// disable the PWM
	HAL_disablePwm(halHandle);

	// turn on the DRV8301 if present
	HAL_enableDrv(halHandle);
	// initialize the DRV8301 interface
	HAL_setupDrvSpi(halHandle, &gDrvSpi8301Vars);

	// enable DC bus compensation
	CTRL_setFlag_enableDcBusComp(ctrlHandle, true);

	// compute scaling factors for flux and torque calculations
	gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
	gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
	gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
	gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

	gMotorVars.Flag_enableSys = true;
	gMotorVars.Flag_enableSpeedCtrl = false;
	gMotorVars.Flag_enableUserParams = true;
	gMotorVars.Flag_enableRsRecalc = false;
	gMotorVars.Flag_enableOffsetcalc = true;
	gMotorVars.Flag_enableForceAngle = true;
	gMotorVars.Flag_enablePowerWarp = false;

	for (;;) {

		// Waiting for enable system flag to be set
		while (!(gMotorVars.Flag_enableSys));

		// Dis-able the Library internal PI.  Iq has no reference now
		CTRL_setFlag_enableSpeedCtrl(ctrlHandle,
				gMotorVars.Flag_enableSpeedCtrl);

		// enable/disable the use of motor parameters being loaded from user.h
		CTRL_setFlag_enableUserMotorParams(ctrlHandle,
				gMotorVars.Flag_enableUserParams);

		// enable/disable automatic calculation of bias values
		CTRL_setFlag_enableOffset(ctrlHandle,
				gMotorVars.Flag_enableOffsetcalc);

		// enable or disable power warp
		CTRL_setFlag_enablePowerWarp(ctrlHandle,
				gMotorVars.Flag_enablePowerWarp);

        // enable/disable Rs recalibration during motor startup
		EST_setFlag_enableRsRecalc(ctrlHandle->estHandle,
				gMotorVars.Flag_enableRsRecalc);

		// enable/disable the forced angle
		EST_setFlag_enableForceAngle(ctrlHandle->estHandle,
				gMotorVars.Flag_enableForceAngle);

		// loop while the enable system flag is true
		while (gMotorVars.Flag_enableSys) {

			CTRL_Obj *obj = (CTRL_Obj *) ctrlHandle;

			// increment counters
			gCounter_updateGlobals++;

			if (CTRL_isError(ctrlHandle) )
			{
				// set the enable controller flag to false
				CTRL_setFlag_enableCtrl(ctrlHandle, false);

				// set the enable system flag to false
				gMotorVars.Flag_enableSys = false;

				// disable the PWM
				HAL_disablePwm(halHandle);
			} else {
				// update the controller state
				bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

				// enable or disable the control
				if((!gDrvSpi8301Vars.Stat_Reg_1.FAULT)&&(gStatus!=0x00)&&(gFault == 0))
					gMotorVars.Flag_Run_Identify = true;
				else
				{
					// disable the PWM
					HAL_disablePwm(halHandle);
					gMotorVars.Flag_Run_Identify = false;
				}

				CTRL_setFlag_enableCtrl(ctrlHandle,
						gMotorVars.Flag_Run_Identify);

				if (flag_ctrlStateChanged) {
					CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

					if (ctrlState == CTRL_State_OffLine) {
						// enable the PWM
						HAL_enablePwm(halHandle);
					} else if (ctrlState == CTRL_State_OnLine) {
						HAL_turnLedOn(halHandle, (GPIO_Number_e) HAL_Gpio_LED4);
						if (gMotorVars.Flag_enableOffsetcalc == true) {
							// update the ADC bias values
							HAL_updateAdcBias(halHandle);
						} else {
							// set the current bias
							HAL_setBias(halHandle, HAL_SensorType_Current, 0,
									_IQ(I_A_offset));
							HAL_setBias(halHandle, HAL_SensorType_Current, 1,
									_IQ(I_B_offset));
							HAL_setBias(halHandle, HAL_SensorType_Current, 2,
									_IQ(I_C_offset));

							// set the voltage bias
							HAL_setBias(halHandle, HAL_SensorType_Voltage, 0,
									_IQ(V_A_offset));
							HAL_setBias(halHandle, HAL_SensorType_Voltage, 1,
									_IQ(V_B_offset));
							HAL_setBias(halHandle, HAL_SensorType_Voltage, 2,
									_IQ(V_C_offset));
						}

						// Return the bias value for currents
						gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,
								HAL_SensorType_Current, 0);
						gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,
								HAL_SensorType_Current, 1);
						gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,
								HAL_SensorType_Current, 2);

						// Return the bias value for voltages
						gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,
								HAL_SensorType_Voltage, 0);
						gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,
								HAL_SensorType_Voltage, 1);
						gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,
								HAL_SensorType_Voltage, 2);

						// enable the PWM
						HAL_enablePwm(halHandle);

					} else if (ctrlState == CTRL_State_Idle) {
						HAL_turnLedOff(halHandle, (GPIO_Number_e) HAL_Gpio_LED4);

						// disable the PWM
						HAL_disablePwm(halHandle);
						gMotorVars.Flag_Run_Identify = false;
					}

					if ((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true)
							&& (ctrlState > CTRL_State_Idle)
							&& (gMotorVars.CtrlVersion.minor == 6)) {
						// call this function to fix 1p6
						USER_softwareUpdate1p6(ctrlHandle);
					}

				}
			}

			if (EST_isMotorIdentified(obj->estHandle)) {
				// set the current ramp
				EST_setMaxCurrentSlope_pu(obj->estHandle, gMaxCurrentSlope);
				gMotorVars.Flag_MotorIdentified = true;

				if (Flag_Latch_softwareUpdate) {
					Flag_Latch_softwareUpdate = false;

					USER_calcPIgains(ctrlHandle);
				}

			} else {
				Flag_Latch_softwareUpdate = true;

				// the estimator sets the maximum current slope during identification
				gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
			}

			// when appropriate, update the global variables
			if (gCounter_updateGlobals
					>= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE) {
				// reset the counter
				gCounter_updateGlobals = 0;

				updateGlobalVariables_motor(ctrlHandle);
			}

			if(cnt_mainIsr%1000 == 0) //100ms读一次温度
				HAL_readAdcDataLsp(halHandle, &gAdcData);

			//
			if (gMotorVars.Flag_enableSpeedCtrl) {
				CTRL_setSpd_ref_krpm(ctrlHandle, gMotorVars.SpeedRef_krpm);
			} else {
				// update Iq reference
				updateIqRef(ctrlHandle);
			}

	        // update CPU usage
	        updateCPUusage();

	        //sysctrl
	        //input
	        if(!HAL_readGpio(halHandle,(GPIO_Number_e)HAL_Fan0_Fb))
	        	gSysVars.Ack_Fan0 = true;
	        else
	        	gSysVars.Ack_Fan0 = false;

	        if(!HAL_readGpio(halHandle,(GPIO_Number_e)HAL_Fan1_Fb))
	        	gSysVars.Ack_Fan1 = true;
	        else
	        	gSysVars.Ack_Fan1 = false;

	        //output
	        if(gSysVars.Flag_openVavleLeft)
	        	HAL_openVavle(halHandle,(GPIO_Number_e)HAL_Vavle_L);
	        else
	        	HAL_closeVavle(halHandle,(GPIO_Number_e)HAL_Vavle_L);

	        if(gSysVars.Flag_openVavleRight)
	        	HAL_openVavle(halHandle,(GPIO_Number_e)HAL_Vavle_R);
	        else
	        	HAL_closeVavle(halHandle,(GPIO_Number_e)HAL_Vavle_R);

	        if(gSysVars.Flag_openFanSource)
	        	HAL_enableFanSrc(halHandle,(GPIO_Number_e)HAL_FanSrc);
	        else
	        	HAL_disableFanSrc(halHandle,(GPIO_Number_e)HAL_FanSrc);

	        if(gSysVars.Flag_openFan)
	        	HAL_openFan(halHandle,(GPIO_Number_e)HAL_Fan_All);
	        else
	        	HAL_closeFan(halHandle,(GPIO_Number_e)HAL_Fan_All);

	        //	HAL_resetI2CBus(halHandle,(GPIO_Number_e)HAL_I2CBus_Rst);
	        //	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_I2CBus_Rst);


			HAL_writeDrvData(halHandle, &gDrvSpi8301Vars);

			HAL_readDrvData(halHandle, &gDrvSpi8301Vars);

		} // end of while(gFlag_enableSys) loop

		// disable the PWM
		HAL_disablePwm(halHandle);

		// set the default controller parameters (Reset the control to re-identify the motor)
		CTRL_setParams(ctrlHandle, &gUserParams);
		RtTorque = 0.0;
		gMotorVars.Flag_Run_Identify = false;

	} // end of for(;;) loop

} // end of main() function

interrupt void mainISR(void) {

	// read the timer 1 value and update the CPU usage module
	uint32_t timer1Cnt = HAL_readTimerCnt(halHandle, 1);
	CPU_USAGE_updateCnts(cpu_usageHandle, timer1Cnt);

	//
//	gCntTimer0 = HAL_readTimerCnt(halHandle, 0);
//	gCntTimer1 = HAL_readTimerCnt(halHandle, 1);
//	gCntTimer2 = HAL_readTimerCnt(halHandle, 2);

	//	isrCnt1 = isrCnt2;
	//	isrCnt2 = timer1Cnt;
	//
	//	if(isrCnt1<isrCnt2)
	//		isrCntDelta = isrCnt1 +((uint32_t)(90 * (float_t)10000.0) - 1) - isrCnt2 +1;
	//	else
	//		isrCntDelta = isrCnt1 -isrCnt2 +1;

	// toggle status LED
	if (++gLEDcnt >= (uint_least32_t) (USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz)) {
		HAL_toggleLed(halHandle, (GPIO_Number_e) HAL_Gpio_LED2);
		gLEDcnt = 0;
	}

	////	test();

	// counter
	counter();

	// convert the ADC data
	HAL_readAdcData(halHandle, &gAdcData);

	// HALL
//	hallHandle->tsCounter = halHandle->capHandle[0]->TSCTR;
//	hallHandle->flag_cap1 = CAP_getInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);
//	if(hallHandle->flag_cap1)
//	{
//		gHall_GpioData = (HAL_readGpio(halHandle,GPIO_Number_11) & 0x1)<<2;
//		gHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_15) & 0x1)<<1;
//		gHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_9) & 0x1)<<0;
//		hallHandle->hallSta = gHall_GpioData&(7<<0);
//
//		HallACap1 = CAP_getCap1(halHandle->capHandle[0]);
//		hallHandle->capCounter = HallACap1;
//
//		CAP_clearInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);
//	}
//
//	HALL_run(hallHandle);

//	hallHandle->tsCounter = halHandle->capHandle[1]->TSCTR;
//	hallHandle->flag_cap1 = CAP_getInt(halHandle->capHandle[1],CAP_Int_Type_CEVT1);
//	if(hallHandle->flag_cap1)
//	{
//		gHall_GpioData = (HAL_readGpio(halHandle,GPIO_Number_11) & 0x1)<<2;
//		gHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_15) & 0x1)<<1;
//		gHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_9) & 0x1)<<0;
//		hallHandle->hallSta = gHall_GpioData&(7<<0);
//
//		HallBCap1 = CAP_getCap1(halHandle->capHandle[1]);
//		hallHandle->capCounter = HallBCap1;
//
//		CAP_clearInt(halHandle->capHandle[1],CAP_Int_Type_CEVT1);
//	}
//	HALL_run(hallHandle);


//	hallHandle->tsCounter = halHandle->capHandle[2]->TSCTR;
//	hallHandle->flag_cap1 = CAP_getInt(halHandle->capHandle[2],CAP_Int_Type_CEVT1);
//	if(hallHandle->flag_cap1)
//	{
//		gHall_GpioData = (HAL_readGpio(halHandle,GPIO_Number_11) & 0x1)<<2;
//		gHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_15) & 0x1)<<1;
//		gHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_9) & 0x1)<<0;
//		hallHandle->hallSta = gHall_GpioData&(7<<0);
//
//		HallCCap1 = CAP_getCap1(halHandle->capHandle[2]);
//		hallHandle->capCounter = HallCCap1;
//
//		CAP_clearInt(halHandle->capHandle[2],CAP_Int_Type_CEVT1);
//	}
//
//	HALL_run(hallHandle);

	// encoder
	encHandle1->dirFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_QDF; // 0 ccw, 1 cw

	encHandle1->UTOFlag = QEP_read_interrupt_flag(halHandle->qepHandle[0],QEINT_Uto);
	if(encHandle1->UTOFlag)
	{
		encHandle1->qposlat = QEP_read_posn_latch(halHandle->qepHandle[0]);
		QEP_clear_interrupt_flag(halHandle->qepHandle[0],QEINT_Uto);
	}

	encHandle1->UPEVNTFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_UPEVNT;
	if(encHandle1->UPEVNTFlag)
	{
		encHandle1->qcprdlat = QEP_read_capture_period_latch(halHandle->qepHandle[0]);
		encHandle1->COEFFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_COEF;
		QEP_reset_status(halHandle->qepHandle[0], UPEVNT);
		QEP_reset_status(halHandle->qepHandle[0], COEF);
	}

	ENC_run(encHandle1);

//	encHandle2->dirFlag = QEP_read_status(halHandle->qepHandle[1])&QEP_QEPSTS_QDF; // 0 ccw, 1 cw
//
//	encHandle2->UTOFlag = QEP_read_interrupt_flag(halHandle->qepHandle[1],QEINT_Uto);
//	if(encHandle2->UTOFlag)
//	{
//		encHandle2->qposlat = QEP_read_posn_latch(halHandle->qepHandle[1]);
//		QEP_clear_interrupt_flag(halHandle->qepHandle[1],QEINT_Uto);
//	}
//
//	encHandle2->UPEVNTFlag = QEP_read_status(halHandle->qepHandle[1])&QEP_QEPSTS_UPEVNT;
//	if(encHandle2->UPEVNTFlag)
//	{
//		encHandle2->qcprdlat = QEP_read_capture_period_latch(halHandle->qepHandle[1]);
//		encHandle2->COEFFlag = QEP_read_status(halHandle->qepHandle[1])&QEP_QEPSTS_COEF;
//		QEP_reset_status(halHandle->qepHandle[1], UPEVNT);
//		QEP_reset_status(halHandle->qepHandle[1], COEF);
//	}
//
//	ENC_run(encHandle2);


	//est
	//	estAngle = _IQtoF(EST_getAngle_pu(ctrlHandle->estHandle))*360.0;
	//	float_t temp = estAngle - prevEstAngle;
	//	if(temp<0.0)
	//		temp += 360.0;
	////	estSpeed = temp/360/0.0001*60;
	//	estSpeed = temp/0.0006;
	//	prevEstAngle = estAngle;

	//	estAngle = _IQtoF(EST_getAngle_pu(ctrlHandle->estHandle));
	//	float_t temp = estAngle - prevEstAngle;
	//	if(temp<0.0)
	//		temp += 1.0;
	//	estSpeed = temp/0.0001*60.0;
	//	prevEstAngle = estAngle;

	//	fm = EST_getFm(ctrlHandle->estHandle);
	//	fe = EST_getFe(ctrlHandle->estHandle);

	//	syncAngle += hallHandle->elec_speed_rpm*60.0*hallHandle->sample_period;
	//	if(syncAngle<0.0)
	//		syncAngle += 360.0;
	//	else if(syncAngle>360.0)
	//		syncAngle -= 360.0;

	//	gDeltaAngle1 = estAngle - syncAngle;
	//	gDeltaAngle2 = hallHandle->elec_angle_degree - syncAngle;

	//	gDeltaAngle = hallHandle->elec_angle_degree - estAngle;


	// protect
	protect();

	// chopper
//	chopper();


	// run the controller
	CTRL_run(ctrlHandle, halHandle, &gAdcData, &gPwmData);

	//--------------------------user controller-----------------------//
//	Udq.value[0] = _IQ(gUd/48.0);
//	Udq.value[1] = _IQ(0.0);
//
//	IPARK_setup(iparkHandle,_IQ(gAngle/360.0));
//	IPARK_run(iparkHandle,&Udq,&Uab);
//
//	SVGEN_run(svgenHandle,&Uab,&gPwmData.Tabc);
	//---------------------------------------------------------------//

	// write the PWM compare values
	HAL_writePwmData(halHandle, &gPwmData);
//  HAL_writePwmData(halHandle,&gPwmDataSim);

	// setup the controller
	CTRL_setup(ctrlHandle);

	// SCI
//	sciTTL();

	// read the timer 1 value and update the CPU usage module
	timer1Cnt = HAL_readTimerCnt(halHandle, 1);
	CPU_USAGE_updateCnts(cpu_usageHandle, timer1Cnt);

	// run the CPU usage module
	CPU_USAGE_run(cpu_usageHandle);

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

	return;
} // end of mainISR() function

void updateGlobalVariables_motor(CTRL_Handle handle) {
	CTRL_Obj *obj = (CTRL_Obj *) handle;

	// get the speed estimate
	gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

	// get the torque estimate
	gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle,
			gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

	// get the magnetizing current
	gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

	// get the rotor resistance
	gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

	// get the stator resistance
	gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

	// get the stator inductance in the direct coordinate direction
	gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

	// get the stator inductance in the quadrature coordinate direction
	gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

	// get the flux in V/Hz in floating point
	gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

	// get the flux in Wb in fixed point
	gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

	// get the controller state
	gMotorVars.CtrlState = CTRL_getState(handle);

	// get the estimator state
	gMotorVars.EstState = EST_getState(obj->estHandle);

	// Get the DC buss voltage
	gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,
			_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

	return;
} // end of updateGlobalVariables_motor() function

void updateIqRef(CTRL_Handle handle) {
	_iq iq_ref_A;
	_iq iq_ref;
	_iq iq_step_A = _IQ(0.01);

	if(RtTorque>20.0)
		RtTorque = 20.0;
	else if(RtTorque<0.0)
		RtTorque = 0.0;

//	iq_ref_A = _IQ(-RtTorque/(1.5*USER_MOTOR_NUM_POLE_PAIRS*USER_MOTOR_RATED_FLUX/MATH_TWO_PI));
	iq_ref_A = _IQ(RtTorque/(1.5*USER_MOTOR_NUM_POLE_PAIRS*USER_MOTOR_RATED_FLUX/MATH_TWO_PI));

	if((gMotorVars.IqRef_A + iq_step_A)<iq_ref_A)
		gMotorVars.IqRef_A += iq_step_A;
	else if((gMotorVars.IqRef_A - iq_step_A)>iq_ref_A)
		gMotorVars.IqRef_A -= iq_step_A;

//	gMotorVars.IqRef_A = _IQ(-RtTorque/(1.5*USER_MOTOR_NUM_POLE_PAIRS*USER_MOTOR_RATED_FLUX/MATH_TWO_PI));

//	iq_ref = _IQmpy(iq_ref_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
	iq_ref = _IQmpy(gMotorVars.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));

	// set the speed reference so that the forced angle rotates in the correct direction for startup
	if (_IQabs(gMotorVars.Speed_krpm) < _IQ(0.01)) {
		if (iq_ref < _IQ(0.0)) {
			CTRL_setSpd_ref_krpm(handle, _IQ(-0.01));
		} else if (iq_ref > _IQ(0.0)) {
			CTRL_setSpd_ref_krpm(handle, _IQ(0.01));
		}
	}

	// Set the Iq reference that use to come out of the PI speed control
	CTRL_setIq_ref_pu(handle, iq_ref);

	return;
} // end of updateIqRef() function

void updateCPUusage(void)
{
  uint32_t minDeltaCntObserved = CPU_USAGE_getMinDeltaCntObserved(cpu_usageHandle);
  uint32_t avgDeltaCntObserved = CPU_USAGE_getAvgDeltaCntObserved(cpu_usageHandle);
  uint32_t maxDeltaCntObserved = CPU_USAGE_getMaxDeltaCntObserved(cpu_usageHandle);
  uint16_t pwmPeriod = HAL_readPwmPeriod(halHandle,PWM_Number_1);
  float_t  cpu_usage_den = (float_t)pwmPeriod * (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK * 2.0;

  // calculate the minimum cpu usage percentage
  gCpuUsagePercentageMin = (float_t)minDeltaCntObserved / cpu_usage_den * 100.0;

  // calculate the average cpu usage percentage
  gCpuUsagePercentageAvg = (float_t)avgDeltaCntObserved / cpu_usage_den * 100.0;

  // calculate the maximum cpu usage percentage
  gCpuUsagePercentageMax = (float_t)maxDeltaCntObserved / cpu_usage_den * 100.0;

  return;
} // end of updateCPUusage() function

void SCIB_TX_PRE(void){
	uint16_t i;
	uint16_t xor = 0;
	uint16_t status = 0x55;
	uint16_t request = 0xAA;
	uint16_t id = 0x11;

	if (txSta == 0){ //不在发送状态
		TX[0] = 0x01; //目标地址
		TX[1] = 0x0A; //帧长
		TX[2] = 0x02; //功能码
		TX[3] = status & 0xFF;
		TX[4] = ((uint16_t) (gMotorVars.Torque_Nm * 10.0)) >> 8 & 0xFF;
		TX[5] = ((uint16_t) (gMotorVars.Torque_Nm * 10.0)) & 0xFF;
		TX[6] = request & 0xFF;
		TX[7] = id >> 8 & 0xFF;
		TX[8] = id & 0xFF;
		for (i = 0; i < 9; i++) {
			xor ^= TX[i];
		}
		TX[9] = xor;

		SCI_resetTxFifo(halHandle->sciBHandle);
		SCI_enableTxFifo(halHandle->sciBHandle);

		txSta = 1;
	}
}

void SCIB_TX(void) {

	if(txSta == 1){
		if(SCI_getTxFifoStatus(halHandle->sciBHandle) <= 1){
			halHandle->sciBHandle->SCITXBUF = TX[txPos];
			txPos++;
		}

		if(txPos>=10){
			txSta = 0;
			txPos = 0;
		}
	}
}

void SCIB_RX(void) {
	uint16_t rxTmp = 0;

	if(rxSta == 1){
		if (SCI_getRxFifoStatus(halHandle->sciBHandle) >= 1) {
			rxTmp = halHandle->sciBHandle->SCIRXBUF & 0xFF;
			cnt_Rx = 0;

			if(flag1 == 0){
				rxPos = 0;
				if(rxTmp == 0x02)
					flag1 = 1;
			}else if (flag2 == 0){
				rxPos++;
				if(rxTmp == 0x0B)
					flag2 = 1;
				else
					flag1 = 0;
			}else if(flag3 == 0){
				rxPos++;
				if(rxTmp == 0x01)
					flag3 = 1;
				else
					flag1 = 0;
			}else
				rxPos++;

			if(flag1 == 0){
				flag2 = 0;
				flag3 = 0;
				rxPos = 0;
			}
			else
				RX[rxPos] = rxTmp;
		}

		if(rxPos>=10||cnt_Rx > 1000){
			flag1 = 0;
			flag2 = 0;
			flag3 = 0;
			rxPos = 0;
			rxSta = 0;
		}
	}
}

void SCIB_RX_RSL(void)
{
	uint16_t i = 0;
	uint16_t xor = 0;


	if(rxSta == 0){
		for(i=0;i<10;i++)
			xor ^=RX[i];

		gXor = xor;

//		if (RX[10] == xor)
		if ((RX[0] == 0x02)&&(RX[1] == 0x0B)&&(RX[2] == 0x01)&&(RX[10] == 0xFF))
		{
			RtTorque = (float) (RX[3] << 8 | RX[4])/10.0;
			LiftTorque = (float) (RX[5] << 8 | RX[6])/10.0;
			RollbackTorque = (float) (RX[7] << 8 | RX[8])/10.0;
			gStatus = RX[9];
		}

		SCI_resetRxFifo(halHandle->sciBHandle);
		SCI_clearRxFifoOvf(halHandle->sciBHandle);
		SCI_enableRxFifo(halHandle->sciBHandle);

		rxSta = 1;
	}
}

void chopper(void)
{
	if(gAdcData.dcBus > _IQ(1.28))
		HAL_enableCHOPPER();
	else if(gAdcData.dcBus < _IQ(1.23))
		HAL_disableCHOPPER();

//	if(VDcTrs > _IQ(1.28))
//		HAL_enableCHOPPER();
//	else if(VDcTrs < _IQ(1.23))
//		HAL_disableCHOPPER();
}

void counter(void){
	if (++cnt_mainIsr >= 10000)
		cnt_mainIsr = 0;

	if(cnt_Rx != 65535)
		cnt_Rx ++;

	if(cnt_UDCOL != 65535)
		cnt_UDCOL ++;
	if(cnt_UDCUL != 65535)
		cnt_UDCUL ++;
	if(cnt_IAOL != 65535)
		cnt_IAOL ++;
	if(cnt_IBOL != 65535)
		cnt_IBOL ++;
	if(cnt_ICOL != 65535)
		cnt_ICOL ++;
}

void protect(void){
	if (gAdcData.dcBus > _IQ(1.3)) {
			if (cnt_UDCOL > 5)
				gFault |= BIT_UDCOL;
		} else
			cnt_UDCOL = 0;

		if (gAdcData.dcBus < _IQ(0.9)) {
			if (cnt_UDCUL > 5)
				gFault |= BIT_UDCUL;
		} else
			cnt_UDCUL = 0;

		if ((gAdcData.I.value[0] > _IQ(1.0))||(gAdcData.I.value[0] < _IQ(-1.0))) {
			if (cnt_IAOL > 5)
				gFault |= BIT_IAOL;
		} else
			cnt_IAOL = 0;

		if ((gAdcData.I.value[1] > _IQ(1.0))||(gAdcData.I.value[1] < _IQ(-1.0))) {
			if (cnt_IBOL > 5)
				gFault |= BIT_IBOL;
		} else
			cnt_IBOL = 0;

		if ((gAdcData.I.value[2] > _IQ(1.0))||(gAdcData.I.value[2] < _IQ(-1.0))) {
			if (cnt_ICOL > 5)
				gFault |= BIT_ICOL;
		} else
			cnt_ICOL = 0;
}

void sciTTL(void){
	if(cnt_mainIsr%5000 == 0)
		SCIB_TX_PRE();
	if(cnt_mainIsr%10==0)
		SCIB_TX();

	if(cnt_mainIsr%10==5)
		SCIB_RX();
	if(cnt_mainIsr%5000 == 2500)
		SCIB_RX_RSL();
}

void test(void)
{
	// 右电磁阀
//	HAL_openVavle(halHandle,(GPIO_Number_e)HAL_Vavle_R);
//	HAL_toggleGpio(halHandle,(GPIO_Number_e)HAL_Vavle_R);

//	HAL_openVavle(halHandle,(GPIO_Number_e)HAL_Vavle_L);
//	HAL_toggleGpio(halHandle,(GPIO_Number_e)HAL_Vavle_L);

//	HAL_openFan(halHandle,(GPIO_Number_e)HAL_Fan_All);
//	HAL_toggleGpio(halHandle,(GPIO_Number_e)HAL_Fan_All);

//	HAL_enableFanSrc(halHandle,(GPIO_Number_e)HAL_FanSrc);
//	HAL_disableFanSrc(halHandle,(GPIO_Number_e)HAL_FanSrc);

//	HAL_resetI2CBus(halHandle,(GPIO_Number_e)HAL_I2CBus_Rst);
//	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_I2CBus_Rst);
}

ENC_Handle ENC_init(void *pMemory,const size_t numBytes)
{
	ENC_Handle handle;

	if(numBytes < sizeof(ENC_Obj))
		return((ENC_Handle)NULL);

	handle = (ENC_Handle)pMemory;

	return(handle);
}


void ENC_setParams(ENC_Handle handle,uint16_t numSlots,float_t smpPrd,uint16_t ccps,uint16_t upps)
{
	ENC_Obj *obj = (ENC_Obj*)handle;

	obj->qposlat = 0;
	obj->qcprdlat = 0;
	obj->dirFlag = 0;
	obj->UTOFlag = 0;
	obj->UPEVNTFlag = 0;

	obj->mech_speed_rpm = 0.0;
	obj->mech_speed_radps = 0.0;

	obj->num_enc_slots = numSlots; // 4*80 pulse per revolution
	obj->sample_period = smpPrd; // sysclkout 90MHz,QUPRD 900000 ,unit time 100Hz ,10ms calculated once
//	obj->ccps = 128; // capclock 1/128 sysclkout,90MHz/128
//	obj->upps = 4; // 1/4 qclk
	obj->ccps = ccps; // capclock 1/64 sysclkout,90MHz/64
	obj->upps = upps; // 1/16 qclk

	obj->prev_enc = 0;
	obj->delta_enc = 0;
	obj->rpm_cycle_way = 0.0;
	obj->rpm_freq_way = 0.0;

	return;
}

void ENC_run(ENC_Handle handle)
{
	ENC_Obj *obj = (ENC_Obj*)handle;

	uint16_t dirFlag = handle->dirFlag;
	uint16_t UTOFlag = handle->UTOFlag;
	uint16_t UPEVNTFlag = handle->UPEVNTFlag;
	uint32_t qposlat = handle->qposlat;
	uint16_t qcprdlat = handle->qcprdlat;


	if(!dirFlag)
		obj->dir = 0;
	else
		obj->dir = 1;

	// 测周法
	if(UTOFlag)
	{
		if(!dirFlag)
		{
			if(qposlat < obj->prev_enc)
			{
				obj->delta_enc = obj->prev_enc - qposlat;
				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
			}
			else if(qposlat == obj->prev_enc)
			{
				obj->delta_enc = 0;
				obj->rpm_cycle_way = obj->mech_speed_rpm;
			}
			else
			{
				obj->delta_enc = obj->prev_enc - qposlat + 0xFFFFFFFF;
				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
			}
		}
		else
		{
			if(qposlat > obj->prev_enc)
			{
				obj->delta_enc =  qposlat - obj->prev_enc;
				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
			}
			else if(qposlat == obj->prev_enc)
			{
				obj->delta_enc = 0;
				obj->rpm_cycle_way = obj->mech_speed_rpm;
			}
			else
			{
				obj->delta_enc = qposlat - obj->prev_enc + 0xFFFFFFFF;
				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
			}
		}
		obj->prev_enc = qposlat;
	}

	// 测频法
	if(UPEVNTFlag)
	{
		if(!obj->COEFFlag)
		{
			obj->rpm_freq_way = 60.0*obj->upps/(obj->num_enc_slots*4.0)/(obj->ccps/90.0e6)/qcprdlat;
		}
		else
		{
			obj->rpm_freq_way = 0.0;
		}
	}

	//
//	if(rpm_cycle_way<=0.01)
//		obj->mech_speed_rpm = 0;
//	else if(rpm_cycle_way<=500)
//		obj->mech_speed_rpm = rpm_freq_way;
//	else if(rpm_cycle_way<=3000)
//		obj->mech_speed_rpm = rpm_cycle_way;
//	else
//		obj->mech_speed_rpm = 3000;
}

HALL_Handle HALL_init(void *pMemory, const size_t numBytes)
{
	HALL_Handle handle;

	if(numBytes < sizeof(HALL_Obj))
		return((HALL_Handle)NULL);

	handle = (HALL_Handle)pMemory;

	return(handle);
}

//typedef struct _HALL_Obj_
//{
//	// input
//	uint16_t hallSta;
//	uint32_t capCounter;
//
//	// output
//	float_t elec_angle_degree;
//	float_t elec_speed_radps;
//	float_t elec_speed_rpm;
//	uint16_t dir;
//
//	// parameter
//	float_t sample_period;
//
//	// static
//	uint16_t sector;
//	uint16_t prevSector;
//	bool Flag_secChanged;
//
//}HALL_Obj,*HALL_Handle;

void HALL_setParams(HALL_Handle handle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	obj->hallSta = 5;
	obj->capCounter = 0xFFFFFFFF;
	obj->tsCounter = 0x0;

	obj->elec_angle_degree = 0.0;
	obj->elec_speed_radps = 0.0;
	obj->elec_speed_rpm = 0.0;
	obj->dir = 1;

	obj->sample_period = 0.0001;

	obj->sector = 1;
	obj->prevSector = 1;
	obj->Flag_secChanged = 0;

	return;
}

void HALL_run(HALL_Handle handle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;
//	uint16_t index = obj->hallSta -1;
//	int16_t deltaSec;

//	obj->elec_speed_rpm = 90.0e6/obj->capCounter*60.0;
//	if(obj->elec_speed_rpm > 6900.0)
//		obj->elec_speed_rpm = 6900.0;
//	else if(obj->elec_speed_rpm < 2.0)
//		obj->elec_speed_rpm = 0.0;

//	obj->elec_speed_radps = obj->elec_speed_rpm*MATH_TWO_PI/60.0;

//	if(index>5)
//		return;
//	else
//		obj->sector = gHallSectorIndex[index];

//	if(obj->sector!=obj->prevSector)
//	{
//		obj->Flag_secChanged = 1;
//
//		deltaSec = obj->sector-obj->prevSector;
//
//		if(deltaSec == 1||deltaSec == -5)
//			obj->dir = 1;
//		else if(deltaSec == -1||deltaSec == 5)
//			obj->dir = 0;
//
//		if(obj->dir == 1)
////			obj->elec_angle_degree = obj->sector * 60.0 - 30.0 + obj->tsCounter/obj->capCounter * 360.0;
//		{
//			if(obj->sector == 4)
//				obj->elec_angle_degree = obj->sector * 60.0 - 30.0 + obj->tsCounter/obj->capCounter * 360.0;
//		}
//		else if(obj->dir == 0)
////			obj->elec_angle_degree = obj->sector * 60.0 + 30.0 - obj->tsCounter/obj->capCounter * 360.0;
//		{
//			if(obj->sector == 6)
//				obj->elec_angle_degree = obj->sector * 60.0 + 30.0 - obj->tsCounter/obj->capCounter * 360.0;
//		}
//
//		obj->prevSector = obj->sector;
//	}
//	else
//	{
//		obj->Flag_secChanged = 0;
//
//		if(obj->dir == 1)
//			obj->elec_angle_degree += obj->elec_speed_rpm*60.0*obj->sample_period;
//		else if(obj->dir == 0)
//			obj->elec_angle_degree -= obj->elec_speed_rpm*60.0*obj->sample_period;
//	}

//	if(obj->flag_cap1)
//	{
//		obj->elec_angle_degree = 210.0;
//	}
//	else
//	{
//		obj->elec_angle_degree = 210.0 + 1.0*obj->tsCounter/obj->capCounter*360.0;
//	}

//	if(obj->flag_cap1)
//	{
//		obj->elec_angle_degree = 330.0;
//	}
//	else
//	{
//		obj->elec_angle_degree = 330.0 + 1.0*obj->tsCounter/obj->capCounter*360.0;
//	}

	if(obj->flag_cap1)
	{
		obj->elec_angle_degree = 90.0;
	}
	else
	{
		obj->elec_angle_degree = 90.0 + 1.0*obj->tsCounter/obj->capCounter*360.0;
	}

//	obj->elec_angle_degree += obj->elec_speed_rpm*60.0*obj->sample_period;

	if(obj->elec_angle_degree<0.0)
		obj->elec_angle_degree += 360.0;
	else if(obj->elec_angle_degree>360.0)
		obj->elec_angle_degree -= 360.0;

	float_t tmp;
	if(obj->elec_angle_degree<obj->elec_angle_degree_prev)
		tmp = obj->elec_angle_degree + 360.0 - obj->elec_angle_degree_prev;
	else
		tmp = obj->elec_angle_degree - obj->elec_angle_degree_prev;

	obj->elec_speed_rpm = tmp/360.0/0.0001*60;

	obj->elec_angle_degree_prev = obj->elec_angle_degree;

	return;
}
//@} //defgroup
// end of file

