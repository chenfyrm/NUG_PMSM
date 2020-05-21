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
//! \brief  Using InstaSPIN-FOC only as a torque controller
//!
//! (C) Copyright 2011, Texas Instruments, Inc.
//! \defgroup PROJ_LAB04 PROJ_LAB04
//@{
//! \defgroup PROJ_LAB04_OVERVIEW Project Overview
//!
//! Running InstaSPIN-FOC only as a Torque controller
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"


// **************************************************************************
// the defines

//!
//!
#define LED_BLINK_FREQ_Hz   5

//! \brief Defines the number of main iterations before global variables are updated
//!
#define NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE  1

// **************************************************************************
// the typedefs


// **************************************************************************
// the globals

uint16_t gLEDcnt = 0;

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

//------------------------------------------------------------------------
USER_Params gUserParams;

HAL_Handle halHandle;

HAL_AdcData_t gAdcData;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

CTRL_Handle ctrlHandle;
//-----------------------------------------------------------------------

volatile MOTOR_Vars_t gMotorVars;
volatile SYS_Vars_t gSysVars;

// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;

_iq gIsRef_A = _IQ(0.0);

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
		extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif

//
CPU_USAGE_Handle cpu_usageHandle;
CPU_USAGE_Obj    cpu_usage;
float_t          gCpuUsagePercentageMin = 0.0;
float_t          gCpuUsagePercentageAvg = 0.0;
float_t          gCpuUsagePercentageMax = 0.0;

ENC_Handle encHandle[2];
ENC_Obj enc[2];

HALL_Handle hallHandle;
HALL_Obj hall;

// **************************************************************************
// the function prototypes

// the interrupt function
#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

interrupt void mainISR(void);


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

	//--------------------------------------USER----------------------------------------
	{
		// check for errors in user parameters
		USER_checkForErrors(&gUserParams);

		// store user parameter error in global variable
		gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

		// do not allow code execution if there is a user parameter error
		if (gMotorVars.UserErrorCode != USER_ErrorCode_NoError) {
			for (;;) {
				gSysVars.Flag_enableSys = false;
			}
		}

		// initialize the user parameters
		USER_setParams(&gUserParams);

		SYS_setParams(&gSysVars);

		MOTOR_setParams(&gMotorVars);
	}

	//---------------------------------------HAL-------------------------------------------
	{
		// initialize the hardware abstraction layer
		halHandle = HAL_init(&hal, sizeof(hal));

		// set the hardware abstraction layer parameters
		HAL_setParams(halHandle, &gUserParams);
	}

	//----------------------------------------CTRL------------------------------------------------
	{
		// initialize the controller
		ctrlHandle = CTRL_initCtrl(0, 0);  	//v1p6 format (06xF and 06xM devices)

		CTRL_Version version;

		// get the version number
		CTRL_getVersion(ctrlHandle, &version);

		gMotorVars.CtrlVersion = version;

		// set the default controller parameters
		CTRL_setParams(ctrlHandle, &gUserParams);

		// initialize the cpu usage handle
		cpu_usageHandle = CPU_USAGE_init(&cpu_usage, sizeof(cpu_usage));
		CPU_USAGE_setParams(cpu_usageHandle, HAL_getTimerPeriod(halHandle, 1),
				(uint32_t) USER_ISR_FREQ_Hz);

		// initialize the encoder handle
		encHandle[0] = ENC_init(&enc[0],sizeof(enc[0]));
		ENC_setParams(encHandle[0],4500,23);

		// initialize the encoder handle
		encHandle[1] = ENC_init(&enc[1],sizeof(enc[1]));
		ENC_setParams(encHandle[1],64,1);

		// initialize the hall handle
		hallHandle = HALL_init(&hall,sizeof(hall));
		HALL_setParams(hallHandle);

	}

	//-------------------------------ENABLE INT---------------------------------
	{
		// initialize the interrupt vector table
		HAL_initIntVectorTable(halHandle);

		// enable the ADC interrupts
		HAL_enableAdcInts(halHandle);

		// enable global interrupts
		HAL_enableGlobalInts(halHandle);

		// enable debug interrupts
		HAL_enableDebugInt(halHandle);
	}

	//-------------------------------ENABLE DRV--------------------------------------
	{
		// setup faults
		HAL_setupFaults(halHandle);

		// disable the PWM
		HAL_disablePwm(halHandle);

		// turn on the DRV8301 if present
		HAL_enableDrv(halHandle);
		// initialize the DRV8301 interface
		HAL_setupDrvSpi(halHandle, &gDrvSpi8301Vars);
	}

	//--------------------------------ENABLE SYS-----------------------------------------------
	gSysVars.Flag_enableSys = true;

	//---------------------------- Begin the background loop-------------------------------------
	for (;;) {

//		// Waiting for enable system flag to be set
//		while (!(gSysVars.Flag_enableSys));

		//UART_run();

		// loop while the enable system flag is true
		while (gSysVars.Flag_enableSys) {

			CTRL_Obj *obj = (CTRL_Obj *) ctrlHandle;

			// increment counters
			gCounter_updateGlobals++;

			// enable/disable the use of motor parameters being loaded from user.h
			CTRL_setFlag_enableUserMotorParams(ctrlHandle,
					gMotorVars.Flag_enableUserParams);

			//
			if(EST_getState(obj->estHandle) >= EST_State_MotorIdentified)
			{
				gMotorVars.Flag_enableRsRecalc = false;
			}

			EST_setFlag_enableRsRecalc(ctrlHandle->estHandle,
					gMotorVars.Flag_enableRsRecalc);

			if (CTRL_isError(ctrlHandle) )
			{
				// set the enable controller flag to false
				CTRL_setFlag_enableCtrl(ctrlHandle, false);

				// set the enable system flag to false
				gSysVars.Flag_enableSys = false;

				// disable the PWM
				HAL_disablePwm(halHandle);
			} else {
				// update the controller state
				bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

				// enable or disable the control
				CTRL_setFlag_enableCtrl(ctrlHandle,
						gMotorVars.Flag_enableCtrl);

				if (flag_ctrlStateChanged) {
					CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);
					EST_State_e estState = EST_getState(obj->estHandle);

					if (ctrlState == CTRL_State_OffLine) {
						// Dis-able the Library internal PI.  Iq has no reference now
						CTRL_setFlag_enableSpeedCtrl(ctrlHandle,
								gMotorVars.Flag_enableSpeedCtrl);

						// enable the PWM
						HAL_enablePwm(halHandle);
					} else if (ctrlState == CTRL_State_OnLine) {

						HAL_turnLedOn(halHandle, (GPIO_Number_e) HAL_Gpio_LED4);

						// update the ADC bias values
						HAL_updateAdcBias(halHandle);

						// enable the PWM
						HAL_enablePwm(halHandle);
					} else if (ctrlState == CTRL_State_Idle) {

						HAL_turnLedOff(halHandle, (GPIO_Number_e) HAL_Gpio_LED4);

						// disable the PWM
						HAL_disablePwm(halHandle);
						gMotorVars.Flag_enableCtrl = false;
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


				if (Flag_Latch_softwareUpdate) {
					Flag_Latch_softwareUpdate = false;

					USER_calcPIgains(ctrlHandle);

					PID_setGains(hallHandle->pidHandle_Is, PID_getKp(ctrlHandle->pidHandle_Iq), PID_getKi(ctrlHandle->pidHandle_Iq), PID_getKd(ctrlHandle->pidHandle_Iq));
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

				updateGlobalVariables_motor(ctrlHandle, &gMotorVars);
			}

			if(gSysVars.cnt_mainIsr%1000 == 0) //100ms读一次温度
				HAL_readAdcDataLsp(halHandle, &gAdcData);

//			if(gSysVars.cnt_mainIsr%1000 == 500) //100ms读一次码盘速度
//			{
//				ENC_run(encHandle[0]);
//				ENC_run(encHandle[1]);
//			}

	        // when indentifying recalculate Kp and Ki gains to fix the R/L limitation of 2000.0, and Kp limit to 0.11
	        recalcKpKi(ctrlHandle);

			//
			if (CTRL_getFlag_enableSpeedCtrl(ctrlHandle)) {
				CTRL_setSpd_ref_krpm(ctrlHandle, gMotorVars.SpeedRef_krpm);
			} else {
				// update Iq reference
				updateIqRef(ctrlHandle);
			}


			_iq IsRef_pu = _IQmpy(gIsRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
			HALL_setIs_ref_pu(hallHandle, IsRef_pu);


	        // update CPU usage
	        updateCPUusage();

	        {
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
	        }

			HAL_writeDrvData(halHandle, &gDrvSpi8301Vars);

			HAL_readDrvData(halHandle, &gDrvSpi8301Vars);

		} // end of while(gFlag_enableSys) loop


		// set the default controller parameters (Reset the control to re-identify the motor)
		SYS_setParams(&gSysVars);
		MOTOR_setParams(&gMotorVars);
		CTRL_setParams(ctrlHandle, &gUserParams);

	} // end of for(;;) loop

} // end of main() function

interrupt void mainISR(void) {

	// read the timer 1 value and update the CPU usage module
	uint32_t timer1Cnt = HAL_readTimerCnt(halHandle, 1);
	CPU_USAGE_updateCnts(cpu_usageHandle, timer1Cnt);

	// toggle status LED
	if (++gLEDcnt >= (uint_least32_t) (USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz)) {
		HAL_toggleLed(halHandle, (GPIO_Number_e) HAL_Gpio_LED2);
		gLEDcnt = 0;
	}

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

	// convert the ADC data
	HAL_readAdcData(halHandle, &gAdcData);

	//-------------------------------------- HALL angle initial position-----------------------------------------------
	HALL_checkState(hallHandle, halHandle);

	//--------------------------------------encoder angle position---------------------------------------------------------
	encHandle[0]->qposcnt = halHandle->qepHandle[0]->QPOSCNT;
	encHandle[0]->dirFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_QDF;

	ENC_calcCombElecAngle(encHandle[0]);

	//-------------------------------------------------------------------------------------------------------
	encHandle[1]->qposcnt = halHandle->qepHandle[1]->QPOSCNT;
	encHandle[1]->dirFlag = QEP_read_status(halHandle->qepHandle[1])&QEP_QEPSTS_QDF; // 0 ccw, 1 cw

	ENC_calcMechAngle(encHandle[1]);

	// protect
//	protect();

	// chopper
	chopper(&gAdcData);

	// run the controller
	CTRL_run(ctrlHandle, halHandle, &gAdcData, &gPwmData);

	if((CTRL_getState(ctrlHandle) == CTRL_State_OnLine)&&(EST_getState(ctrlHandle->estHandle) >= EST_State_MotorIdentified))
		HALL_Ctrl_run(hallHandle,ctrlHandle,&gAdcData,&gPwmData);

	// write the PWM compare values
	HAL_writePwmData(halHandle, &gPwmData);

	// setup the controller
	CTRL_setup(ctrlHandle);

	// if we are forcing alignment, using the Rs Recalculation, align the eQEP angle with the rotor angle
	if((EST_getState(ctrlHandle->estHandle) == EST_State_Rs) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
	{
		ENC_setElecInitOffset(encHandle[0], (uint32_t)(HAL_getQepPosnMaximum(halHandle) - HAL_getQepPosnCounts(halHandle)),_IQ(0.0));
	}
	else if(hallHandle->flag_cap)
	{
		ENC_setElecInitOffset(encHandle[0], (uint32_t)(HAL_getQepPosnMaximum(halHandle) - HAL_getQepPosnCounts(halHandle)),hallHandle->elec_initAngle_pu);
	}

	// 分时脉冲计算
	if (++gSysVars.cnt_mainIsr >= 10)
		gSysVars.cnt_mainIsr = 0;

	switch(gSysVars.cnt_mainIsr)
	{

	case 0:
	{
		break;
	}
	case 1:
	{
		break;
	}
	case 2:
	{
		break;
	}
	case 3:
	{
		break;
	}
	case 4:
	{
		break;
	}
	case 5:
	{
		break;
	}
	case 6:
	{
		break;
	}
	case 7:
	{
		break;
	}
	case 8:
	{
		break;
	}
	case 9:
	{
		break;
	}

	}

	// read the timer 1 value and update the CPU usage module
	timer1Cnt = HAL_readTimerCnt(halHandle, 1);
	CPU_USAGE_updateCnts(cpu_usageHandle, timer1Cnt);

	// run the CPU usage module
	CPU_USAGE_run(cpu_usageHandle);

	return;
} // end of mainISR() function


//void updateGlobalVariables_motor(CTRL_Handle handle) {
//	CTRL_Obj *obj = (CTRL_Obj *) handle;
//
//	// get the speed estimate
//	gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);
//
//	// get the stator resistance
//	gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);
//
//	// get the stator inductance in the direct coordinate direction
//	gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);
//
//	// get the stator inductance in the quadrature coordinate direction
//	gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);
//
//	// get the flux in V/Hz in floating point
//	gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);
//
//	// get the controller state
//	gMotorVars.CtrlState = CTRL_getState(handle);
//
//	// get the estimator state
//	gMotorVars.EstState = EST_getState(obj->estHandle);
//
//	// Get the DC buss voltage
//	gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,
//			_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
//
//	return;
//} // end of updateGlobalVariables_motor() function

void updateIqRef(CTRL_Handle handle) {
	_iq iq_ref;

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

//void chopper(void)
//{
//	if(gAdcData.dcBus > _IQ(1.35))
//		HAL_enableCHOPPER();
//	else if(gAdcData.dcBus < _IQ(1.30))
//		HAL_disableCHOPPER();
//}

//void recalcKpKi(CTRL_Handle handle)
//{
//  CTRL_Obj *obj = (CTRL_Obj *)handle;
//  EST_State_e EstState = EST_getState(obj->estHandle);
//
//  if((EST_isMotorIdentified(obj->estHandle) == false) && (EstState == EST_State_Rs))
//    {
//      float_t Lhf = CTRL_getLhf(handle);
//      float_t Rhf = CTRL_getRhf(handle);
//      float_t RhfoverLhf = Rhf/Lhf;
//      _iq Kp = _IQ(0.25*Lhf*USER_IQ_FULL_SCALE_CURRENT_A/(USER_CTRL_PERIOD_sec*USER_IQ_FULL_SCALE_VOLTAGE_V));
//      _iq Ki = _IQ(RhfoverLhf*USER_CTRL_PERIOD_sec);
//
//      // set Rhf/Lhf
//      CTRL_setRoverL(handle,RhfoverLhf);
//
//      // set the controller proportional gains
//      CTRL_setKp(handle,CTRL_Type_PID_Id,Kp);
//      CTRL_setKp(handle,CTRL_Type_PID_Iq,Kp);
//
//      // set the Id controller gains
//      CTRL_setKi(handle,CTRL_Type_PID_Id,Ki);
//      PID_setKi(obj->pidHandle_Id,Ki);
//
//      // set the Iq controller gains
//      CTRL_setKi(handle,CTRL_Type_PID_Iq,Ki);
//      PID_setKi(obj->pidHandle_Iq,Ki);
//    }
//
//  return;
//} // end of recalcKpKi() function

//@} //defgroup
// end of file

