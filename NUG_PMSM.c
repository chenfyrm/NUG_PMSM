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
//! \brief  Using InstaSPIN�FOC only as a torque controller
//!
//! (C) Copyright 2011, Texas Instruments, Inc.
//! \defgroup PROJ_LAB04 PROJ_LAB04
//@{
//! \defgroup PROJ_LAB04_OVERVIEW Project Overview
//!
//! Running InstaSPIN�FOC only as a Torque controller
//!
// **************************************************************************
// the includes
// system includes
#include "NUG_main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5

// **************************************************************************
// the globals

uint16_t gLEDcnt = 0;

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

HAL_Handle halHandle;

USER_Params gUserParams;

HAL_AdcData_t gAdcData;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

CTRL_Handle ctrlHandle;
CTRL_Obj *controller_obj;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
HAL_PwmData_t gPwmDataSim = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
		extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
		extern uint16_t *econst_start, *econst_end, *econst_ram_load;
		extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
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

void SCIB_TX_PRE(void);
void SCIB_TX(void);

void SCIB_RX(void);
void SCIB_RX_RSL(void);
//void SCIB_RX_INIT(void);

void chopper(void);


//void SCIB_RX_INIT(void)
//{
//	HAL_Obj *obj = (HAL_Obj *)halHandle;
//
//	SCI_resetRxFifo(obj->sciBHandle);
//	SCI_clearRxFifoOvf(obj->sciBHandle);
//	SCI_enableRxFifo(obj->sciBHandle);
//}

void SCIB_TX_PRE(void){
	uint16_t i;
	uint16_t xor = 0;
	uint16_t status = 0x55;
	uint16_t request = 0xAA;
	uint16_t id = 0x11;

	//���ڷ���״̬
	if(txSta == 0){
		TX[0] = 0x01;//Ŀ���ַ
		TX[1] = 0x0A;//֡��
		TX[2] = 0x02;//������
		TX[3] = status&0xFF;
		TX[4] = ((uint16_t)(gMotorVars.Torque_Nm*10.0))>>8&0xFF;
		TX[5] = ((uint16_t)(gMotorVars.Torque_Nm*10.0))&0xFF;
		TX[6] = request&0xFF;
		TX[7] = id>>8&0xFF;
		TX[8] = id&0xFF;
		for(i=0;i<9;i++)
		{
			xor ^=TX[i];
		}
		TX[9] = xor;

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

			RX[rxPos] = rxTmp;
		}

		if(rxPos>=10){
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

		if (RX[10] == xor) {
			RtTorque = (float) (RX[3] << 8 | RX[4])/10.0;
			LiftTorque = (float) (RX[5] << 8 | RX[6])/10.0;
			RollbackTorque = (float) (RX[7] << 8 | RX[8])/10.0;
			gStatus = RX[9];
		}

		rxSta = 1;
	}
}

void chopper(void)
{
	if(gAdcData.dcBus > _IQ(1.1))
		HAL_enableCHOPPER();
	else if(gAdcData.dcBus < _IQ(1.05))
		HAL_disableCHOPPER();
}

// **************************************************************************
// the functions

void main(void) {
//	uint16_t i;

	// Only used if running from FLASH
	// Note that the v+ariable FLASH is defined by the project
#ifdef FLASH
	// Copy time critical code and Flash setup code to RAM
	// The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);

#ifdef CSM_ENABLE
	//copy .econst to unsecure RAM
	if(*econst_end - *econst_start)
	{
		memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
	}

	//copy .switch ot unsecure RAM
	if(*switch_end - *switch_start)
	{
		memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
	}
#endif
#endif

	// initialize the hardware abstraction layer
	halHandle = HAL_init(&hal, sizeof(hal));

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

	// initialize the user parameters
	USER_setParams(&gUserParams);

	// set the hardware abstraction layer parameters
	HAL_setParams(halHandle, &gUserParams);
	RtTorque = 0.0;

	// initialize the controller
	ctrlHandle = CTRL_initCtrl(0, 0);  	//v1p6 format (06xF and 06xM devices)

	controller_obj = (CTRL_Obj *) ctrlHandle;

	CTRL_Version version;

	// get the version number
	CTRL_getVersion(ctrlHandle, &version);

	gMotorVars.CtrlVersion = version;

	// set the default controller parameters
	CTRL_setParams(ctrlHandle, &gUserParams);

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

	HAL_turnLedOff(halHandle, (GPIO_Number_e) HAL_Gpio_LED4);

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
				if((!gDrvSpi8301Vars.Stat_Reg_1.FAULT)&&(gStatus!=0x00))
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

			//
			if (gMotorVars.Flag_enableSpeedCtrl) {
				CTRL_setSpd_ref_krpm(ctrlHandle, gMotorVars.SpeedRef_krpm);
			} else {
				// update Iq reference
				updateIqRef(ctrlHandle);
			}



#ifdef DRV8301_SPI
			HAL_writeDrvData(halHandle, &gDrvSpi8301Vars);

			HAL_readDrvData(halHandle, &gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
			HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

			HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif
		} // end of while(gFlag_enableSys) loop

		// disable the PWM
		HAL_disablePwm(halHandle);

		// set the default controller parameters (Reset the control to re-identify the motor)
		CTRL_setParams(ctrlHandle, &gUserParams);
		gMotorVars.Flag_Run_Identify = false;

		RtTorque = 0.0;

	} // end of for(;;) loop

} // end of main() function

interrupt void mainISR(void) {
	// toggle status LED
	if (++gLEDcnt >= (uint_least32_t) (USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz)) {
		HAL_toggleLed(halHandle, (GPIO_Number_e) HAL_Gpio_LED2);
		gLEDcnt = 0;
	}

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

	// convert the ADC data
	HAL_readAdcData(halHandle, &gAdcData);

	/********************************protect**********************************/

	/******************************************************************/

	/******************************inverter************************************/
	// run the controller
	CTRL_run(ctrlHandle, halHandle, &gAdcData, &gPwmData);

	if (cnt_mainIsr < 10000)
		cnt_mainIsr++;
	else
		cnt_mainIsr = 0;

	if(cnt_mainIsr%5000 == 0)
		SCIB_TX_PRE();
	if(cnt_mainIsr%10==0)
		SCIB_TX();

	if(cnt_mainIsr%10==5)
		SCIB_RX();
	if(cnt_mainIsr%5000 == 2500)
		SCIB_RX_RSL();

	// setup the controller
	CTRL_setup(ctrlHandle);
	/**********************************************************************/

	/*******************************chopper***************************************/
	chopper();
	/**********************************************************************/

	// write the PWM compare values
	HAL_writePwmData(halHandle, &gPwmData);
//  HAL_writePwmData(halHandle,&gPwmDataSim);

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
//	_iq iq_ref_A;
//	_iq iq_ref;

	if(RtTorque>20.0)
		RtTorque = 20.0;

	gMotorVars.IqRef_A = _IQ(-RtTorque/(1.5*USER_MOTOR_NUM_POLE_PAIRS*USER_MOTOR_RATED_FLUX/MATH_TWO_PI));

//	iq_ref = _IQmpy(iq_ref_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));


	_iq iq_ref = _IQmpy(gMotorVars.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));

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

//@} //defgroup
// end of file
