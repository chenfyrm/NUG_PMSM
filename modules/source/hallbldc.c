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
//! \file   modules/hallbldc/src/32b/hallbldc.c
//! \brief  Portable C fixed point code.  These functions define the
//!         Clarke transform module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "hallbldc.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals

//! brief Define relationships between hallStatus and hallSector
//! hallStatus:HallA MSB ,HallC LSB
const uint16_t gHallSectorIndex[6] = {4,2,3,6,5,1};

// **************************************************************************
// the functions

HALLBLDC_Handle HALLBLDC_init(void *pMemory,const size_t numBytes)
{
  HALLBLDC_Handle handle;

  if(numBytes < sizeof(HALLBLDC_Obj))
    return((HALLBLDC_Handle)NULL);

  // assign the handle
  handle = (HALLBLDC_Handle)pMemory;

  return(handle);
} // end of HALLBLDC_init() function

////! \brief
//void HALLBLDC_Ctrl_Run(void)
//{
//	if(gHall_Flag_EnableStartup == true)
//	{
//		gHall_PwmState = gHall_PwmIndex[gHall_State];
//
//		if(_IQabs(speed_est_pu)  < gHall_speed_FastToBldc_low_pu)	// FAST to Hall
//		{
//		  if(gHall_Flag_EnableBldc == false)
//		  {
//			  if(gHall_Bldc_Cnt > 20)
//			  {
//				gHall_Flag_EnableBldc = true;
//				gHall_Bldc_Cnt = 0;
//
//				if(gHall_Flag_CurrentCtrl == true)		// Torque Control Mode
//				{
//					// The following instructions load the parameters for the speed PI
//					// controller.
//					PID_setGains(pidHandle[0],_IQ(0.1),_IQ(0.005),_IQ(0.0));
//
//					// Set the initial condition value for the integrator output to 0
//					PID_setUi(pidHandle[3], _IQmpy(pid[2].Ui, gFast2Hall_Ui_coef));
//				}
//				else		// Speed Control Mode
//				{
//					// The following instructions load the parameters for the speed PI
//					// controller.
//					PID_setGains(pidHandle[0],_IQ(0.1),_IQ(0.005),_IQ(0.0));
//
//					gHall_PwmDuty = pid[2].Ui;
//
//					// Set the initial condition value for the integrator output to 0
//					PID_setUi(pidHandle[0], _IQmpy(pid[2].Ui, gFast2Hall_Spd_coef));
//				}
//			  }
//			  else
//				gHall_Bldc_Cnt++;
//		  }
//		}
//		else if(_IQabs(gHall_speed_fdb_pu) > gHall_speed_BldcToFast_high_pu)			// Hall to FAST
//		{
//		  if(gHall_Flag_EnableBldc == true)
//		  {
//			  if(gHall_Fast_Cnt > 20)
//			  {
//				  gHall_Flag_EnableBldc = false;
//				  gHall_Fast_Cnt = 0;
//
//				  if(gHall_Flag_CurrentCtrl == true)		// Torque Control
//				  {
//					  // The following instructions load the parameters for the speed PI
//					  // controller.
//					  PID_setGains(pidHandle[0],_IQ(2.0),_IQ(0.02),_IQ(0.0));
//
//					  // Set the initial condition value for the integrator output to 0, Id
//					  PID_setUi(pidHandle[1],_IQ(0.0));
//
//					  // Set the initial condition value for the integrator output to 0, Iq
//					  PID_setUi(pidHandle[2], _IQmpy(pid[3].Ui, _IQ(0.25)));
//
//				  }
//				  else				// speed control
//				  {
//					  // The following instructions load the parameters for the speed PI
//					  // controller.
//					  PID_setGains(pidHandle[0],_IQ(2.0),_IQ(0.02),_IQ(0.0));
//
//					  // Set the initial condition value for the integrator output to 0, speed
//					  PID_setUi(pidHandle[0], _IQmpy(gHall_PwmDuty, gHall2Fast_Spd_Coef));
//
//					  // Set the initial condition value for the integrator output to 0, Iq
//					  PID_setUi(pidHandle[2], _IQmpy(gHall_PwmDuty, gHall2Fast_Iq_coef));
//
//					  // Set the initial condition value for the integrator output to 0, Id
//					  PID_setUi(pidHandle[1],_IQmpy(gHall_PwmDuty, _IQ(0.0)));
//				  }
//
//				  PWM_setSocAPulseSrc(hal.pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualZero);
//
//				  HAL_enablePwm(halHandle);
//			  }
//			  else
//				  gHall_Fast_Cnt++;
//		  }
//		}
//
//		if(gHall_Flag_EnableBldc)
//		{
//			angle_pu = angle_est_pu;
//			speed_pu = gHall_speed_fdb_pu;
//
//			if(gHall_Flag_CurrentCtrl == true)		// Torque Control Mode
//			{
//				gHall_BLDC_Is_fdb_pu = gAdcData.I.value[gHall_BLDC_Flag_Is_fdb];
//				gHall_BLDC_Is_ref_pu = speed_pid_out;
//
//				// BLDC current loop
//				PID_run(pidHandle[3],gHall_BLDC_Is_ref_pu,gHall_BLDC_Is_fdb_pu,&gHall_PwmDuty);
//
//				HALLBLDC_Ctrl_PwmSet(gHall_PwmState, gHall_PwmDuty);
//			}
//			else									// Speed Control Mode
//			{
//				gHall_PwmDuty = speed_pid_out;
//				HALLBLDC_Ctrl_PwmSet(gHall_PwmState, gHall_PwmDuty);
//			}
//		}
//		else
//		{
//		  angle_pu = angle_est_pu;
//		  speed_pu = speed_est_pu;
//		}
//	}
//	else	//(gHall_Flag_EnableStartup == false)
//	{
//   	    angle_pu = angle_est_pu;
//		speed_pu = speed_est_pu;
//	}
//
//	return;
//}
//
////! \brief
//void HALLBLDC_State_Check(void)
//{
//// Hall_A, Hall_B, Hall_C
//	gHall_GpioData  = (~HAL_readGpio(halHandle, (GPIO_Number_e)HAL_HallGpio_C) & 0x1) << 2;   //CAP1->J10/J4_1->green
//	gHall_GpioData += (~HAL_readGpio(halHandle, (GPIO_Number_e)HAL_HallGpio_B) & 0x1) << 1;   //CAP2->J10/J4_2->green&white
//	gHall_GpioData += (~HAL_readGpio(halHandle, (GPIO_Number_e)HAL_HallGpio_A) & 0x1);        //CAP3->J10/J4_3->gray&white
//
//	gHall_State = gHall_GpioData;
//
//	if(gHall_State != gHall_PrevState)
//	{
//		gHall_timer_now = HAL_readTimerCnt(halHandle,2);
//		gHall_time_delta_now = gHall_timer_prev - gHall_timer_now;
//		gHall_timer_prev = gHall_timer_now;
//
//		gHall_time_delta = (gHall_time_delta_now + gHall_time_delta_prev)>>1;
//		gHall_time_delta_prev = gHall_time_delta_now;
//
//		gHall_speed_fdb_0p01Hz = gHall_speed_scale/gHall_time_delta;
//		gHall_speed_fdb_pu = gHall_speed_fdb_0p01Hz*gHall_Speed_0p01hz_to_pu_sf;
//
////	   	if(TRAJ_getIntValue(trajHandle_spd) < _IQ(0.0))
////		{
////			gHall_speed_fdb_pu = -gHall_speed_fdb_pu;
////		}
//
//	   	//direction check
//		gHall_State_delta = gHall_PwmIndex[gHall_State] - gHall_PwmIndex[gHall_PrevState];
//
//		if((gHall_State_delta == -1) || (gHall_State_delta == 5))
//		{
//			gHall_dir = 1; 		// positive direction
//		}
//	   	else if((gHall_State_delta == 1) || (gHall_State_delta == -5))
//		{
//			gHall_dir = 2; 		// negative direction
//			gHall_speed_fdb_pu = -gHall_speed_fdb_pu;
//		}
//		else
//		{
//			gHall_dir = 0; 		// direction change
//		}
//
//		//check if dirction is chagned.
//		//if direction is changed, speed feedback is reset.
//		if(gHall_dir != gHall_dir_prev)
//		{
//			gHall_speed_fdb_pu = _IQ(0.0);
//			gHall_dir_change = 1;
//		}
//
//		gHall_LastState = gHall_PrevState;
//		gHall_PrevState = gHall_State;
//		gHall_dir_prev = gHall_dir;
//
//		gHall_Flag_State_Change = true;
//		gHall_PwmCnt = 0;
//	}
//	else
//	{
//		gHall_PwmCnt++;
//		if(gHall_PwmCnt > gHall_PwmCntMax)
//		{
//			gHall_speed_fdb_pu = _IQ(0.0);
//			gHall_PwmCnt = 0;
//		}
//	}
//
//	return;
//}
//
////! \brief
//void HALLBLDC_Ctrl_Stop(void)
//{
//	gHall_Flag_EnableBldc = true;
//	gHall_Flag_State_Change = false;
//	gHall_speed_fdb_pu = _IQ(0.0);
//
//    // Set the initial condition value for the integrator output to 0
//    PID_setUi(pidHandle[0],_IQ(0.0));
//    PID_setUi(pidHandle[1],_IQ(0.0));
//    PID_setUi(pidHandle[2],_IQ(0.0));
//    PID_setUi(pidHandle[3],_IQ(0.0));
//}
//
////! \brief
//void HALLBLDC_Ctrl_PwmSet(uint16_t PwmState, _iq PwmDuty)
//{
//  switch(PwmState)
//  {
//	case 0:		// V+/W-
//	{
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = _IQ(0.0);
//		gPwmData.Tabc.value[1] = PwmDuty;
//		gPwmData.Tabc.value[2] = -PwmDuty;
//
//		gHall_BLDC_Flag_Is_fdb = 1;
//		break;
//	}
//	case 1:		// U+/W-
//	{
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = PwmDuty;
//		gPwmData.Tabc.value[1] = _IQ(0.0);
//		gPwmData.Tabc.value[2] = -PwmDuty;
//
//		gHall_BLDC_Flag_Is_fdb = 0;
//		break;
//	}
//	case 2:		//U+/V-
//	{
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = PwmDuty;
//		gPwmData.Tabc.value[1] = -PwmDuty;
//		gPwmData.Tabc.value[2] = _IQ(0.0);
//
//		gHall_BLDC_Flag_Is_fdb = 0;
//		break;
//	}
//	case 3:		//W+/V-
//	{
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = _IQ(0.0);
//		gPwmData.Tabc.value[1] = -PwmDuty;
//		gPwmData.Tabc.value[2] = PwmDuty;
//
//		gHall_BLDC_Flag_Is_fdb = 2;
//		break;
//	}
//	case 4:		//W+/U-
//	{
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = -PwmDuty;
//		gPwmData.Tabc.value[1] = _IQ(0.0);
//		gPwmData.Tabc.value[2] = PwmDuty;
//
//		gHall_BLDC_Flag_Is_fdb = 2;
//		break;
//	}
//	case 5:		// V+/U-
//	{
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = -PwmDuty;
//		gPwmData.Tabc.value[1] = PwmDuty;
//		gPwmData.Tabc.value[2] = _IQ(0.0);
//
//		gHall_BLDC_Flag_Is_fdb = 1;
//		break;
//	}
//	case 6:		// V+/W-
//	{
//		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
//		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);
//
//		gPwmData.Tabc.value[0] = _IQ(0.0);
//		gPwmData.Tabc.value[1] = PwmDuty;
//		gPwmData.Tabc.value[2] = -PwmDuty;
//
//		gHall_BLDC_Flag_Is_fdb = 1;
//		break;
//	}
//	default:	// N/A
//		break;
//  }
//}

// end of file
