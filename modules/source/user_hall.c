/*
 * hall.c
 *
 *  Created on: 2020-5-6
 *      Author: 700363
 */


// **************************************************************************
// the includes

#include "user_hall.h"

// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

// **************************************************************************
// the globals

//! brief Define relationships between hallStatus and hallSector
//! hallStatus:HallA MSB ,HallC LSB
//! hallStatus->sector: 1->3 2->1 3->2 4->5 5->4 6->6
//! sector->degree: forward 1->30 backward 1->90 etc
const uint16_t gHallSectorIndex[8] = {6,3,1,2,5,4,6,3};

// **************************************************************************
// the function prototypes

// **************************************************************************
// the functions

HALL_Handle HALL_init(void *pMemory, const size_t numBytes)
{
	HALL_Handle handle;

	if(numBytes < sizeof(HALL_Obj))
		return((HALL_Handle)NULL);

	handle = (HALL_Handle)pMemory;

	return(handle);
}

void HALL_setParams(HALL_Handle handle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	obj->hallSta = 5;
	obj->capCounter = 0xFFFFFFFF;
	obj->tsCounter = 0x0;
	obj->flag_cap = false;

	obj->sector = 4;
	obj->prevSector = 4;
	obj->Flag_secChanged = false;
	obj->dir = 1;

	obj->elec_initAngle_pu = _IQ(0.0);

	obj->flag_enableSpeedCtrl = false;
	obj->flag_enableBldc = false;

	obj->bldcCnt = 0;
	obj->focCnt = 0;

	obj->Is_ref_pu = _IQ(0.0);
	obj->Is_fdb_sel = 0;
	obj->Is_fbk_pu = _IQ(0.0);

	obj->speed_fbk_pu = _IQ(0.0);

	obj->speed_BldcToFoc_high_pu = _IQ(300.0*23/(60.0*200.0));
	obj->speed_FocToBldc_low_pu = _IQ(250.0*23/(60.0*200.0));


	return;
}

void HALL_checkState(HALL_Handle handle, HAL_Handle halHandle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	int16_t deltaSec;
	uint16_t pHall_GpioData;

	pHall_GpioData = (HAL_readGpio(halHandle,GPIO_Number_11) & 0x1)<<2;
	pHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_15) & 0x1)<<1;
	pHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_9) & 0x1)<<0;

	hallHandle->hallSta = pHall_GpioData&(7<<0);

	if(obj->hallSta > 7)
		return;
	else
		obj->sector = gHallSectorIndex[obj->hallSta];

	if(obj->sector!=obj->prevSector)
	{
		obj->Flag_secChanged = true;

		deltaSec = obj->sector-obj->prevSector;

		if(deltaSec == 1||deltaSec == -5)
			obj->dir = 1;
		else if(deltaSec == -1||deltaSec == 5)
			obj->dir = 0;

		obj->prevSector = obj->sector;
	}
	else
		obj->Flag_secChanged = false;

	hallHandle->flag_cap = CAP_getInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);

	if(hallHandle->flag_cap)
	{

		hallHandle->tsCounter = halHandle->capHandle[0]->TSCTR;
		hallHandle->capCounter = CAP_getCap1(halHandle->capHandle[0]);

		CAP_clearInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);

		if(obj->dir == 1)
			obj->elec_initAngle_pu = _IQ(0.5833333 + 1.0*obj->tsCounter/obj->capCounter);
		else if(obj->dir == 0)
			obj->elec_initAngle_pu = _IQ(0.0833333 - 1.0*obj->tsCounter/obj->capCounter);

		obj->elec_initAngle_pu &= ((uint32_t) 0x00ffffff);
	}

	return;
}

void HALL_calcElecSpd(HALL_Handle handle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;
	float_t temp;

	temp = 5.4e6/obj->capCounter; // 90e6*60/1000  kRPM
	obj->elec_speed_kRPM = _IQ(temp);

	return;
}

void HALL_Ctrl_run(HALL_Handle handle, CTRL_Handle ctrlHandle, const HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	obj->speed_fbk_pu = EST_getFe_pu(ctrlHandle->estHandle);

	if(_IQabs(obj->speed_fbk_pu)  < obj->speed_FocToBldc_low_pu)
	{
		if(obj->flag_enableBldc == false)
		{
			if(obj->bldcCnt > 20)
			{
				obj->flag_enableBldc = true;
				obj->bldcCnt = 0;

				PID_setUi(obj->pidHandle_Is, _IQ(0.0));
			}
			else
				obj->bldcCnt++;
		}
	}
	else if(_IQabs(obj->speed_fbk_pu)  > obj->speed_BldcToFoc_high_pu)
	{
		if(obj->flag_enableBldc == true)
		{
			if(obj->focCnt > 20)
			{
				obj->flag_enableBldc = false;
				obj->focCnt = 0;

				PID_setUi(ctrlHandle->pidHandle_Id, _IQ(0.0));
				PID_setUi(ctrlHandle->pidHandle_Iq, _IQ(0.0));

				PWM_setSocAPulseSrc(hal.pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualZero);

				HAL_enablePwm((HAL_Handle)&hal);
			}
			else
				obj->focCnt++;
		}
	}

	if(obj->flag_enableBldc)
	{
		_iq Is_ref_pu = obj->Is_ref_pu;
		_iq Is_fdb_pu = pAdcData->I.value[obj->Is_fdb_sel];

		if(obj->flag_enableSpeedCtrl)
		{
			Is_ref_pu = ctrlHandle->spd_out;
		}

		obj->Is_ref_pu = Is_ref_pu;
		obj->Is_fbk_pu = Is_fdb_pu;

		// BLDC current loop
		PID_run(obj->pidHandle_Is,Is_ref_pu,Is_fdb_pu,&obj->PwmDuty);

		HALL_Ctrl_PwmSet(handle, pPwmData);
	}

	return;
}


//! \brief
void HALL_Ctrl_PwmSet(HALL_Handle handle, HAL_PwmData_t *pPwmData)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	//----------------------------1-6-------------------------------
	switch(obj->sector)
	{
	//-------------------------------sector 1 30-90 BA----------------------------------
	case 1:// V+/U-
	{
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_3]);

		pPwmData->Tabc.value[0] = -obj->PwmDuty;
		pPwmData->Tabc.value[1] = obj->PwmDuty;
		pPwmData->Tabc.value[2] = _IQ(0.0);

		obj->Is_fdb_sel = 1;//BÏàµçÁ÷
		break;
	}
	//-------------------------------sector 2 90-150 CA----------------------------------
	case 2:// W+/U-
	{
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_2]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

		pPwmData->Tabc.value[0] = -obj->PwmDuty;
		pPwmData->Tabc.value[1] = _IQ(0.0);
		pPwmData->Tabc.value[2] = obj->PwmDuty;

		obj->Is_fdb_sel = 2;
		break;
	}
	//-------------------------------sector 3 150-210 CB----------------------------------
	case 3:// W+/V-
	{
		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

		pPwmData->Tabc.value[0] = _IQ(0.0);
		pPwmData->Tabc.value[1] = -obj->PwmDuty;
		pPwmData->Tabc.value[2] = obj->PwmDuty;

		obj->Is_fdb_sel = 2;
		break;
	}
	//-------------------------------sector 4 210-270 AB----------------------------------
	case 4:// U+/V-
	{
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_3]);

		pPwmData->Tabc.value[0] = obj->PwmDuty;
		pPwmData->Tabc.value[1] = -obj->PwmDuty;
		pPwmData->Tabc.value[2] = _IQ(0.0);

		obj->Is_fdb_sel = 0;
		break;
	}
	//-------------------------------sector 5 270-330 AC----------------------------------
	case 5:// U+/W-
	{
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_2]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

		pPwmData->Tabc.value[0] = obj->PwmDuty;
		pPwmData->Tabc.value[1] = _IQ(0.0);
		pPwmData->Tabc.value[2] = -obj->PwmDuty;

		obj->Is_fdb_sel = 0;
		break;
	}
	//-------------------------------sector 6 330-30 BC----------------------------------
	case 6:// V+/W-
	{
		PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
		PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

		pPwmData->Tabc.value[0] = _IQ(0.0);
		pPwmData->Tabc.value[1] = obj->PwmDuty;
		pPwmData->Tabc.value[2] = -obj->PwmDuty;

		obj->Is_fdb_sel = 1;
		break;
	}
	default:	// N/A
		break;
	}

	return;
}
