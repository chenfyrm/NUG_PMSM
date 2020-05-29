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
	HALL_Obj *obj;

	if(numBytes < sizeof(HALL_Obj))
		return((HALL_Handle)NULL);

	handle = (HALL_Handle)pMemory;

	obj = (HALL_Obj*)handle;

	obj->pidHandle_Is = PID_init(&obj->pid_Is,sizeof(PID_Obj));

//	obj->pidHandle_spd = PID_init(&obj->pid_spd,sizeof(PID_Obj));

	return(handle);
}

void HALL_setParams(HALL_Handle handle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	_iq Kp,Ki,Kd;
	_iq outMin,outMax;

	obj->hallSta = 5;
	obj->capCounter = 0xFFFFFFFF;
	obj->tsCounter = 0x0;
	obj->flag_cap = false;

	obj->sector = 4;
	obj->prevSector = 4;
	obj->Flag_secChanged = false;
	obj->dir = 1;

	obj->elec_initAngle_pu = _IQ(0.0);
//	obj->elec_speed_pu = _IQ(0.0);

//	obj->flag_enableCurrentCtrl = true;
	obj->flag_enableSpeedCtrl = false;
	obj->flag_enableBldc = false;

	obj->bldcCnt = 0;
	obj->focCnt = 0;

	obj->Is_ref_pu = _IQ(0.0);
	obj->Is_fdb_sel = 0;
	obj->Is_fbk_pu = _IQ(0.0);

//	obj->speed_ref_pu = _IQ(0.0);
	obj->speed_fbk_pu = _IQ(0.0);

	obj->speed_BldcToFoc_high_pu = _IQ(300.0*23/(60.0*200.0));
	obj->speed_FocToBldc_low_pu = _IQ(250.0*23/(60.0*200.0));


	// set the default the Is PID controller parameters
	Kp = _IQ(0.1);
	Ki = _IQ(0.025);
	Kd = _IQ(0.0);
	outMin = _IQ(-0.4);
	outMax = _IQ(0.4);

	PID_setGains(obj->pidHandle_Is,Kp,Ki,Kd);
	PID_setUi(obj->pidHandle_Is,_IQ(0.0));
	PID_setMinMax(obj->pidHandle_Is,outMin,outMax);

//	// set the default the speed PID controller parameters
//	Kp = _IQ(0.1);
//	Ki = _IQ(0.025);
//	Kd = _IQ(0.0);
//	outMin = _IQ(-0.4);
//	outMax = _IQ(0.4);
//
//	PID_setGains(obj->pidHandle_spd,Kp,Ki,Kd);
//	PID_setUi(obj->pidHandle_spd,_IQ(0.0));
//	PID_setMinMax(obj->pidHandle_spd,outMin,outMax);

	return;
}

//#ifdef FLASH
//#pragma CODE_SECTION(HALL_checkState,"ramfuncs");
//#endif

void HALL_checkState(HALL_Handle handle, HAL_Handle halHandle)
{
	HALL_Obj *obj = (HALL_Obj*)handle;

	int16_t deltaSec;
	uint16_t pHall_GpioData;

	pHall_GpioData = (HAL_readGpio(halHandle,GPIO_Number_11) & 0x1)<<2;
	pHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_15) & 0x1)<<1;
	pHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_9) & 0x1)<<0;

	hallHandle->hallSta = pHall_GpioData&(7<<0);

	hallHandle->flag_cap = CAP_getInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);

	if(hallHandle->flag_cap)
	{

		hallHandle->tsCounter = halHandle->capHandle[0]->TSCTR;
		hallHandle->capCounter = CAP_getCap1(halHandle->capHandle[0]);

		CAP_clearInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);
	}

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

//		if(obj->dir == 1)
//			obj->elec_initAngle_pu = _IQ(0.1666667*obj->sector - 0.08333333 + 1.0*obj->tsCounter/obj->capCounter);
//		else if(obj->dir == 0)
//			obj->elec_initAngle_pu = _IQ(0.1666667*obj->sector + 0.08333333 - 1.0*obj->tsCounter/obj->capCounter);
//
//		obj->elec_initAngle_pu &= ((uint32_t) 0x00ffffff);

		obj->prevSector = obj->sector;
	}
	else
		obj->Flag_secChanged = false;

	if(obj->flag_cap){
		if(obj->dir == 1)
			obj->elec_initAngle_pu = _IQ(0.5833333 + 1.0*obj->tsCounter/obj->capCounter);
		else if(obj->dir == 0)
			obj->elec_initAngle_pu = _IQ(0.0833333 - 1.0*obj->tsCounter/obj->capCounter);

		obj->elec_initAngle_pu &= ((uint32_t) 0x00ffffff);
	}

//	obj->elec_speed_pu = _IQ(4.5e5/obj->capCounter); //cpu usage 60%

	return;
}

//#ifdef FLASH
//#pragma CODE_SECTION(HALL_Ctrl_run,"ramfuncs");
//#endif

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

//#ifdef FLASH
//#pragma CODE_SECTION(HALL_Ctrl_PwmSet,"ramfuncs");
//#endif

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

		obj->Is_fdb_sel = 1;//B相电流
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

//void HALL_run(HALL_Handle handle)
//{
//	HALL_Obj *obj = (HALL_Obj*)handle;
//
//	uint16_t index = obj->hallSta -1;
//	int16_t deltaSec;
//
//	if(index>5)
//		return;
//	else
//		obj->sector = gHallSectorIndex[index];
//
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
//			obj->elec_angle_pu = _IQ(0.1666667*obj->sector - 0.08333333 + 1.0*obj->tsCounter/obj->capCounter);
//		else if(obj->dir == 0)
//			obj->elec_angle_pu = _IQ(0.1666667*obj->sector + 0.08333333 - 1.0*obj->tsCounter/obj->capCounter);
//
//		obj->elec_angle_pu &= ((uint32_t) 0x00ffffff);
//
////		if(obj->dir == 1)
////			obj->trigCounter ++;
////		else if(obj->dir == 0)
////			obj->trigCounter --;
////
////		if(obj->trigCounter == 0x00ffffff)
////			obj->trigCounter = 0;
////		else if(obj->trigCounter == 0)
////			obj->trigCounter = 0x00ffffff;
//
//		obj->prevSector = obj->sector;
//	}
//	else
//		obj->Flag_secChanged = 0;
//
////	if(obj->ticker++>=1000)
////	{
////		obj->ticker = 0;
////
////		if(obj->dir == 1)
////		{
////			if(obj->trigCounter >= obj->trigCounter_prev)
////				obj->elec_speed_rpm = 1.0*(obj->trigCounter - obj->trigCounter_prev)/6.0/0.1; //deltaCounter/6.0/0.1  速度采样 0.1s 360电角度6个脉冲 精度1.667rpm
////			else
////				obj->elec_speed_rpm = 1.0*(0x00ffffff + obj->trigCounter - obj->trigCounter_prev )/6.0/0.1;
////		}
////		else
////		{
////			if(obj->trigCounter <= obj->trigCounter_prev)
////				obj->elec_speed_rpm = 1.0*(obj->trigCounter_prev - obj->trigCounter)/6.0/0.1; //deltaCounter/6.0/0.1  速度采样 0.1s 360电角度6个脉冲 精度1.667rpm
////			else
////				obj->elec_speed_rpm = 1.0*(0x00ffffff + obj->trigCounter_prev - obj->trigCounter)/6.0/0.1;
////		}
////
////		obj->trigCounter_prev = obj->trigCounter;
////	}
//
//	return;
//}

//void HALL_run(HALL_Handle handle)
//{
//	HALL_Obj *obj = (HALL_Obj*)handle;
//	uint16_t index = obj->hallSta -1;
//	int16_t deltaSec;
//
////	obj->elec_speed_rpm = 90.0e6/obj->capCounter*60.0;
////	if(obj->elec_speed_rpm > 6900.0)
////		obj->elec_speed_rpm = 6900.0;
////	else if(obj->elec_speed_rpm < 2.0)
////		obj->elec_speed_rpm = 0.0;
//
////	obj->elec_speed_radps = obj->elec_speed_rpm*MATH_TWO_PI/60.0;
//
//	if(index>5)
//		return;
//	else
//		obj->sector = gHallSectorIndex[index];
//
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
//			obj->elec_angle_degree = obj->sector * 60.0 - 30.0 + obj->tsCounter/obj->capCounter * 360.0;
////		{
////			if(obj->sector == 4)
////				obj->elec_angle_degree = obj->sector * 60.0 - 30.0 + obj->tsCounter/obj->capCounter * 360.0;
////		}
//		else if(obj->dir == 0)
//			obj->elec_angle_degree = obj->sector * 60.0 + 30.0 - obj->tsCounter/obj->capCounter * 360.0;
////		{
////			if(obj->sector == 6)
////				obj->elec_angle_degree = obj->sector * 60.0 + 30.0 - obj->tsCounter/obj->capCounter * 360.0;
////		}
//
////		if(obj->dir == 1)
////			obj->trigCounter++;
////		else
////			obj->trigCounter--;
////
////		obj->trigCounter &= 0x00FFFFFF;
//
//		obj->prevSector = obj->sector;
//	}
//	else
//	{
//		obj->Flag_secChanged = 0;
//
////		if(obj->dir == 1)
////			obj->elec_angle_degree += obj->elec_speed_rpm*60.0*obj->sample_period;
////		else if(obj->dir == 0)
////			obj->elec_angle_degree -= obj->elec_speed_rpm*60.0*obj->sample_period;
//	}
//
//
//
//
////	if(obj->flag_cap1)
////	{
////		obj->elec_angle_degree = 210.0;
////	}
////	else
////	{
////		obj->elec_angle_degree = 210.0 + 1.0*obj->tsCounter/obj->capCounter*360.0;
////	}
//
////	if(obj->flag_cap1)
////	{
////		obj->elec_angle_degree = 330.0;
////	}
////	else
////	{
////		obj->elec_angle_degree = 330.0 + 1.0*obj->tsCounter/obj->capCounter*360.0;
////	}
//
////	if(obj->flag_cap1)
////	{
////		obj->elec_angle_degree = 90.0;
////	}
////	else
////	{
////		obj->elec_angle_degree = 90.0 + 1.0*obj->tsCounter/obj->capCounter*360.0;
////	}
//
////	obj->elec_angle_degree += obj->elec_speed_rpm*60.0*obj->sample_period;
//
////	if(obj->elec_angle_degree<0.0)
////		obj->elec_angle_degree += 360.0;
////	else if(obj->elec_angle_degree>360.0)
////		obj->elec_angle_degree -= 360.0;
////
////	float_t tmp;
////	if(obj->elec_angle_degree<obj->elec_angle_degree_prev)
////		tmp = obj->elec_angle_degree + 360.0 - obj->elec_angle_degree_prev;
////	else
////		tmp = obj->elec_angle_degree - obj->elec_angle_degree_prev;
////
////	obj->elec_speed_rpm = tmp/360.0/0.0001*60;
////
////	obj->elec_angle_degree_prev = obj->elec_angle_degree;
//
//	return;
//}

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
