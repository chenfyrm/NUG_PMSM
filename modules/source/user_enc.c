/*
 * enc.c
 *
 *  Created on: 2020-5-6
 *      Author: 700363
 */

// **************************************************************************
// the includes

#include "user_enc.h"

// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

// **************************************************************************
// the functions

ENC_Handle ENC_init(void *pMemory,const size_t numBytes)
{
	ENC_Handle handle;

	if(numBytes < sizeof(ENC_Obj))
		return((ENC_Handle)NULL);

	handle = (ENC_Handle)pMemory;

	return(handle);
}


void ENC_setParams(ENC_Handle handle, uint16_t numSlots, uint16_t polePairs)
{
	ENC_Obj *obj = (ENC_Obj*)handle;

	obj->qposcnt = 0;
	obj->dirFlag = 0;

	obj->elec_angle_pu = _IQ(0.0);
	obj->elec_angle_pu_flt = _IQ(0.0);
	obj->mech_pos_pu = _IQ(0.0);
	obj->mech_speed_kRPM = _IQ(0.0);

	obj->num_enc_slots = numSlots;
	obj->num_pole_pairs = polePairs;
	obj->mech_angle_gain = (_iq)((((uint32_t)1)<<24)/(4*numSlots)); // 4*(600*7.5) pulse per revolution
	obj->sample_time = _IQ(0.001);

	obj->elec_init_offset = 0;
	obj->elec_init_angle = _IQ(0.0);

	obj->mech_init_offset = 0;

	obj->mech_prevPos_pu = _IQ(0.0);

	return;
}


void ENC_calcElecAngle(ENC_Handle handle)
{
	ENC_Obj *obj = (ENC_Obj *) handle;
	uint32_t temp;

	temp = (obj->elec_init_offset + obj->qposcnt)*obj->mech_angle_gain;
	temp *= obj->num_pole_pairs;
	temp += obj->elec_init_angle;
	temp &= ((uint32_t) 0x00ffffff);

	obj->elec_angle_pu = (_iq)temp;
	obj->elec_angle_pu_flt = _IQmpy(obj->elec_angle_pu_flt, _IQ(0.7)) + _IQmpy(obj->elec_angle_pu, _IQ(0.3));

	return;
}


void ENC_calcMechPos(ENC_Handle handle)
{
	ENC_Obj *obj = (ENC_Obj *) handle;
	uint32_t temp;

	temp = (obj->mech_init_offset + obj->qposcnt)*obj->mech_angle_gain;

	temp &= ((uint32_t) 0x0fffffff);
	temp = (uint32_t)_IQ(16.0) - temp;
	obj->mech_pos_pu = (_iq)temp;

	if(obj->mech_pos_pu > _IQ(15.0))
		obj->mech_pos_pu = _IQ(0.0);

	return;
}


void ENC_calcMechSpd(ENC_Handle handle)
{
	ENC_Obj *obj = (ENC_Obj *) handle;
	_iq temp;

	if(!obj->dirFlag)
	{
		if(obj->mech_pos_pu >= obj->mech_prevPos_pu)
			temp = obj->mech_pos_pu - obj->mech_prevPos_pu;
		else
			temp = obj->mech_pos_pu + _IQ(16.0) - obj->mech_prevPos_pu;
	}
	else
	{
		if(obj->mech_pos_pu >= obj->mech_prevPos_pu)
			temp = obj->mech_prevPos_pu + _IQ(16.0) - obj->mech_pos_pu;
		else
			temp = obj->mech_prevPos_pu - obj->mech_pos_pu;
	}

	obj->mech_speed_kRPM = _IQmpy(_IQdiv(temp, obj->sample_time),_IQ(0.06));

	obj->mech_prevPos_pu = obj->mech_pos_pu;

	return;

}

