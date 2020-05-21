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

	obj->mech_angle_pu = _IQ(0.0);
	obj->mech_prevAngle_pu = _IQ(0.0);
	obj->mech_deltaAngle_pu = _IQ(0.0);
	obj->speed_pu = _IQ(0.0);

	obj->mech_rev_cnt = 0;

	obj->flag_outOfGuage = false;

	obj->num_pole_pairs = polePairs;
	obj->num_enc_slots = numSlots;
	obj->mech_angle_gain = (_iq)((((uint32_t)1)<<24)/(4*numSlots)); // 4*(600*7.5) pulse per revolution

	obj->elec_init_offset = 0;
	obj->elec_init_angle = _IQ(0.0);

	obj->mech_init_offset = 0;
	obj->mech_init_angle = _IQ(0.0);

	return;
}

void ENC_calcCombElecAngle(ENC_Handle handle)
{
	ENC_Obj *obj = (ENC_Obj *) handle;
	uint32_t temp;

	temp = (obj->elec_init_offset + obj->qposcnt)*obj->mech_angle_gain;
	temp *= obj->num_pole_pairs;
	temp += obj->elec_init_angle;
	temp &= ((uint32_t) 0x00ffffff);

	obj->elec_angle_pu = (_iq)temp;

	return;
}

void ENC_calcMechAngle(ENC_Handle handle)
{
	ENC_Obj *obj = (ENC_Obj *) handle;
	uint32_t temp;

	temp = (obj->mech_init_offset + obj->qposcnt)*obj->mech_angle_gain;
	temp += obj->mech_init_angle;
	temp &= ((uint32_t) 0x00ffffff);

	obj->mech_angle_pu = (_iq)temp;

	if(obj->dirFlag)
	{
		if(obj->mech_angle_pu > obj->mech_prevAngle_pu)
			obj->speed_pu = (obj->mech_angle_pu - obj->mech_prevAngle_pu)*50.0;
		else
			obj->speed_pu = (obj->mech_angle_pu + _IQ(1.0) - obj->mech_prevAngle_pu)*50.0;
	}
	else
	{
		if(obj->mech_prevAngle_pu > obj->mech_angle_pu)
			obj->speed_pu = (obj->mech_prevAngle_pu - obj->mech_angle_pu)*50.0;
		else
			obj->speed_pu = (obj->mech_prevAngle_pu + _IQ(1.0) - obj->mech_angle_pu)*50.0;
	}

	obj->mech_prevAngle_pu = obj->mech_angle_pu;

	return;
}

//void ENC_run(ENC_Handle handle)
//{
//	ENC_Obj *obj = (ENC_Obj*)handle;
//
//	uint16_t dirFlag = handle->dirFlag;
//	uint16_t UTOFlag = handle->UTOFlag;
//	uint16_t UPEVNTFlag = handle->UPEVNTFlag;
//	uint32_t qposlat = handle->qposlat;
//	uint16_t qcprdlat = handle->qcprdlat;
//
//
//	if(!dirFlag)
//		obj->dir = 0;
//	else
//		obj->dir = 1;
//
//	// 测周法
//	//-----------------------------------转速太慢记周精度降低 转速太块记周会翻转  -------------------------------------------------
//	if(UTOFlag)
//	{
//		if(!dirFlag)
//		{
//			if(qposlat < obj->prev_enc)
//			{
//				obj->delta_enc = obj->prev_enc - qposlat;
//				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
//			}
//			else if(qposlat == obj->prev_enc)
//			{
//				obj->delta_enc = 0;
//				obj->rpm_cycle_way = obj->mech_speed_rpm;
//			}
//			else
//			{
//				obj->delta_enc = obj->prev_enc - qposlat + 0xFFFFFFFF;
//				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
//			}
//		}
//		else
//		{
//			if(qposlat > obj->prev_enc)
//			{
//				obj->delta_enc =  qposlat - obj->prev_enc;
//				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
//			}
//			else if(qposlat == obj->prev_enc)
//			{
//				obj->delta_enc = 0;
//				obj->rpm_cycle_way = obj->mech_speed_rpm;
//			}
//			else
//			{
//				obj->delta_enc = qposlat - obj->prev_enc + 0xFFFFFFFF;
//				obj->rpm_cycle_way = 60.0*obj->sample_period/(obj->num_enc_slots*4.0)*obj->delta_enc;
//			}
//		}
//		obj->prev_enc = qposlat;
//	}
//
//	// 测频法
//	//-----------------------------------转速太慢记时会翻转  转速太快记时精度降低-------------------------------------------------
//	if(UPEVNTFlag)
//	{
//		if(!obj->COEFFlag)
//		{
//			obj->rpm_freq_way = 60.0*obj->upps/(obj->num_enc_slots*4.0)/(obj->ccps/90.0e6)/qcprdlat;
//		}
//		else
//		{
//			obj->rpm_freq_way = 0.0;
//		}
//	}
//
//	//
////	if(rpm_cycle_way<=0.01)
////		obj->mech_speed_rpm = 0;
////	else if(rpm_cycle_way<=500)
////		obj->mech_speed_rpm = rpm_freq_way;
////	else if(rpm_cycle_way<=3000)
////		obj->mech_speed_rpm = rpm_cycle_way;
////	else
////		obj->mech_speed_rpm = 3000;
//}


//encHandle1->dirFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_QDF; // 0 ccw, 1 cw
//
//	encHandle1->UTOFlag = QEP_read_interrupt_flag(halHandle->qepHandle[0],QEINT_Uto);
//	if(encHandle1->UTOFlag)
//	{
//		encHandle1->qposlat = QEP_read_posn_latch(halHandle->qepHandle[0]);
//		QEP_clear_interrupt_flag(halHandle->qepHandle[0],QEINT_Uto);
//	}
//
//	encHandle1->UPEVNTFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_UPEVNT;
//	if(encHandle1->UPEVNTFlag)
//	{
//		encHandle1->qcprdlat = QEP_read_capture_period_latch(halHandle->qepHandle[0]);
//		encHandle1->COEFFlag = QEP_read_status(halHandle->qepHandle[0])&QEP_QEPSTS_COEF;
//		QEP_reset_status(halHandle->qepHandle[0], UPEVNT);
//		QEP_reset_status(halHandle->qepHandle[0], COEF);
//	}
//
//	ENC_run(encHandle1);

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
