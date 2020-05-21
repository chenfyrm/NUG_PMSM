/*
 * hall.h
 *
 *  Created on: 2020-5-6
 *      Author: 700363
 */

#ifndef HALL_H_
#define HALL_H_

// **************************************************************************
// the includes

#include "user_math.h"
#include "NUG_hal.h"
#include "NUG_ctrl.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

typedef struct _HALL_Obj_
{
	PID_Handle 		pidHandle_Is;
	PID_Obj	   		pid_Is;

	// input
	uint16_t hallSta;
	uint32_t capCounter;
	uint32_t tsCounter;

	_iq 	 Is_ref_pu;
	_iq		 Is_fbk_pu;

	_iq		 speed_fbk_pu;

	bool	 flag_cap;

	// output
	_iq	elec_initAngle_pu;

	uint16_t dir;
	_iq PwmDuty;

	// parameter
	_iq speed_FocToBldc_low_pu;
	_iq speed_BldcToFoc_high_pu;

	// static
	uint16_t Is_fdb_sel;
	uint16_t sector;
	uint16_t prevSector;
	uint16_t bldcCnt;
	uint16_t focCnt;
	bool Flag_secChanged;
	bool flag_enableBldc;

}HALL_Obj,*HALL_Handle;

// **************************************************************************
// the globals
extern HALL_Handle hallHandle;

// **************************************************************************
// the function prototypes

HALL_Handle HALL_init(void *pMemory, const size_t numBytes);

void HALL_setParams(HALL_Handle handle);

inline void HALL_setIs_ref_pu(HALL_Handle handle, const _iq IsRef_pu)
{
	HALL_Obj *obj = (HALL_Obj *)handle;

	obj->Is_ref_pu = IsRef_pu;

	return;
}

//inline void HALL_readData(HALL_Handle handle, HAL_Handle halHandle)
//{
//	uint16_t pHall_GpioData;
//
//	pHall_GpioData = (HAL_readGpio(halHandle,GPIO_Number_11) & 0x1)<<2;
//	pHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_15) & 0x1)<<1;
//	pHall_GpioData += (HAL_readGpio(halHandle,GPIO_Number_9) & 0x1)<<0;
//
//	hallHandle->hallSta = pHall_GpioData&(7<<0);
//
//	hallHandle->flag_cap = CAP_getInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);
//
//	if(hallHandle->flag_cap)
//	{
//
//		hallHandle->tsCounter = halHandle->capHandle[0]->TSCTR;
//		hallHandle->capCounter = CAP_getCap1(halHandle->capHandle[0]);
//
//		CAP_clearInt(halHandle->capHandle[0],CAP_Int_Type_CEVT1);
//	}
//}

void HALL_checkState(HALL_Handle handle, HAL_Handle halHandle);

extern void HALL_Ctrl_run(HALL_Handle handle, CTRL_Handle ctrlHandle, const HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData);

void HALL_Ctrl_PwmSet(HALL_Handle handle, HAL_PwmData_t *pPwmData);
// **************************************************************************
// the functions

#ifdef __cplusplus
}
#endif // extern "C"


#endif /* HALL_H_ */
