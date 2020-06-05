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
	_iq elec_speed_kRPM;

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

	bool flag_enableSpeedCtrl;

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

void HALL_checkState(HALL_Handle handle, HAL_Handle halHandle);
void HALL_calcElecSpd(HALL_Handle handle);

extern void HALL_Ctrl_run(HALL_Handle handle, CTRL_Handle ctrlHandle, const HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData);

void HALL_Ctrl_PwmSet(HALL_Handle handle, HAL_PwmData_t *pPwmData);
// **************************************************************************
// the functions

#ifdef __cplusplus
}
#endif // extern "C"


#endif /* HALL_H_ */
