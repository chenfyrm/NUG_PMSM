/*
 * enc.h
 *
 *  Created on: 2020-5-6
 *      Author: 700363
 */

#ifndef ENC_H_
#define ENC_H_

// **************************************************************************
// the includes

#include "user_math.h"

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the typedefs

typedef struct _ENC_Obj_
{
	// input
	uint32_t qposcnt;   // 当前脉冲计数
	uint16_t dirFlag;

	// output
	_iq elec_angle_pu;

	_iq mech_angle_pu;
	_iq mech_prevAngle_pu;
	_iq mech_deltaAngle_pu;
	_iq speed_pu;

	uint16_t mech_rev_cnt;

	bool flag_outOfGuage;

	// parameter
	uint16_t num_pole_pairs;
	uint16_t num_enc_slots;		//!< number of encoder slots
	_iq mech_angle_gain;

	// static
	uint32_t elec_init_offset;     //!< encoder zero offset in counts
	_iq	elec_init_angle;

	uint32_t mech_init_offset;
	_iq mech_init_angle;
} ENC_Obj,*ENC_Handle;


// **************************************************************************
// the globals
extern ENC_Handle encHandle[2];

// **************************************************************************
// the function prototypes

ENC_Handle ENC_init(void *pMemory, const size_t numBytes);
void ENC_setParams(ENC_Handle handle, uint16_t numSlots, uint16_t polePairs);
void ENC_calcCombElecAngle(ENC_Handle handle);
void ENC_calcMechAngle(ENC_Handle handle);
void ENC_run(ENC_Handle handle);

inline _iq ENC_getElecAngle_pu(ENC_Handle handle)
{
	return (handle->elec_angle_pu);
}

////! \brief Sets the value for the encoder object zero offset
////! \param[in] encHandle                        Handle to the ENC object
////! \param[in] zeroOffset                       New zero offset
//inline void ENC_setZeroOffset(ENC_Handle encHandle, uint32_t zeroOffset) {
//	ENC_Obj *enc = (ENC_Obj *)encHandle;
//
//	enc->enc_zero_offset = zeroOffset;
//
//	return;
//}

//! \brief Sets the value for the encoder object zero offset
//! \param[in] encHandle                        Handle to the ENC object
//! \param[in] zeroOffset                       New zero offset
inline void ENC_setElecInitOffset(ENC_Handle handle, uint32_t initOffset, _iq initAngle) {
	ENC_Obj *obj = (ENC_Obj *)handle;

	obj->elec_init_offset = initOffset;
	obj->elec_init_angle = initAngle;

	return;
}

#ifdef __cplusplus
}
#endif // extern "C"

#endif /* ENC_H_ */
