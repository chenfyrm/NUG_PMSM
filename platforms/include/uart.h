/* --COPYRIGHT--
 * Copyright (c) 2020, New United Group
 * All rights reserved.
 * --/COPYRIGHT--*/

#ifndef _UART_H_
#define _UART_H_


// **************************************************************************
// the includes

#include "user_math.h"


//!
//! \defgroup CRTLPAN

//!
//! \ingroup CRTLPAN
//@{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines



//! \brief Defines the transmitted data address
//!
#define UART_TX_DATA_ADDRESS			(0x01)

//! \brief Defines the transmitted data length
//!
#define UART_TX_DATA_LENGTH				(10)

//! \brief Defines the transmitted function code
//!
#define UART_TX_DATA_FUNCCODE			(0x02)

//! \brief Defines the received data address
//!
#define UART_RX_DATA_ADDRESS			(0x02)

//! \brief Defines the received data length
//!
#define UART_RX_DATA_LENGTH				(11)

//! \brief Defines the received function code
//!
#define UART_RX_DATA_FUNCCODE			(0x01)

//! \brief

// **************************************************************************
// the typedefs

//! \brief Defines the CTRLPAN object
//!
typedef struct _UART_Obj_
{
	SCI_Handle		sciHandle;
	uint16_t 		RxSta;

	uint16_t 		TxBuf[UART_TX_DATA_LENGTH];
	uint16_t 		RxBuf[UART_RX_DATA_LENGTH];

	float_t			RealtimeForce;
	float_t			PullForce;
	float_t			RollbackForce;
	uint16_t		SubDevSta;

	bool			RxTimeOut;
	bool			enableTimeOut;
} UART_Obj;

//! \brief Defines the CTRLPAN handle
//!
typedef struct _UART_Obj_ *CTRLPAN_Handle;

// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes



#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CTRLPAN_H_ definition
