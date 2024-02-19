
/*******************************************************************************
 *  [FILE NAME]   :      <OBD2_CFG.h>                                          *
 *  [AUTHOR]      :      <David S. Alexander>                                  *
 *  [DATE CREATED]:      <Feb 16, 2024>                                        *
 *  [Description} :      <Header file for OBD2 driver configuration>           *
 *******************************************************************************/

#ifndef INC_OBD2_CFG_H_
#define INC_OBD2_CFG_H_

/* Define OBD2 CAN Interface */
#define OBD2CAN                 hcan       /* CAN Interface for OBD-II communication */

/* Define Serial Debug Interface */
#define SERIAL_DEBUG            huart2     /* Serial Debug Interface (USART/UART) for debugging purposes */

/* Define Debug Message Buffer Size */
#define DEBUG_MESSAGE_BUFFER    0X64       /* Size of the debug message buffer in bytes */


#endif /* INC_OBD2_CFG_H_ */
