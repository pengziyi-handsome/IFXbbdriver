/*******************************************************************************
* 
* file	   : i2c.h
* author   : Peng ZiYi
* brief    : 
* revision :
* 1.0     date   Peng ZiYi     initial version
*******************************************************************************/

#ifndef I2C_H
#define I2C_H
/*******************************************************************************
* includes
*******************************************************************************/
#include "Std_Types.h"
#include "cy_scb_i2c.h"

/*******************************************************************************
* defines[global]
*******************************************************************************/
#define SCB7_I2C_SLAVE_ADDR         34
#define SCB_I2C_NUM                 2

/*******************************************************************************
* data types[global]
*******************************************************************************/
typedef enum 
{ 
    scb7   ,   /* bl i2c */   
} en_scb_i2c_channel_t;

typedef void (* SCB_I2C_IntrISR_t)(void);
typedef void (* SCB_I2C_Callback_t)(uint32_t locEvents);
/*******************************************************************************
* data definitions[global]
*******************************************************************************/

/*******************************************************************************
* function prototypes[global]
*******************************************************************************/
extern void SCB7_I2C_vInit();  /* bl i2c init */
extern void SCB_I2C_Callback_Register(en_scb_i2c_channel_t en_i2c_channel, SCB_I2C_Callback_t SCB_I2C_Callback);
extern uint8_t I2C_AsyncWrite(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength);  
extern uint8_t I2C_AsyncRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8RData, uint8_t u8RLength);
extern uint8_t I2C_AsyncWriteRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength, uint8_t *u8RData, uint8_t u8RLength);  
extern uint8_t I2C_SyncWrite(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength);
extern uint8_t I2C_SyncRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8RData, uint8_t u8RLength);
extern uint8_t I2C_SyncWriteRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength, uint8_t *u8RData, uint8_t u8RLength);

#endif
/*******************************************************************************
* end of file
*******************************************************************************/



