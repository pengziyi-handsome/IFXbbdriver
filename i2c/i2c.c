/*******************************************************************************
 *
 * file	    : i2c.c
 * author   : Peng ZiYi
 * brief    : i2c driver
 * revision : 1.0     2023/5/11   Peng ZiYi     initial version
 *
 *******************************************************************************/

/*******************************************************************************
 * includes
 *******************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include "Std_Types.h"
#include "i2c.h"
// #include "MAX25512.h"

/*******************************************************************************
 * defines[internal]
 *******************************************************************************/
#define DIVIDER_NO_1            (1u)

/* ----------------------Slave Address------------------------ */
#if (CY_USE_PSVP == 1)
  #define E_SOURCE_CLK_FREQ     (24000000ul) // fixed
#else
  #define E_SOURCE_CLK_FREQ     (80000000u)  // fixed
#endif

/* -------------------SCB7 Select Frequency------------------- */
#define SCB7_I2C_INCLK_TARGET_FREQ (2000000ul)  // modifiable
#define SCB7_I2C_DATARATE          (100000ul)   // modifiable

/* ----------------------SCB7 PORT---------------------- */
#define SCB7_I2C_SDA_PORT     GPIO_PRT0
#define SCB7_I2C_SDA_PORT_PIN (0)
#define SCB7_I2C_SDA_PORT_MUX P0_0_SCB7_I2C_SDA 

#define SCB7_I2C_SCL_PORT     GPIO_PRT0
#define SCB7_I2C_SCL_PORT_PIN (1)
#define SCB7_I2C_SCL_PORT_MUX P0_1_SCB7_I2C_SCL 

/*******************************************************************************
 * function prototypes[internal]
 *******************************************************************************/
static void SetPeripheFracDiv24_5(en_scb_i2c_channel_t en_i2c_channel, uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum);
static void Scb7_I2C_IntrISR(void);
static void Scb7_I2C_Master_Event(uint32_t locEvents);

/*******************************************************************************
 * data types[internal]
 *******************************************************************************/

/*******************************************************************************
 * data definitions[internal]
 *******************************************************************************/
static volatile stc_SCB_t* stc_SCB[SCB_I2C_NUM] = 
{
    SCB7,
};

static uint8_t  SCB_I2C_Slave_Addr[SCB_I2C_NUM] = 
{
    SCB7_I2C_SLAVE_ADDR,
};

static en_clk_dst_t en_Clk_Dst[SCB_I2C_NUM] = 
{
    PCLK_SCB7_CLOCK,
};

static uint32_t SCB_I2C_INCLK_TARGET_FREQ[SCB_I2C_NUM] = 
{
    SCB7_I2C_INCLK_TARGET_FREQ,
};

static uint32_t SCB_I2C_DATARATE[SCB_I2C_NUM] = 
{
    SCB7_I2C_DATARATE,
};

static volatile stc_GPIO_PRT_t* stc_I2C_SDA_Port[SCB_I2C_NUM] = 
{
    SCB7_I2C_SDA_PORT,
};

static uint8_t stc_I2C_SDA_Port_Pin[SCB_I2C_NUM] = 
{
    SCB7_I2C_SDA_PORT_PIN,
};

static en_hsiom_sel_t en_I2C_SDA_Port_Mux[SCB_I2C_NUM] = 
{
    SCB7_I2C_SDA_PORT_MUX,
};

static volatile stc_GPIO_PRT_t* stc_I2C_SCL_Port[SCB_I2C_NUM] = 
{
    SCB7_I2C_SCL_PORT,
};

static uint8_t stc_I2C_SCL_Port_Pin[SCB_I2C_NUM] = 
{
    SCB7_I2C_SCL_PORT_PIN,
};

static en_hsiom_sel_t en_I2C_SCL_Port_Mux[SCB_I2C_NUM] = 
{
    SCB7_I2C_SCL_PORT_MUX,
};

static cy_stc_gpio_pin_config_t I2C_port_pin_cfg =
{
    .outVal    = 0ul,
    .driveMode = 0ul,            /* Will be updated in runtime */
    .hsiom     = HSIOM_SEL_GPIO, /* Will be updated in runtime */
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

static cy_stc_sysint_irq_t irq_cfg[SCB_I2C_NUM] =
{
    {
        .sysIntSrc  = scb_7_interrupt_IRQn,
        .intIdx     = CPUIntIdx3_IRQn,
        .isEnabled  = true,
    },
};

/* SCB - I2C Configuration */
static cy_stc_scb_i2c_context_t g_stc_i2c_context[SCB_I2C_NUM];
static const cy_stc_scb_i2c_config_t  g_stc_i2c_config [SCB_I2C_NUM] =
{
    {
        .i2cMode             = CY_SCB_I2C_MASTER,
        .useRxFifo           = true,
        .useTxFifo           = true,
        .slaveAddress        = SCB7_I2C_SLAVE_ADDR,
        .slaveAddressMask    = SCB7_I2C_SLAVE_ADDR,
        .acceptAddrInFifo    = false,
        .ackGeneralAddr      = false,
        .enableWakeFromSleep = false
    },
};

static cy_stc_scb_i2c_master_xfer_config_t g_stc_i2c_master_config[SCB_I2C_NUM] =
{
    {
        .slaveAddress = SCB7_I2C_SLAVE_ADDR,
        .buffer       = 0,
        .bufferSize   = 0,
        .xferPending  = false   
    },
};

static SCB_I2C_IntrISR_t SCB_I2C_IntrISR[SCB_I2C_NUM] = 
{
    Scb7_I2C_IntrISR,
};
 
static scb_i2c_handle_events_t SCB_I2C_Master_Event[SCB_I2C_NUM] = 
{
    Scb7_I2C_Master_Event,
}; 

static SCB_I2C_Callback_t SCB_I2C_Callback_Fun[SCB_I2C_NUM] = 
{
    NULL,
    NULL,
};


static uint8_t* SCB_I2C_Rxbuffer [SCB_I2C_NUM] = {0};
static uint8_t  SCB_I2C_RxBufferLen [SCB_I2C_NUM] = {0};

/*******************************************************************************
 * function implementation[internal]
 *******************************************************************************/
static void SetPeripheFracDiv24_5(en_scb_i2c_channel_t en_i2c_channel, uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
    uint64_t temp = ((uint64_t)sourceFreq << 5ull);
    uint32_t divSetting;

    divSetting = (uint32_t)(temp / targetFreq);
    Cy_SysClk_PeriphSetFracDivider(
//                                   Cy_SysClk_GetClockGroup(en_Clk_Dst[en_i2c_channel]), 
                                   CY_SYSCLK_DIV_24_5_BIT,
                                   divNum, 
                                   (((divSetting >> 5u) & 0x00000FFFul) - 1ul), 
                                   (divSetting & 0x0000001Ful)
                                     
                                     );
}



static void Scb7_I2C_IntrISR(void)
{
    /* I2C interrupt handler for High-Level APIs */
    Cy_SCB_I2C_Interrupt(stc_SCB[scb7], &g_stc_i2c_context[scb7]);
}

static void Scb7_I2C_Master_Event(uint32_t locEvents)
{
    uint8_t Result = true;
    uint32_t masterStatus;

    switch (locEvents)
    {
    case CY_SCB_I2C_MASTER_WR_IN_FIFO_EVENT:
        /* do nothing */
        break;
    case CY_SCB_I2C_MASTER_WR_CMPLT_EVENT:
        if (SCB_I2C_RxBufferLen[scb7] != 0)
        {
            g_stc_i2c_master_config[scb7].xferPending = false;
            Result = I2C_AsyncRead(scb7, SCB_I2C_Rxbuffer[scb7], SCB_I2C_RxBufferLen[scb7]);

            if (Result == true)
            {
                /* do nothing */
            }
            else
            {
                /* Handle read error */
            }

            SCB_I2C_RxBufferLen[scb7] = 0;   /* Prevents write-only process */
        }
        else
        {
            /* do nothing */
            /* write-only, not need to write befor read  */
        }
        break;
    case CY_SCB_I2C_MASTER_RD_CMPLT_EVENT:
        /* do nothing */
        break;
    case CY_SCB_I2C_MASTER_ERR_EVENT:
        /* An error occurred while transmitting data. */
        break;
    default:
        break;
    }

    masterStatus = Cy_SCB_I2C_MasterGetStatus(stc_SCB[scb7], &g_stc_i2c_context[scb7]);
    if (SCB_I2C_Callback_Fun[scb7] != NULL)
    {
        SCB_I2C_Callback_Fun[scb7](masterStatus);
    }
    else
    {
        /* do nothing */
    }
}

static void I2C_Clock_Config(en_scb_i2c_channel_t en_i2c_channel)
{
    Cy_SysClk_PeriphAssignDivider(en_Clk_Dst[en_i2c_channel], CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
    SetPeripheFracDiv24_5(en_i2c_channel, SCB_I2C_INCLK_TARGET_FREQ[en_i2c_channel], E_SOURCE_CLK_FREQ, DIVIDER_NO_1);
    Cy_SysClk_PeriphEnableDivider( CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
}

static void I2C_Port_Config(en_scb_i2c_channel_t en_i2c_channel)
{
    /* ------------------------SDA------------------------------- */
    I2C_port_pin_cfg.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
    I2C_port_pin_cfg.hsiom     = en_I2C_SDA_Port_Mux[en_i2c_channel];
    Cy_GPIO_Pin_Init(stc_I2C_SDA_Port[en_i2c_channel], stc_I2C_SDA_Port_Pin[en_i2c_channel], &I2C_port_pin_cfg);

    /* ------------------------SCL------------------------------- */
    I2C_port_pin_cfg.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
    I2C_port_pin_cfg.hsiom     = en_I2C_SCL_Port_Mux[en_i2c_channel];
    Cy_GPIO_Pin_Init(stc_I2C_SCL_Port[en_i2c_channel], stc_I2C_SCL_Port_Pin[en_i2c_channel], &I2C_port_pin_cfg);
}

static void I2C_Int_Config(en_scb_i2c_channel_t en_i2c_channel)
{
    Cy_SysInt_InitIRQ(&irq_cfg[en_i2c_channel]);
    Cy_SysInt_SetSystemIrqVector(irq_cfg[en_i2c_channel].sysIntSrc, SCB_I2C_IntrISR[en_i2c_channel]);
    NVIC_SetPriority(irq_cfg[en_i2c_channel].intIdx, 3ul);
    NVIC_EnableIRQ(irq_cfg[en_i2c_channel].intIdx);
}

static void I2C_Init_Config(en_scb_i2c_channel_t en_i2c_channel)
{
    Cy_SCB_I2C_DeInit(stc_SCB[en_i2c_channel]);
    Cy_SCB_I2C_Init(stc_SCB[en_i2c_channel], &g_stc_i2c_config[en_i2c_channel], &g_stc_i2c_context[en_i2c_channel]);
    Cy_SCB_I2C_SetDataRate(stc_SCB[en_i2c_channel], SCB_I2C_DATARATE[en_i2c_channel], SCB_I2C_INCLK_TARGET_FREQ[en_i2c_channel]);
    Cy_SCB_I2C_RegisterEventCallback(stc_SCB[en_i2c_channel], (scb_i2c_handle_events_t)SCB_I2C_Master_Event[en_i2c_channel], &g_stc_i2c_context[en_i2c_channel]);
    Cy_SCB_I2C_Enable(stc_SCB[en_i2c_channel]);
}

static void I2C_vInit(en_scb_i2c_channel_t en_i2c_channel)
{
    /* Clock Configuration */
    I2C_Clock_Config(en_i2c_channel);
    
    /* Port Configuration */
    I2C_Port_Config(en_i2c_channel);

    /* Interrupt Configuration */
    I2C_Int_Config(en_i2c_channel);

    /* Initialize & Enable I2C  */
    I2C_Init_Config(en_i2c_channel);
}
/*******************************************************************************
 * function implementation
 *******************************************************************************/

void SCB7_I2C_vInit(void)
{
    I2C_vInit(scb7);
}

void  SCB_I2C_Callback_Register(en_scb_i2c_channel_t en_i2c_channel, SCB_I2C_Callback_t SCB_I2C_Callback)
{
    SCB_I2C_Callback_Fun[en_i2c_channel] = SCB_I2C_Callback;
}

uint8_t I2C_AsyncWrite(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength)
{   
    uint8_t Result = true;
    cy_en_scb_i2c_status_t en_scb_i2c_status = CY_SCB_I2C_SUCCESS;
    g_stc_i2c_master_config[en_i2c_channel].buffer = u8WData;
    g_stc_i2c_master_config[en_i2c_channel].bufferSize = u8WLength;

    /* This function is not blocking, use interrupt to write data */
    en_scb_i2c_status = Cy_SCB_I2C_MasterWrite(stc_SCB[en_i2c_channel], &g_stc_i2c_master_config[en_i2c_channel], &g_stc_i2c_context[en_i2c_channel]);

    switch (en_scb_i2c_status)
    {
    case CY_SCB_I2C_SUCCESS :
        Result = true;
        break;

    case CY_SCB_I2C_MASTER_NOT_READY :
        /* */
        Result = false;
        break;
   
    default:
        break;
    }
    
    return(Result);
}

uint8_t I2C_AsyncRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8RData, uint8_t u8RLength)
{
    uint8_t Result = true;
    cy_en_scb_i2c_status_t en_scb_i2c_status = CY_SCB_I2C_SUCCESS;
    g_stc_i2c_master_config[en_i2c_channel].buffer = u8RData;
    g_stc_i2c_master_config[en_i2c_channel].bufferSize = u8RLength;

    /* This function is not blocking, use interrupt to read data */
    en_scb_i2c_status = Cy_SCB_I2C_MasterRead(stc_SCB[en_i2c_channel], &g_stc_i2c_master_config[en_i2c_channel], &g_stc_i2c_context[en_i2c_channel]);

    switch (en_scb_i2c_status)
    {
    case CY_SCB_I2C_SUCCESS :
        Result = true;
        break;

    case CY_SCB_I2C_MASTER_NOT_READY :
        /*  */
        Result = false;
        break;
    default:
        break;
    }

    return(Result);
}

uint8_t I2C_AsyncWriteRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength, uint8_t *u8RData, uint8_t u8RLength)
{
    uint8_t Result = true;
    cy_en_scb_i2c_status_t en_scb_i2c_status = CY_SCB_I2C_SUCCESS;
    g_stc_i2c_master_config[en_i2c_channel].xferPending = true;
    g_stc_i2c_master_config[en_i2c_channel].buffer = u8WData;
    g_stc_i2c_master_config[en_i2c_channel].bufferSize = u8WLength;

    /* After the write process is complete, the data is read */
    SCB_I2C_Rxbuffer[en_i2c_channel] = u8RData;
    SCB_I2C_RxBufferLen[en_i2c_channel] = u8RLength;

    /* The function is not blocking */
    en_scb_i2c_status = Cy_SCB_I2C_MasterWrite(stc_SCB[en_i2c_channel], &g_stc_i2c_master_config[en_i2c_channel], &g_stc_i2c_context[en_i2c_channel]);
    switch (en_scb_i2c_status)
    {
    case CY_SCB_I2C_SUCCESS :
        Result = true;
        break;

    case CY_SCB_I2C_MASTER_NOT_READY :
        /*  */
        Result = false;
        break;
   
    default:
        break;
    }

    return(Result);
}

uint8_t I2C_SyncWrite(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength)
{
    uint8_t Result = true;
    cy_en_scb_i2c_status_t en_scb_i2c_status = CY_SCB_I2C_SUCCESS;

    /* Send START and Receive ACK/NACK */
    en_scb_i2c_status = Cy_SCB_I2C_MasterSendStart(stc_SCB[en_i2c_channel], SCB_I2C_Slave_Addr[en_i2c_channel], CY_SCB_I2C_WRITE_XFER, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
    {
        /* Transmit Data */
        for (uint8_t i = 0; i < u8WLength; i++)
        {
            /* Transmit one byte */
            en_scb_i2c_status = Cy_SCB_I2C_MasterWriteByte(stc_SCB[en_i2c_channel], *u8WData++, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

            if (en_scb_i2c_status != CY_SCB_I2C_SUCCESS)
                break;
        }

        if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
        {
            en_scb_i2c_status = Cy_SCB_I2C_MasterSendWriteStop(stc_SCB[en_i2c_channel], 2000ul, &g_stc_i2c_context[en_i2c_channel]);

            if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
            {
                /* do nothing */
            } 
            else
            {
                /* handle send write stop error */
            }
        }
        else
        {
            /* Handle transmit data error */
        }
    }
    else
    {
        /* Handle send STARTerror */
    }

    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
    {
        Result = true;
    }
    else
    {
        Result = false;
    }

    return(Result);
}


uint8_t I2C_SyncRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8RData, uint8_t u8RLength)
{
    uint8_t Result = true;
    cy_en_scb_i2c_status_t en_scb_i2c_status = CY_SCB_I2C_SUCCESS;

    en_scb_i2c_status = Cy_SCB_I2C_MasterSendStart(stc_SCB[en_i2c_channel], SCB_I2C_Slave_Addr[en_i2c_channel], CY_SCB_I2C_READ_XFER, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
    {
        /* Receive Data and return ACK */
        for (uint8_t i = 0; i < u8RLength; i++)
        {
            /* Receive one byte */
            en_scb_i2c_status = Cy_SCB_I2C_MasterReadByte(stc_SCB[en_i2c_channel], CY_SCB_I2C_ACK, u8RData++, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

            if (en_scb_i2c_status != CY_SCB_I2C_SUCCESS)
                break;
        }

        if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
        {
            en_scb_i2c_status = Cy_SCB_I2C_MasterSendReadStop(stc_SCB[en_i2c_channel], 2000ul, &g_stc_i2c_context[en_i2c_channel]);

            if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
            {
                /* do noting */
            }
            else
            {
                /* Handle send read stop error */
            }
            
        }
        else
        {
            /* Handle receive data error */
        }
        
    }
    else
    {
        /* Handle send START error */
    }
    
    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
    {
        Result = true;
    }
    else
    {
        Result = false;
    }
    
    return(Result);
}


uint8_t I2C_SyncWriteRead(en_scb_i2c_channel_t en_i2c_channel, uint8_t *u8WData, uint8_t u8WLength, uint8_t *u8RData, uint8_t u8RLength)
{
    uint8_t Result = true;
    cy_en_scb_i2c_status_t en_scb_i2c_status = CY_SCB_I2C_SUCCESS;

    /* while(Cy_SCB_GetNumInTxFifo(USER_I2C_SCB_TYPE) != 0ul); */

    en_scb_i2c_status = Cy_SCB_I2C_MasterSendStart(stc_SCB[en_i2c_channel], SCB_I2C_Slave_Addr[en_i2c_channel], CY_SCB_I2C_READ_XFER, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
    {
        /* Transmit Data */
        for(uint8_t i = 0; i < u8WLength; i++)
        {
            en_scb_i2c_status = Cy_SCB_I2C_MasterWriteByte(stc_SCB[en_i2c_channel], *u8WData++, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

            if (en_scb_i2c_status != CY_SCB_I2C_SUCCESS)
                break;
        }

        if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
        {
            en_scb_i2c_status = Cy_SCB_I2C_MasterSendReStart(stc_SCB[en_i2c_channel], SCB_I2C_Slave_Addr[en_i2c_channel], CY_SCB_I2C_READ_XFER, 2000ul, &g_stc_i2c_context[en_i2c_channel]);

            if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
            {
                /* Receive Data and return ACK */
                for(uint8_t i = 0; i < u8RLength; i++)
                {
                    en_scb_i2c_status = Cy_SCB_I2C_MasterReadByte(stc_SCB[en_i2c_channel], CY_SCB_I2C_ACK, u8RData++, 2000ul, &g_stc_i2c_context[en_i2c_channel]);
                    if (en_scb_i2c_status != CY_SCB_I2C_SUCCESS)
                        break;
                }

                if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
                {
                    Cy_SCB_I2C_MasterSendReadStop(stc_SCB[en_i2c_channel], 2000ul, &g_stc_i2c_context[en_i2c_channel]);
                    
                    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
                    {
                        /* do noting, success precess*/
                    }
                    else
                    {
                        /* Handle send read stop error */
                    }
                    
                }
                else
                {
                    /* Handle receive data error */
                }
            }
            else
            {
                /* Handle send restart error */
            }
        }
        else
        {
            /* Handle transmit data error */
        }
    }
    else
    {
        /* Handle send start error */
    }
    
    if (en_scb_i2c_status == CY_SCB_I2C_SUCCESS)
    {
        Result = true;
    }
    else
    {
        Result = false;
    }
    
    return(Result);
}   

/*******************************************************************************
 * end of file
 *******************************************************************************/




