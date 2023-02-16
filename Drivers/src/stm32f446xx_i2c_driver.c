/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Feb 13, 2020
 *      Author: nemanja
 */

#include <stm32f446xx_i2c_driver.h>

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn          - I2C_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given I2C port
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}


/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn	        - I2C_Init
 *
 * @brief       - This function initialize I2C peripherals
 *
 * @param[in]   - Pointer to I2C Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    /* Enabling clock for I2C peripheral */
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    /* ACK control bit */
    tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    /* CR2 FREQ filed configuration */
    tempreg = 0;
    tempreg|= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

    /* Programming device own address */
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << I2C_OAR1_ADDMODE);
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    /* CCR calculations */
    uint16_t ccr_value = 0;
    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* Standard mode */
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempreg |= (ccr_value & 0xFFF);
    }
    else
    {
        /* Fast mode */
        tempreg |= (1 << 15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        else
        {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tempreg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;

    /* TRISE Configuration */
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* Standard mode */
        tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
    }
    else
    {
        /* Fast mode */
        tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000U ) + 1;
    }

    pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


/*****************************************************************
 * @fn          - I2C_DeInit
 *
 * @brief       - This function de-initialize I2C peripherals
 *
 * @param[in]   - Base address of the I2C peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if(pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if(pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if(pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
}


/*****************************************************************
 * @fn          - I2C_GetFlagStatus
 *
 * @brief       - This function returns if bit in register is
 *                set or not
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Name of flag
 *
 * @return      - Flag status (True/False)
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if(pI2Cx->SR1 & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}


/*****************************************************************
 * @fn          - I2C_PeripheralControl
 *
 * @brief       - This function sets I2C peripheral control
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Enable or Disable command
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}


/*****************************************************************
 * @fn          - I2C_MasterSendData
 *
 * @brief       - I2C Master sends data to slaves
 *
 * @param[in]   - Pointer to I2C Handle structure
 * @param[in]   - Pointer to transmit buffer
 * @param[in]   - Length of transmit buffer
 * @param[in]   - Slave address
 * @param[in]   - State of status register
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr)
{
    /* Generating start condition */
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    /* Confirming start generation is completed by checking SB flag in SR1 */
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

    /* Send the address of the slave with r/nm bit set to w(0) (total 8 bits) */
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    /* Confirming address phase is completed by checking ADDR flag in SR1 */
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

    /* Clearing the address flag according to its software sequence */
    I2C_ClearAddrFlag(pI2CHandle);

    /* Send the data until length reaches 0 value */
    while(Length > 0)
    {
        /* Waiting untill TX is set */
        while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Length--;
    }

    /* Waiting for TXE=1 and BTF=1 before generating STOP condition */
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

    /* Generate STOP condition and master not need to wait for the completion of stop condition */
    if(Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}


/*****************************************************************
 * @fn          - I2C_MasterReceiveData
 *
 * @brief       - I2C Master receive data from slaves
 *
 * @param[in]   - Pointer to I2C Handle structure
 * @param[in]   - Pointer to receive buffer
 * @param[in]   - Length of receive buffer
 * @param[in]   - Slave address
 * @param[in]   - State of status register
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr)
{
    /* Generating start condition */
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    /* Confirming start generation is completed by checking SB flag in SR1 */
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

    /* Send the address of the slave with r/nm bit set to w(0) (total 8 bits) */
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    /* Confirming address phase is completed by checking ADDR flag in SR1 */
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

    /* Procedure to read only 1 byte from slave */
    if(Length == 1)
    {
        /* Disable acknowledgment */
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        /* Clear address flag */
        I2C_ClearAddrFlag(pI2CHandle);

        /* Wait until RXNE becomes 1 */
        while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

        /* Generate STOP condition and master not need to wait for the completion of stop condition */
        if(Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        /* Read the data into buffer */
        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    }

    if(Length > 1)
    {
        /* Clear address flag */
        I2C_ClearAddrFlag(pI2CHandle);

        /* Read the data into buffer until length becomes 0 */
        for(uint32_t i = Length; i > 0; i--)
        {
            /* Wait until RXNE becomes 1 */
            while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

            /* If last 2 bits are remaining */
            if(i == 2)
            {
                /* Clear the ACK bit */
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                /* Generate STOP condition and master not need to wait for the completion of stop condition */
                if(Sr == I2C_DISABLE_SR)
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
            }

            /* Read the data from data register in to the buffer */
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            /* Increment the buffer address */
            pRxBuffer++;

        }
    }

    /* Re-enable ACK */
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}


/*****************************************************************
 * @fn          - I2C_MasterSendDataInterruptMode
 *
 * @brief       - This function sends data over I2C
 *                peripheral in Interrupt mode
 *
 * @param[in]   - Pointer to I2C Handle structure
 * @param[in]   - Pointer to transmit buffer
 * @param[in]   - Length of transmit buffer
 * @param[in]   - Slave address
 * @param[in]   - State of status register
 *
 * @return      - Tx State
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t I2C_MasterSendDataInterruptMode(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Length;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        /* Generating start condition */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* Enable ITBUFEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        /* Enable ITEVTEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        /* Enable ITERREN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}


/*****************************************************************
 * @fn          - I2C_MasterReceiveDataInterruptMode
 *
 * @brief       - This function receives data over I2C
 *                peripheral in Interrupt mode
 *
 * @param[in]   - Pointer to I2C Handle structure
 * @param[in]   - Pointer to receive buffer
 * @param[in]   - Length of receive buffer
 * @param[in]   - Slave address
 * @param[in]   - State of status register
 *
 * @return      - Rx State
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t I2C_MasterReceiveDataInterruptMode(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr)
{

    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Length;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        /* RxSize is used in the ISR code to manage the data reception */
        pI2CHandle->RxSize = Length;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        /* Generating start condition */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* Enable ITBUFEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        /* Enable ITEVTEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        /* Enable ITERREN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}


/*****************************************************************
 * @fn          - I2C_ManageAcking
 *
 * @brief       - This function manages acknowledgment bit
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Name of flag
 *
 * @return      - Flag status (True/False)
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == I2C_ACK_ENABLE)
    {
        /* Enable ACK */
        pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
    }
    else
    {
        /* Disable ACK */
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
    }
}


/*****************************************************************
 * @fn          - I2C_SlaveSendData
 *
 * @brief       - This function sends data in I2C Slave mode
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Data
 *
 * @return      - Flag status (True/False)
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}


/*****************************************************************
 * @fn          - I2C_SlaveReceiveData
 *
 * @brief       - This function receives data in I2C Slave mode
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Data
 *
 * @return      - Flag status (True/False)
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t)pI2Cx->DR;
}


/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn          - I2C_IRQInterruptConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ISER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ISER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            /* Program ICER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ICER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ICER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
}


/*****************************************************************
 * @fn          - I2C_IRQPriorityConfig
 *
 * @brief       - This function configures interrupt priority
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ interrupt priority
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*****************************************************************
 * @fn          - I2C_EV_IRQHandling
 *
 * @brief       - Interrupt handling for different I2C events
 *
 * @param[in]   - Pointer to I2C Handle structure

 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    /* Interrupt handling for both master and slave mode of a device */
    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    /* Handling for interrupt generated by SB event */
    if(temp1 && temp3)
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

    /* Handling for interrupt generated by ADDR event */
    if(temp1 && temp3)
    {
        I2C_ClearAddrFlag(pI2CHandle);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

    /* Handling for interrupt generated by BTF(Byte Transfer Finished) event */
    if(temp1 && temp3)
    {
        /* BTF flag is set */
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            /* TXE flag is also set */
            if(pI2CHandle->pI2Cx->SR1 && (1 << I2C_SR1_TXE))
            {
                if(pI2CHandle->TxLen == 0)
                {
                    /* Generate STOP condition */
                    if(pI2CHandle->Sr == I2C_DISABLE_SR)
                    {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    /* Reset all member elements of the handle structure */
                    I2C_CloseSendData(pI2CHandle);

                    /* Notify application about transmission complete */
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            ;
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

    /* Handling for interrupt generated by STOPF event */
    if(temp1 && temp3)
    {
        /* Clear STOPF flag */
        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        /* STOPF generated by master */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

    /* Handling for interrupt generated by TXE event */
    if(temp1 && temp2 && temp3)
    {
        /* Checking device mode */
        if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        }
        else
        {
            /* Checking if slave is really in Tx mode */
            if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
            }
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

    /* Handling for interrupt generated by RXNE event */
    if(temp1 && temp2 && temp3)
    {
        /* Checking device mode */
        if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            /* RXNE flag is set */
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            /* Checking if slave is really in Rx mode */
            if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
            {
                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
            }
        }
    }
}


/*****************************************************************
 * @fn          - I2C_ER_IRQHandling
 *
 * @brief       - Interrupt handling for different I2C errors
 *
 * @param[in]   - Pointer to I2C Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    /* Status of ITERREN control bit in CR2 */
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /* Checking for bus error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if(temp1 && temp2)
    {
        /* Clear the bus error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /* Checking for arbitration lost error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if(temp1 && temp2)
    {
        /* Clear the arbitration lost error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_ARLO);
    }

    /* Checking for ACK failure error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if(temp1 && temp2)
    {
        /* Clear the ACK failure error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_AF);
    }

    /* Checking for Overrun/Underrun error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if(temp1 && temp2)
    {
        /* Clear the Overrun/Underrun error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_OVR);
    }

    /* Checking for Time out error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if(temp1 && temp2)
    {
        /* Clear the Time out error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_TIMEOUT);
    }
}


/*****************************************************************
 * @fn          - I2C_CloseReceiveData
 *
 * @brief       - Closing I2C communication when data is
 *                received
 *
 * @param[in]   - Pointer to I2C Handle structure
 *
 * @return      - None
 *
 * @Note        - Disabling all interrupts
 *
 *****************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    /* Disabling ITBUFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    /* Disabling ITEVFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
    }

}


/*****************************************************************
 * @fn          - I2C_CloseSendData
 *
 * @brief       - Closing I2C communication when data is sent
 *
 * @param[in]   - Pointer to I2C Handle structure
 *
 * @return      - None
 *
 * @Note        - Disabling all interrupts
 *
 *****************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    /* Disabling ITBUFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    /* Disabling ITEVFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}


/*****************************************************************
 * @fn          - I2C_GenerateStopCondition
 *
 * @brief       - Generate stop condition for I2C
 *
 * @param[in]   - Base address of the I2C peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


/*****************************************************************
 * @fn          - I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief       - Generate stop condition for I2C
 *
 * @param[in]   - Base address of the I2C peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}


/*****************************************************************
 *               Helper functions implementation                 *
 *****************************************************************/
/*****************************************************************
 * @fn          - I2C_GenerateStartCondition
 *
 * @brief       - Generate start condition for I2C
 *
 * @param[in]   - Base address of the I2C peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/*****************************************************************
 * @fn          - I2C_ExecuteAddressPhaseWrite
 *
 * @brief       - Send the address of the slave with r/nm
 *                bit set to r/nw(0) (total 8 bits)
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Slave address
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    /* SlaveAddr is Slave address + r/nw bit=0 */
    SlaveAddr &= ~(1);
    pI2Cx->DR = SlaveAddr;
}


/*****************************************************************
 * @fn          - I2C_ExecuteAddressPhaseRead
 *
 * @brief       - Send the address of the slave with r/nm
 *                bit set to r/nw(1) (total 8 bits)
 *
 * @param[in]   - Base address of the I2C peripheral
 * @param[in]   - Slave address
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    /* SlaveAddr is Slave address + r/nw bit=1 */
    SlaveAddr |= 1;
    pI2Cx->DR = SlaveAddr;
}


/*****************************************************************
 * @fn          - I2C_ClearAddrFlag
 *
 * @brief       - Clear address flag
 *
 * @param[in]   - Base address of the I2C peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;

    /* Checking for device mode */
    if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
    {
        /* Device is in master mode */
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->RxSize == 1)
            {
                /* Disabling ACK */
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

                /* Clearing ADDR flag, reading SR1 and SR2 */
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read;
            }
        }
        else
        {
            /* Clearing ADDR flag, reading SR1 and SR2 */
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read;
        }
    }
    else
    {
        /* Device is in slave mode */
        /* Clearing ADDR flag, reading SR1 and SR2 */
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}


/*****************************************************************
 * @fn          - I2C_MasterHandleTXEInterrupt
 *
 * @brief       - Interrupt handling master TXE mode
 *
 * @param[in]   - Pointer to I2C Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
    /* TXE flag is set */
    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
    {
        if(pI2CHandle->TxLen > 0)
        {
            /* Loading data into DR */
            pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

            /* Decrement Tx Length */
            pI2CHandle->TxLen--;

            /* Increment Tx buffer address */
            pI2CHandle->pTxBuffer++;
        }
    }
    /* DBG->Check comments */
}


/*****************************************************************
 * @fn          - I2C_MasterHandleRXNEInterrupts
 *
 * @brief       - Interrupt handling master RXNE mode
 *
 * @param[in]   - Pointer to I2C Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->RxSize == 1)
    {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxSize > 1)
    {
        if(pI2CHandle->RxLen == 2)
        {
            /* Clearing ACK bit */
            I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        }

        /* Reading DR */
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxLen == 0)
    {
        /* Closing I2C data reception and notifying application */

        /* Generating stop condition */
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        /* Closing I2C RX */
        I2C_CloseReceiveData(pI2CHandle);

        /* Notify application about transmission complete */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}


/*****************************************************************
 * @fn          - I2C_ApplicationEventCallback
 *
 * @brief       - Application event callback function
 *
 * @param[in]   - Handle structure
 * @param[in]   - Application event
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
    /* This is a week implementation. The application may override this function. */
}
