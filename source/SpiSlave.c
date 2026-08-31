/******************************************************************************
* File Name:   SpiSlave.c
*
* Description: This file contains function definitions for SPI Slave.
*
*
*******************************************************************************
* (c) 2021-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#include "SpiSlave.h"
#include "Interface.h"


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
cy_stc_scb_spi_context_t sSPI_context;

/* Assign SPI interrupt number and priority */
#define sSPI_INTR_PRIORITY   (3U)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static void SPI_Isr(void);

/*******************************************************************************
 * Function Name: sSPI_Interrupt
 *******************************************************************************
 *
 * Invokes the Cy_SCB_SPI_Interrupt() PDL driver function.
 *
 *******************************************************************************/
static void SPI_Isr(void)
{
    Cy_SCB_SPI_Interrupt(sSPI_HW, &sSPI_context);
}

/*******************************************************************************
* Function Name: init_slave
********************************************************************************
*
* Summary:
*  This function initializes the SPI Slave based on the
*  configuration done in design.modus file.
*
* Parameters:
*  None
*
* Return:
*  (uint32) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32_t init_slave(void)
{
    cy_en_scb_spi_status_t spi_status;
    cy_en_sysint_status_t intr_status;

    /* Configure the SPI block */
    spi_status = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &sSPI_context);

    /* If the initialization fails, return failure status */
    if(spi_status != CY_SCB_SPI_SUCCESS)
    {
        return(INIT_FAILURE);
    }

    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(sSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    /* Populate configuration structure */
    const cy_stc_sysint_t spi_intr_config =
    {
        .intrSrc      = sSPI_IRQ,
        .intrPriority = sSPI_INTR_PRIORITY,
    };

    /* Hook interrupt service routine and enable interrupt */
    intr_status = Cy_SysInt_Init(&spi_intr_config, &SPI_Isr);

    if(intr_status != CY_SYSINT_SUCCESS)
    {
        return(INIT_FAILURE);
    }

    NVIC_EnableIRQ(sSPI_IRQ);

    /* Enable the SPI Slave block */
    Cy_SCB_SPI_Enable(sSPI_HW);

    /* Initialization completed */
    return(INIT_SUCCESS);
}


/******************************************************************************
* Function Name: read_packet
*******************************************************************************
*
* Summary:
*  This function reads the data received by the slave. Note that
*  the below function is blocking until the required number of
*  bytes is received by the slave.
*
* Parameters:
*  - (uint8_t *) rxBuffer - Pointer to the receive buffer where data
*                          needs to be stored
*  - (uint32_t) transferSize - Number of bytes to be received
*
* Return:
*  - (uint32_t) - Returns TRANSFER_COMPLETE if SPI transfer is completed or
*                 returns TRANSFER_FAILURE if SPI tranfer is not successfull
*
******************************************************************************/
uint32_t read_packet(uint8_t *rxBuffer, uint32_t transferSize)
{
    uint32_t slave_status;
    cy_en_scb_spi_status_t status;

    /* Prepare for a transfer. */
    status = Cy_SCB_SPI_Transfer(sSPI_HW, NULL, rxBuffer, transferSize, &sSPI_context);

    if(status == CY_SCB_SPI_SUCCESS)
    {
        /* Blocking wait for transfer completion */
        while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE &\
                          Cy_SCB_SPI_GetTransferStatus(sSPI_HW, &sSPI_context)))
        {
        }

        /* Check start and end of packet markers */
        if ((rxBuffer[PACKET_SOP_POS] == PACKET_SOP) &&\
                                       (rxBuffer[PACKET_EOP_POS] == PACKET_EOP))
        {
            /* Data received correctly */
            slave_status = TRANSFER_COMPLETE;
        }
        else
        {
            /* Data was not received correctly */
            slave_status = TRANSFER_FAILURE;
        }
    }
    else
    {
        /* SPI transfer not initiated */
        slave_status = TRANSFER_FAILURE;
    }

   return slave_status;
}



