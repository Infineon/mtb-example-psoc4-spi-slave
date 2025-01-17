/******************************************************************************
* File Name:   SpiSlave.c
*
* Description: This file contains function definitions for SPI Slave.
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
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



