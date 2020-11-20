/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4 SPI Slave code example for
*              ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "SpiSlave.h"
#include "Interface.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* Number of elements in the transmit and receive buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS   (3UL)
#define SIZE_OF_ELEMENT      (1UL)
#define SIZE_OF_PACKET       (NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
/* Function to turn ON or OFF the LED based on the SPI Master command. */
static void update_led(uint8_t);

/* Function to handle the error */
static void handle_error(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - configure the SCB block as SPI slave
*  - check for spi transfer complete status
*  - update the LED status based on the command received from the SPI master
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Buffer to save the received data by the slave */
    uint32_t status = 0;

    uint8_t rx_buffer[SIZE_OF_PACKET] = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the SPI Slave */
    status = init_slave();

    if(status == INIT_FAILURE)
    {
        /* NOTE: This function will block the CPU forever */
        handle_error();
    }

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Get the bytes received by the slave */
        status = read_packet(rx_buffer, SIZE_OF_PACKET);

        /* Check whether the slave succeeded in receiving the required number
         * of bytes and in the right format */
        if(status == TRANSFER_COMPLETE)
        {
            /* Communication succeeded. Update the LED. */
            update_led(rx_buffer[PACKET_CMD_POS]);
        }
        else
        {
            /* Communication failed */
            handle_error();
        }
    }
}


/*******************************************************************************
* Function Name: update_led
********************************************************************************
*
* Summary:
*  This function updates the LED based on the command received by
*  the SPI Slave from Master.
*
* Parameters:
*  (uint8_t) LED_Cmd - command to turn LED ON or OFF
*
* Return:
*  None
*
*******************************************************************************/
static void update_led(uint8_t LED_Cmd)
{
    /* Control the LED based on command received from Master */
    if(LED_Cmd == CYBSP_LED_STATE_ON)
    {
        /* Turn ON the LED */
        Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
    }

    if(LED_Cmd == CYBSP_LED_STATE_OFF)
    {
        /* Turn OFF the LED */
        Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
    }
}


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
*
* Summary:
*  This is a blocking function. It disables the interrupt and waits
*  in an infinite loop. This function is called when an error is
*  encountered during initialization of the blocks or during
*  SPI communication.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}
}

/* [] END OF FILE */
