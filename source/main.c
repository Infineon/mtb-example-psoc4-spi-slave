/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4 SPI Slave code example for
*              ModusToolbox.
*
* Related Document: See README.md 
*
*******************************************************************************
* (c) 2020-2026, Infineon Technologies AG, or an affiliate of Infineon
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

#if defined  CY_DEVICE_PSOC4HV144K
#define PACKET_CMD_DATA0     (0UL)
#define PACKET_CMD_DATA1     (1UL)
#endif

/*******************************************************************************
* Function Prototypes
********************************************************************************/
#if defined  CY_DEVICE_PSOC4HV144K
/* Function to output for port */
static void Port_output(uint8_t);

#else
/* Function to turn ON or OFF the LED based on the SPI Master command. */
static void update_led(uint8_t);
#endif

/* Function to handle the error */
static void handle_error(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. The main function performs following action:
*  1. initial setup of device
*  2. configure the SCB block as SPI slave
*  3. check for spi transfer complete status
*  4. update the LED status based on the command received from the SPI master
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
#if defined  CY_DEVICE_PSOC4HV144K
            /* Communication succeeded. Output signal port. */
            Port_output(rx_buffer[PACKET_CMD_POS]);
#else
            /* Communication succeeded. Update the LED. */
            update_led(rx_buffer[PACKET_CMD_POS]);
#endif
        }
        else
        {
            /* Communication failed */
            handle_error();
        }
    }
}

/*******************************************************************************
* Function Name: Port_output
********************************************************************************
*
* Summary:
*  This function is only used for PSOC4 HVPA 144k Lite kit.
*  This function outputs low and high signal by received command data in SPI Slave.
*  If the command data is 1, it outputs a high signal to the port. 
*  If the command data is 0, it outputs a low signal to the port.
* Parameters:
*  (uint8_t) Cmd - Received command data to output signal
*
* Return:
*  None
*
*******************************************************************************/
#if defined  CY_DEVICE_PSOC4HV144K
static void Port_output(uint8_t Cmd)
{
    /* Control the LED based on command received from Master */
    if(Cmd == PACKET_CMD_DATA0)
    {
      /* Output low signal */
      Cy_GPIO_Write(CYBSP_SW3_PORT, CYBSP_SW3_NUM, 0);
    }
    else if(Cmd == PACKET_CMD_DATA1)
    {
      /* Output low signal */
      Cy_GPIO_Write(CYBSP_SW3_PORT, CYBSP_SW3_NUM, 1);
    }
}
#else
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
#endif

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
