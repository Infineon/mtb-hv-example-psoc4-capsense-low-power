/******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the PSoC 4: CAPSENSE Low Power
 *              code example for ModusToolbox.
 *
 * Related Document: See README.md
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
 
/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "stdio.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CY_ASSERT_FAILED                 (0u)

/* CapSense tuner interface settings */
#define TUNER_UART                      (1u) // Enabling CapSense Tuner with UART
#define TUNER_I2C                       (2u) // Enabling CapSense Tuner with I2C
#define TUNER_PROTOCOL                  TUNER_UART // Selecting Tuner interface

/* ILO Frequency in Hz */
#define ILO_FREQUENCY_HZ                (40000U)

/* WDT interrupt period in milliseconds */
#define WDT_INTERRUPT_INTERVAL_MS        (10U)

/* WDT interrupt priority */
#define WDT_INTERRUPT_PRIORITY           (3u)

/* Define desired delay in microseconds */
#define DESIRED_WDT_INTERVAL             (WDT_INTERRUPT_INTERVAL_MS  * 1000U)

/* LED STATE TOGGLE MACROS */
#define LED_ON    (1u)
#define LED_OFF   (0u)

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/
#if(TUNER_PROTOCOL == TUNER_I2C)
    /* I2C tuner interrupt priority must be higher than CapSense interrupt. */
    #define EZI2C_INTR_PRIORITY              (2u)
    cy_stc_scb_ezi2c_context_t i2c_context;
#else
    /* Uart tuner interrupt priority must be higher than CapSense interrupt. */
    #define UART_INTR_PRIORITY              (2u)

    cy_stc_scb_uart_context_t CYBSP_UART_context; 	   //for CapSense Tuner communication

    /* Define UART ring buffer size to hold extra tuner command packets.
     *  Additional byte is required by the UART ring buffer implementation.
     */
    #define UART_RINGBUFFER_SIZE (CY_CAPSENSE_COMMAND_PACKET_SIZE * 2u + 1u)
#endif /*TUNER PROTOCOL SELECTION*/

/* WDT interrupt service routine configuration */
const cy_stc_sysint_t wdt_isr_cfg =
{
    .intrSrc = srss_wdt_irq_IRQn,    /* Interrupt source */
    .intrPriority = WDT_INTERRUPT_PRIORITY /* Interrupt priority is 3 */
};

/* Variable to check whether WDT interrupt is triggered */
volatile bool interrupt_flag = false;

/* Variable to store the counts required after ILO compensation */
static uint32_t ilo_compensated_counts = 0U;
static uint32_t ilo_cycles  = 0U;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/* WDT interrupt service routine */
void wdt_isr(void);
/* WDT function */
void wdt_trigger(void);

static void initialize_capsense(void);
static void capsense_msc0_isr(void);
static void initialize_capsense_tuner(void);
static void tuner_isr(void);

static void control(void);

#if(TUNER_PROTOCOL == TUNER_UART)

void tuner_send(void * context);
void tuner_receive(uint8_t ** packet, uint8_t ** tunerPacket, void * context);

#endif /* TUNER_UART */

char uartString[50] = " ";

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE
 *  - scan touch input continuously
 *
 * Return:
 *  int
 *
 * Parameters:
 *  void
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and enable interrupt */
    cy_en_sysint_status_t sysintStatus = Cy_SysInt_Init(&wdt_isr_cfg, wdt_isr);
    if(sysintStatus != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_EnableIRQ(wdt_isr_cfg.intrSrc);

    /* Unmask the WDT interrupt */
    Cy_WDT_UnmaskInterrupt();

    /* Enable WDT */
    Cy_WDT_Unlock();
    Cy_WDT_Enable();
    Cy_WDT_Lock();

    /* Initialize CapSense */
    initialize_capsense();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

#if(TUNER_PROTOCOL == TUNER_I2C)
    cy_stc_syspm_callback_params_t ezi2cCallbackParams =
    {
        .base       = CYBSP_EZI2C_HW,
        .context    = &ezi2c_context
    };

    cy_stc_syspm_callback_t ezi2cCallback =
    {
        .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &ezi2cCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
    };
#else
    cy_stc_syspm_callback_params_t uartCallbackParams =
    {
        .base       = scb_1_HW,
        .context    = &CYBSP_UART_context
    };

    cy_stc_syspm_callback_t uartCallback =
    {
        .callback       = (Cy_SysPmCallback)&Cy_SCB_UART_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &uartCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
    };
#endif /* TUNER_UART */

    cy_stc_syspm_callback_params_t sysClkCallbackParams0 =
    {
        .base       = msc_0_msc_0_HW,
        .context    = &cy_capsense_context
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback =
    {
        .callback       = &Cy_SysClk_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams0,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };

#if(TUNER_PROTOCOL == TUNER_I2C)
    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);
#else
    /* Register UART Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&uartCallback);
#endif /* TUNER_UART */

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback);

    /* Start the first scan*/
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);


    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Turns LED ON/OFF and prints the Slider position based on Slider status */
            control();

            /* WDT interrupt source */
            wdt_trigger();

            Cy_CapSense_ScanAllSlots(&cy_capsense_context);

            /* Establishes synchronized communication with the CapSense Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);
        }
    }
}

/*******************************************************************************
 * Function Name: wdt_trigger
 ********************************************************************************
 * Summary:
 *  - Updates the set match value to the WDT block.
 *  - Enters into deep sleep mode.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
void wdt_trigger(void)
{
    if (interrupt_flag)
    {
        /* Clear the interrupt flag */
        interrupt_flag = false;
    }

    /* Start ILO measurement */
    Cy_SysClk_IloStartMeasurement();

    /* Get the ILO compensated counts i.e. the actual counts for the
     desired ILO frequency. ILO default accuracy is +/- 60%.
     Note that DESIRED_WDT_INTERVAL should be less than the total
     count time */
    while (CY_SYSCLK_SUCCESS != Cy_SysClk_IloCompensate(DESIRED_WDT_INTERVAL, &ilo_cycles));
    ilo_compensated_counts = (uint32_t)ilo_cycles;
    /* Stop ILO measurement before entering deep sleep mode */
    Cy_SysClk_IloStopMeasurement();
    /* Enter deep sleep mode */
    Cy_SysPm_CpuEnterDeepSleep();
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense and configures the CapSense
 *  interrupt.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* Capture the CSD/MSC HW block and initialize it to the default state */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

#if(TUNER_PROTOCOL == TUNER_UART)
    /* Register communication callback for UART*/
    cy_capsense_context.ptrInternalContext->ptrTunerSendCallback = tuner_send;
    cy_capsense_context.ptrInternalContext->ptrTunerReceiveCallback = tuner_receive;
#endif /*TUNER_UART*/

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the README.md file
         */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*****************************************************************************
 * Function Name: wdt_isr
 ******************************************************************************
 * Summary:
 * This function is the handler for the WDT interrupt
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *****************************************************************************/
void wdt_isr(void)
{
    /* Clears the WDT match flag */
    Cy_WDT_ClearInterrupt();
    /* Set the interrupt flag */
    interrupt_flag = true;
}

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  UART/I2C module to communicate with the CapSense Tuner tool.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
#if(TUNER_PROTOCOL == TUNER_I2C)

    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t i2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &i2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&i2c_intr_config, tuner_isr);
    NVIC_EnableIRQ(i2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &i2c_context);

    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

#else

    static uint8_t uartRingBuffer[UART_RINGBUFFER_SIZE];
    cy_en_scb_uart_status_t initstatus;

    /* UART interrupt configuration structure */
    const cy_stc_sysint_t uart_intr_config =
    {
        .intrSrc = scb_1_IRQ,
        .intrPriority = UART_INTR_PRIORITY,
    };

    /* Configure and enable UART communication */
    initstatus = Cy_SCB_UART_Init(scb_1_HW, &scb_1_config, &CYBSP_UART_context);

    if(initstatus != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&uart_intr_config, tuner_isr);
    NVIC_EnableIRQ(uart_intr_config.intrSrc);

    Cy_SCB_UART_StartRingBuffer(scb_1_HW, uartRingBuffer, UART_RINGBUFFER_SIZE, &CYBSP_UART_context);

    Cy_SCB_UART_Enable(scb_1_HW);
#endif /*TUNER PROTOCOL SELECTION*/
}

/*******************************************************************************
 * Function Name: tuner_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from EZI2C block.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void tuner_isr(void)
{
#if(TUNER_PROTOCOL == TUNER_I2C)
    Cy_SCB_EZI2C_Interrupt(CYBSP_I2C_HW, &i2c_context);
#else
    Cy_SCB_UART_Interrupt(scb_1_HW, &CYBSP_UART_context);
#endif /*TUNER PROTOCOL SELECTION*/
}

/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(msc_0_msc_0_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: control
********************************************************************************
* Summary:
* Turning LED ON/OFF and prints the Slider position based on Slider status
*
*******************************************************************************/
static void control(void)
{
    if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON0_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM, LED_OFF);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM, LED_ON);
    }
    if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_NUM, LED_OFF);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_NUM, LED_ON);
    }
}

#if(TUNER_PROTOCOL == TUNER_UART)
/*******************************************************************************
* Function Name: tuner_send
********************************************************************************
* Summary:
* Tuner send callback function for UART block
*
*******************************************************************************/
void tuner_send(void * context)
{
    uint8_t uartTxHeader[] = {0x0Du, 0x0Au};
    uint8_t uartTxTail[] = {0x00u, 0xFFu, 0xFFu};
    (void)context;
    Cy_SCB_UART_PutArrayBlocking(scb_1_HW, &(uartTxHeader[0u]), sizeof(uartTxHeader));
    Cy_SCB_UART_PutArrayBlocking(scb_1_HW, (uint8_t *)&cy_capsense_tuner, sizeof(cy_capsense_tuner));
    Cy_SCB_UART_PutArrayBlocking(scb_1_HW, &(uartTxTail[0u]), sizeof(uartTxTail));
    /* Blocking wait for transfer completion */
    while (!Cy_SCB_UART_IsTxComplete(scb_1_HW))
    {
    }
}

/*******************************************************************************
* Function Name: tuner_receive
********************************************************************************
* Summary:
* Tuner received callback function for UART block
*
*******************************************************************************/
void tuner_receive(uint8_t ** packet, uint8_t ** tunerPacket, void * context)
{
    uint32_t i;
    (void)context;
    static uint32_t dataIndex = 0u;
    static uint8_t commandPacket[CY_CAPSENSE_COMMAND_PACKET_SIZE] = {0u};
    uint32_t numBytes;
    while(0u != Cy_SCB_UART_GetNumInRingBuffer(scb_1_HW, &CYBSP_UART_context))
    {
        numBytes = Cy_SCB_UART_GetNumInRingBuffer(scb_1_HW, &CYBSP_UART_context);
        /* Calculate number of bytes to read from ring buffer */
        if ((CY_CAPSENSE_COMMAND_PACKET_SIZE - dataIndex) < numBytes)
        {
            numBytes = CY_CAPSENSE_COMMAND_PACKET_SIZE - dataIndex;
        }
        /* Add received data to the end of commandPacket */
        Cy_SCB_UART_Receive(scb_1_HW, &commandPacket[dataIndex], numBytes, &CYBSP_UART_context);
        dataIndex += numBytes;
        if(CY_CAPSENSE_COMMAND_PACKET_SIZE <= dataIndex)
        {
            if (CY_CAPSENSE_COMMAND_OK == Cy_CapSense_CheckTunerCmdIntegrity(&commandPacket[0u]))
            {
                /* Found a correct command, reset data index and assign pointers to buffers */
                dataIndex = 0u;
                *tunerPacket = (uint8_t *)&cy_capsense_tuner;
                *packet = &commandPacket[0u];
                break;
            }
            else
            {
                /* Command is not correct, remove the first byte in commandPacket buffer */
                dataIndex--;
                for (i = 0u; i < (CY_CAPSENSE_COMMAND_PACKET_SIZE - 1u); i++)
                {
                    commandPacket[i] = commandPacket[i + 1u];
                }
            }
        }
    }
}
#endif /* TUNER_UART */

/* [] END OF FILE */
