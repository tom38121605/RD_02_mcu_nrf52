/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port_platform.h"
#include "deca_device_api.h"

/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/


/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/****************************************************************************//**
 *
 *                              Time section
 *
 *******************************************************************************/

#if 0
/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t
portGetTickCnt(void)
{
    return HAL_GetTick();
}
#endif

#if 0
/* @fn    usleep
 * @brief precise usleep() delay
 * */
int usleep(useconds_t usec)
{
    int i,j;
#pragma GCC ivdep
    for(i=0;i<usec;i++)
    {
#pragma GCC ivdep
        for(j=0;j<2;j++)
        {
            __NOP();
            __NOP();
        }
    }
    return 0;
}
#endif

#if 0
/* @fn    Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
    HAL_Delay(x);
}
#endif
/****************************************************************************//**
 *
 *                              END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              Configuration section
 *
 *******************************************************************************/

#if 0
/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
    return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
            (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);
}
#endif
/****************************************************************************//**
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          DW1000 port section
 *
 *******************************************************************************/




/* @fn      setup_DW1000RSTnIRQ
 * @brief   setup the DW_RESET pin mode
 *          0 - output Open collector mode
 *          !0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable)
{
#if 0
    GPIO_InitTypeDef GPIO_InitStruct;

    if(enable)
    {
        // Enable GPIO used as DECA RESET for interrupt
        GPIO_InitStruct.Pin = DW_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(EXTI0_IRQn);     //pin #0 -> EXTI #0
        HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);    //pin #0 -> EXTI #0

        //put the pin back to tri-state ... as
        //output open-drain (not active)
        GPIO_InitStruct.Pin = DW_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_SET);
    }
#endif
}



/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
	#if 0
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);
    Sleep(1);
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
    Sleep(7);                       //wait 7ms for DW1000 XTAL to stabilise
	#endif
}

/* @fn      port_wakeup_dw1000_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup time
 * */
void port_wakeup_dw1000_fast(void)
{
	#if 0
    #define WAKEUP_TMR_MS   (10)

    uint32_t x = 0;
    uint32_t timestamp = HAL_GetTick(); //protection

    setup_DW1000RSTnIRQ(0);         //disable RSTn IRQ
    signalResetDone = 0;            //signalResetDone connected to RST_PIN_IRQ
    setup_DW1000RSTnIRQ(1);         //enable RSTn IRQ
    port_SPIx_clear_chip_select();  //CS low

    //need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt is not reliable
    //when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL lock (in 5 us)
    while((signalResetDone == 0) && \
          ((HAL_GetTick() - timestamp) < WAKEUP_TMR_MS))
    {
        x++;     //when DW1000 will switch to an IDLE state RSTn pin will high
    }
    setup_DW1000RSTnIRQ(0);         //disable RSTn IRQ
    port_SPIx_set_chip_select();    //CS high

    //it takes ~35us in total for the DW1000 to lock the PLL, download AON and go to IDLE state
    usleep(35);
		#endif
}


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
}

//================================================================================================
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
  uint8 idatabuf[DATALEN1]={0};
  uint8 itempbuf[DATALEN1]={0};
	
	uint8 * p1;
	uint32 idatalength=0;
	
	memset(idatabuf, 0, DATALEN1);
	memset(itempbuf, 0, DATALEN1);	
	
	p1=idatabuf;	
	memcpy(p1,headerBuffer, headerLength);
	
	p1 += headerLength;
	memset(p1,0x00,readlength);
	
	idatalength= headerLength + readlength;

	spi_xfer_done = false;
  nrf_drv_spi_transfer(&spi, idatabuf, idatalength, itempbuf, idatalength);
	while(!spi_xfer_done)				
	  ;
	
	p1=itempbuf + headerLength;
	
  memcpy(readBuffer, p1, readlength);
	
	return 0;
} 


int writetospi( uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
  uint8 idatabuf[DATALEN1]={0};
  uint8 itempbuf[DATALEN1]={0};
	
	uint8 * p1;
	uint32 idatalength=0;
	
	memset(idatabuf, 0, DATALEN1);
	memset(itempbuf, 0, DATALEN1);		 
	
	p1=idatabuf;	
	memcpy(p1,headerBuffer, headerLength);
	p1 += headerLength;
	memcpy(p1,bodyBuffer,bodylength);
	
	idatalength= headerLength + bodylength;
	
	spi_xfer_done = false;
	nrf_drv_spi_transfer(&spi, idatabuf, idatalength, itempbuf, idatalength);
	while(!spi_xfer_done)
				;

	return 0;
} 

//------------------------------other---------------------------

#define NRF_DRV_SPI_DEFAULT_CONFIG_2M(id)                       \
{                                                            \
    .sck_pin      = CONCAT_3(SPI, id, _CONFIG_SCK_PIN),      \
    .mosi_pin     = CONCAT_3(SPI, id, _CONFIG_MOSI_PIN),     \
    .miso_pin     = CONCAT_3(SPI, id, _CONFIG_MISO_PIN),     \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = CONCAT_3(SPI, id, _CONFIG_IRQ_PRIORITY), \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_2M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}


#define NRF_DRV_SPI_DEFAULT_CONFIG_8M(id)                       \
{                                                            \
    .sck_pin      = CONCAT_3(SPI, id, _CONFIG_SCK_PIN),      \
    .mosi_pin     = CONCAT_3(SPI, id, _CONFIG_MOSI_PIN),     \
    .miso_pin     = CONCAT_3(SPI, id, _CONFIG_MISO_PIN),     \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = CONCAT_3(SPI, id, _CONFIG_IRQ_PRIORITY), \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_8M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
  nrf_gpio_cfg_output2(DW1000_RST);   
	nrf_gpio_pin_clear(DW1000_RST);  
	nrf_delay_ms(200); 
	nrf_gpio_pin_set(DW1000_RST);  
	nrf_delay_ms(50); 
  nrf_gpio_cfg_input(DW1000_RST, NRF_GPIO_PIN_NOPULL); 
	nrf_delay_ms(2); 
}

/* @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 *          n
 * */
void port_set_dw1000_slowrate(void)
{
	nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG_2M(SPI_INSTANCE);
	spi_config.ss_pin = SPI_CS_PIN;
	APP_ERROR_CHECK( nrf_drv_spi_init(&spi, &spi_config, spi_event_handler) );
	nrf_delay_ms(2);	
}

/* @fn      port_set_dw1000_fastrate
 * @brief   set 8MHz
 *         
 * */
void port_set_dw1000_fastrate(void)
{
	nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG_8M(SPI_INSTANCE);
	spi_config.ss_pin = SPI_CS_PIN;
	APP_ERROR_CHECK( nrf_drv_spi_init2(&spi, &spi_config, spi_event_handler) );
	nrf_delay_ms(2);	
}


void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}


// currently do nothing
uint16 decamutexon(void)           
{
//	u16 j = (u16)(NVIC->ISER[0] & (1 << 5));

	//	if(j) 
//  {
//		NVIC->ISER[0] &= ~(1 << 5); //disable the external interrupt line
//	}
//	return j ; 

return 0;	
}


// currently do nothing
void decamutexoff(uint16 j)       
{
//	if(j) 

	//	{                 
//		NVIC->ISER[0] |= (1 << 5);;
//	}
	;	
}

/****************************************************************************//**
 *
 *                          End APP port section
 *
 *******************************************************************************/



/****************************************************************************//**
 *
 *                              IRQ section
 *
 *******************************************************************************/
#if 0
/* @fn      HAL_GPIO_EXTI_Callback
 * @brief   IRQ HAL call-back for all EXTI configured lines
 *          i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DW_RESET_Pin)
    {
        signalResetDone = 1;
    }
    else if (GPIO_Pin == DW_IRQn_Pin)
    {
        process_deca_irq();
    }
    else
    {
    }
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW1000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW1000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {

        dwt_isr();

    } //while DW1000 IRQ line active
}


/* @fn      port_DisableEXT_IRQ
 * @brief   wrapper to disable DW_IRQ pin IRQ
 *          in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
    NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_EnableEXT_IRQ
 * @brief   wrapper to enable DW_IRQ pin IRQ
 *          in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
    NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}


/* @fn      port_GetEXT_IRQStatus
 * @brief   wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
    return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}


/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return HAL_GPIO_ReadPin(DECAIRQ_GPIO, DW_IRQn_Pin);
}

#endif
/****************************************************************************//**
 *
 *                              END OF IRQ section
 *
 *******************************************************************************/


#if 0

/* DW1000 IRQ handler definition. */
port_deca_isr_t port_deca_isr = NULL;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr)
{
    /* Check DW1000 IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
    if (en)
    {
        port_DisableEXT_IRQ();
    }
    port_deca_isr = deca_isr;
    if (en)
    {
        port_EnableEXT_IRQ();
    }
}

#endif

/****************************************************************************//**
 *
 *******************************************************************************/

