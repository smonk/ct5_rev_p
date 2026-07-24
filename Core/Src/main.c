/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "i2s.h"
#include "octospi.h"
// #include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "hope_hal.h"
#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"
#include "my_fx_common.h"
#include "my_fx_pass_through.h"

#include "midi.h"

#include "ct5_buffer.h"
#include "ct5_fx_ct5.h"
#include "ct5_fx_m2r.h"
#include "ct5_fx_m3t.h"
// #include "dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* io structs */
//internal button and switch
hope_btn_and_sw_struct my_btn_and_sw[ HOPE_NUM_BTN_AND_SW ];

//internal pots and cv
hope_pot_and_cvin_struct my_pot_and_cvin[ HOPE_NUM_POTS_AND_CVIN ];

//dac
// hope_dac_struct my_dac[ HOPE_CT5_P_NUM_EXT_DAC ];

//rgb led via pwm
// hope_ext_rgb_led_struct my_ext_rgb_led[ HOPE_856_NUM_EXT_RGB_LEDS ];
hope_pwm_rgb_led_struct my_pwm_rgb_led[ HOPE_NUM_PWM_RGB_LEDS ];

//relays
hope_relay_struct my_relay[ HOPE_NUM_RELAYS ];

/* for ls data port/packet */

//data packet tx
hope_data_packet_struct tx_data_packet_struct_0;
hope_data_packet_struct * my_ls_tx_data_packet = & tx_data_packet_struct_0;

//data packet rx buffer
hope_data_packet_struct rx_data_packet_struct_0;
hope_data_packet_struct * my_ls_rx_data_packet = & rx_data_packet_struct_0;

//this is so the cli can use dma instead of sending 1 char every transmission
hope_data_packet_struct tx_data_packet_struct_1;
hope_data_packet_struct * my_tx_data_packet_for_general_cli = & tx_data_packet_struct_1;

//for embedded cli
EmbeddedCli *cli = NULL;//



/* for midi port */
hope_midi_buffer_struct midi_tx_buffer;
hope_midi_buffer_struct * my_midi_tx_buffer = & midi_tx_buffer;

hope_midi_buffer_struct midi_rx_buffer;
hope_midi_buffer_struct * my_midi_rx_buffer = & midi_rx_buffer;


/*for patching i2s data to dsp algorithms */
hope_dsp_buffer_struct dsp_buffer_struct_0;
hope_dsp_buffer_struct * my_input_dsp_buffer = & dsp_buffer_struct_0;

hope_dsp_buffer_struct dsp_buffer_struct_1;
hope_dsp_buffer_struct * my_output_dsp_buffer = & dsp_buffer_struct_1;

/* for using spi flash dma */
hope_spi_flash_buffer_struct * my_spi_flash_buffer = NULL;


/* a global function pointer for what the led does when not bypassed */
void (*hope_ct5_led_on_setting)( hope_pwm_rgb_led_struct * ) = NULL;
void (*hope_ct5_led_off_setting)( hope_pwm_rgb_led_struct * ) = NULL;

/* ticks */

//for systick
uint8_t my_tick_flag = 0;
uint32_t my_tick_count;
uint8_t my_tick_expired;

//for codec dma/dsp
uint8_t my_tx_tick_flag = 1;
uint8_t my_rx_tick_flag = 1;
uint8_t my_i2s_tick_expired = 0;
uint8_t my_i2s_tick_count = 0;



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	// MX_GPIO_Init();
	// MX_ADC1_Init();
	// MX_I2C1_Init();
	// MX_I2S1_Init();
	// MX_OCTOSPI1_Init();
	// MX_TIM4_Init();
	// MX_USART1_UART_Init();
	// MX_USART3_UART_Init();
	// MX_SPI4_Init();
	MX_DMA_Init();

	/* USER CODE BEGIN 2 */
  __enable_irq();

  //put the codec in reset 
  hope_codec_ctl_port_enter_reset();

	//uses timer 6 as a system tick
	hope_timer_as_tick_init();

	//sets up the gpios
	hope_btn_and_sw_init();

	//init the button struct for ct5
	hope_ct5_btn_and_sw_init( my_btn_and_sw );

  //sets up the pwm hardware gpios and timer for rgb leds
  hope_pwm_rgb_led_init();
  // hope_pwm_rgb_test_just_as_gpio();
  //set teh on off functions
  hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_blue;
  hope_ct5_led_off_setting = hope_ct5_pwm_rgb_set_off;

  //init the led struct for the pwm rgb leds
  hope_ct5_pwm_rgb_leds_init( my_pwm_rgb_led );

  //setup the relays
  //sets up the gpios
  hope_relay_init();
  hope_relay_struct_init( my_relay );

	//for debug, only 2 leds on this one
	hope_dgb_btn_and_led_init();


	//the ls data port
	hope_ls_data_port_init();
	hope_ls_data_packet_tx_buffer_init( my_ls_tx_data_packet ); // it is already a pointer
	hope_ls_data_packet_rx_buffer_init( my_ls_rx_data_packet );
	hope_ls_cli_tx_buffer_init( my_tx_data_packet_for_general_cli );

	//sets up the embedded cli
	EmbeddedCliConfig *config = embeddedCliDefaultConfig();
	config->maxBindingCount = 16;
	cli = embeddedCliNew(config);
	cli->writeChar = write_char_to_dma_buffer;
	init_cli_bindings();

    /* midi init */

    //sets up the gpio and the dma tx/rx of the uart
    hope_midi_init();

    //initialize the midi buffers
    hope_midi_tx_buffer_init( my_midi_tx_buffer ); //we dont need this but do it for compatibility
    hope_midi_rx_buffer_init( my_midi_rx_buffer );

    //turn on the rx dma
    hope_midi_rx_dma_receive_enable();

    //this is the init function from libmidi, we use that to recieve midi
    midi_init();

    //register the callbacks
    hope_midi_register_callbacks();

    //this is for setting up the cli
    hope_ls_data_port_rx_dma_receive_enable();
    embeddedCliProcess(cli);

    /* memories init */

    //qspi sram init - gpios and peripheral
    hope_qspi_ram_init();

    //now initialize the apm6404 chip
    hope_qspi_ram_test();

    hope_qspi_ram_zero_out();
    hope_qspi_ram_float_read_write_test();

/* flash init */
    // W25Q128JVSJM flash init
    hope_spi_flash_init();

    hope_spi_flash_buffer_init( my_spi_flash_buffer );

    // hope_spi_flash_driver( my_spi_flash_buffer );

    /* pot and cvin init */
    //initializes the gpio, peripheral, and dma
    pot_and_cvin_init();
    //initializes the hope data structure
    hope_ct5_pot_and_cvin_struct_init( my_pot_and_cvin );

    /* i2c init of codec */
    // this function configures the i2c hardware on the hope processor but does not initialize the actual codec. 
    hope_codec_ctl_port_init();
    // initializes a specific codec
   hope_codec_ctl_port_cs4270_init();
   hope_codec_ctl_port_cs4270_on();
    /* i2s init */
    hope_codec_stream_port_init();

    /*dsp buffers init */
    hope_dsp_init_input_buffer( my_input_dsp_buffer );
    hope_dsp_init_output_buffer( my_output_dsp_buffer );

    //starts the audio stream
    hope_codec_stream_port_start_duplex_communication();



	/* USER CODE END 2 */
 

  hope_dbg_btn_and_led_led_on(1);
  hope_dbg_btn_and_led_led_off(2);
  uint32_t i = 0;
  uint32_t n = 0;
  uint32_t N = 100;

  uint32_t time_out = 0;

  while (1)
  {
    if(time_out)
    {
      while(1);
    }
    while( ( my_tx_tick_flag  == 1 ) || ( my_rx_tick_flag == 1 ) ){};
    my_tx_tick_flag = 1;
    my_rx_tick_flag = 1;
    my_i2s_tick_count++;

    time_out = 1;

    n++;
    if( n > N )
    {
      n = 0;
      // HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_3 );
      hope_dbg_btn_and_led_led_toggle(1);
      hope_dbg_btn_and_led_led_toggle(2);
    }

    //the dsp function
    // my_fx_pass_through( my_input_dsp_buffer, my_output_dsp_buffer );
    // ct5_fx_ct5( my_input_dsp_buffer, my_output_dsp_buffer );
    // ct5_fx_m2r( my_input_dsp_buffer, my_output_dsp_buffer );
    ct5_fx_m3t( my_input_dsp_buffer, my_output_dsp_buffer );

    //start an adc conversion
    hope_pot_and_cvin_start_dma_conversion();

    //scans the buttons local to hope processor
    hope_btn_and_sw_update( my_btn_and_sw );

    //midi rx and process
    hope_midi_read_all_pending_bytes( my_midi_rx_buffer );

    //for the terminal, this function handles periodic rx and tx    
    hope_port_uart_cli_test( cli, my_ls_rx_data_packet );

    //update led
    // hope_ct5_pwm_rgb_leds_tick( my_pwm_rgb_led );
  
    //this is the bypass functioanlity
    static uint8_t bypass = 0;
    if( my_btn_and_sw[7].pressed_event_flag )
    {
      //bypass = 1 means active effect
      //bypass = 0 means bypassed
      my_btn_and_sw[7].pressed_event_flag = 0;
      if( bypass )
      {
        bypass = 0;
      }
      else
      {
        bypass = 1;
      }
      my_relay[0].current_state = bypass;
      my_relay[1].current_state = bypass;

      //now turn on or off the pwm leds based on bypass
      // float temp_brightness = 0;
      // if( bypass )
      // {
      //   temp_brightness = 0.75;
      // }
      // else
      // {
      //   temp_brightness = 0;
      // }

      // for( uint8_t i = 0; i < HOPE_NUM_EXT_PWM_LEDS; i++ )
      // {
      //   my_ext_pwm_led[i].target_brightness = temp_brightness;
      //   my_ext_pwm_led[i].mode = PWM_LED_MODE_STATIC;

      // }
      

    }
    if(bypass)
    {

      // hope_ct5_pwm_rgb_leds_driver( my_pwm_rgb_led );
      hope_ct5_led_on_setting( my_pwm_rgb_led );
    }
    else
    {
      // my_pwm_rgb_led[0].target_brightness[0] = 0;
      // my_pwm_rgb_led[0].target_brightness[1] = 0;
      // my_pwm_rgb_led[0].target_brightness[2] = 0;

      hope_ct5_led_off_setting( my_pwm_rgb_led );
    }
    hope_ct5_pwm_rgb_leds_tick( my_pwm_rgb_led );

    hope_relay_tick( my_relay );

    time_out = 0;

  }



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //for internal systick testing

  while (1)
  {
    while( my_tick_flag  == 1 ){}; 

    my_tick_flag = 1;
    my_tick_count++;

    n++;
    if( n > N )
    {
      n = 0;
      hope_dbg_btn_and_led_led_toggle(1);
      hope_dbg_btn_and_led_led_toggle(2);

      i++;
      if( i > 2 )
      {
        i = 0;
      }

      //clear gpiod pin 12 13 14 with ll driver
      LL_GPIO_ResetOutputPin( GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 );

      switch( i )
      {
      case 0:
        //set gpiod pin 12 to high with ll gpio driver
        LL_GPIO_SetOutputPin( GPIOD, LL_GPIO_PIN_12 );
        break;

      case 1:
        //set gpiod pin 13 to low with ll gpio driver
        LL_GPIO_SetOutputPin( GPIOD, LL_GPIO_PIN_13 );
        break;

      case 2:
        //set gpiod pin 14 to high with ll gpio driver
        LL_GPIO_SetOutputPin( GPIOD, LL_GPIO_PIN_14 );
        break;

      }


    }

    //a function to test if the rgb led is working properly
    hope_ct5_pwm_rgb_leds_driver( my_pwm_rgb_led );

    //test cli
    hope_port_uart_cli_test( cli, my_ls_rx_data_packet );

    //the button and switch tick/update
    hope_btn_and_sw_update( my_btn_and_sw );


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_3)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1Q_Enable();
  LL_RCC_PLL1R_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(4);
  LL_RCC_PLL1_SetN(90);
  LL_RCC_PLL1_SetP(1);
  LL_RCC_PLL1_SetQ(3);
  LL_RCC_PLL1_SetR(4);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {

  }
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);
  LL_SetSystemCoreClock(540000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(8);
  LL_RCC_PLL2_SetN(160);
  LL_RCC_PLL2_SetP(3);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{

  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  // /* Disables the MPU */
  // LL_MPU_Disable();

  // /** Initializes and configures the Region and the memory to be protected
  // */
  // LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER0, 0x87, 0x0, LL_MPU_REGION_SIZE_4GB|LL_MPU_TEX_LEVEL0|LL_MPU_REGION_NO_ACCESS|LL_MPU_INSTRUCTION_ACCESS_DISABLE|LL_MPU_ACCESS_SHAREABLE|LL_MPU_ACCESS_NOT_CACHEABLE|LL_MPU_ACCESS_NOT_BUFFERABLE);
  
  // LL_MPU_ConfigRegion(
  //     LL_MPU_REGION_NUMBER1, 0x87, 0x0, LL_MPU_REGION_SIZE_8MB|LL_MPU_TEX_LEVEL0|LL_MPU_REGION_FULL_ACCESS|LL_MPU_INSTRUCTION_ACCESS_DISABLE|LL_MPU_ACCESS_SHAREABLE|LL_MPU_ACCESS_NOT_CACHEABLE|LL_MPU_ACCESS_NOT_BUFFERABLE);
  
  // /* Enables the MPU */
  // LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}