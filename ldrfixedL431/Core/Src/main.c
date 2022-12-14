/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "DTW_counter.h"
#include "system_reset.h"
#include "morse.h"
#include "canwinch_ldrproto.h"

#define SYSCLOCKFREQ 16000000

/* &&&&&&&&&&&&& Each node on the CAN bus gets a unit number &&&&&&&&&&&&&&&&&&&&&&&&&& */
#include "db/gen_db.h"
#define   IAMUNITNUMBER   CANID_UNIT_BMS03  /* Fixed loader (serial number concept) */
#define   BOARDTYPE   3   /* Board type (e.g. shaft encoder, manifold pressure, tension,... */
/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */

#if 0
/* Specify msg buffer and max useage for TX, RX0, and RX1. */
const struct CAN_INIT msginit = { \
96, /* Total number of msg blocks. */\
32, /* TX can use this huge ammount. */\
16, /* RX0 can use this many. */\
8 /* RX1 can use this piddling amount. */\
};

struct CAN_CTLBLOCK* pctl1;
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 uint32_t flashblocksize1;
 uint32_t unique_id[3];
 uint16_t flashsize;
 uint8_t ldr_phase;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
char* buffer = "\n\rX ldrfixedL431 started 123";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif



PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   uint32_t dtw; // DTW time
/* --------------------- Type of RESET detection and dispatch ------------------------------------- */
  extern void* __appjump; // Defined in ldr.ld file
  /* Check type of RESET and set us on the correct journey. */
  uint32_t rcc_csr = RCC->CSR;  // Get reset flags
  RCC->CSR |= (1 << 23); // Bit 23 RMVF: Remove reset flag (prep for next RESET)
  if (rcc_csr & (1 << 29))  // Was it Independent watchdog reset flag?
  { // Here, yes. This should be the result of a valid load process

    /* Shift vector table to new position. */
    *(uint32_t*)ADDR_SCB_VTOR = 0x8000;

    __DSB(); // Data barrier sync, JIC

    /* Jump to app. */
    (*(  (void (*)(void))__appjump)  )(); // Indirect via label in .ld file
  }
  /* Here, not the IWDG flag, so printf some stuff and wait for possible download, before app jump. */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  DTW_counter_init();

   printf("\n\n\n\r######### ldrfixedL431 STARTS");

  #define DTW_INC_printf (1000 * 16000) // 16 MHz sysclock
  uint32_t DTW_next_LED = DTWTIME;
  #define DTW_INC_LED (250 * 16000)
  uint32_t DTW_next_printf = DTWTIME;
 
  unsigned int mctr = 0;

  printf ("\n\rControl/status register (RCC_CSR) : %08x\n\r",(unsigned int)RCC->CSR);
  RCC->CSR |= (1 << 24);
  printf ("Control/status register (RCC_CSR) : %08x After RMVF written\n\r",(unsigned int)RCC->CSR);
  RCC->CSR |= (7 << 29);
  printf ("Control/status register (RCC_CSR) : %08x After LPWR written\n\r",(unsigned int)RCC->CSR);

  flashblocksize1 = 2048; // 
  unique_id[0] = *(uint32_t*)(ADDR_UNIQUE_ID+0);
  unique_id[1] = *(uint32_t*)(ADDR_UNIQUE_ID+1);
  unique_id[2] = *(uint32_t*)(ADDR_UNIQUE_ID+2);
  flashsize = *(uint16_t*)ADDR_FLASH_SIZE;
  printf("\n\rUnique ID : %08X%08X%08X",(unsigned int)unique_id[0],(unsigned int)unique_id[1],(unsigned int)unique_id[2]);
  printf("\n\rFlash size:     %uK\n\r",(unsigned int)flashsize);

  /* ----------------------- Header for columns of CAN error printf ------------------------------------- */
//canwinch_pod_common_systick2048_printerr_header();
/* ---------------- When CAN interrupts are enabled reception of msgs begins! ------------------------ */
//  can_msg_reset_init(pctl1, IAMUNITNUMBER); // Specify CAN ID for this unit for msg caused RESET

// RX msgs begin immediately following enabling CAN interrupts.  Get 'peek' 'toss' of RX msgs going soon.
//  can_driver_enable_interrupts(); // Enable CAN interrupts
/* -------------- Get the program loader stuff setup -------------------------------------- */
  canwinch_ldrproto_init(IAMUNITNUMBER);

//  uint32_t* pcrcblk = (uint32_t*)((uint32_t)((uint8_t*)*&__appjump + 7 + 0)); // First table entry = number of crcblocks  
//  printf(  "(uint32_t)*pcrcblk: %08X\n\r", (unsigned int)*pcrcblk++ );

//  uint32_t flashincrement = SYSCLOCKFREQ/6;

  // for debug multipy the increment to give the hapless Op time to think
  uint32_t can_waitdelay_ct = (DTWTIME + 5*SYSCLOCKFREQ); // Set number secs to wait before jumping to app

  printf("\n\r\nAddresses: &__appjump %08X   __appjump %08X\n\r\n",(unsigned int)&__appjump, (unsigned int)__appjump);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    HAL_Delay(1000);

    if ((int32_t)(DTWTIME - DTW_next_printf) > 0)
    {
      DTW_next_printf = DTW_next_printf + DTW_INC_printf;
      printf("\n\r%5u ldrfixedL431 waiting",mctr++);
    }

    /* LED blinking */
    if ((int32_t)(DTWTIME - DTW_next_LED) > 0)
    {
      DTW_next_LED = DTW_next_LED + DTW_INC_LED;
      if ((GPIOB->ODR & (1<<1)) == 0) 
           GPIOB->BSRR = (1<<1);
      else 
           GPIOB->BSRR = (1<<(1+16));
    }

    /* Do loader'ing, if there are applicable msgs. */
//    canwinch_ldrproto_poll();

    /* Have we written to flash?  If so, don't jump to the the app unless commanded. */
    if (ldr_phase == 0)
    { // Here, we haven't done anything to disturb the integrity of the app
      if (  ((int)can_waitdelay_ct - (int)(DTWTIME)) < 0 )
      { // We timed out.
        if (((unsigned int)&__appjump > (unsigned int)__appjump) || ((unsigned int)__appjump >= (unsigned int)0x08040000))
        { // Here, jump address is bogus
          printf("\n\r\n#### At offset %08X address %08X is bogus ####\n\r\n",(unsigned int)&__appjump, (unsigned int)__appjump);
          dtw = (DTWTIME + (SYSCLOCKFREQ/2)); // Wait 1/2 sec for printf to complete
          while (  ((int)dtw - (int)(DTWTIME)) > 0 );
          system_reset(); // Software reset
        }
        dtw = (DTWTIME + (SYSCLOCKFREQ/2)); // Wait 1/2 sec for printf to complete
        while (  ((int)dtw - (int)(DTWTIME)) > 0 );
        /* Set Indpendent Watch Dog and let it cause a reset. */
        RCC->CSR |= (1<<0);   // LSI enable, necessary for IWDG
        while ((RCC->CSR & (1<<1)) == 0);  // wait till LSI is ready
          IWDG->KR  = 0x5555; // enable write to PR, RLR
          IWDG->PR  = 0;      // Init prescaler
          IWDG->RLR = 0x02;   // Init RLR
          IWDG->KR  = 0xAAAA; // Reload the watchdog
          IWDG->KR  = 0xCCCC; // Start the watchdog
        while (1==1);
      }
    }         
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 16;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GRN_Pin|LED_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_GRN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GRN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
