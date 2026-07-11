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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor3508.h"
#include "remote_control.h"
#include "servo.h"
#include "stepper.h"
#include "actions.h"
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Control Timing ---- */
#define PID_DT               0.001f /* PID loop period: 1 ms (1000 Hz) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static TIM_HandleTypeDef buzzer_tim = { .Instance = TIM12 };
static uint8_t  g_ch6_armed   = 1;   /* CH6 centre → armed; action/RC-key → disarmed */
static uint8_t  g_ch6_trig_cnt = 0;  /* trigger debounce: consecutive action-zone samples */
#define CH6_TRIG_DEBOUNCE_CNT   2    /* ~20 ms stable in action zone before firing */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  /* ---- Initialize Chassis Motors (CAN1 + DC24V power) ---- */
  Motor3508_Init();

  /* ---- Initialize Servos (TIM2 + TIM5 PWM, 50 Hz) ---- */
  Servo_Init();

  /* ---- Initialize Stepper Motors (GPIO) ---- */
  Stepper_Init();

  /* ---- A-board LEDs: RED=PE11, GREEN=PF14 (active-low: 0=ON) ---- */
  {
      GPIO_InitTypeDef l = {0};
      l.Mode  = GPIO_MODE_OUTPUT_PP;
      l.Pull  = GPIO_NOPULL;
      l.Speed = GPIO_SPEED_FREQ_LOW;

      __HAL_RCC_GPIOE_CLK_ENABLE();
      l.Pin = GPIO_PIN_11;
      HAL_GPIO_Init(GPIOE, &l);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);  /* off */

      __HAL_RCC_GPIOF_CLK_ENABLE();
      l.Pin = GPIO_PIN_14;
      HAL_GPIO_Init(GPIOF, &l);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);  /* off */
  }

  /* Allow everything to stabilize (C620 self-check already done in Motor3508_Init) */
  HAL_Delay(1000);

  /* Home all servos to idle positions */
  Actions_ServoHome();

  /* ---- Initialize JY61P IMU (USART2, PD5=TX, PD6=RX) ---- */
  uint8_t imu_status = IMU_Init();
  if (imu_status != IMU_OK)
  {
      for (uint8_t i = 0; i < imu_status; i++)
      {
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
          HAL_Delay(80);
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
          HAL_Delay(80);
      }
  }
  else
  {
      uint32_t imu_t0 = HAL_GetTick();
      while (!IMU_IsReady() && (HAL_GetTick() - imu_t0 < 500))
      {
          IMU_Update();
          HAL_Delay(5);
      }
      if (IMU_IsReady())
      {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
          HAL_Delay(200);
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
      }
  }

  /* ---- Initialize RC SBUS receiver ---- */
  Remote_Control_Init();

  /* ---- Ready: green LED flash ---- */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);

  /* ---- Buzzer: PH6 = TIM12_CH1, 3 kHz PWM ---- */
  {
      __HAL_RCC_TIM12_CLK_ENABLE();
      __HAL_RCC_GPIOH_CLK_ENABLE();

      GPIO_InitTypeDef g = {0};
      g.Mode      = GPIO_MODE_AF_PP;
      g.Pull      = GPIO_NOPULL;
      g.Speed     = GPIO_SPEED_FREQ_LOW;
      g.Alternate = GPIO_AF9_TIM12;
      g.Pin       = GPIO_PIN_6;
      HAL_GPIO_Init(GPIOH, &g);

      buzzer_tim.Init.Prescaler         = 29;       /* 90 MHz / 30 = 3 MHz */       /* 180 MHz / 60 = 3 MHz */
      buzzer_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
      buzzer_tim.Init.Period            = 999;      /* 3 MHz / 1000 = 3 kHz */
      buzzer_tim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
      buzzer_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
      HAL_TIM_PWM_Init(&buzzer_tim);

      TIM_OC_InitTypeDef oc = {0};
      oc.OCMode     = TIM_OCMODE_PWM1;
      oc.Pulse      = 500;                         /* 50% duty */
      oc.OCPolarity = TIM_OCPOLARITY_HIGH;
      oc.OCFastMode = TIM_OCFAST_DISABLE;
      HAL_TIM_PWM_ConfigChannel(&buzzer_tim, &oc, TIM_CHANNEL_1);
  }

  /* ---- Arming Phase 1: sticks to corners (CH1+CH2+CH4 > 1900us), CH6 1400-1600us ---- */
  while (1)
  {
      Motor3508_RxPoll();
      IMU_Update();
      if (SBUS_IsValid())
      {
          uint16_t ch6 = SBUS_GetChannel(5);
          if (SBUS_GetChannel(1) > SBUS_ARM_THRESHOLD &&   /* CH2: forward */
              SBUS_GetChannel(0) > SBUS_ARM_THRESHOLD &&   /* CH1: lateral */
              SBUS_GetChannel(3) > SBUS_ARM_THRESHOLD &&   /* CH4: rotation */
              ch6 > SBUS_CH6_ARM_LOW &&                     /* CH6 > 1400us */
              ch6 < SBUS_CH6_ARM_HIGH)                      /* CH6 < 1600us */
              break;
      }
      HAL_Delay(10);
  }

  /* ---- Arming Phase 2: sticks return to center (1024 ± 200), CH6 1400-1600us ---- */
  while (1)
  {
      Motor3508_RxPoll();
      IMU_Update();
      if (SBUS_IsValid())
      {
          uint16_t ch1 = SBUS_GetChannel(0);
          uint16_t ch2 = SBUS_GetChannel(1);
          uint16_t ch3 = SBUS_GetChannel(2);
          uint16_t ch4 = SBUS_GetChannel(3);
          uint16_t ch6 = SBUS_GetChannel(5);
          if (ch1 > 824 && ch1 < 1224 &&
              ch2 > 824 && ch2 < 1224 &&
              ch3 > 824 && ch3 < 1224 &&
              ch4 > 824 && ch4 < 1224 &&
              ch6 > SBUS_CH6_ARM_LOW &&
              ch6 < SBUS_CH6_ARM_HIGH)
              break;
      }
      HAL_Delay(10);
  }

  /* Armed: buzzer beep 200 ms */
  HAL_TIM_PWM_Start(&buzzer_tim, TIM_CHANNEL_1);
  HAL_Delay(200);
  HAL_TIM_PWM_Stop(&buzzer_tim, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- Poll CAN and IMU ---- */
    Motor3508_RxPoll();
    IMU_Update();

    /* ---- CH6 mode dispatch ---- */
    if (SBUS_IsValid())
    {
        uint16_t ch6 = SBUS_GetChannel(5);

        /* Arm when CH6 is in the centre safe zone (action wait-loop already debounces) */
        if (ch6 > SBUS_CH6_ARM_LOW && ch6 < SBUS_CH6_ARM_HIGH)
            g_ch6_armed = 1;

        if (ch6 > SBUS_CH6_HIGH)
        {
            /* Grap zone — debounced trigger */
            if (g_ch6_armed)
            {
                if (g_ch6_trig_cnt < CH6_TRIG_DEBOUNCE_CNT)
                    g_ch6_trig_cnt++;
                if (g_ch6_trig_cnt >= CH6_TRIG_DEBOUNCE_CNT)
                {
                    g_ch6_armed    = 0;
                    g_ch6_trig_cnt = 0;
                    uint16_t ch3 = SBUS_GetChannel(2);
                    if (ch3 > 1592)  /* CH3 > 1875 us PWM → Grap2 (27cm) */
                        Actions_Grap2();
                    else
                        Actions_Grap1();
                }
            }
        }
        else if (ch6 < SBUS_CH6_LOW)
        {
            /* Build zone — debounced trigger */
            if (g_ch6_armed)
            {
                if (g_ch6_trig_cnt < CH6_TRIG_DEBOUNCE_CNT)
                    g_ch6_trig_cnt++;
                if (g_ch6_trig_cnt >= CH6_TRIG_DEBOUNCE_CNT)
                {
                    g_ch6_armed    = 0;
                    g_ch6_trig_cnt = 0;
                    Actions_Build();
                }
            }
        }
        else
        {
            /* RC zone */
            g_ch6_trig_cnt = 0;
            Remote_Control();
            /* If CH6 still centre after RC exit → key-press → disarm */
            uint16_t ch6_after = SBUS_GetChannel(5);
            if (ch6_after > SBUS_CH6_ARM_LOW && ch6_after < SBUS_CH6_ARM_HIGH)
                g_ch6_armed = 0;
        }
    }

    HAL_Delay(10);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
