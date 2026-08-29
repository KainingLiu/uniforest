/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — Pi-controlled executor architecture
  *
  *  Architecture:
  *    Raspberry Pi (上位机) ←→ STM32 A-board (下位机) via UART7
  *    STM32 executes low-level commands, Pi handles all decision logic.
  *
  *  Control loops (ISR-driven, non-blocking):
  *    TIM6 @ 1 kHz  → chassis speed PID + torque output
  *    TIM7 @ 100 kHz → stepper pulse generation
  *
  *  Safety:
  *    Communication timeout (200ms) → emergency stop + RC fallback
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
#include "remote_control.h"   /* SBUS — kept as backup */
#include "servo.h"
#include "suction.h"
#include "stepper.h"
#include "imu.h"
#include "protocol.h"         /* UART7 Pi communication */
#include "debug_telem.h"      /* VOFA+ JustFloat debug telemetry (USART3) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PID_DT               0.001f   /* 1 ms (1000 Hz) */

/* ---- RC Fallback Yaw PID ---- */
#define RC_YAW_KP            100.0f
#define RC_YAW_KI              0.8f
#define RC_YAW_MAX_RPM       3000.0f
#define RC_MAX_LINEAR_CM_S    250.0f
#define RC_MAX_ANGULAR_DEG_S  300.0f

/* ---- Chassis Speed PID (TIM6 @ 1 kHz) ---- */
#define CHASSIS_PID_DT         0.001f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static TIM_HandleTypeDef htim6_chassis = { .Instance = TIM6 };
static TIM_HandleTypeDef buzzer_tim   = { .Instance = TIM12 };

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

  /* ---- Initialize Chassis Motors (CAN1, DC24V, PID) ---- */
  Motor3508_Init();

  /* ---- Initialize Servos (TIM2 + TIM5 PWM, 50 Hz) ---- */
  Servo_Init();

  /* ---- Suction pump and release valve: PD12 / PD13 ---- */
  Suction_Init();

  /* ---- Initialize Steppers (GPIO + TIM7 @ 100 kHz) ---- */
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

  /* Allow everything to stabilize */
  HAL_Delay(1000);

  /* Home all servos to idle positions */
  Servo_HomeAll();

  /* ---- Initialize JY61P IMU (USART2) ---- */
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

  /* ---- Initialize RC SBUS receiver (backup only) ---- */
  Remote_Control_Init();

  /* ---- Initialize Pi Communication (UART7, 115200 bps) ---- */
  Protocol_Init();

  /* USART3 debug telemetry is disabled: the single DAPLink is used by UART7. */

  /* ---- TIM6: 1 kHz chassis PID loop ---- */
  /* APB1 timer clock = 90 MHz → PSC=89, ARR=899 → 90M/90/1000 = 1 kHz */
  {
      __HAL_RCC_TIM6_CLK_ENABLE();
      htim6_chassis.Init.Prescaler         = 89;
      htim6_chassis.Init.CounterMode       = TIM_COUNTERMODE_UP;
      htim6_chassis.Init.Period            = 899;
      htim6_chassis.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
      htim6_chassis.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
      HAL_TIM_Base_Init(&htim6_chassis);

      __HAL_TIM_ENABLE_IT(&htim6_chassis, TIM_IT_UPDATE);
      HAL_TIM_Base_Start_IT(&htim6_chassis);

      HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);   /* TIM6 IRQ = 54 */
      HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }

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

      buzzer_tim.Init.Prescaler         = 29;       /* 90 MHz / 30 = 3 MHz */
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

  /* ---- Ready: green LED flash ---- */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);

  /* ---- Pi will set telemetry rate via CMD_SET_TELEM_RATE ---- */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- 1. CAN motor feedback is accumulated in CAN1_RX0 IRQ ---- */

    /* ---- 2. Poll IMU ---- */
    IMU_Update();

    /* ---- 3. Process inbound Pi commands (UART7) ---- */
    Protocol_RxPoll();

    /* Non-blocking 1-second release valve timeout. */
    Suction_Update();

    /* ---- 4. Send telemetry (at configured rate) ---- */
    Protocol_TelemTick();

    /* ---- 5. Communication timeout → emergency stop ---- */
    if (!Protocol_IsAlive())
    {
        /* Pi communication lost — stop chassis motors */
        Motor3508_StopAll();
        Suction_AllOff();

        /* Blink red LED to indicate comm loss */
        static uint32_t last_blink = 0;
        if (HAL_GetTick() - last_blink > 500)
        {
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
            last_blink = HAL_GetTick();
        }
    }

    /* ---- 6. Loop delay (1 ms → ~1 kHz main loop) ---- */
    HAL_Delay(1);
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

/**
 * @brief  TIM6 ISR — Chassis speed PID @ 1 kHz
 * @note   Runs Motor3508_UpdateAllSpeedPID each tick.
 *         This is the core closed-loop control for chassis motors.
 */
void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim6_chassis, TIM_FLAG_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim6_chassis, TIM_FLAG_UPDATE);
        Motor3508_UpdateAllSpeedPID(CHASSIS_PID_DT);
    }
}

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
