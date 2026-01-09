/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @details        : 本程序基于 STM32G030F6Px，使用 TIM14 约 38.4kHz 中断
  *                 : 实现 7 路 LED 软件 PWM（BSRR 一次写），提供 8 种灯效
  *                 :（流水/呼吸/爆闪/全亮），带伽马校正与占空比双缓冲。
  *                 : 10ms 低频任务完成按键去抖、短按切模式、长按进出设置，
  *                 : 并支持按模式调整速度/爆闪周期/全亮亮度等参数档位显示。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 数学函数用于初始化伽马查找表，仅在启动时执行 */
#include <math.h>
/* memcpy 用于双缓冲拷贝 */
#include <string.h>
#include "flash_cfg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 灯数量与软件 PWM 参数 */
#define LED_COUNT 7U
#define PWM_BITS 6U
#define PWM_STEPS (1U << PWM_BITS)
#define PWM_MAX (PWM_STEPS - 1U)
#define DITHER_BITS (8U - PWM_BITS)
#define DITHER_CYCLE (1U << DITHER_BITS)
#define BAM_FRAME_UNITS ((1U << PWM_BITS) - 1U)
/* 64MHz / (20kHz * 63) ≈ 50.8 -> 51 ticks per BAM base unit. */
#define BAM_BASE_TICKS 51U
#if (PWM_STEPS != (1U << PWM_BITS))
#error "PWM_STEPS must be a power-of-two defined by PWM_BITS."
#endif
/* 低频效果节拍 */
#define EFFECT_TICK_MS 10U
/* 流水步进周期 */
#define STEP_MS 120U
/* 呼吸包络每 tick 变化量 */
#define ENV_DELTA 4U
/* 呼吸曲线参数 */
#define BREATH_STEPS 256U
#define BREATH_CYCLE (BREATH_STEPS * 2U)
/* 爆闪翻转周期 */
#define FLASH_TOGGLE_MS 100U
/* 按键消抖计数 */
#define KEY_DEBOUNCE_TICKS 3U
/* 长按阈值 */
#define LONGPRESS_MS 2000U
/* 配置保存延迟，避免频繁写入 */
#define SAVE_DELAY_MS 500U

/* LED 相关位掩码，仅影响 LED 引脚 */
#define LED_MASK (LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin | LED5_Pin | LED6_Pin | LED7_Pin)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

/* PWM 相位与双缓冲占空比 */
static volatile uint8_t bam_bit = 0;
static volatile uint8_t dither_phase = 0;
static volatile uint8_t duty_next[LED_COUNT];
static uint32_t bam_on_buf_a[DITHER_CYCLE][PWM_BITS];
static uint32_t bam_on_buf_b[DITHER_CYCLE][PWM_BITS];
static uint32_t (*bam_on_cur)[PWM_BITS] = bam_on_buf_a;
static uint32_t (*bam_on_next)[PWM_BITS] = bam_on_buf_b;
static volatile uint8_t bam_table_ready = 0;
static uint16_t bam_arr_table[PWM_BITS];

/* 伽马校正查找表 */
static uint8_t gamma_lut[256];

/* 效果状态机变量 */
static uint8_t mode = 1;
static uint8_t pos = 0;
static uint8_t group = 0;
static uint16_t step_accum_ms = 0;
static uint16_t flash_accum_ms = 0;
static uint8_t flash_on = 0;
static uint8_t env = 0;
static uint16_t breath_phase = 0;
static uint32_t breath_phase_accum = 0;
static uint8_t breath_lut_cur = 0;
static uint8_t save_pending = 0;
static uint16_t save_delay_ms = 0;

/* 按键消抖状态 */
static uint8_t key_stable = 1;
static uint8_t key_cnt = 0;
static uint16_t key_press_ms = 0;
static uint8_t key_long_fired = 0;
static uint8_t key_last_stable = 1;

typedef enum
{
  RUN_STATE_RUN = 0,
  RUN_STATE_SETUP = 1
} RunState_t;

static RunState_t run_state = RUN_STATE_RUN;

/* 参数档位（index by mode 1..8） */
static uint8_t param_level[10] = {0, 4, 4, 4, 4, 4, 4, 4, 4, 4};

static const uint16_t step_ms_table[7] = {240, 200, 160, 130, 110, 90, 70};
static const uint16_t flash_ms_table[7] = {300, 240, 200, 160, 130, 110, 90};
static const uint8_t bright_cmd_table[7] = {100, 120, 140, 170, 200, 230, 255};
static const uint16_t breath_delta_q8_table[7] = {218, 246, 281, 328, 393, 492, 655};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* 初始化伽马查找表 */
static void Gamma_Init(void);
/* 低频效果更新（按键/模式/包络） */
static void Effect_Tick(void);
/* 将线性亮度命令映射并写入下一帧占空比 */
static void Apply_CmdToDutyNext(const uint8_t cmd[LED_COUNT]);
static void Apply_Config(const Config *cfg);
static void Save_Config_Now(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* LED 顺序：LED1..LED7 映射到 PA4,5,6,1,2,3,7（用于 GPIOA BSRR 原子更新）*/
static const uint16_t led_bits[LED_COUNT] = {
  LED4_Pin, LED5_Pin, LED6_Pin, LED1_Pin, LED2_Pin, LED3_Pin, LED7_Pin
};

/* Setup level (1..7) -> LED index mapping, aligned with led_bits order. */
static const uint8_t setup_level_to_index[LED_COUNT] = {0, 1, 2, 3, 4, 5, 6};

static uint8_t Breath_FromPhase(uint16_t phase)
{
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);

  return gamma_lut[tri];
}

static uint8_t Breath_Triangle(uint16_t phase)
{
  /* Linear triangle wave (0..255) with gamma correction. */
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);

  return gamma_lut[tri];
}

static void Gamma_Init(void)
{
  /* 用户代码函数：初始化伽马校正查找表（仅启动执行一次） */
  /* 启动时一次性生成伽马表，避免 ISR 内浮点运算 */
  const float gamma = 2.2f;
  for (uint16_t i = 0; i < 256; i++)
  {
    float x = (float)i / 255.0f;
    float y = powf(x, gamma);
    uint32_t duty = (uint32_t)(y * 255.0f + 0.5f);
    if (duty > 255U)
    {
      duty = 255U;
    }
    gamma_lut[i] = (uint8_t)duty;
  }
}

static void Apply_CmdToDutyNext(const uint8_t cmd[LED_COUNT])
{
  /* Map cmd->gamma and precompute the dithered phase table. */
  __disable_irq();
  if (bam_table_ready != 0U)
  {
    __enable_irq();
    return;
  }
  __enable_irq();

  for (uint8_t i = 0; i < LED_COUNT; i++)
  {
    duty_next[i] = gamma_lut[cmd[i]];
  }

  for (uint8_t d = 0; d < DITHER_CYCLE; d++)
  {
    for (uint8_t b = 0; b < PWM_BITS; b++)
    {
      bam_on_next[d][b] = 0U;
    }
  }

  for (uint8_t i = 0; i < LED_COUNT; i++)
  {
    /* 4-frame temporal dither to approximate 8-bit gamma with 6-bit BAM. */
    uint32_t scaled = ((uint32_t)duty_next[i] * (PWM_MAX * DITHER_CYCLE) + 127U) / 255U;
    uint8_t base = (uint8_t)(scaled >> DITHER_BITS);
    uint8_t frac = (uint8_t)(scaled & (DITHER_CYCLE - 1U));
    for (uint8_t d = 0; d < DITHER_CYCLE; d++)
    {
      uint8_t value = base;
      if ((base < PWM_MAX) && (d < frac))
      {
        value = (uint8_t)(base + 1U);
      }
      if (value > PWM_MAX)
      {
        value = PWM_MAX;
      }
      for (uint8_t b = 0; b < PWM_BITS; b++)
      {
        if ((value >> b) & 0x1U)
        {
          bam_on_next[d][b] |= led_bits[i];
        }
      }
    }
  }

  __disable_irq();
  bam_table_ready = 1U;
  __enable_irq();
}

static void Apply_Config(const Config *cfg)
{
  mode = cfg->mode;
  if ((mode < 1U) || (mode > 9U))
  {
    mode = 1U;
  }
  param_level[0] = 0U;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    uint8_t level = cfg->param_level[i];
    if ((level < 1U) || (level > 7U))
    {
      level = 4U;
    }
    param_level[i] = level;
  }
}

static void Save_Config_Now(void)
{
  Config cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = mode;
  cfg.run_state = RUN_STATE_RUN;
  cfg.param_level[0] = 0U;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    uint8_t level = param_level[i];
    if ((level < 1U) || (level > 7U))
    {
      level = 4U;
    }
    cfg.param_level[i] = level;
  }

  HAL_TIM_Base_Stop_IT(&htim14);
  __disable_irq();
  (void)FlashCfg_Append(&cfg);
  __enable_irq();
  HAL_TIM_Base_Start_IT(&htim14);
}

static void Effect_Tick(void)
{
  /* 用户代码函数：10ms 低频任务，按键事件/状态机/参数/图案更新 */
  /* 10 ms 低频节拍：按键消抖、模式切换、包络与图案更新 */
  uint8_t key_raw = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 0U : 1U;
  if (key_raw != key_stable)
  {
    if (++key_cnt >= KEY_DEBOUNCE_TICKS)
    {
      key_cnt = 0;
      key_stable = key_raw;
    }
  }
  else
  {
    key_cnt = 0;
  }

  /* 生成短按/长按事件（仅在稳定状态下计时） */
  uint8_t short_press = 0;
  uint8_t long_press = 0;
  if (key_stable == 0U)
  {
    key_press_ms += EFFECT_TICK_MS;
    if ((key_press_ms >= LONGPRESS_MS) && (key_long_fired == 0U))
    {
      key_long_fired = 1U;
      long_press = 1U;
    }
  }
  else
  {
    if ((key_last_stable == 0U) && (key_long_fired == 0U) && (key_press_ms > 0U))
    {
      short_press = 1U;
    }
    key_press_ms = 0;
    key_long_fired = 0U;
  }
  key_last_stable = key_stable;

  /* RUN/SETUP 状态机 */
  if (run_state == RUN_STATE_RUN)
  {
    if (long_press)
    {
      run_state = RUN_STATE_SETUP;
      save_pending = 0U;
      save_delay_ms = 0U;
    }
    else if (short_press)
    {
      mode++;
      if (mode > 9U)
      {
        mode = 1U;
      }
      save_pending = 1U;
      save_delay_ms = 0U;
    }
  }
  else
  {
    if (long_press)
    {
      run_state = RUN_STATE_RUN;
      save_pending = 0U;
      save_delay_ms = 0U;
      Save_Config_Now();
    }
    else if (short_press)
    {
      uint8_t level = param_level[mode];
      level++;
      if (level > 7U)
      {
        level = 1U;
      }
      param_level[mode] = level;
    }
  }

  /* 根据当前模式参数更新节拍 */
  uint8_t level_idx = (param_level[mode] > 0U) ? (uint8_t)(param_level[mode] - 1U) : 0U;
  uint16_t step_ms = STEP_MS;
  uint16_t flash_ms = FLASH_TOGGLE_MS;
  if (mode >= 1U && mode <= 6U)
  {
    step_ms = step_ms_table[level_idx];
  }
  else if (mode == 7U)
  {
    flash_ms = flash_ms_table[level_idx];
  }
  if (mode == 9U)
  {
    uint32_t delta_q8 = breath_delta_q8_table[level_idx];
    breath_phase_accum += delta_q8;
    if (breath_phase_accum >= (uint32_t)BREATH_CYCLE << 8)
    {
      breath_phase_accum -= (uint32_t)BREATH_CYCLE << 8;
    }
    breath_phase = (uint16_t)(breath_phase_accum >> 8);
  }
  else
  {
    breath_phase = (uint16_t)(breath_phase + ENV_DELTA);
    if (breath_phase >= BREATH_CYCLE)
    {
      breath_phase = (uint16_t)(breath_phase - BREATH_CYCLE);
    }
  }
  {
    uint16_t breath = Breath_FromPhase(breath_phase);
    uint16_t boosted = (uint16_t)((breath * 255U) / 32U);
    if (boosted > 255U)
    {
      boosted = 255U;
    }
    env = (uint8_t)boosted;
    if (mode == 9U)
    {
      breath_lut_cur = Breath_Triangle(breath_phase);
    }
    else
    {
      breath_lut_cur = (uint8_t)breath;
    }
  }

  step_accum_ms += EFFECT_TICK_MS;
  if (step_accum_ms >= step_ms)
  {
    step_accum_ms -= step_ms;
    /* 根据模式推进位置或组 */
    switch (mode)
    {
      case 1U:
      case 4U:
        pos = (uint8_t)((pos + 1U) % LED_COUNT);
        break;
      case 2U:
      case 5U:
        pos = (uint8_t)((pos + LED_COUNT - 1U) % LED_COUNT);
        break;
      case 3U:
      case 6U:
        group = (uint8_t)((group + 1U) % 4U);
        break;
      default:
        break;
    }
  }

  flash_accum_ms += EFFECT_TICK_MS;
  if (flash_accum_ms >= flash_ms)
  {
    flash_accum_ms -= flash_ms;
    /* 爆闪翻转 */
    flash_on = (uint8_t)!flash_on;
  }

  if ((save_pending != 0U) && (run_state == RUN_STATE_RUN))
  {
    save_delay_ms += EFFECT_TICK_MS;
    if (save_delay_ms >= SAVE_DELAY_MS)
    {
      save_delay_ms = 0U;
      save_pending = 0U;
      Save_Config_Now();
    }
  }

  uint8_t cmd[LED_COUNT] = {0};
  if (run_state == RUN_STATE_SETUP)
  {
    /* SETUP 档位指示：单灯半亮显示当前档位 */
    uint8_t level = param_level[mode];
    if (level < 1U)
    {
      level = 1U;
    }
    if (level > 7U)
    {
      level = 7U;
    }
    cmd[setup_level_to_index[level - 1U]] = 255U;
    Apply_CmdToDutyNext(cmd);
    return;
  }

  switch (mode)
  {
    case 1U:
    case 2U:
      /* 模式1/2：正/反向单灯流水 */
      cmd[pos] = 255U;
      break;
    case 3U:
      /* 模式3：中心向两侧分组流水 */
      switch (group)
      {
        case 0U:
          cmd[3] = 255U;
          break;
        case 1U:
          cmd[2] = 255U;
          cmd[4] = 255U;
          break;
        case 2U:
          cmd[1] = 255U;
          cmd[5] = 255U;
          break;
        default:
          cmd[0] = 255U;
          cmd[6] = 255U;
          break;
      }
      break;
    case 4U:
    case 5U:
      {
        uint8_t env_breath = env;
        if (env_breath < 100U)
        {
          env_breath = 100U;
        }
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          uint8_t dist = (i > pos) ? (i - pos) : (pos - i);
          if (dist > (LED_COUNT / 2U))
          {
            dist = (uint8_t)(LED_COUNT - dist);
          }
          uint8_t base = 0U;
          if (dist == 0U)
          {
            base = 255U;
          }
          else if (dist == 1U)
          {
            base = 220U;
          }
          else if (dist == 2U)
          {
            base = 128U;
          }
          cmd[i] = (uint8_t)(((uint16_t)base * env_breath) / 255U);
        }
      }
      break;
    case 6U:
      {
        uint8_t env_breath = env;
        if (env_breath < 100U)
        {
          env_breath = 100U;
        }
        switch (group)
        {
          case 0U:
            cmd[3] = env_breath;
            break;
          case 1U:
            cmd[2] = env_breath;
            cmd[4] = env_breath;
            break;
          case 2U:
            cmd[1] = env_breath;
            cmd[5] = env_breath;
            break;
          default:
            cmd[0] = env_breath;
            cmd[6] = env_breath;
            break;
        }
      }
      break;
    case 7U:
      /* 模式7：100 ms 频率全亮/全灭爆闪 */
      if (flash_on)
      {
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = 255U;
        }
      }
      break;
    case 9U:
      /* Mode 9: triangle breathing for all LEDs. */
      {
        uint8_t env_breath = breath_lut_cur;
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = env_breath;
        }
      }
      break;
    default:
      /* 模式8：全亮 */
      for (uint8_t i = 0; i < LED_COUNT; i++)
      {
        uint8_t lvl = param_level[8];
        uint8_t idx = (lvl > 0U) ? (uint8_t)(lvl - 1U) : 0U;
        cmd[i] = bright_cmd_table[idx];
      }
      break;
  }

  Apply_CmdToDutyNext(cmd);
}

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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  key_stable = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 0U : 1U;
  key_last_stable = key_stable;
  key_cnt = 0;
  key_press_ms = 0;
  key_long_fired = 0;


  {
    Config cfg;
    if (FlashCfg_Load(&cfg))
    {
      Apply_Config(&cfg);
    }
    else
    {
      FlashCfg_InitDefaults(&cfg);
      Apply_Config(&cfg);
      (void)FlashCfg_Append(&cfg);
    }
  }

  /* 初始化伽马表并生成首帧 duty */
  Gamma_Init();
  Effect_Tick();
  for (uint8_t b = 0; b < PWM_BITS; b++)
  {
    bam_arr_table[b] = (uint16_t)((BAM_BASE_TICKS * (1U << b)) - 1U);
  }
  bam_bit = 0U;
  dither_phase = 0U;
  if (bam_table_ready != 0U)
  {
    uint32_t (*tmp)[PWM_BITS] = bam_on_cur;
    bam_on_cur = bam_on_next;
    bam_on_next = tmp;
    bam_table_ready = 0U;
  }
  {
    uint32_t on_bits = bam_on_cur[dither_phase][bam_bit];
    uint32_t off_bits = LED_MASK & ~on_bits;
    GPIOA->BSRR = (on_bits & LED_MASK) | (off_bits << 16);
  }
  __HAL_TIM_SET_AUTORELOAD(&htim14, bam_arr_table[bam_bit]);
  /* Start TIM14 interrupt for BAM PWM. */
  HAL_TIM_Base_Start_IT(&htim14);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 使用 SysTick 驱动 10 ms 效果节拍 */
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_tick) >= EFFECT_TICK_MS)
    {
      last_tick += EFFECT_TICK_MS;
      Effect_Tick();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */
  /*
   * TIM14 BAM timing:
   * base_ticks = 64MHz / (20kHz * 63) ≈ 50.8 -> BAM_BASE_TICKS = 51
   * frame_ticks = base_ticks * 63 = 3213 -> PWM base ≈ 19.94kHz
   */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = (BAM_BASE_TICKS - 1U);
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED4_Pin|LED5_Pin|LED6_Pin|LED1_Pin
                          |LED2_Pin|LED3_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED5_Pin LED6_Pin LED1_Pin
                           LED2_Pin LED3_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED5_Pin|LED6_Pin|LED1_Pin
                          |LED2_Pin|LED3_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Note: for cleaner 20kHz edges, consider GPIO_SPEED_FREQ_MEDIUM if rise time looks soft. */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Timer ISR: update BAM output for current bit. */
  if (htim->Instance == TIM14)
  {
    /* BAM: 6-bit per frame @ 20kHz, update rate ~120kHz. */
    uint32_t on_bits = bam_on_cur[dither_phase][bam_bit];
    uint32_t off_bits = LED_MASK & ~on_bits;
    /* Single BSRR write for atomic LED update. */
    GPIOA->BSRR = (on_bits & LED_MASK) | (off_bits << 16);

    bam_bit++;
    if (bam_bit >= PWM_BITS)
    {
      bam_bit = 0;
      dither_phase++;
      if (dither_phase >= DITHER_CYCLE)
      {
        dither_phase = 0;
      }
      if (bam_table_ready != 0U)
      {
        uint32_t (*tmp)[PWM_BITS] = bam_on_cur;
        bam_on_cur = bam_on_next;
        bam_on_next = tmp;
        bam_table_ready = 0U;
        dither_phase = 0U;
      }
    }
    __HAL_TIM_SET_AUTORELOAD(htim, bam_arr_table[bam_bit]);
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
