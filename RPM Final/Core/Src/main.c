/**
  ******************************************************************************
  * @file    main.c
  * @brief   NUCLEO-G474RE 模擬 RPM 提示燈(可直接覆蓋 Core/Src/main.c)
  *
  *  功能概述:
  *   - 按 B1(PC13, EXTI13, Rising)模擬引擎脈衝,量測兩次按下的時間差 Δt
  *   - RPM = 60,000,000 / Δt(us) x K,K = 30
  *   - 三段狀態機:
  *        RPM <  4000        -> STATE 0:綠燈亮 (PA1 / TIM2_CH2 PWM)
  *        4000 ~ 6500        -> STATE 1:黃燈亮 (PA0 / TIM2_CH1 PWM)
  *        RPM >  6500        -> STATE 2:紅燈 10Hz 爆閃 (PA4 GPIO),
  *                                      蜂鳴器 1.2kHz 同步鳴叫 (PA15 / TIM8_CH1 PWM)
  *   - 超過 2 秒沒按 -> RPM 歸零,回到綠燈
  *   - 按鈕去彈跳 20ms(以 TIM6 微秒時間戳判斷)
  *   - USART2 (PA2/PA3, 115200 8N1) 每 200ms 輸出一行狀態
  *   - TIM6 當 1us 計時器:不開中斷,主迴圈 / EXTI 內輪詢 UIF 旗標累計溢位,
  *     組成 32-bit 微秒時間
  ******************************************************************************
  */

#include "main.h"      /* CubeMX 產生的標頭,內含 stm32g4xx_hal.h */
#include <stdio.h>     /* snprintf */
#include <string.h>    /* memset   */

/* =========================================================================
 *  1. 可調參數與腳位巨集(此處自訂名稱,避免與 main.h 內的標籤巨集衝突)
 * ========================================================================= */
#define RPM_K                 30UL                 /* 係數 K                         */
#define RPM_NUMERATOR         (60000000UL * RPM_K) /* 60,000,000 x 30 = 1.8e9,32-bit 放得下 */
#define DEBOUNCE_US           20000UL              /* 按鈕去彈跳 20ms                */
#define TIMEOUT_US            2000000UL            /* 2 秒沒按 -> RPM 歸零           */
#define RPM_YELLOW_MIN        4000UL               /* >=4000 進入黃燈                */
#define RPM_RED_ABOVE         6500UL               /* >6500 進入紅燈                 */
#define LED_PWM_DUTY          500U                 /* 黃/綠燈亮度 (ARR=999 -> 50%)   */
#define BUZZ_PWM_PULSE        416U                 /* 蜂鳴器 Pulse (ARR=832 -> 50%)  */
#define RED_TOGGLE_MS         50U                  /* 紅燈每 50ms 翻轉 => 10Hz       */
#define UART_PERIOD_MS        200U                 /* UART 輸出週期                  */
#define SELFTEST_STEP_MS      500U                 /* 開機自我測試每步 0.5 秒        */

#define LED_Y_CHANNEL         TIM_CHANNEL_1        /* 黃燈 TIM2_CH1 (PA0) */
#define LED_G_CHANNEL         TIM_CHANNEL_2        /* 綠燈 TIM2_CH2 (PA1) */
#define BUZZ_CHANNEL          TIM_CHANNEL_1        /* 蜂鳴器 TIM8_CH1 (PA15) */

#define RED_PORT              GPIOA
#define RED_PIN               GPIO_PIN_4           /* 紅燈 PA4 */
#define BTN_PORT              GPIOC
#define BTN_PIN               GPIO_PIN_13          /* 按鈕 B1 PC13 */

/* 狀態機狀態編號(也是 UART 輸出的 STATE 值) */
#define STATE_GREEN           0U
#define STATE_YELLOW          1U
#define STATE_RED             2U

/* =========================================================================
 *  2. 全域變數
 * ========================================================================= */
TIM_HandleTypeDef  htim2;   /* 黃/綠燈 PWM  */
TIM_HandleTypeDef  htim6;   /* 1us 計時器   */
TIM_HandleTypeDef  htim8;   /* 蜂鳴器 PWM   */
UART_HandleTypeDef huart2;  /* 序列埠輸出   */

/* TIM6 溢位計數(每溢位一次 = 65536us),由 Micros() 輪詢 UIF 累計 */
static volatile uint32_t g_tim6_ovf      = 0;

/* 以下變數在 EXTI 中斷內寫入、主迴圈讀取 */
static volatile uint32_t g_last_press_us = 0;  /* 上一次有效按下的時間戳 (us)  */
static volatile uint32_t g_dt_us         = 0;  /* 最近一次 Δt (us)             */
static volatile uint32_t g_rpm           = 0;  /* 最近一次計算出的 RPM         */
static volatile uint32_t g_exti_count    = 0;  /* 有效(通過去彈跳)的脈衝數   */
static volatile uint8_t  g_has_prev      = 0;  /* 1 = g_last_press_us 有效     */

/* =========================================================================
 *  3. 函式原型
 * ========================================================================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);

uint32_t Micros(void);
static void Outputs_Set(uint8_t yellow, uint8_t green, uint8_t red, uint8_t buzzer);
static void Delay_Poll(uint32_t ms);
static void Self_Test(void);
static uint8_t Rpm_To_State(uint32_t rpm);
static void Button_Pulse_Handler(uint16_t pin);

/* EXTI callback 三種名稱的原型(G4 HAL 實際只呼叫 HAL_GPIO_EXTI_Callback) */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* =========================================================================
 *  4. 32-bit 微秒時間:輪詢 TIM6 UIF 累計溢位
 *     TIM6 以 1MHz 計數、ARR=65535,每 65.536ms 溢位一次。
 *     主迴圈與 EXTI 都會呼叫本函式,因此只要主迴圈不被卡住超過 65ms,
 *     就不會漏掉溢位。32-bit 微秒約 71.6 分鐘才會繞回,
 *     用「無號減法」計算差值即使繞回也正確。
 * ========================================================================= */
uint32_t Micros(void)
{
  /* 暫時關閉中斷:避免主迴圈與 EXTI 同時處理 UIF 造成重複累計 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* 第一次檢查:若已溢位,清旗標並累計 */
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    g_tim6_ovf++;
  }

  uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim6);

  /* 第二次檢查:若在「檢查旗標」與「讀計數值」之間剛好溢位,
     則補計一次並重新讀取計數值,避免時間倒退 65ms */
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    g_tim6_ovf++;
    cnt = __HAL_TIM_GET_COUNTER(&htim6);
  }

  uint32_t t = (g_tim6_ovf << 16) | (cnt & 0xFFFFU);

  __set_PRIMASK(primask);
  return t;
}

/* =========================================================================
 *  5. 輸出控制:一次設定黃 / 綠 / 紅 / 蜂鳴器
 *     黃、綠燈與蜂鳴器皆為 PWM 持續運行,
 *     "亮" = Compare 設為指定值,"滅" = Compare 設為 0(輸出恆為低),
 *     這樣不需啟停 Timer,腳位也不會浮接。
 * ========================================================================= */
static void Outputs_Set(uint8_t yellow, uint8_t green, uint8_t red, uint8_t buzzer)
{
  __HAL_TIM_SET_COMPARE(&htim2, LED_Y_CHANNEL, yellow ? LED_PWM_DUTY : 0U);
  __HAL_TIM_SET_COMPARE(&htim2, LED_G_CHANNEL, green  ? LED_PWM_DUTY : 0U);
  HAL_GPIO_WritePin(RED_PORT, RED_PIN, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, BUZZ_CHANNEL, buzzer ? BUZZ_PWM_PULSE : 0U);
}

/* =========================================================================
 *  6. 延時(期間持續輪詢 TIM6,避免漏掉溢位)
 * ========================================================================= */
static void Delay_Poll(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms)
  {
    (void)Micros();
  }
}

/* =========================================================================
 *  7. 開機自我測試:綠 -> 黃 -> 紅+蜂鳴器,各 0.5 秒
 * ========================================================================= */
static void Self_Test(void)
{
  Outputs_Set(0, 1, 0, 0);            /* 綠燈 */
  Delay_Poll(SELFTEST_STEP_MS);

  Outputs_Set(1, 0, 0, 0);            /* 黃燈 */
  Delay_Poll(SELFTEST_STEP_MS);

  Outputs_Set(0, 0, 1, 1);            /* 紅燈 + 蜂鳴器 */
  Delay_Poll(SELFTEST_STEP_MS);

  Outputs_Set(0, 0, 0, 0);            /* 全部關閉 */

  /* 清除自我測試期間可能誤觸的按鈕資料 */
  __disable_irq();
  g_has_prev   = 0;
  g_rpm        = 0;
  g_dt_us      = 0;
  g_exti_count = 0;
  __enable_irq();
  __HAL_GPIO_EXTI_CLEAR_IT(BTN_PIN);
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

/* =========================================================================
 *  8. RPM -> 狀態判斷
 * ========================================================================= */
static uint8_t Rpm_To_State(uint32_t rpm)
{
  if (rpm < RPM_YELLOW_MIN)  return STATE_GREEN;    /* < 4000        */
  if (rpm <= RPM_RED_ABOVE)  return STATE_YELLOW;   /* 4000 ~ 6500   */
  return STATE_RED;                                 /* > 6500        */
}

/* =========================================================================
 *  9. 按鈕脈衝處理(在 EXTI 中斷內執行)
 *     - 以 TIM6 微秒時間戳做 20ms 去彈跳
 *     - 計算 Δt 與 RPM = 60,000,000 x K / Δt
 *     - 若與上次間隔 > 2 秒,視為新的第一次按下(不計算 RPM)
 * ========================================================================= */
static void Button_Pulse_Handler(uint16_t pin)
{
  if (pin != BTN_PIN)
  {
    return;
  }

  uint32_t now = Micros();

  if (g_has_prev)
  {
    uint32_t dt = now - g_last_press_us;      /* 無號減法,繞回也正確 */

    if (dt < DEBOUNCE_US)
    {
      return;                                 /* 20ms 內視為彈跳,忽略 */
    }

    if (dt <= TIMEOUT_US)
    {
      g_dt_us = dt;
      g_rpm   = RPM_NUMERATOR / dt;           /* 先乘後除,保留精度 */
    }
    else
    {
      g_dt_us = 0;                            /* 間隔過久,視為重新開始 */
      g_rpm   = 0;
    }
  }

  g_last_press_us = now;                      /* 記錄這次有效按下時間 */
  g_has_prev      = 1;
  g_exti_count++;                             /* 有效脈衝計數 */
}

/* =========================================================================
 *  10. EXTI callback:同時提供三種名稱,不論 HAL 版本呼叫哪一個都能運作
 *      (G4 HAL 只會呼叫 HAL_GPIO_EXTI_Callback;
 *       Rising/Falling 版本留給其他版本 HAL 或之後改用的情況)
 * ========================================================================= */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  Button_Pulse_Handler(GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  Button_Pulse_Handler(GPIO_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Button_Pulse_Handler(GPIO_Pin);
}

/* =========================================================================
 *  11. main()
 * ========================================================================= */
int main(void)
{
  /* ---- 初始化 ---- */
  HAL_Init();                 /* HAL、SysTick(1ms)初始化 */
  SystemClock_Config();       /* HSI -> PLL -> 170MHz    */

  MX_GPIO_Init();             /* PA4 輸出、PC13 EXTI     */
  MX_TIM2_Init();             /* 黃/綠燈 PWM             */
  MX_TIM8_Init();             /* 蜂鳴器 PWM              */
  MX_TIM6_Init();             /* 1us 計時器              */
  MX_USART2_UART_Init();      /* 序列埠                  */

  /* ---- 啟動 PWM(Compare 先為 0,不亮不響)---- */
  HAL_TIM_PWM_Start(&htim2, LED_Y_CHANNEL);
  HAL_TIM_PWM_Start(&htim2, LED_G_CHANNEL);
  HAL_TIM_PWM_Start(&htim8, BUZZ_CHANNEL);

  /* ---- 啟動 TIM6(不開中斷,純計數)---- */
  HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);  /* 清除初始化時 UG 產生的 UIF */
  g_tim6_ovf = 0;

  /* ---- 開機自我測試 ---- */
  Self_Test();

  /* ---- 主迴圈用的區域變數 ---- */
  uint8_t  state     = STATE_GREEN;       /* 目前狀態                 */
  uint8_t  red_phase = 0;                 /* 紅燈閃爍相位 (1=亮)      */
  uint32_t red_tick  = 0;                 /* 上次紅燈翻轉時刻 (ms)    */
  uint32_t uart_tick = HAL_GetTick();     /* 上次 UART 輸出時刻 (ms)  */

  Outputs_Set(0, 1, 0, 0);                /* 初始:綠燈亮             */

  while (1)
  {
    uint32_t rpm, dt, cnt, now;

    /* ---- (a) 取得資料快照,並檢查 2 秒逾時(關中斷保持一致性)---- */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    now = Micros();                       /* 同時輪詢 TIM6 UIF */

    if (g_has_prev && ((now - g_last_press_us) > TIMEOUT_US))
    {
      g_has_prev = 0;                     /* 超過 2 秒沒按:RPM 歸零 */
      g_rpm      = 0;
      g_dt_us    = 0;
    }

    rpm = g_rpm;
    dt  = g_dt_us;
    cnt = g_exti_count;

    __set_PRIMASK(primask);

    /* ---- (b) 狀態機:狀態改變時切換輸出 ---- */
    uint8_t new_state = Rpm_To_State(rpm);
    if (new_state != state)
    {
      state = new_state;

      switch (state)
      {
        case STATE_GREEN:
          Outputs_Set(0, 1, 0, 0);        /* 綠燈亮 */
          break;

        case STATE_YELLOW:
          Outputs_Set(1, 0, 0, 0);        /* 黃燈亮 */
          break;

        case STATE_RED:
        default:
          red_phase = 1;                  /* 進入紅燈:先亮,接著 10Hz 閃 */
          red_tick  = HAL_GetTick();
          Outputs_Set(0, 0, 1, 1);        /* 紅燈 + 蜂鳴器 同步 */
          break;
      }
    }

    /* ---- (c) 紅燈狀態:每 50ms 翻轉,蜂鳴器與紅燈同步 ---- */
    if (state == STATE_RED)
    {
      if ((HAL_GetTick() - red_tick) >= RED_TOGGLE_MS)
      {
        red_tick  = HAL_GetTick();
        red_phase ^= 1U;
        Outputs_Set(0, 0, red_phase, red_phase);
      }
    }

    /* ---- (d) 每 200ms 由 USART2 輸出一行狀態 ---- */
    if ((HAL_GetTick() - uart_tick) >= UART_PERIOD_MS)
    {
      uart_tick = HAL_GetTick();

      char buf[80];
      int  n = snprintf(buf, sizeof(buf),
                        "RPM:%lu,STATE:%u,DT:%lu us,EXTI:%lu\r\n",
                        (unsigned long)rpm,
                        (unsigned int)state,
                        (unsigned long)dt,
                        (unsigned long)cnt);
      if (n > 0)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
      }
    }
  }
}

/* =========================================================================
 *  12. 系統時鐘:HSI(16MHz) -> /M=4 -> x N=85 -> /R=2 => 170MHz
 *      VCO = 16/4 x 85 = 340MHz,SYSCLK = 340/2 = 170MHz
 *      AHB / APB1 / APB2 皆不分頻 => 170MHz(Timer 時鐘亦為 170MHz)
 * ========================================================================= */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 170MHz 必須使用 Range 1 Boost 模式 */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /* 振盪器:開 HSI,並設定 PLL */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 匯流排時鐘:SYSCLK 來源 PLL,Flash 等待週期 8 (170MHz) */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
}

/* =========================================================================
 *  13. GPIO 初始化
 *      PA4  : 紅燈 推挽輸出
 *      PC13 : B1 按鈕 EXTI13,Rising 觸發(板上有外部上拉,故不再設內部上拉下拉)
 *      NVIC : EXTI15_10 中斷
 *      註:PWM 腳位(PA0/PA1/PA15)與 UART 腳位(PA2/PA3)在各自 Init 中設定
 * ========================================================================= */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* 紅燈預設熄滅 */
  HAL_GPIO_WritePin(RED_PORT, RED_PIN, GPIO_PIN_RESET);

  /* PA4:紅燈輸出 */
  GPIO_InitStruct.Pin   = RED_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_PORT, &GPIO_InitStruct);

  /* PC13:B1 外部中斷,上升緣觸發 */
  GPIO_InitStruct.Pin  = BTN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

  /* EXTI15_10 中斷:優先權 1(低於 SysTick 的 0) */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* =========================================================================
 *  14. TIM2:黃燈 PA0 (CH1) / 綠燈 PA1 (CH2) PWM
 *      170MHz / (169+1) = 1MHz,ARR=999 => PWM 1kHz,Duty 500/1000 = 50%
 * ========================================================================= */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef      sConfigOC     = {0};
  GPIO_InitTypeDef        GPIO_InitStruct = {0};

  /* 時鐘與腳位(PA0 / PA1 -> AF1 = TIM2_CH1 / CH2) */
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 計時器基本參數 */
  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 169;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 999;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* PWM 通道設定:PWM Mode 1,極性高,起始 Pulse = 0(先不亮) */
  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, LED_Y_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, LED_G_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

/* =========================================================================
 *  15. TIM8:蜂鳴器 PA15 (CH1) PWM
 *      170MHz / (169+1) = 1MHz,ARR=832 => 1MHz/833 ≈ 1200.5Hz,
 *      Pulse = 416 => 約 50% 占空比
 *      PA15 沒有 TIM3_CH1,使用 AF2 = TIM8_CH1
 *      TIM8 為進階計時器,需設定 Break/Dead-time 並由 HAL_TIM_PWM_Start 開啟 MOE
 * ========================================================================= */
static void MX_TIM8_Init(void)
{
  TIM_MasterConfigTypeDef        sMasterConfig = {0};
  TIM_OC_InitTypeDef             sConfigOC     = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  GPIO_InitTypeDef               GPIO_InitStruct = {0};

  memset(&sBreakDeadTimeConfig, 0, sizeof(sBreakDeadTimeConfig));

  /* 時鐘與腳位(PA15 -> AF2 = TIM8_CH1) */
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_15;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 計時器基本參數 */
  htim8.Instance               = TIM8;
  htim8.Init.Prescaler         = 169;
  htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim8.Init.Period            = 832;
  htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* PWM 通道:Mode 1,起始 Pulse = 0(先不響) */
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, BUZZ_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  /* Break / Dead-time:全部停用(未使用煞車輸入) */
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter      = 0;
  sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter     = 0;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* =========================================================================
 *  16. TIM6:1us 計時器(不使用中斷)
 *      170MHz / (169+1) = 1MHz => 每 1us 加 1,ARR=65535 => 65.536ms 溢位
 *      溢位由 Micros() 輪詢 UIF 旗標累計
 * ========================================================================= */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance               = TIM6;
  htim6.Init.Prescaler         = 169;
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.Period            = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* =========================================================================
 *  17. USART2:PA2 (TX) / PA3 (RX),115200 8N1(經 ST-LINK 虛擬 COM 輸出)
 *      PA2/PA3 -> AF7 = USART2
 * ========================================================================= */
static void MX_USART2_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 時鐘與腳位 */
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* UART 參數 */
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = 115200;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* =========================================================================
 *  18. 錯誤處理:關閉中斷後停在原地
 * ========================================================================= */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
