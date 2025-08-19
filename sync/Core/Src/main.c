#include "main.h"
#include "usb_host.h"

void SystemClock_Config(void);
#define TIMER_FREQ    72000000UL
#define TOTAL_CYCLES  90
#define T0H_CYCLES    30
#define T1H_CYCLES    60
#define RESET_SLOTS   50
#define MAX_LED		  84
#define USE_BRIGHTNESS 1
#define PI 3.14159265


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);


#define LED_PER_BANDS 14
#define NUM_BANDS 6

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];

int datasentflag=0;
uint16_t pwmData[(24*MAX_LED)+50];

const uint8_t colors_pixel[14][3] = {
    {255, 0, 0},
    {255, 127, 0},
    {255, 255, 0},
    {0, 255, 0},
    {0, 255, 255},
    {0, 0, 255},
    {75, 0, 130},
    {255, 255, 255},
    {255, 182, 193},
    {255, 165, 0},
    {0, 191, 255},
    {255, 20, 147},
    {0, 128, 128},
    {128, 0, 128}
};
uint16_t effStep = 0;
uint16_t pixelPos = 0;
uint16_t colorIndex = 0;
uint16_t fadeStep = 0;

typedef enum {
    MODE_RAINBOW,
	MODE_FFT
} EffectMode;

EffectMode mode = MODE_RAINBOW;

uint8_t tick_divider,  count_tick = 0;
volatile uint8_t call_effect_flag= 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

void Set_Brightness (int brightness);
void Set_LED (int LEDnum, int Red, int Green, int Blue) ;
void Reset_LED (void);
void WS2812_Send(void);

uint8_t rainbow_effect();
uint8_t sync_effect();

//------------ Audio management section-----------------
#define ADC_MEAN   2048.0f
extern ADC_HandleTypeDef hadc1;

#define FFT_SIZE 256
uint16_t audioBuffer[FFT_SIZE * 2]; // 2 ping pong buffer
volatile uint8_t ready_buffer = 0;
volatile uint8_t current_buffer = 0;
//------------ End Audio management section-----------------
//-------------FFT define section----------------
#define ARM_MATH_CM4
#include "arm_math.h"

#define SAMPLE_RATE 8000

arm_rfft_fast_instance_f32 fftInstance;

typedef struct {
    uint16_t start_bin;
    uint16_t end_bin;
    float magnitude;
    uint8_t color[3];
} FrequencyBand;

FrequencyBand frequency_bands[NUM_BANDS];

float band_frequencies[NUM_BANDS + 1] = {
    20,    // end low bass, start bass
    150,
	250,   // end bass, start upper bass
	500,
    1000,   // end upper bass, start low mid
    2000,  // end low mid, start mid
    4000,  // end mid, start high mid
};
const uint8_t band_colors[NUM_BANDS][3] = {
    {255, 0, 0},    // Band 0: Red (Bass)
    {255, 127, 0},  // Band 1: Orange
    {255, 255, 0},  // Band 2: Yellow
    {0, 255, 0},    // Band 3: Green
    {0, 255, 255},  // Band 4: Cyan
    {0, 0, 255}     // Band 5: Blue
};

void FFT_INIT(void);
void FFT_PROCESS(void);
//-------------END FFT define section----------------

/**
 * @brief  Bass-reactive pulse: brightness & color based on sound level.
 * @retval 0x01 always, 0x00 on ADC error.
 */

#include <stdbool.h>
/**
 * @brief  Invoked by TIM3 interrupt: selects and runs current effect.
 */
void call_effect(void)
{

    call_effect_flag = 0;
    switch (mode)
    {
        case MODE_RAINBOW:
            tick_divider = 10;
            rainbow_effect();
            break;

        case MODE_FFT:
        	sync_effect();
            break;

        default:
            break;
    }
}
  // Bắt đầu chuyển đổi ADC để đọc cường độ âm thanh
//  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */
  int main(void) {
	  HAL_Init();
	  SystemClock_Config();
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_I2C1_Init();
	  MX_SPI1_Init();
	  MX_USB_HOST_Init();
	  MX_ADC1_Init();
	  MX_TIM2_Init();
	  MX_TIM1_Init();
	  MX_TIM3_Init();

	  HAL_TIM_Base_Start(&htim2);
	  HAL_TIM_Base_Start_IT(&htim3);
	  FFT_INIT();
	  Reset_LED();
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)audioBuffer, FFT_SIZE);

      mode = MODE_RAINBOW;

      while (1)
      {
          if (call_effect_flag)
        	  call_effect();

          HAL_Delay(5);
      }
  }

 // ===========LED transmission section ===========
void WS2812_Send(void) {
      uint32_t indx = 0;
      uint32_t color;

      for (int led = 0; led < MAX_LED; led++) {
  #if USE_BRIGHTNESS
          color = ((LED_Mod[led][1]<<16) | (LED_Mod[led][2]<<8) | LED_Mod[led][3]);
  #else
          color = ((LED_Data[led][1]<<16) | (LED_Data[led][2]<<8) | LED_Data[led][3]);
  #endif
          for (int bit = 23; bit >= 0; bit--) {
              pwmData[indx++] = (color & (1UL << bit)) ? T1H_CYCLES : T0H_CYCLES;
          }
      }

      for (int i = 0; i < RESET_SLOTS; i++) {
          pwmData[indx++] = 0;
      }

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
      while (!datasentflag) {}
      datasentflag = 0;
}
void Reset_LED (void) {
      for (int i=0; i< MAX_LED; i++) {
    	  LED_Mod[i][0] = i;
          LED_Mod[i][1] = 0;
          LED_Mod[i][2] = 0;
          LED_Mod[i][3] = 0;
      }
}
void Set_Brightness (int brightness) {
#if USE_BRIGHTNESS
    if (brightness > 45) brightness = 45;
    for (int i=0; i < MAX_LED; i++) {
        LED_Mod[i][0] = LED_Data[i][0];
        for (int j=1; j<4; j++) {
            float angle = 90-brightness;
            angle = angle*PI / 180;
            LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
        }
    }
#endif
}
void Set_LED (int LEDnum, int Red, int Green, int Blue) {
    LED_Mod[LEDnum][0] = LEDnum;
    LED_Mod[LEDnum][1] = Green;
    LED_Mod[LEDnum][2] = Red;
    LED_Mod[LEDnum][3] = Blue;
}
// ===========END LED transmission section===========
// ===========Custom callback section ================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	current_buffer = !current_buffer;
	ready_buffer = 1;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&audioBuffer[current_buffer * FFT_SIZE], FFT_SIZE);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    datasentflag=1;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		count_tick++;
	if (htim->Instance == TIM3 && tick_divider == count_tick ) {
		count_tick = 0;
		call_effect_flag = 1;

	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
      if(GPIO_Pin == GPIO_PIN_0) // INT Source is pin A9
      {
    	  mode = (EffectMode)((mode + 1) % 2);
      }
}
// =========== End Custom callback section===========
// =========== LED effect section ==================
uint8_t rainbow_effect() {
	    // Strip ID: 0 - Effect: Rainbow - LEDS: 8
	    // Steps: 13 - Delay: 54
	    // Colors: 3 (255.0.0, 0.255.0, 0.0.255)
	    // Options: rainbowlen=8, toLeft=true,
	//  if(millis() - strip_0.effStart < 54 * (strip_0.effStep)) return 0x00;

	  float factor1, factor2;
	  uint16_t ind;
	  for(uint16_t j=0;j< MAX_LED;j++) {
	    ind = effStep + j * 1.625;
	    switch((int)((ind % 13) / 4.333333333333333)) {
	      case 0: factor1 = 1.0 - ((float)(ind % 13 - 0 * 4.333333333333333) / 4.333333333333333);
	              factor2 = (float)((int)(ind - 0) % 13) / 4.333333333333333;
	              /************ chnaged here *********/
	              Set_LED(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
	              WS2812_Send();
	              break;
	      case 1: factor1 = 1.0 - ((float)(ind % 13 - 1 * 4.333333333333333) / 4.333333333333333);
	              factor2 = (float)((int)(ind - 4.333333333333333) % 13) / 4.333333333333333;
	              Set_LED(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
	              WS2812_Send();
	              break;
	      case 2: factor1 = 1.0 - ((float)(ind % 13 - 2 * 4.333333333333333) / 4.333333333333333);
	              factor2 = (float)((int)(ind - 8.666666666666666) % 13) / 4.333333333333333;
	              Set_LED(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
	              WS2812_Send();
	              break;
	    }
	  }
	  if(effStep >= 13) {effStep=0; return 0x03; }
	  else effStep++;
	  return 0x01;
	}
//----------------FFT subsection-----------------------
void FFT_INIT(void)
{
	arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);
	float freq_resolution = (float)SAMPLE_RATE / FFT_SIZE;

	for (int i = 0; i < NUM_BANDS; i++) {
	        frequency_bands[i].start_bin = (uint16_t)(band_frequencies[i] / freq_resolution);
	        frequency_bands[i].end_bin = (uint16_t)(band_frequencies[i + 1] / freq_resolution);
	        frequency_bands[i].magnitude = 0.0f;


	        frequency_bands[i].color[0] = band_colors[i][0];
	        frequency_bands[i].color[1] = band_colors[i][1];
	        frequency_bands[i].color[2] = band_colors[i][2];


	        if (frequency_bands[i].end_bin > FFT_SIZE/2)
	            frequency_bands[i].end_bin = FFT_SIZE/2;
	}
}

void FFT_PROCESS(void)
{
	float32_t inputBuffer[FFT_SIZE * 2],
			 outputBuffer[FFT_SIZE];
	float32_t fftBuffer[FFT_SIZE * 2];
	for (int i = 0; i < FFT_SIZE; i++) {
	        inputBuffer[i] = ((float32_t)audioBuffer[(!current_buffer) * FFT_SIZE + i] -  ADC_MEAN);
	        // normalize signal
	        inputBuffer[i] /= ADC_MEAN;
	    }
	    // Biến đổi RFFT
	    arm_rfft_fast_f32(&fftInstance, inputBuffer, fftBuffer, 0);

	    // Tính cường độ
	    arm_cmplx_mag_f32(fftBuffer, outputBuffer, FFT_SIZE / 2);
	    // TÍnh trung bình cường độ của mỗi band
	    for (int band = 0; band < NUM_BANDS; band++) {
	        float sum = 0.0f;
	        int bin_count = 0;

	        for (int bin = frequency_bands[band].start_bin; bin < frequency_bands[band].end_bin; bin++) {
	            if (bin < FFT_SIZE / 2) {
	                sum += outputBuffer[bin];
	                bin_count++;
	            }
	        }
	        float avg =  sum / bin_count;
	        if (bin_count > 0) {
	            frequency_bands[band].magnitude = avg;
	        }
	    }
	}

const float max_magnitude[NUM_BANDS] = {20.0f, 20.0f, 20.0f, 20.0f, 15.0f, 10.0f};
const float min_magnitude[NUM_BANDS] = {5.0f, 5.0f, 5.0f, 5.0, 2.5f, 1.0f};

uint8_t top_index[NUM_BANDS] ={0};

uint8_t sync_effect()
{
    if (!ready_buffer)
        return 0;

    FFT_PROCESS();
    ready_buffer = 0;

    const int leds_per_band = 14;
    for (int band = 0; band < NUM_BANDS; band++)
    {
        float magnitude = frequency_bands[band].magnitude;
        float max  = max_magnitude[band];
        // Clamp magnitude
        if (magnitude < 0.0f) magnitude = 0.0f;
        if (magnitude > max_magnitude[band]) magnitude = max_magnitude[band];

        // Calculate activate led
        int active_leds = (int)((magnitude / max) * leds_per_band);
        if (active_leds > leds_per_band) active_leds = leds_per_band;

        uint8_t r = (uint8_t)(band_colors[band][0]);
        uint8_t g = (uint8_t)(band_colors[band][1]);
        uint8_t b = (uint8_t)(band_colors[band][2]);

        if (top_index[band] <= active_leds)
        	top_index[band] = active_leds;
        else
        	top_index[band]--;
        // Bật LED theo cường độ
        for (int i = 0; i < leds_per_band; i++) {
            int led_index = leds_per_band * band + i;
            if (i < active_leds) {
                Set_LED(led_index, r, g, b);
            } else {
                Set_LED(led_index, 0, 0, 0);
            }
        }
        Set_LED(top_index[band], 155 , 155 , 155);
        WS2812_Send();
        HAL_Delay(5);
    }

    return 1;
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
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
//    RCC_OscInitStruct.PLL.PLLM = 2;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();
    }
  }
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	    hadc1.Instance = ADC1;
	    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;      // ADC clock = 72MHz/4 = 18MHz
	    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	    hadc1.Init.ScanConvMode = DISABLE;
	    hadc1.Init.ContinuousConvMode = DISABLE;
	    hadc1.Init.DiscontinuousConvMode = DISABLE;
	    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO; // Trigger từ TIM2
	    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	    hadc1.Init.NbrOfConversion = 1;
	    hadc1.Init.DMAContinuousRequests = ENABLE;                 // Cho phép DMA Circular
	    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	    if (HAL_ADC_Init(&hadc1) != HAL_OK)
	    {
	        Error_Handler();
	    }

	    sConfig.Channel = ADC_CHANNEL_1;                  // Kênh ADC1_IN1 (PA1)
	    sConfig.Rank = 1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;   // 28 cycles
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;                   // 72MHz/72 = 1MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 124;                     // 1MHz/125 = 8000Hz → mỗi 125µs overflow
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Cho TIM2 tạo TRGO mỗi khi counter overflow (update event)
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim3, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
	  hdma_adc1.Instance = DMA2_Stream0;  // or another available stream
	  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
	  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	  hdma_adc1.Init.Mode = DMA_CIRCULAR;
	  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
	  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;


	    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
	        Error_Handler();
	    }
	    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

	  hdma_tim1_ch1.Instance                 = DMA2_Stream1;
      hdma_tim1_ch1.Init.Channel             = DMA_CHANNEL_6;
      hdma_tim1_ch1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_tim1_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_tim1_ch1.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_tim1_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      hdma_tim1_ch1.Init.Mode                = DMA_NORMAL;
      hdma_tim1_ch1.Init.Priority            = DMA_PRIORITY_HIGH;
      hdma_tim1_ch1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
      {
        Error_Handler();
      }

      __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_IRQn , 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);


/* USER CODE END MX_GPIO_Init_2 */
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
