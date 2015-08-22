// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stm32f0xx.h>
#include "diag/Trace.h"
#include "lcd_stm32f0.h"

#define TRUE            1
#define FALSE           0

#define DEBOUNCE_MS     20
#define ADC_GRAIN       806 // ADC uV per bits
#define ADC_MULTIPLIER  (float)1000000 // Grain multiplier

// == Type Definitions
typedef enum {
  PROG_STATE_INIT,
  PROG_STATE_WAIT_FOR_SW0,
  PROG_STATE_WAIT_FOR_BUTTON,
  PROG_STATE_BUCKET_TIP
} programState_t;

typedef enum {
  DISP_RAIN_BUCKET,
  DISP_RAINFALL,
  DISP_BAT,
  DISP_WELCOME,
  DISP_MENU
} displayType_t;

// == Global Variables
programState_t programState; // To keep track of the program state throughout execution
uint32_t rainCounter; // Keep track of the rain

// == Function Prototypes
static void init_ports(void);
static void init_ADC(void);
static void init_NVIC(void);
static void init_EXTI(void);

static void lcd_put2String(uint8_t *string1, uint8_t *string2);
void delay(unsigned int microseconds);

static uint8_t getSW(uint8_t pb);
static void check_battery(void);
static uint16_t getADC(void);
static void display(displayType_t displayType, void value);

// == Program Code
int main(int argc, char* argv[]) {
  // Initialisations
  programState = PROG_STATE_INIT;

  init_LCD();
  init_ports();
  init_EXTI();
  init_NVIC();
  init_ADC();

  lcd_put2String("EEE3017W Prac 6", "Sean Wood");
  programState = PROG_STATE_WAIT_FOR_SW0;

  // Infinite loop
  while (1) {
    check_battery();
    //    __asm("nop");
  }
}

// == Function Definitions

/*
 * @brief Initialise the GPIO ports for pushbuttons, LEDs and the ADC
 * @params None
 * @retval None
 */
static void init_ports(void) {
  // Enable the clock for ports used
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;

  // Initialise PB0 - PB7, PB10 and PB11 for RG Led
  GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 |
                  GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
                  GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                  GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 |
                  GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0;
  GPIOB->ODR &= ~(GPIO_ODR_10 | GPIO_ODR_11); // Make sure they are not on

  // Initialise PA0, PA1, PA2 and PA3 for SW0, SW1, SW2 and SW3
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2
      | GPIO_MODER_MODER3);
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0
      | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0; // Enable pullup resistors

  // Initialise PA5 for ADC1
  GPIOA->MODER |= GPIO_MODER_MODER5;
}

/*
 * @brief Initialise the ADC to POT0
 * @params None
 * @retval None
 */
static void init_ADC(void) {
  // Enable the ADC clock in the RCC
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  // Select ADC channel 5 for POT0
  ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

  // Enable the ADC peripheral
  ADC1->CR |= ADC_CR_ADEN;

  // Wait for the ADC to become ready
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
    __asm("nop");
  }
}

/*
 * @brief Initialise the NVIC for pushbutton interrupts
 * @params None
 * @retval None
 */
static void init_NVIC(void) {
  NVIC_EnableIRQ(EXTI0_1_IRQn); // For lines 0 and 1
  NVIC_EnableIRQ(EXTI2_3_IRQn); // For lines 2 and 3
}

/*
 * @brief Initialise the EXTI lines for pushbutton interrupts
 * @params None
 * @retval None
 */
static void init_EXTI(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable the SYSCFG and COMP RCC clock
  SYSCFG->EXTICR[1] &= ~(0xFFFF); // Map PA0 and PA1 to external interrupt lines

  EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1 | EXTI_FTSR_TR2 | EXTI_FTSR_TR3; // Configure trigger to falling edge
  EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2 | EXTI_IMR_MR3; // Umask the interrupts
}

/*
 * @brief Rational addition of a safe 2 line write to the LCD
 * @params *string1: Pointer to the string to be written to line 1
 *         *string2: Pointer to the string to be written to line 2
 * @retval None
 */
static void lcd_put2String(uint8_t *string1, uint8_t *string2) {
  lcd_command(CURSOR_HOME);
  lcd_command(CLEAR);
  lcd_putstring(string1);
  lcd_command(LINE_TWO);
  lcd_putstring(string2);
}

/*
 * @brief Get the state of the specified switch, with debouncing of predefined length
 * @params pb: Pushbutton number
 * @retval True or false when pressed and not pressed rsp.
 */
static uint8_t getSW(uint8_t pb) {
  uint8_t pbBit;

  switch (pb) {
  case 0:
    pbBit = GPIO_IDR_0;
    break;
  case 1:
    pbBit = GPIO_IDR_1;
    break;
  case 2:
    pbBit = GPIO_IDR_2;
    break;
  case 3:
    pbBit = GPIO_IDR_3;
    break;
  default:
    return FALSE;
  }

  if (!(GPIOA->IDR & pbBit)) {
    delay(DEBOUNCE_MS * 1000);
    if (!(GPIOA->IDR & pbBit)) {
      return TRUE;
    } else {
      return FALSE;
    }
  } else {
    return FALSE;
  }
}

static uint16_t getADC(void) {
  // Start a conversion
  ADC1->CR |= ADC_CR_ADSTART;

  // Wait for the conversion to finish
  while (!(ADC1->ISR & ADC_ISR_EOC)) {
    __asm("nop");
  }

  // Return the result of the conversion
  return (uint16_t)(ADC1->DR);
}

/*
 * @brief Interrupt Request Handler for EXTI Lines 2 and 3 (PB0 and PB1)
 * @params None
 * @retval None
 */
void EXTI0_1_IRQHandler(void) {
  if (getSW(0)) {
    switch (programState) {
    case PROG_STATE_WAIT_FOR_SW0:
      display(DISP_MENU);
      programState = PROG_STATE_WAIT_FOR_BUTTON;
      break;
    default:
      break;
    }
  } else if (getSW(1)) {
    switch (programState) {
    case PROG_STATE_WAIT_FOR_BUTTON:
      rainCounter++; // Increment the rain counter
      display(DISP_RAIN_BUCKET); // Notify the user
      break;
    default:
      break;
    }
  }

  EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR1; // Clear the interrupt pending bit
}

/*
 * @brief Interrupt Request Handler for EXTI Lines 2 and 3 (PB2 and PB3)
 * @params None
 * @retval None
 */
void EXTI2_3_IRQHandler(void) {
  if (getSW(2)) {
    switch (programState) {
    case PROG_STATE_WAIT_FOR_BUTTON:
      lcd_put2String("Rainfall:", "0.0000000mm");
      break;
    default:
      break;
    }
  }
  EXTI->PR |= EXTI_PR_PR2 | EXTI_PR_PR3; // Clear the interrupt pending bit
}

/*
 * @brief Check the "battery voltage" and
 * @params None
 * @retval None
 */
static void check_battery(void) {
  uint16_t adcVal = getADC();
  uint32_t uVoltage = adcVal * ADC_GRAIN;
  float voltage = uVoltage/ADC_MULTIPLIER;

  display(DISP_BAT);
}

/*
 * @brief Display the specified data on the screen
 * @params displayType: What to display on the screen
 *         value: Data to display for the given type
 * @retval None
 */
void display(displayType_t displayType, void value) {
  switch (displayType) {
  case DISP_BAT:
    break;
  case DISP_RAINFALL:
    lcd_putString("Rainfall:");
    lcd_command(LINE_TWO);
    break;
  case DISP_RAIN_BUCKET:
    lcd_put2String("Rain bucket tip", "");
    break;
  case DISP_MENU:
    lcd_put2String("Weather Station", "Press SW2 or SW3");
    break;
  case DISP_WELCOME:
    break;
  default:
    break;
  }
}

// ----------------------------------------------------------------------------
