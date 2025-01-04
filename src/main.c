//
// See "system/include/cmsis/stm32f051x8.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f051x8.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "stm32f0xx.h"
#include "stm32f051x8.h"

#pragma GCC diagnostic push

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t) 0xFFFFFFFF)

#define VDD (3.3)
#define ADC_MAX_VALUE (4095.0)
#define RESITANCE_MAX_VALUE (5000.0)

// Globals
SPI_HandleTypeDef SPI_Handle;

uint32_t Frequency = 0;
uint32_t Resistance = 0;

uint8_t timer_triggered = 0;
uint8_t in_signal = 0; // Initially measuring timer 555

// LED Display initialization commands
const uint8_t OLED_init_cmds[] = {
    0xAE,
    0x20, 0x00,
    0x40,
    0xA0 | 0x01,
    0xA8, 0x40 - 1,
    0xC0 | 0x08,
    0xD3, 0x00,
    0xDA, 0x32,
    0xD5, 0x80,
    0xD9, 0x22,
    0xDB, 0x30,
    0x81, 0xFF,
    0xA4,
    0xA6,
    0xAD, 0x30,
    0x8D, 0x10,
    0xAE | 0x01,
    0xC0,
    0xA0
};

//
// Character specifications for LED Display (1 row = 8 bytes = 1 ASCII character)
// Example: to display '4', retrieve 8 data bytes stored in ASCII_CHARS[20][X] row
//          (where X = 0, 1, ..., 7) and send them one by one to LED Display.
const uint8_t ASCII_CHARS[][8] = {
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b01011111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // !
    {0b00000000, 0b00000111, 0b00000000, 0b00000111, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // "
    {0b00010100, 0b01111111, 0b00010100, 0b01111111, 0b00010100, 0b00000000, 0b00000000, 0b00000000},  // #
    {0b00100100, 0b00101010, 0b01111111, 0b00101010, 0b00010010, 0b00000000, 0b00000000, 0b00000000},  // $
    {0b00100011, 0b00010011, 0b00001000, 0b01100100, 0b01100010, 0b00000000, 0b00000000, 0b00000000},  // %
    {0b00110110, 0b01001001, 0b01010101, 0b00100010, 0b01010000, 0b00000000, 0b00000000, 0b00000000},  // &
    {0b00000000, 0b00000101, 0b00000011, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // '
    {0b00000000, 0b00011100, 0b00100010, 0b01000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // (
    {0b00000000, 0b01000001, 0b00100010, 0b00011100, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // )
    {0b00010100, 0b00001000, 0b00111110, 0b00001000, 0b00010100, 0b00000000, 0b00000000, 0b00000000},  // *
    {0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000, 0b00000000, 0b00000000, 0b00000000},  // +
    {0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // ,
    {0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00000000, 0b00000000, 0b00000000},  // -
    {0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // .
    {0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010, 0b00000000, 0b00000000, 0b00000000},  // /
    {0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110, 0b00000000, 0b00000000, 0b00000000},  // 0
    {0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // 1
    {0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110, 0b00000000, 0b00000000, 0b00000000},  // 2
    {0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001, 0b00000000, 0b00000000, 0b00000000},  // 3
    {0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000, 0b00000000, 0b00000000, 0b00000000},  // 4
    {0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001, 0b00000000, 0b00000000, 0b00000000},  // 5
    {0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000, 0b00000000, 0b00000000, 0b00000000},  // 6
    {0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111, 0b00000000, 0b00000000, 0b00000000},  // 7
    {0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110, 0b00000000, 0b00000000, 0b00000000},  // 8
    {0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110, 0b00000000, 0b00000000, 0b00000000},  // 9
    {0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // :
    {0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // ;
    {0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // <
    {0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00000000, 0b00000000, 0b00000000},  // =
    {0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000, 0b00000000, 0b00000000, 0b00000000},  // >
    {0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110, 0b00000000, 0b00000000, 0b00000000},  // ?
    {0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110, 0b00000000, 0b00000000, 0b00000000},  // @
    {0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110, 0b00000000, 0b00000000, 0b00000000},  // A
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110, 0b00000000, 0b00000000, 0b00000000},  // B
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010, 0b00000000, 0b00000000, 0b00000000},  // C
    {0b01111111, 0b01000001, 0b01000001, 0b00100010, 0b00011100, 0b00000000, 0b00000000, 0b00000000},  // D
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001, 0b00000000, 0b00000000, 0b00000000},  // E
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001, 0b00000000, 0b00000000, 0b00000000},  // F
    {0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010, 0b00000000, 0b00000000, 0b00000000},  // G
    {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111, 0b00000000, 0b00000000, 0b00000000},  // H
    {0b01000000, 0b01000001, 0b01111111, 0b01000001, 0b01000000, 0b00000000, 0b00000000, 0b00000000},  // I
    {0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001, 0b00000000, 0b00000000, 0b00000000},  // J
    {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000, 0b00000000, 0b00000000},  // K
    {0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b00000000, 0b00000000, 0b00000000},  // L
    {0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111, 0b00000000, 0b00000000, 0b00000000},  // M
    {0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111, 0b00000000, 0b00000000, 0b00000000},  // N
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110, 0b00000000, 0b00000000, 0b00000000},  // O
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110, 0b00000000, 0b00000000, 0b00000000},  // P
    {0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110, 0b00000000, 0b00000000, 0b00000000},  // Q
    {0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110, 0b00000000, 0b00000000, 0b00000000},  // R
    {0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001, 0b00000000, 0b00000000, 0b00000000},  // S
    {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001, 0b00000000, 0b00000000, 0b00000000},  // T
    {0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111, 0b00000000, 0b00000000, 0b00000000},  // U
    {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111, 0b00000000, 0b00000000, 0b00000000},  // V
    {0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111, 0b00000000, 0b00000000, 0b00000000},  // W
    {0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011, 0b00000000, 0b00000000, 0b00000000},  // X
    {0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111, 0b00000000, 0b00000000, 0b00000000},  // Y
    {0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011, 0b00000000, 0b00000000, 0b00000000},  // Z
    {0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // [
    {0b00010101, 0b00010110, 0b01111100, 0b00010110, 0b00010101, 0b00000000, 0b00000000, 0b00000000},  // back slash
    {0b00000000, 0b00000000, 0b00000000, 0b01000001, 0b01111111, 0b00000000, 0b00000000, 0b00000000},  // ]
    {0b00000100, 0b00000010, 0b00000001, 0b00000010, 0b00000100, 0b00000000, 0b00000000, 0b00000000},  // ^
    {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b00000000, 0b00000000, 0b00000000},  // _
    {0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // `
    {0b00100000, 0b01010100, 0b01010100, 0b01010100, 0b01111000, 0b00000000, 0b00000000, 0b00000000},  // a
    {0b01111111, 0b01001000, 0b01000100, 0b01000100, 0b00111000, 0b00000000, 0b00000000, 0b00000000},  // b
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00100000, 0b00000000, 0b00000000, 0b00000000},  // c
    {0b00111000, 0b01000100, 0b01000100, 0b01001000, 0b01111111, 0b00000000, 0b00000000, 0b00000000},  // d
    {0b00111000, 0b01010100, 0b01010100, 0b01010100, 0b00011000, 0b00000000, 0b00000000, 0b00000000},  // e
    {0b00001000, 0b01111110, 0b00001001, 0b00000001, 0b00000010, 0b00000000, 0b00000000, 0b00000000},  // f
    {0b00001100, 0b01010010, 0b01010010, 0b01010010, 0b00111110, 0b00000000, 0b00000000, 0b00000000},  // g
    {0b01111111, 0b00001000, 0b00000100, 0b00000100, 0b01111000, 0b00000000, 0b00000000, 0b00000000},  // h
    {0b00000000, 0b01000100, 0b01111101, 0b01000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // i
    {0b00100000, 0b01000000, 0b01000100, 0b00111101, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // j
    {0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // k
    {0b00000000, 0b01000001, 0b01111111, 0b01000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // l
    {0b01111100, 0b00000100, 0b00011000, 0b00000100, 0b01111000, 0b00000000, 0b00000000, 0b00000000},  // m
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b01111000, 0b00000000, 0b00000000, 0b00000000},  // n
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00111000, 0b00000000, 0b00000000, 0b00000000},  // o
    {0b01111100, 0b00010100, 0b00010100, 0b00010100, 0b00001000, 0b00000000, 0b00000000, 0b00000000},  // p
    {0b00001000, 0b00010100, 0b00010100, 0b00011000, 0b01111100, 0b00000000, 0b00000000, 0b00000000},  // q
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00001000, 0b00000000, 0b00000000, 0b00000000},  // r
    {0b01001000, 0b01010100, 0b01010100, 0b01010100, 0b00100000, 0b00000000, 0b00000000, 0b00000000},  // s
    {0b00000100, 0b00111111, 0b01000100, 0b01000000, 0b00100000, 0b00000000, 0b00000000, 0b00000000},  // t
    {0b00111100, 0b01000000, 0b01000000, 0b00100000, 0b01111100, 0b00000000, 0b00000000, 0b00000000},  // u
    {0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100, 0b00000000, 0b00000000, 0b00000000},  // v
    {0b00111100, 0b01000000, 0b00111000, 0b01000000, 0b00111100, 0b00000000, 0b00000000, 0b00000000},  // w
    {0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100, 0b00000000, 0b00000000, 0b00000000},  // x
    {0b00001100, 0b01010000, 0b01010000, 0b01010000, 0b00111100, 0b00000000, 0b00000000, 0b00000000},  // y
    {0b01000100, 0b01100100, 0b01010100, 0b01001100, 0b01000100, 0b00000000, 0b00000000, 0b00000000},  // z
    {0b00000000, 0b00001000, 0b00110110, 0b01000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // {
    {0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // |
    {0b00000000, 0b01000001, 0b00110110, 0b00001000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},  // }
    {0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000, 0b00000000, 0b00000000, 0b00000000},  // ~
};

void OLED_write_cmd(uint8_t value);
void OLED_set_cursor_pos(uint8_t page_number, uint8_t segment);
void OLED_write_data(uint8_t data);

// Call this function to boost the STM32F0xx clock to 48 MHz
void SystemClock48MHz(void)
{
    // Disable the PLL
    RCC->CR &= ~(RCC_CR_PLLON);

    // Wait for the PLL to unlock
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );

    // Configure the PLL for a 48MHz system clock
    RCC->CFGR = 0x00280000;

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait for the PLL to lock
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

    // Switch the processor to the PLL clock source
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

    // Update the system with the new clock frequency
    SystemCoreClockUpdate();
}

void GPIOA_init(void)
{
    // Enable clock for GPIOA peripheral
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA0, PA1, PA2 as input
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2);

    // Configure no pull-up/pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2);
}

void GPIOB_init(void)
{
    // Enable clock for GPIOB peripheral
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB3, PB5 for alternate function
    GPIOB->MODER |= (0x2UL << GPIO_MODER_MODER3_Pos | 0x2UL << GPIO_MODER_MODER5_Pos);

    // Configure PB4, PB6, PB7 for output
    GPIOB->MODER |= (0x1UL << GPIO_MODER_MODER4_Pos | 0x1UL << GPIO_MODER_MODER6_Pos | 0x1UL << GPIO_MODER_MODER7_Pos);
}

void TIM2_init(void)
{
    /* Enable clock for TIM2 peripheral */
    // Relevant register: RCC->APB1ENR
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    // Relevant register: TIM2->CR1
    TIM2->CR1 = ((uint16_t) 0x008C);

    /* Set clock prescaler value */
    TIM2->PSC = myTIM2_PRESCALER;
    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

    /* Update timer registers */
    // Relevant register: TIM2->EGR
    TIM2->EGR = ((uint16_t) 0x0001);

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority
    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable TIM2 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    // Relevant register: TIM2->DIER
    TIM2->DIER |= TIM_DIER_UIE;
}

void EXTI_init(void)
{
    /* Map EXTI2 line to PA2 */
    // Relevant register: SYSCFG->EXTICR[0]
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2;

    /* EXTI2 line interrupts: set rising-edge trigger */
    // Relevant register: EXTI->RTSR
    EXTI->RTSR |= EXTI_RTSR_TR2;

    /* Unmask interrupts from EXTI2 line: we don't do this initially as we start with this disabled and
    // using the values of the 555 timer.
    // Relevant register: EXTI->IMR
    // EXTI->IMR |= EXTI_IMR_MR2;

    /* Assign EXTI2 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[2], or use NVIC_SetPriority
    NVIC_SetPriority(EXTI2_3_IRQn, 1);

    /* Enable EXTI2 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    // Same as above for button
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->IMR |= EXTI_IMR_MR0;

    // Same as above for 555 Timer
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1;
    EXTI->RTSR |= EXTI_RTSR_TR1;
    EXTI->IMR |= EXTI_IMR_MR1;
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

// This handler is declared in system/src/cmsis/vectors_stm32f051x8.c
void TIM2_IRQHandler(void)
{
    // Check if update interrupt flag is indeed set
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        trace_printf("\n*** Overflow! ***\n");
        /* Clear update interrupt flag */
        // Relevant register: TIM2->SR
        TIM2->SR &= ~(TIM_SR_UIF);
        /* Restart stopped timer */
        // Relevant register: TIM2->CR1
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

// This handler is declared in system/src/cmsis/vectors_stm32f051x8.c
// Used for the function generator
void EXTI2_3_IRQHandler(void)
{
    // Check if EXTI2 interrupt pending flag is indeed set
    if ((EXTI->PR & EXTI_PR_PR2) != 0)
    {
        if(!timer_triggered) {
            // Detect the first edge
            timer_triggered = 1;

            // Reset and start the timer
            TIM2->CNT = 0;
            TIM2->CR1 |= TIM_CR1_CEN;

        } else {
            // stop the timer
            TIM2->CR1 &= ~TIM_CR1_CEN;

            // Set global calculated frequency
            Frequency = (1.0 / (((float) TIM2->CNT) / SystemCoreClock)) + 0.5;

            timer_triggered = 0;
        }
        // Clear EXTI2 interrupt pending flag (EXTI->PR).
        // NOTE: A pending register (PR) bit is cleared
        // by writing 1 to it.
        //
        EXTI->PR |= EXTI_PR_PR2;
    }
}

// Used for user button and timer 555
void EXTI0_1_IRQHandler(void)
{
    // Handle button
    if ((EXTI->PR & EXTI_PR_PR0) != 0) {
        // Button pressed code:
        in_signal = !in_signal; // Toggle the in_signal
        timer_triggered = 0; // Reset timer flag - switching input methods

        if (in_signal == 0)
            EXTI->IMR &= ~EXTI_IMR_MR2; // switch to 555 timer. Disable function generator interrupts
        else
            EXTI->IMR |= EXTI_IMR_MR2; // Switch to function generator. Enable interrupts

        // slight delay to prevent button debouncing
        for (uint32_t i = 0; i < 0xFFFFF; ++i);

        EXTI->PR |= EXTI_PR_PR0; // Clear interrupt flag and return
        return;
    }
    
    // Handle 555 Timer
    if ((EXTI->PR & EXTI_PR_PR1) != 0) {
        EXTI->PR |= EXTI_PR_PR1;

        // Early return if reading from the function generator
        if (in_signal == 1)
            return;

        if(!timer_triggered) {
            // Detect the first edge
            timer_triggered = 1;

            // Reset and start the timer
            TIM2->CNT = 0;
            TIM2->CR1 |= TIM_CR1_CEN;

        } else {
            // stop the timer
            TIM2->CR1 &= ~TIM_CR1_CEN;

            // Set global calculated frequency
            Frequency = 1.0 / (((float) TIM2->CNT) / SystemCoreClock);

            timer_triggered = 0;
        }
    }
}

void ADC_init(void)
{
    // Enable the clock for the ADC
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Configure PA[5] as analog
    GPIOA->MODER |= GPIO_MODER_MODER5;

    // Set 12 bit data resolution (i.e. 0x0FFF)
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;

    // Set data alignment = right
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;

    // If the ADC conversion has finished, and we haven't read out the data,
    // overwrite it with the latest data
    ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;

    // Automatically start the next conversion once the first conversion has finished (continuously)
    //ADC1->CFGR1 |= ADC_CFGR1_CONT;

    // We configured PA[5] as analog, now configure the ADC to use that channel
    ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

    // Take as many clock cycles when sampling as needed to produce a reliable capture
    ADC1->SMPR |= ADC_SMPR_SMP;

    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;

    // Wait for ADC to be ready to start conversion
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

void DAC_init(void)
{
    // Start the clock for the DAC
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Configure PA[4] as analog
    GPIOA->MODER |= GPIO_MODER_MODER4;

    // Enable channel 1
    DAC->CR |= DAC_CR_EN1;

    // Channel 1 output buffer enabled and channel1 trigger disabled
    DAC->CR &= ~(DAC_CR_BOFF1 | DAC_CR_TEN1);
}

uint32_t POT_read(void)
{
    // Start the ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // wait for the conversion to finish
    while ((ADC1->ISR & ADC_ISR_EOC) == 0);

    // Read and return the value
    return ADC1->DR & 0x0000FFFF;
}

void OLED_init(void)
{
    // Enable SPI1 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // SPI configs for the HAL
    SPI_Handle.Instance = SPI1;
    SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
    SPI_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_Handle.Init.CRCPolynomial = 7;

    // Initialize the SPI interface
    HAL_SPI_Init(&SPI_Handle);

    // Enable the SPI
    __HAL_SPI_ENABLE(&SPI_Handle);

    /* Reset LED Display (RES# = PB4):
       - make pin PB4 = 0, wait for a few ms
       - make pin PB4 = 1, wait for a few ms
    */
    GPIOB->ODR &= ~GPIO_ODR_4;
    for (uint16_t i = 0; i < 0xFFFF; ++i);

    GPIOB->ODR |= GPIO_ODR_4;
    for (uint16_t i = 0; i < 0xFFFF; ++i);

    // Send initialization commands to LED Display
    for (uint8_t i = 0; i < sizeof(OLED_init_cmds); i++)
        OLED_write_cmd(OLED_init_cmds[i]);

    // Fill LED Display data memory (GDDRAM) with zeros: ('Graphics Display DRAM')
    for (uint8_t page = 0; page < 8; ++page) {
        OLED_set_cursor_pos(page, 2);

        for (uint8_t seg = 0; seg < 128; ++seg)
            OLED_write_data(0x00);
    }
}

void OLED_set_cursor_pos(uint8_t page_number, uint8_t segment)
{
    // Set the page number (line [2, 7]) which must be >= 2
    // EX commands:0xB3, 0xB4,..., 0xB7
    OLED_write_cmd(0xB0 | page_number);

    OLED_write_cmd(0x00 | (segment & 0x0F));         // Set lower nibble (---- 0011)
    OLED_write_cmd(0x10 | ((segment & 0xF0) >> 4)); // Set upper nibble (0000 ----)
}

// LED Display Functions
void OLED_update(void)
{
    // At most 16 characters per PAGE + terminating '\0'
    static uint8_t buffer[17];

    snprintf(buffer, sizeof(buffer), "R: %5u Ohms", Resistance);

    // Set LED starting position
    OLED_set_cursor_pos(2, 3);

    // Write the formatted string to the display
    for (uint8_t i = 0; buffer[i] != '\0'; ++i)
        for (uint8_t byte = 0; byte < 8; ++byte)
            OLED_write_data(ASCII_CHARS[buffer[i] - ' '][byte]);

    snprintf(buffer, sizeof(buffer), "F: %5u Hz", Frequency);

    OLED_set_cursor_pos(4, 3); // 2 pages down

    for (uint8_t i = 0; buffer[i] != '\0'; ++i)
        for (uint8_t byte = 0; byte < 8; ++byte)
            OLED_write_data(ASCII_CHARS[buffer[i] - ' '][byte]);

    // Slight delay for screen refresh
    for (uint32_t i = 0; i < 0xFFFFF; ++i);

}

void OLED_write(uint8_t value)
{
    // Wait until SPI1 is ready for writing (TXE = 1 in SPI1_SR) */
    while ((SPI1->SR & SPI_SR_TXE) == 0);

    /* Send one 8-bit character:
       - This function also sets BIDIOE = 1 in SPI1_CR1
    */
    HAL_SPI_Transmit(&SPI_Handle, &value, 1, HAL_MAX_DELAY);

    //Wait until transmission is complete (TXE = 1 in SPI1_SR) */
    while ((SPI1->SR & SPI_SR_TXE) == 0);
}

void OLED_write_cmd(uint8_t cmd)
{
    // Pull CS high to disable it while preparing => make PB6 = CS# = 1
    GPIOB->ODR |= GPIO_ODR_6;

    // Drive D/C low for command (1 for data, 0 for command) => make PB7 = D/C# = 0
    GPIOB->ODR &= ~GPIO_ODR_7;

    // Drive CS low now that were ready to transmit => make PB6 = CS# = 0
    GPIOB->ODR &= ~GPIO_ODR_6;

    // Transmit 8 bits of information through SPI (8 pulses)
    OLED_write(cmd);

    // Deactivate => make PB6 = CS# = 1
    GPIOB->ODR |= GPIO_ODR_6;
}

void OLED_write_data(uint8_t data)
{
    // Pull CS high to disable it while preparing => make PB6 = CS# = 1
    GPIOB->ODR |= GPIO_ODR_6;

    // Drive D/C high for data (1 for data, 0 for command) => make PB7 = D/C# = 1
    GPIOB->ODR |= GPIO_ODR_7;

    // Drive CS low now that were ready to transmit => make PB6 = CS# = 0
    GPIOB->ODR &= ~GPIO_ODR_6;

    // Transmit 8 bits of information through SPI (8 pulses)
    OLED_write(data);

    // Deactivate => make PB6 = CS# = 1
    GPIOB->ODR |= GPIO_ODR_6;
}

int main(void)
{
    // Boost clock to 48 MHz
    SystemClock48MHz();
    trace_printf("System clock: %u Hz\n", SystemCoreClock);

    GPIOA_init();
    GPIOB_init();
    TIM2_init();
    EXTI_init();
    ADC_init();
    DAC_init();
    OLED_init();

    while (1)
    {
        uint32_t pot_value = POT_read(); // Should read [0, 4095]

        // Relay digital value to DAC
        DAC->DHR12R1 = pot_value;

        // Calculate and set the Resistance value
        Resistance = ((uint32_t)((pot_value / ADC_MAX_VALUE) * RESITANCE_MAX_VALUE))

        OLED_update();
    }
}

#pragma GCC diagnostic pop