#ifndef _GENERAL_H
#define _GENERAL_H

#include "stm32f7xx_hal.h"


#define LED_G_PORT GPIOB
#define LED_G_PIN (uint32_t)GPIO_PIN_0
#define LED_R_PORT GPIOB
#define LED_R_PIN (uint32_t)GPIO_PIN_14
#define LED_B_PORT GPIOB
#define LED_B_PIN (uint32_t)GPIO_PIN_7

#define LED_G_ON() LED_G_PORT->BSRR = LED_G_PIN
#define LED_R_ON() LED_R_PORT->BSRR = LED_R_PIN
#define LED_B_ON() LED_B_PORT->BSRR = LED_B_PIN

#define LED_G_OFF() LED_G_PORT->BSRR = LED_G_PIN << 16
#define LED_R_OFF() LED_R_PORT->BSRR = LED_R_PIN << 16
#define LED_B_OFF() LED_B_PORT->BSRR = LED_B_PIN << 16

#define LED_G_TOG() HAL_GPIO_TogglePin(LED_G_PORT, LED_G_PIN)
#define LED_R_TOG() HAL_GPIO_TogglePin(LED_R_PORT, LED_R_PIN)
#define LED_B_TOG() HAL_GPIO_TogglePin(LED_B_PORT, LED_B_PIN)


//#define DBG_0(state) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, state)

#define DBG_0_PORT GPIOC
#define DBG_0_PIN (uint32_t)GPIO_PIN_2
#define DBG_1_PORT GPIOB
#define DBG_1_PIN (uint32_t)GPIO_PIN_1
#define DBG_2_PORT GPIOF
#define DBG_2_PIN (uint32_t)GPIO_PIN_4
#define DBG_3_PORT GPIOB
#define DBG_3_PIN (uint32_t)GPIO_PIN_6
#define DBG_4_PORT GPIOB
#define DBG_4_PIN (uint32_t)GPIO_PIN_2

#define DBG_0_ON() DBG_0_PORT->BSRR = DBG_0_PIN
#define DBG_1_ON() DBG_1_PORT->BSRR = DBG_1_PIN
#define DBG_2_ON() DBG_2_PORT->BSRR = DBG_2_PIN
#define DBG_3_ON() DBG_3_PORT->BSRR = DBG_3_PIN
#define DBG_4_ON() DBG_4_PORT->BSRR = DBG_4_PIN

#define DBG_0_OFF() DBG_0_PORT->BSRR = DBG_0_PIN << 16
#define DBG_1_OFF() DBG_1_PORT->BSRR = DBG_1_PIN << 16
#define DBG_2_OFF() DBG_2_PORT->BSRR = DBG_2_PIN << 16
#define DBG_3_OFF() DBG_3_PORT->BSRR = DBG_3_PIN << 16
#define DBG_4_OFF() DBG_4_PORT->BSRR = DBG_4_PIN << 16

#define AL_PORT GPIOG
#define AL_PIN GPIO_PIN_14
#define AH_PORT GPIOF
#define AH_PIN GPIO_PIN_13

#define BL_PORT GPIOD
#define BL_PIN GPIO_PIN_14
#define BH_PORT GPIOB
#define BH_PIN GPIO_PIN_4

#define CL_PORT GPIOC
#define CL_PIN GPIO_PIN_7
#define CH_PORT GPIOG
#define CH_PIN GPIO_PIN_9

#define AL_ON() AL_PORT->BSRR = AL_PIN
#define AH_ON() AH_PORT->BSRR = AH_PIN
#define BL_ON() BL_PORT->BSRR = BL_PIN
#define BH_ON() BH_PORT->BSRR = BH_PIN
#define CL_ON() CL_PORT->BSRR = CL_PIN
#define CH_ON() CH_PORT->BSRR = CH_PIN

#define AL_OFF() AL_PORT->BSRR = AL_PIN << 16
#define AH_OFF() AH_PORT->BSRR = AH_PIN << 16
#define BL_OFF() BL_PORT->BSRR = BL_PIN << 16
#define BH_OFF() BH_PORT->BSRR = BH_PIN << 16
#define CL_OFF() CL_PORT->BSRR = CL_PIN << 16
#define CH_OFF() CH_PORT->BSRR = CH_PIN << 16


#define nop() asm("NOP")
#define sev() asm("SEV")
#define nop10() {nop(); nop(); nop(); nop(); nop(); nop(); nop(); nop(); nop(); nop();}
#define delay25ns() {nop10()}
#define delay100ns() {delay25ns(); delay25ns(); delay25ns(); delay25ns();}
#define delay200ns() {delay100ns(); delay100ns();}

void GeneralInit(void);
float lim_float(float v, float lim_lo, float lim_hi);


#endif /* _GENERAL_H */
