/*
 * stm32f446xx.h
 *
 *  Created on: Aug 23, 2019
 *      Author: neman
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Base addresses of FLASH and  SRAM memories
 */
#define FLASH_BASEADDR						0x80000000U 	/* Base address of FLASH memory */
#define SRAM1_BASEADDR						0x20000000U 	/* Base address of SRAM1 memory */
#define SRAM2_BASEADDR						0x20001C00U 	/* Base address of SRAM2 memory */
#define ROM_BASEADDR						0x1FFF0000U 	/* Base address of ROM memory 	*/
#define SRAM_BASEADDR 						SRAM1_BASEADDR  /* Base address of SRAM memory 	*/

/*
 * AHBx and APBx Bus peripheral base addresses
 */
#define PERIPH_BASE							0x40000000U		/* Base address of PERIPHERALS memory 	  */
#define APB1PERIPH_BASE						PERIPH_BASE		/* Base address of APB1 peripheral memory */
#define APB2PERIPH_BASE						0x40001000U		/* Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASE						0x40002000U		/* Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASE						0x50000000U		/* Base address of AHB2 peripheral memory */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASE + 0x0000) /* GPIOA base address */
#define GPIOB_BASEADDR						(AHB1PERIPH_BASE + 0x0400) /* GPIOB base address */
#define GPIOC_BASEADDR						(AHB1PERIPH_BASE + 0x0800) /* GPIOC base address */
#define GPIOD_BASEADDR						(AHB1PERIPH_BASE + 0x0C00) /* GPIOD base address */
#define GPIOE_BASEADDR						(AHB1PERIPH_BASE + 0x1000) /* GPIOE base address */
#define GPIOF_BASEADDR						(AHB1PERIPH_BASE + 0x1400) /* GPIOF base address */
#define GPIOG_BASEADDR						(AHB1PERIPH_BASE + 0x1800) /* GPIOG base address */
#define GPIOH_BASEADDR						(AHB1PERIPH_BASE + 0x1C00) /* GPIOH base address */
#define RCC_BASEADDR						(AHB1PERIPH_BASE + 0x3800) /* RCC base address	 */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400) /* I2C1 base address   */
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800) /* I2C2 base address   */
#define I2C3_BASEADDR						(APB1PERIPH_BASE + 0x5C00) /* I2C3 base address   */
#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800) /* SPI2 base address   */
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00) /* SPI3 base address   */
#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400) /* USART2 base address */
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800) /* USART3 base address */
#define UART4_BASEADDR						(APB1PERIPH_BASE + 0x4C00) /* UART4 base address  */
#define UART5_BASEADDR						(APB1PERIPH_BASE + 0x5000) /* UART5 base address  */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define SPI1_BASEADDR						(APB2PERIPH_BASE + 0x3000) /* SPI1 base address   */
#define USART1_BASEADDR						(APB2PERIPH_BASE + 0x1000) /* USART1 base address */
#define USART6_BASEADDR						(APB2PERIPH_BASE + 0x1400) /* USART6 base address */
#define SYSCFG_BASEADDR						(APB2PERIPH_BASE + 0x3800) /* SYSCFG base address */
#define EXTI_BASEADDR						(APB2PERIPH_BASE + 0x3C00) /* EXTI base address   */


/**********************************************************************************************
 * Peripheral register definition structures
 **********************************************************************************************/

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;  						/* GPIO port mode register, 	    		Address offset: 0x00 */
	__vo uint32_t OTYPER; 					    /* GPIO port output type register,  		Address offset: 0x04 */
	__vo uint32_t OSPEEDER;  				    /* GPIO port output speed register, 		Address offset: 0x08 */
	__vo uint32_t PUPDR; 					    /* GPIO port pull-up/down register, 		Address offset: 0x0C */
	__vo uint32_t IDR; 							/* GPIO port input data register,    		Address offset: 0x10 */
	__vo uint32_t ODR; 							/* GPIO port output data register, 	 		Address offset: 0x14 */
	__vo uint32_t BSRR; 						/* GPIO port bit set/reset register, 		Address offset: 0x18 */
	__vo uint32_t LCKR; 						/* GPIO port configuration lock register,	Address offset: 0x1C */
	__vo uint32_t AFR[2];						/* AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register, Address offset: 0x20-24 */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t CR;							/* RCC clock control register, 	    								Address offset: 0x00 	*/
	__vo uint32_t PLLCFGR;						/* RCC PLL configuration register, 	    							Address offset: 0x04 	*/
	__vo uint32_t CFGR;							/* RCC clock configuration register, 	   							Address offset: 0x08 	*/
	__vo uint32_t CIR;							/* RCC clock interrupt register, 	    							Address offset: 0x0C 	*/
	__vo uint32_t AHB1RSTR;						/* RCC AHB1 peripheral reset register, 	    						Address offset: 0x10 	*/
	__vo uint32_t AHB2RSTR;						/* RCC AHB2 peripheral reset register, 	    						Address offset: 0x14 	*/
	__vo uint32_t AHB3RSTR;						/* RCC AHB3 peripheral reset register, 	    						Address offset: 0x18 	*/
	uint32_t 	  RESERVED0;					/* RCC reserved register, 											Address offset: 0x1C 	*/
	__vo uint32_t APB1RSTR;						/* RCC APB1 peripheral reset register, 	    						Address offset: 0x20 	*/
	__vo uint32_t APB2RSTR;						/* RCC APB2 peripheral reset register, 	    						Address offset: 0x24 	*/
	uint32_t 	  RESERVED1[2];					/* RCC reserved register, 											Address offset: 0x28-2C */
	__vo uint32_t AHB1ENR;						/* RCC AHB1 peripheral clock enable register, 	    				Address offset: 0x30 	*/
	__vo uint32_t AHB2ENR;						/* RCC AHB2 peripheral clock enable register, 	    				Address offset: 0x34 	*/
	__vo uint32_t AHB3ENR;						/* RCC AHB3 peripheral clock enable register, 	    				Address offset: 0x38 	*/
	uint32_t 	  RESERVED2;					/* RCC reserved register, 											Address offset: 0x3C 	*/
	__vo uint32_t APB1ENR;						/* RCC APB1 peripheral clock enable register, 	    				Address offset: 0x40 	*/
	__vo uint32_t APB2ENR;						/* RCC APB2 peripheral clock enable register, 	    				Address offset: 0x44 	*/
	uint32_t 	  RESERVED3[2];					/* RCC reserved register, 											Address offset: 0x48-4C */
	__vo uint32_t AHB1LPENR;					/* RCC AHB1 peripheral clock enable in low power mode register,		Address offset: 0x50 	*/
	__vo uint32_t AHB2LPENR;					/* RCC AHB2 peripheral clock enable in low power mode register,		Address offset: 0x54 	*/
	__vo uint32_t AHB3LPENR;					/* RCC AHB3 peripheral clock enable in low power mode register,		Address offset: 0x58 	*/
	uint32_t 	  RESERVED4;					/* RCC reserved register, 											Address offset: 0x5C 	*/
	__vo uint32_t APB1LPENR;					/* RCC APB1 peripheral clock enable in low power mode register,		Address offset: 0x60 	*/
	__vo uint32_t APB2LPENR;					/* RCC APB2 peripheral clock enable in low power mode register,		Address offset: 0x64 	*/
	uint32_t 	  RESERVED5[2];					/* RCC reserved register, 											Address offset: 0x68-6C */
	__vo uint32_t BDCR;							/* RCC back up domain control register, 							Address offset: 0x70 	*/
	__vo uint32_t CSR;							/* RCC clock control & status register, 							Address offset: 0x74 	*/
	uint32_t 	  RESERVED6[2];					/* RCC reserved register, 											Address offset: 0x78-7C */
	__vo uint32_t SSCGR;						/* RCC spread spectrum clock generation register, 					Address offset: 0x80 	*/
	__vo uint32_t PLLI2SCFGR;					/* RCC PLLI2S configuration register, 								Address offset: 0x84 	*/
	__vo uint32_t PLLSAICFGR;					/* RCC PLL configuration register, 									Address offset: 0x88 	*/
	__vo uint32_t DCKCFGR;						/* RCC dedicated clock configuration register, 						Address offset: 0x8C 	*/
	__vo uint32_t CKGATENR;						/* RCC clock gated enable register, 								Address offset: 0x90 	*/
	__vo uint32_t DCKCFGR2;						/* RCC dedicated clock configuration register 2, 					Address offset: 0x94 	*/
}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA 								((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 								((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 								((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*) RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()						( RCC->AHB1ENR |= (1 << 0) ) /* GPIOA peripheral clock enabled */
#define GPIOB_PCLK_EN()						( RCC->AHB1ENR |= (1 << 1) ) /* GPIOB peripheral clock enabled */
#define GPIOC_PCLK_EN()						( RCC->AHB1ENR |= (1 << 2) ) /* GPIOC peripheral clock enabled */
#define GPIOD_PCLK_EN()						( RCC->AHB1ENR |= (1 << 3) ) /* GPIOD peripheral clock enabled */
#define GPIOE_PCLK_EN()						( RCC->AHB1ENR |= (1 << 4) ) /* GPIOE peripheral clock enabled */
#define GPIOF_PCLK_EN()						( RCC->AHB1ENR |= (1 << 5) ) /* GPIOF peripheral clock enabled */
#define GPIOG_PCLK_EN()						( RCC->AHB1ENR |= (1 << 6) ) /* GPIOG peripheral clock enabled */
#define GPIOH_PCLK_EN()						( RCC->AHB1ENR |= (1 << 7) ) /* GPIOH peripheral clock enabled */


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()						( RCC->APB1ENR |= (1 << 21) ) /* I2C1 peripheral clock enabled */
#define I2C2_PCLK_EN()						( RCC->APB1ENR |= (1 << 22) ) /* I2C2 peripheral clock enabled */
#define I2C3_PCLK_EN()						( RCC->APB1ENR |= (1 << 23) ) /* I2C3 peripheral clock enabled */


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()						( RCC->APB2ENR |= (1 << 12) ) /* SPI1 peripheral clock enabled */
#define SPI2_PCLK_EN()						( RCC->APB1ENR |= (1 << 14) ) /* SPI2 peripheral clock enabled */
#define SPI3_PCLK_EN()						( RCC->APB1ENR |= (1 << 15) ) /* SPI3 peripheral clock enabled */


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()					( RCC->APB2ENR |= (1 << 4) )  /* USART1 peripheral clock enabled */
#define USART2_PCLK_EN()					( RCC->APB1ENR |= (1 << 17) ) /* USART2 peripheral clock enabled */
#define USART3_PCLK_EN()					( RCC->APB1ENR |= (1 << 18) ) /* USART3 peripheral clock enabled */
#define UART4_PCLK_EN()						( RCC->APB1ENR |= (1 << 19) ) /* USART4 peripheral clock enabled */
#define UART5_PCLK_EN()						( RCC->APB1ENR |= (1 << 20) ) /* USART5 peripheral clock enabled */
#define USART6_PCLK_EN()					( RCC->APB2ENR |= (1 << 5) )  /* USART6 peripheral clock enabled */


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()					( RCC->APB2ENR |= (1 << 14) )  /* SYSCFG peripheral clock enabled */


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 0) ) /* GPIOA peripheral clock disabled */
#define GPIOB_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 1) ) /* GPIOB peripheral clock disabled */
#define GPIOC_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 2) ) /* GPIOC peripheral clock disabled */
#define GPIOD_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 3) ) /* GPIOD peripheral clock disabled */
#define GPIOE_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 4) ) /* GPIOE peripheral clock disabled */
#define GPIOF_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 5) ) /* GPIOF peripheral clock disabled */
#define GPIOG_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 6) ) /* GPIOG peripheral clock disabled */
#define GPIOH_PCLK_DI()						( RCC->AHB1ENR &= ~(1 << 7) ) /* GPIOH peripheral clock disabled */

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 21) ) /* I2C1 peripheral clock disabled */
#define I2C2_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 22) ) /* I2C2 peripheral clock disabled */
#define I2C3_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 23) ) /* I2C3 peripheral clock disabled */


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()						( RCC->APB2ENR &= ~(1 << 12) ) /* SPI1 peripheral clock disabled */
#define SPI2_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 14) ) /* SPI2 peripheral clock disabled */
#define SPI3_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 15) ) /* SPI3 peripheral clock disabled */


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 4) )  /* USART1 peripheral clock disabled */
#define USART2_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 17) ) /* USART2 peripheral clock disabled */
#define USART3_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 18) ) /* USART3 peripheral clock disabled */
#define UART4_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 19) ) /* USART4 peripheral clock disabled */
#define UART5_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 20) ) /* USART5 peripheral clock disabled */
#define USART6_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 5) )  /* USART6 peripheral clock disabled */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 14) )  /* SYSCFG peripheral clock disabled */


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


/*
 * Generic Macros
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F446XX_H_ */

