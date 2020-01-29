################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Tests/006_SPI_SendData.c 

OBJS += \
./Src/Tests/006_SPI_SendData.o 

C_DEPS += \
./Src/Tests/006_SPI_SendData.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Tests/006_SPI_SendData.o: ../Src/Tests/006_SPI_SendData.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"D:/MCU Workspace/STM32CubeIDE_Workspace/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/Tests/006_SPI_SendData.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

