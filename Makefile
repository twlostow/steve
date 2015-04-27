CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

PROJECT_NAME = steve

PROJECT_SRC = src
STM_SRC = Drivers/STM32F4xx_StdPeriph_Driver/src/
OBJ_DIR = bin

vpath %.c $(PROJECT_SRC)
vpath %.c $(STM_SRC)

SRCS = main.c motors.c usart.c vsprintf-xint.c printf.c adc.c biquad.c servo.c rpc.c

SRCS += Device/startup_stm32f401xe.s

#SRCS += stm32f4xx_hal_msp.c
SRCS += stm32f4xx_it.c
SRCS += system_stm32f4xx.c

EXT_SRCS = stm32f4xx_rcc.c \
	   stm32f4xx_gpio.c \
	   stm32f4xx_adc.c \
	   stm32f4xx_tim.c \
	   stm32f4xx_exti.c \
	   stm32f4xx_syscfg.c \
	   stm32f4xx_usart.c \
	    misc.c
#EXT_SRCS += stm32f4xx_hal_rcc.c
#EXT_SRCS += stm32f4xx_hal_gpio.c
#EXT_SRCS += stm32f4xx_hal_cortex.c

EXT_OBJ = $(addprefix $(OBJ_DIR)/, $(EXT_SRCS:.c=.o))

INC_DIRS  = src/
INC_DIRS += Drivers/STM32F4xx_StdPeriph_Driver/inc/
INC_DIRS += Drivers/CMSIS/Device/ST/STM32F4xx/Include/
INC_DIRS += Drivers/CMSIS/Include/

INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS = -DSTM32F401xx -DUSE_STDPERIPH_DRIVER

CFLAGS  = -ggdb -O0 -std=c99
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -Wl,--gc-sections

WFLAGS += -Warray-bounds -Wno-unused-parameter
LFLAGS = -TDevice/gcc.ld --specs=nano.specs

# Create a directory for object files
$(shell mkdir $(OBJ_DIR) > /dev/null 2>&1)

.PHONY: all
all: $(PROJECT_NAME)

.PHONY: $(PROJECT_NAME)
$(PROJECT_NAME): $(PROJECT_NAME).elf

$(PROJECT_NAME).elf: $(SRCS) $(EXT_OBJ)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(WFLAGS) $(LFLAGS) $^ -lm -o $@
	$(OBJCOPY) -O ihex $(PROJECT_NAME).elf   $(PROJECT_NAME).hex
	$(OBJCOPY) -O binary $(PROJECT_NAME).elf $(PROJECT_NAME).bin

$(OBJ_DIR)/%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

clean:
	rm -rf $(OBJ_DIR) $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin

flash: $(PROJECT_NAME).elf
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x_stlink.cfg -f openocd.cfg -c "program $(PROJECT_NAME).elf 0 verify reset"
