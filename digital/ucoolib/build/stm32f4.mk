# ucoolib - Microcontroller object oriented library.
#
# Rules for STM32F4.

LIBOPENCM3_PATH ?= $(BASE)/lib/libopencm3

stm32f4_SUBTARGETS := stm32 arm newlib

stm32f4_CPPFLAGS := $(CPPFLAGS) $(stm32f4_DEFS) \
	-DSTM32F4 -I$(LIBOPENCM3_PATH)/include \
	-DTARGET_STM32F4=1 \
	$(foreach sub,$(stm32f4_SUBTARGETS),-DTARGET_SUB_$(sub)=1)
stm32f4_CFLAGS := $(CFLAGS) \
	-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-Wl,--gc-sections -ffunction-sections
stm32f4_CXXFLAGS := $(sort $(stm32f4_CFLAGS) $(CXXFLAGS))
stm32f4_ASFLAGS := $(ASFLAGS)
stm32f4_LDSCRIPT := stm32f4.ld
stm32f4_LDFLAGS := $(LDFLAGS) -T$(stm32f4_LDSCRIPT) \
	-L$(BASE)/ucoolib/arch/stm32f4 \
	-L$(LIBOPENCM3_PATH)/lib
stm32f4_LDLIBS := -nostartfiles $(LDLIBS) $(stm32f4_LIBS) \
	-lopencm3_stm32f4

stm32f4_PREFIX := arm-none-eabi-
$(eval $(call arch_cmds,stm32f4))

stm32f4_ELF_SUFFIX = .elf

# Rules.

$(eval $(call arch_build_rules,stm32f4))
$(eval $(call arch_lst_rules,stm32f4))
$(eval $(call arch_size_rules,stm32f4))
$(eval $(call arch_misc_rules,stm32f4))
