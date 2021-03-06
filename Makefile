.PHONY: clean all

src = $(CURDIR)
INMATES = $(src)/../jailhouse/inmates
INMATES_LIB = $(INMATES)/lib
export INMATES_LIB

include $(INMATES_LIB)/x86/Makefile.lib

INCLUDES := -I$(INMATES_LIB)/x86 \
	    -I$(INMATES_LIB)/../../hypervisor/arch/x86/include \
	    -I$(INMATES_LIB)/../../hypervisor/include \
	    -I$(src)/freertos/Source/portable/GCC/X86jailhouse \
		-I $(src)/freertos-runtime

CC = gcc
LD = ld
AR = ar
OBJCOPY = objcopy

CFLAGS += -O2 -Wall -MMD -pipe -m32
CFLAGS += $(INCLUDES) -I $(src) -I $(src)/freertos/Source/include
ASFLAGS += -m32 $(INCLUDES) -I $(src) -I $(src)/freertos/Source/include
LDFLAGS += -m elf_i386 -T lscript.lds

EXE_STEM = freertos-demo

FREERTOS_OBJS = freertos/Source/queue.o \
	freertos/Source/list.o \
	freertos/Source/croutine.o \
	freertos/Source/event_groups.o \
	freertos/Source/portable/MemMang/heap_1.o \
	freertos/Source/portable/GCC/X86jailhouse/port.o \
	freertos/Source/portable/GCC/X86jailhouse/portASM.o \
	freertos/Source/timers.o \
	freertos/Source/tasks.o

FREERTOS_RUNTIME_OBJS = freertos-runtime/string.o \
	$(INMATES_LIB)/x86/int.o \
	$(INMATES_LIB)/x86/printk.o


RUNTIME_OBJS = $(FREERTOS_RUNTIME_OBJS) $(FREERTOS_OBJS)
OBJS = main.o boot.o

RUNTIME_AR = libfreertos.a

all: $(EXE_STEM).bin

DEPS := $(OBJS:.o=.d) $(RUNTIME_OBJS:.o=.d)

$(EXE_STEM).elf: $(OBJS) $(RUNTIME_AR)
	@echo " LD "$@
	@$(LD) $(LDFLAGS) -o $@ $^

$(RUNTIME_AR): $(RUNTIME_OBJS)
	@echo "AR "$@
	@$(AR) -srcv $@ $^

%.bin: %.elf
	@echo "objcopy" $@
	@$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(EXE_STEM).elf $(EXE_STEM).bin $(RUNTIME_OBJS) $(RUNTIME_AR)

distclean: clean
	rm -f $(DEPS)

-include $(DEPS)
