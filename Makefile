.PHONY: clean all

src = $(CURDIR)
INMATES = $(src)/../jailhouse/inmates
INMATES_LIB = $(INMATES)/lib
export INMATES_LIB

include $(INMATES_LIB)/x86/Makefile.lib

INCLUDES := -I$(INMATES_LIB)/x86 \
	    -I$(INMATES_LIB)/../../hypervisor/arch/x86/include \
	    -I$(INMATES_LIB)/../../hypervisor/include \
	    -I$(src)/freertos/Source/portable/GCC/X86jailhouse

src = $(CURDIR)

CC = gcc
LD = ld
AR = ar
OBJCOPY = objcopy

CFLAGS += -Wall
CFLAGS += $(INCLUDES) -I $(src) -I $(src)/freertos/Source/include

LDFLAGS += -T lscript.lds

EXE_STEM = freertos-demo

FREERTOS_OBJS = freertos/Source/queue.o \
	freertos/Source/list.o \
	freertos/Source/croutine.o \
	freertos/Source/event_groups.o \
	freertos/Source/portable/MemMang/heap_1.o \
	freertos/Source/timers.o \
	freertos/Source/tasks.o

FREERTOS_RUNTIME_OBJS = freertos-runtime/string.o \
	$(INMATES_LIB)/int.o \
	$(INMATES_LIB)/printk.o


RUNTIME_OBJS = $(FREERTOS_RUNTIME_OBJS) $(FREERTOS_OBJS)
OBJS = main
BOOT = boot

RUNTIME_AR = libfreertos.a

all: $(EXE_STEM).bin

DEPS := $(OBJS:.o=.d) $(RUNTIME_OBJS:.o=.d)

$(BOOT).o: $(BOOT).S
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJS).o: $(OBJS).c
	$(CC) $(CFLAGS) -c -o $@ $^

$(EXE_STEM).elf: $(OBJS).o $(BOOT).o $(RUNTIME_AR)
	$(LD) $(LDFLAGS) -o $@ $^

$(RUNTIME_AR): $(RUNTIME_OBJS)
	$(AR) -srcv $@ $^

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(EXE_STEM).elf $(EXE_STEM).bin $(RUNTIME_OBJS) $(RUNTIME_AR)

distclean: clean
	rm -f $(DEPS)

-include $(DEPS)
