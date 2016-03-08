.PHONY: clean all

INMATES_LIB = $(INMATES)/lib/x86
export INMATES_LIB

INCLUDES := -I$(INMATES_LIB) \
	    -I$(INMATES_LIB)/../hypervisor/arch/x86/include \
	    -I$(INMATES_LIB)/../hypervisor/include

src = $(CURDIR)

CC = gcc
LD = ld
AR = ar
OBJCOPY = objcopy

CFLAGS += -O2
CFLAGS += -Wall -MMD -pipe
CFLAGS += -I $(src) -I $(src)/freertos/Source/include -I $(src)/freertos-runtime

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
	freertos-runtime/serial.o \
	freertos-runtime/printf-stdarg.o \
	freertos-runtime/lib1funcs.o

RUNTIME_OBJS = $(FREERTOS_RUNTIME_OBJS) $(FREERTOS_OBJS)
OBJS = main.o boot.o

RUNTIME_AR = libfreertos.a

all: $(EXE_STEM).bin

DEPS := $(OBJS:.o=.d) $(RUNTIME_OBJS:.o=.d)

$(EXE_STEM).elf: $(OBJS) $(RUNTIME_AR)
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
