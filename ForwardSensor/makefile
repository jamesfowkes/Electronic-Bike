NAME=test

AVR_DIR=C:/WinAVR-20100110/bin

CC=$(AVR_DIR)/avr-gcc
ifdef SystemRoot
    RM = del /Q
	CAT = type
	FIXPATH = $(subst /,\,$1)
else
    RM = rm -f
	CAT = cat
endif

MCU_TARGET=attiny84
LIBS_DIR = $(PROJECTS_PATH)\Libs

OPT_LEVEL=s

INCLUDE_DIRS = \
	-I..\Common \
	-I$(LIBS_DIR)/AVR \
	-I$(LIBS_DIR)/Common \
	-I$(LIBS_DIR)/Devices \
	-I$(LIBS_DIR)/Generics \
	-I$(LIBS_DIR)/Utility

CFILES = \
	main.c \
	io_config.c \
	$(LIBS_DIR)/AVR/lib_clk.c \
	$(LIBS_DIR)/AVR/lib_io.c \
	$(LIBS_DIR)/AVR/lib_fuses.c \
	$(LIBS_DIR)/AVR/lib_adc.c \
	$(LIBS_DIR)/AVR/lib_spi.c \
	$(LIBS_DIR)/AVR/lib_usi.c \
	$(LIBS_DIR)/AVR/lib_tmr8.c \
	$(LIBS_DIR)/AVR/lib_tmr8_tick.c \
	$(LIBS_DIR)/Generics/memorypool.c \

OPTS = \
	-Wall \
	-Wextra \
	-DF_CPU=8000000 \
	-DMEMORY_POOL_BYTES=128

LDFLAGS = \
	-Wl,-Map=$(MAPFILE)

OBJDEPS=$(CFILES:.c=.o)

MAPFILE = $(NAME).map

all: $(NAME).elf errors
	
$(NAME).elf: $(OBJDEPS)
	-$(CC) $(INCLUDE_DIRS) $(OPTS) $(LDFLAGS) -O$(OPT_LEVEL) -mmcu=$(MCU_TARGET) -o $@ $^

%.o:%.c
	-$(CC) $(INCLUDE_DIRS) $(OPTS) -O$(OPT_LEVEL) -mmcu=$(MCU_TARGET) -c $< -o $@
	
clean:
	$(RM) $(call FIXPATH, $(NAME).elf)
	$(RM) $(call FIXPATH, $(OBJDEPS))
	