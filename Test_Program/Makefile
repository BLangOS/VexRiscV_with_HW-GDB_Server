# GCC toolchain programs.
CC = riscv-none-embed-gcc
OC = riscv-none-embed-objcopy
OD = riscv-none-embed-objdump
OS = riscv-none-embed-size
ARCH=rv32i

# Assembly directives.
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -g
ASFLAGS += -Wall
ASFLAGS += -fmessage-length=0
ASFLAGS += -mno-div
ASFLAGS += -march=$(ARCH) # -march=rv32ic
ASFLAGS += -mabi=ilp32
ASFLAGS += -mcmodel=medlow

# C compilation directives
CFLAGS += -c
CFLAGS += -Wall
CFLAGS += -O0
CFLAGS += -g
CFLAGS += -fmessage-length=0
CFLAGS += --specs=nosys.specs
CFLAGS += -mno-div
CFLAGS += -march=$(ARCH) # -march=rv32ic
CFLAGS += -mabi=ilp32
CFLAGS += -mcmodel=medlow

# Linker directives.
LFLAGS += -Wall
LFLAGS += -Wl,--no-relax
LFLAGS += -Wl,--gc-sections
LFLAGS += -nostdlib
LFLAGS += -nostartfiles
LFLAGS += -lc
LFLAGS += -lgcc
LFLAGS += --specs=nosys.specs
LFLAGS += -mno-div
LFLAGS += -march=$(ARCH) # -march=rv32ic
LFLAGS += -mabi=ilp32
LFLAGS += -mcmodel=medlow # Generate code for the medium-low code model
LFLAGS += -TLinker.ld

# Source files.
AS_SRC  = Boot.S
C_SRC   = 

# Header file directories.
#INCLUDE = -I./../common/device_headers

# Object files to build.
OBJS  = $(AS_SRC:.S=.o)
OBJS += $(C_SRC:.c=.o)

# Default rule to build the whole project.
.PHONY: all
all: main.bin main_diss.txt main.hex

# Rule to build assembly files.
%.o: %.S
	$(CC) -x assembler-with-cpp $(ASFLAGS) $(INCLUDE) $< -o $@

# Rule to compile C files.
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

# Rule to create an ELF file from the compiled object files.
main.elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

# Rule to create a raw binary file from an ELF file.
main.bin: main.elf
	$(OC) -S -O binary $< $@
	$(OS) $<

# disassemble for information purpose
main_diss.txt: main.elf

#	$(OD) -h -t -j.vectors -j.text -j.data -j.bss -S $< > $@
	$(OD) -D -S $< > $@
  
# Generate HEX-FIle for memory initialization
main.hex: main.elf
	$(OC) -O ihex $< $@

# Rule to clear out generated build files.
.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f main.elf
	rm -f main.bin
	rm -f main.hex
	rm -f main_diss.txt
  