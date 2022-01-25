path L:\tools\xpack-riscv-none-embed-gcc-10.1.0-1.1\bin
riscv-none-embed-objdump.exe -h -j.text -j.data -j.bss %1.elf
riscv-none-embed-objdump.exe -h -t -j.text -j.data -j.bss -S %1.elf > %1_diss.txt
riscv-none-embed-objcopy.exe -O ihex %1.elf %1.hex
