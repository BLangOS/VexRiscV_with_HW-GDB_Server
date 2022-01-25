path L:\tools\xpack-riscv-none-embed-gcc-10.1.0-1.1\bin;%path%

del gdb.x
echo set serial baud 115200  >gdb.x
echo set debug remote 1      >>gdb.x
echo file main.elf           >>gdb.x
echo target remote \\.\com6  >>gdb.x

riscv-none-embed-gdb -x gdb.x

pause