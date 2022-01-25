path L:\tools\xpack-riscv-none-embed-gcc-10.1.0-1.1\bin;%path%

echo set debug remote 1               >gdb.x
echo file Debug/First.elf             >>gdb.x
echo target extended-remote \\.\com6  >>gdb.x
echo set serial baud 115200           >>gdb.x

riscv-none-embed-gdb -x gdb.x
del gdb.x
pause