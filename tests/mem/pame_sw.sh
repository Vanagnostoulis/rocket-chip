riscv64-unknown-elf-gcc -O2 -Wall mem.c -o mem
riscv64-unknown-elf-objdump -D mem > mem.dump
time ../rocket-chip/emulator/emulator-freechips.rocketchip.system-freechips.rocketchip.system.MyConfig +verbose pk mem 2>&1 | spike-dasm > output
