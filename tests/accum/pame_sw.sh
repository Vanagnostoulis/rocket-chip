riscv64-unknown-elf-gcc -O2 -Wall accum.c -o acc
time ../rocket-chip/emulator/emulator-freechips.rocketchip.system-freechips.rocketchip.system.MyConfig +verbose pk acc 2>&1 | spike-dasm > output
