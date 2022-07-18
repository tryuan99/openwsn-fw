SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\PuTTY\;C:\Users\fmaksimo\AppData\Local\Programs\Python\Python39\Scripts\;C:\Users\fmaksimo\AppData\Local\Programs\Python\Python39\;C:\Users\fmaksimo\Anaconda3;C:\Users\fmaksimo\Anaconda3\Scripts;C:\Program Files\MATLAB\R2018b\runtime\win64;C:\Program Files\MATLAB\R2018b\bin;C:\Program Files (x86)\Pico Technology\PicoScope6\;C:\ProgramData\chocolatey\bin;C:\Program Files\Inkscape\bin;C:\Program Files\TortoiseGit\bin;C:\Program Files\otii;C:\Program Files\CMake\bin;C:\Program Files\dotnet\;C:\MinGW\bin\;C:\Python27;C:\Program Files\Nordic Semiconductor\nrf-command-line-tools\bin\;C:\Program Files (x86)\dotnet\;C:\Program Files\Git\cmd;C:\Users\fmaksimo\AppData\Local\Programs\Python\Python39\Scripts\;C:\Users\fmaksimo\AppData\Local\Programs\Python\Python39\;C:\gnuarmemb\bin;C:\Users\fmaksimo\AppData\Local\Microsoft\WindowsApps;C:\Users\fmaksimo\AppData\Anaconda3;C:\Users\fmaksimo\AppData\Anaconda3\Scripts;C:\Users\fmaksimo\AppData\Local\atom\bin;C:\Users\fmaksimo\.dotnet\tools;C:\MinGW\bin\;C:\Python27;;C:\Users\fmaksimo\AppData\Local\Microsoft\WindowsApps
SET CPU_TYPE=ARMCM0
SET CPU_VENDOR=ARM
SET UV2_TARGET=Target SCuM
SET CPU_CLOCK=0x00B71B00
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\01bsp_radio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\objects\cm0dsasm._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\leds.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\board.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\uart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\debugpins.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\radio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\eui64.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\sctimer.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\retarget.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\adc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\optical.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\scm3c_hw_interface.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\Objects\01bsp_radio.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" ".\Objects\01bsp_radio.axf" --i32combined --output ".\Objects\01bsp_radio.hex"
fromelf --bin .\Objects\01bsp_radio.axf -o .\Objects\01bsp_radio.bin
fromelf -cvf .\Objects\01bsp_radio.axf -o .\Objects\disasm.txt
