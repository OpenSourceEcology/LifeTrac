# Build script (PowerShell) for L072 hello-world.
# Uses Arduino-bundled arm-none-eabi-gcc 7-2017q4.

$ErrorActionPreference = "Stop"
$TC = "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4\bin"
$GCC = "$TC\arm-none-eabi-gcc.exe"
$OBJCOPY = "$TC\arm-none-eabi-objcopy.exe"
$SIZE = "$TC\arm-none-eabi-size.exe"

$SRC = Join-Path $PSScriptRoot "main.c"
$LD  = Join-Path $PSScriptRoot "stm32l072.ld"
$ELF = Join-Path $PSScriptRoot "hello.elf"
$BIN = Join-Path $PSScriptRoot "hello.bin"

$flags = @(
  "-mcpu=cortex-m0plus", "-mthumb",
  "-Os", "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra",
  "-nostdlib", "-nostartfiles", "-fno-builtin",
  "-T", $LD,
  "-Wl,--gc-sections", "-Wl,-Map=$($PSScriptRoot)\hello.map",
  "-o", $ELF, $SRC
)

Write-Host "Compiling..."
& $GCC @flags
if ($LASTEXITCODE -ne 0) { throw "compile failed" }

Write-Host "Stripping to .bin..."
& $OBJCOPY -O binary $ELF $BIN
if ($LASTEXITCODE -ne 0) { throw "objcopy failed" }

Write-Host "Sizes:"
& $SIZE $ELF
$binSz = (Get-Item $BIN).Length
Write-Host "hello.bin = $binSz bytes"
