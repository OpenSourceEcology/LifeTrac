@echo off
setlocal

rem Safe read-only attach for Murata CMWX1ZZABZ-078 / STM32L072 recovery.
rem Requires external ST-Link wired to CN2/test pads: SWDIO, SWCLK, NRST, GND, VTref.

set OCD=C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin\openocd.exe
if not exist "%OCD%" set OCD=openocd.exe

pushd "%~dp0\.."
"%OCD%" ^
  -f openocd\stlink.cfg ^
  -f openocd\stm32l0_recover_under_reset.cfg ^
  -c "init" ^
  -c "halt" ^
  -c "flash banks" ^
  -c "flash info 0" ^
  -c "mdb 0x1FF80000 16" ^
  -c "shutdown"
set RC=%ERRORLEVEL%
popd

exit /b %RC%