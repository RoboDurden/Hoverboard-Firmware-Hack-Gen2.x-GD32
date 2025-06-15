@echo off

:repeat
REM Remove REM below to add 1 sec delay 
REM timeout /t 1 /nobreak > nul
color 08
cls
st-flash --connect-under-reset write hoverboard.bin 0x8000000 || goto :failure


color 0A
echo:
echo ================================
echo ^|       FLASH SUCCESSFUL       ^| 
echo ================================

goto :exit

:failure

color 09
echo:
echo ================================
echo ^|       FLASH FAILED :-(      ^| 
echo ================================
echo:

:exit

pause 