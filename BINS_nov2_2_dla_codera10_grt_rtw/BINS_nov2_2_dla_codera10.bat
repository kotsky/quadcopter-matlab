set MATLAB=D:\Programms\MatLAB

cd .

if "%1"=="" (D:\PROGRA~1\MatLAB\bin\win64\gmake -f BINS_nov2_2_dla_codera10.mk all) else (D:\PROGRA~1\MatLAB\bin\win64\gmake -f BINS_nov2_2_dla_codera10.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
