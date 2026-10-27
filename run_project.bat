@echo off
cd /d "%~dp0"
echo ========================================================================================
echo   EXECUTING ENTIRE QUANSER ACTIVE SUSPENSION BENCHMARK PROJECT END-TO-END
echo ========================================================================================
echo.
echo Running MATLAB verification, 5 road simulations, 45-run sweep, and report generation...
matlab -batch "run_project"
echo.
echo ========================================================================================
echo   PROJECT EXECUTION COMPLETE
echo   Figures:  figures/
echo   Results:  results/sweep_results.csv
echo   PDF:      docs/IEEE_ACCESS_Paper_Active_Suspension_STSMC.pdf
echo ========================================================================================
pause
