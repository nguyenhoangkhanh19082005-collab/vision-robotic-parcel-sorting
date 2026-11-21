@echo off
REM Khoi dong tram phan loai hop nhat (Vision + bang tai + robot 6 truc)
set "ROOT=%~dp0"
set "TEMP=E:\UDMT\.tmp"
set "TMP=E:\UDMT\.tmp"
cd /d "%ROOT%"
"%ROOT%..\.venv\Scripts\python.exe" -m sorting_station.app %*
