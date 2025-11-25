@echo off
REM https://sumo.dlr.de/docs/Downloads.php
REM https://docs.python.org/3/library/venv.html

REM Check that Python and pip are installed
SET COMMAND=python

WHERE python >NUL 2>&1
IF %ERRORLEVEL% EQU 0 SET COMMAND=python

WHERE python3 >NUL 2>&1
IF %ERRORLEVEL% EQU 0 SET COMMAND=python3

WHERE py >NUL 2>&1
IF %ERRORLEVEL% EQU 0 SET COMMAND=py

IF NOT DEFINED COMMAND (
    echo Could not find Python installed in path. Exiting.
    exit /b 1
)

WHERE pip >NUL 2>&1
IF %ERRORLEVEL% NEQ 0 (
    echo Could not find pip installed in path. Exiting.

    pause
    exit /b 1
) ELSE (
    echo pip found
)

REM Install SUMO
SET USING_VENV=0

IF %USING_VENV% EQU 1 (
    %COMMAND% -m venv sumo_env
    cd sumo_env

    REM Activate virtual environment
    if exist Scripts\activate.bat (
        echo Windows detected.
        call Scripts\activate.bat
    ) ELSE (
        echo Could not determine OS type.
        echo Manually activate the virtual environment through the script Scripts\activate.bat.
        echo Manually install sumo to virtual environment through pip:
        echo     pip install eclipse-sumo
        echo     pip install traci
        echo     pip install libsumo
        
        pause
        exit /b 1
    )

    pip install eclipse-sumo
    pip install traci
    pip install libsumo
) ELSE (
    pip install eclipse-sumo
    pip install traci
    pip install libsumo
)

REM Codegen step
cd codegen
start .\install.bat
cd ..