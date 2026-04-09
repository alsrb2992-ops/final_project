@echo off
REM ===================================================================
REM Vivado 프로젝트 생성 도우미 (Windows)
REM ===================================================================

echo ======================================
echo  Vivado Project Setup Script
echo ======================================
echo.

REM 프로젝트 이름
set /p PROJECT_NAME="Project Name [Default: person_detection]: "
if "%PROJECT_NAME%"=="" set PROJECT_NAME=person_detection

REM 프로젝트 경로
set /p PROJECT_DIR="Project Directory [Default: .\build]: "
if "%PROJECT_DIR%"=="" set PROJECT_DIR=.\build

REM 보드 선택
echo.
echo Select Board:
echo   1) ZyboZ7-20
echo   2) ZyboZ7-10
echo   3) Custom (Manual Input)
set /p BOARD_CHOICE="Choose [1-3]: "

if "%BOARD_CHOICE%"=="1" (
    set BOARD_PART=digilentinc.com:zybo-z7-20:part0:1.1
) else if "%BOARD_CHOICE%"=="2" (
    set BOARD_PART=digilentinc.com:zybo-z7-10:part0:1.1
) else if "%BOARD_CHOICE%"=="3" (
    set /p BOARD_PART="Input Board Part: "
) else (
    echo Invalid selection. Proceeding with ZyboZ7-20.
    set BOARD_PART=digilentinc.com:zybo-z7-20:part0:1.1
)

echo.
echo ======================================
echo Configuration:
echo   Project Name: %PROJECT_NAME%
echo   Project Dir:  %PROJECT_DIR%
echo   Board:        %BOARD_PART%
echo ======================================
set /p CONFIRM="Proceed? [y/N]: "

if /i not "%CONFIRM%"=="y" (
    echo Cancelled.
    exit /b 0
)

echo.
echo Creating Vivado project...
vivado -mode batch -source scripts\create_project.tcl -tclargs ^
    "%PROJECT_NAME%" ^
    "%PROJECT_DIR%" ^
    "%BOARD_PART%"

if %errorlevel% equ 0 (
    echo.
    echo [SUCCESS] Project created successfully!
    echo Opening Vivado GUI...
    echo.
    
    REM Vivado GUI 자동 실행
    start vivado %PROJECT_DIR%\%PROJECT_NAME%.xpr
) else (
    echo [ERROR] Project creation failed!
    exit /b 1
)
