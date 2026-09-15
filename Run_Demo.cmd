@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0Outputs" mkdir "%~dp0Outputs"
set "MODE=%~1"
if defined MODE goto run
echo WM9M3 renderer demonstrations
echo 1. Cornell Box - path tracing and guided denoising, 32 SPP
echo 2. MaterialsScene - path tracing and guided denoising, 32 SPP
echo 3. Cornell Box - Light Tracing, 4 light-path rounds
echo 4. Cornell Box - Instant Radiosity, 4 SPP and 64 emitted paths
choice /c 1234 /n /m "Choose 1-4: "
set "MODE=%ERRORLEVEL%"
:run
set "EXE=%~dp0x64\Release\RTBase.exe"
cd /d "%~dp0RTBase"
if "%MODE%"=="1" goto path
if "%MODE%"=="2" goto materials
if "%MODE%"=="3" goto light
if "%MODE%"=="4" goto ir
echo Invalid mode. Choose 1, 2, 3 or 4.
exit /b 1
:path
"%EXE%" -scene cornell-box -integrator path -SPP 32 -threads 0 -denoise guided -aovPrefix "%~dp0Outputs\cornell_aov" -outputFilename "%~dp0Outputs\cornell_path.hdr"
goto finish
:materials
"%EXE%" -scene MaterialsScene -integrator path -SPP 32 -threads 0 -denoise guided -aovPrefix "%~dp0Outputs\materials_aov" -outputFilename "%~dp0Outputs\materials_path.hdr"
goto finish
:light
"%EXE%" -scene cornell-box -integrator light -SPP 4 -outputFilename "%~dp0Outputs\cornell_light.hdr"
goto finish
:ir
"%EXE%" -scene cornell-box -integrator ir -SPP 4 -threads 0 -vplPaths 64 -outputFilename "%~dp0Outputs\cornell_ir.hdr"
:finish
set "RESULT=%ERRORLEVEL%"
echo.
echo Renderer exit code: %RESULT%
echo Output folder: %~dp0Outputs
if not "%~2"=="--no-pause" pause
exit /b %RESULT%

