@echo off
REM Get the current script directory
set "SCRIPT_DIR=%~dp0"

REM Build the docker image (using buildx for arm64, adjust as needed)
docker buildx build -t bjr_base_ros -f "%SCRIPT_DIR%\..\..\..\docker\Dockerfile_base_ros" "%SCRIPT_DIR%\..\..\.."

REM If a container named "bjr_base_ros" is running, clean it up.
for /f "delims=" %%i in ('docker ps --filter "name=bjr_base_ros" --format "{{.Names}}"') do (
    call "%SCRIPT_DIR%\clean_docker_car.bat"
)
