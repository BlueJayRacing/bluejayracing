@echo off
setlocal enabledelayedexpansion
set "SCRIPT_DIR=%~dp0"

REM Check if the image exists; if not, build it.
docker images --format "{{.Repository}}" | findstr /I "bjr_base_ros" >nul
if errorlevel 1 (
    call "%SCRIPT_DIR%\build_base_emulation.bat"
)

REM Attempt to retrieve the container ID from any running container created from the image.
set "CONTAINER_ID="
for /f "delims=" %%i in ('docker ps --filter "ancestor=bjr_base_ros" --format "{{.ID}}"') do (
    set "CONTAINER_ID=%%i"
)

REM If no container is running, start a new one and capture its container ID.
if not defined CONTAINER_ID (
    for /f "delims=" %%i in ('docker run -d ^
        -v "%SCRIPT_DIR%\..\..\..\src:/bjr_ws/src/bjr_packages" ^
        --env "DISPLAY=172.26.128.1:0" ^
        --env "QT_X11_NO_MITSHM=1" ^
        -v "%USERPROFILE%\.ssh:/home/dock/.ssh" ^
        --volume "%USERPROFILE%\.Xauthority:/tmp/.X11-unix:rw" ^
        --network=host ^
        -p 0.0.0.0:9365:9365	 ^
        -i ^
        bjr_base_ros') do (
            set "CONTAINER_ID=%%i"
    )
)

REM Open an interactive bash session using the container's ID.
if defined CONTAINER_ID (
    docker exec -it !CONTAINER_ID! bash
) else (
    echo No container is running.
)
endlocal
