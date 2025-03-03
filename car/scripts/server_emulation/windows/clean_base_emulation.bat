@echo off
for /f "delims=" %%i in ('docker ps -q --filter "ancestor=bjr_base_ros"') do (
    docker rm --force %%i
)
