@echo off
rem Publishes the ROADS website staging folder to the JediArchive web server.
rem Verify after publishing: https://pepperell.net/games/ROADS

set "SRC=%~dp0web"
set "DEST=\\JediArchive\web\pepperellnet\games\ROADS"

echo Publishing ROADS website
echo   from: %SRC%
echo   to:   %DEST%
echo.

rem /XF google_oauth.php: never publish OAuth credentials; the live copy lives
rem in the private data folder outside the publish destination.
robocopy "%SRC%" "%DEST%" /E /R:2 /W:5 /NP /XF google_oauth.php

rem Robocopy exit codes 0-7 indicate success; 8+ indicate failure.
if %ERRORLEVEL% GEQ 8 (
    echo.
    echo PUBLISH FAILED - robocopy exit code %ERRORLEVEL%
    exit /b 1
)

echo.
echo Publish complete. Verify at https://pepperell.net/games/ROADS
exit /b 0
