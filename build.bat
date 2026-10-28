@echo off
REM ===========================================================================
REM  wireless-esp8266-dap one-click build script
REM
REM  Usage:
REM    build.bat            build
REM    build.bat clean      remove build dir, then build
REM    build.bat fullclean  remove sdkconfig + build dir, then build
REM
REM  Why this script exists: running idf.py directly inside Git Bash hits five
REM  traps (see 03_firmware/固件烧录指南.md). This script avoids all five:
REM    1. MSYSTEM makes IDF's export.bat refuse to run -> we clear MSYSTEM
REM    2. must use the IDF venv python, otherwise pkg_resources is missing
REM    3. IDF_PATH must use forward slashes, backslashes break cmake foreach
REM    4. must call the IDF-owned idf.py by absolute path (the repo root has a
REM       same-named ESP8266 idf.py that would otherwise win)
REM    5. corsacOTA is a submodule; init it if missing
REM ===========================================================================

REM --- Environment -----------------------------------------------------------
REM Change these two lines if ESP-IDF lives somewhere else on your machine.
set IDF_TOOLS_PATH=D:\Espressif
set IDF_PATH=D:/Espressif/frameworks/esp-idf-v4.4.2
set IDF_PYTHON_ENV=D:\Espressif\python_env\idf4.4_py3.8_env\Scripts
set IDF_PYTHON=%IDF_PYTHON_ENV%\python.exe
set IDF_PY=%IDF_PATH%\tools\idf.py

REM --- Trap 1: clear MSYSTEM, otherwise IDF's export.bat refuses to run ------
set MSYSTEM=

REM --- Toolchain PATH ---------------------------------------------------------
set PATH=%IDF_PYTHON_ENV%;D:\Espressif\tools\idf-git\2.34.2\cmd;D:\Espressif\tools\xtensa-esp32s3-elf\esp-2021r2-patch3-8.4.0\xtensa-esp32s3-elf\bin;D:\Espressif\tools\cmake\3.23.1\bin;D:\Espressif\tools\ninja\1.10.2;%PATH%

cd /d "%~dp0"

REM --- Trap 5: make sure the submodule is present -----------------------------
if not exist "components\corsacOTA\src\corsacOTA.c" (
    echo [build.bat] corsacOTA submodule missing, initializing...
    git submodule update --init components/corsacOTA || goto :fail
)

REM --- Preflight checks -------------------------------------------------------
if not exist "%IDF_PY%" (
    echo [build.bat] ERROR: idf.py not found
    echo [build.bat]        expected at: %IDF_PY%
    echo [build.bat]        edit IDF_PATH at the top of this script
    goto :fail
)
if not exist "%IDF_PYTHON%" (
    echo [build.bat] ERROR: IDF venv python not found
    echo [build.bat]        expected at: %IDF_PYTHON%
    echo [build.bat]        edit IDF_PYTHON_ENV at the top of this script
    goto :fail
)

REM --- Credential file change detection --------------------------------------
REM wifi_credentials.h is optional and pulled in via __has_include, so the
REM compiler never records "file absent" as a dependency. Adding or removing it
REM would otherwise silently reuse the previous object files. Touch the sources
REM that embed the SSID list to force a rebuild when its presence changes.
set CRED_MARK=.cred_state
set CRED_NOW=absent
if exist "main\wifi_credentials.h" set CRED_NOW=present
set CRED_PREV=absent
if exist "%CRED_MARK%" set /p CRED_PREV=<"%CRED_MARK%"
if not "%CRED_NOW%"=="%CRED_PREV%" (
    echo [build.bat] wifi_credentials.h changed ^(%CRED_PREV% -^> %CRED_NOW%^), forcing rebuild
    if exist "main\main.c" copy /b "main\main.c"+,, "main\main.c" >nul
    if exist "main\wifi_handle.c" copy /b "main\wifi_handle.c"+,, "main\wifi_handle.c" >nul
    >"%CRED_MARK%" echo %CRED_NOW%
)

REM --- Build ------------------------------------------------------------------
echo [build.bat] IDF_PATH = %IDF_PATH%
echo [build.bat] Python   = %IDF_PYTHON%
echo [build.bat] CWD      = %CD%
echo.

if /i "%1"=="fullclean" (
    echo [build.bat] fullclean: backing up sdkconfig, removing build dir
    REM Do NOT delete sdkconfig: it carries CONFIG_IDF_TARGET, and losing it
    REM silently retargets the build to the default esp32 (wrong toolchain).
    REM Keep a backup so menuconfig changes can be re-applied if needed.
    if exist sdkconfig (
        copy /y sdkconfig sdkconfig.backup >nul
        echo [build.bat] sdkconfig backed up to sdkconfig.backup
    )
    if exist build rmdir /s /q build
) else if /i "%1"=="clean" (
    echo [build.bat] clean: removing build dir
    if exist build rmdir /s /q build
)

"%IDF_PYTHON%" "%IDF_PY%" build || goto :fail

echo.
echo [build.bat] BUILD OK. Artifacts:
echo [build.bat]   app:        build\wireless_esp_dap.bin
echo [build.bat]   bootloader: build\bootloader\bootloader.bin
echo [build.bat]   partition:  build\partition_table\partition-table.bin
echo.
echo [build.bat] To flash (hold BOOT, tap RST to enter download mode; after
echo [build.bat] flashing release BOOT and tap RST again):
echo [build.bat]   "%IDF_PYTHON%" "%IDF_PATH%\components\esptool_py\esptool\esptool.py" --chip esp32s3 --port COM18 --baud 921600 write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\wireless_esp_dap.bin
exit /b 0

:fail
echo.
echo [build.bat] BUILD FAILED, exit code %ERRORLEVEL%
exit /b 1
