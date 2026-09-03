@echo off
setlocal

rem Signe le binaire compile par STM32CubeIDE (Debug\DIAS.bin) et le programme
rem dans la flash externe QSPI de la STM32N6570-DK, a l'adresse 0x70000000.
rem Prerequis : le projet doit deja avoir ete compile (Build, PAS Debug/Run) --
rem le bouton Debug ne fait que charger le firmware en RAM (perdu au reboot).
rem Une fois ce script termine avec succes : bascule le switch BOOT de la
rem carte sur "Boot from flash" et fais un cycle d'alimentation.

set "CUBEPROG_BIN=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
set "PROJECT_DIR=%~dp0"
set "BIN_FILE=%PROJECT_DIR%Debug\DIAS.bin"
set "SIGNED_BIN=%PROJECT_DIR%Debug\DIAS_sign.bin"
set "LOADER=%CUBEPROG_BIN%\ExternalLoader\MX66UW1G45G_STM32N6570-DK.stldr"

if not exist "%BIN_FILE%" (
    echo [ERREUR] "%BIN_FILE%" introuvable.
    echo          Compile d'abord le projet dans STM32CubeIDE ^(bouton Build, pas Debug^).
    exit /b 1
)

rem Supprime la sortie d'un lancement precedent : sinon l'outil de signature
rem demande une confirmation interactive (y/n) qu'aucune console non
rem interactive (dont celle d'Eclipse/STM32CubeIDE) ne peut fournir --
rem provoque un blocage silencieux ("Launching delegate" fige indefiniment).
if exist "%SIGNED_BIN%" del /f /q "%SIGNED_BIN%"

rem -align = aligne le payload signe sur 0x400 (1 Ko) a l'interieur du fichier
rem signe. Sans ce flag (comportement par defaut), le payload commence juste
rem apres l'entete (offset 0x240), et le BootROM/FSBL du STM32N6 ne parvient
rem alors JAMAIS a localiser/copier le code : rien ne s'execute (silence
rem total en boot-from-flash, meme pas la toute premiere instruction de
rem Reset_Handler). Confirme par comparaison octet-a-octet avec le binaire
rem de reference ST (qui, lui, est bien aligne sur 0x400) : voir Binary/.
echo === Signature de %BIN_FILE% ===
"%CUBEPROG_BIN%\STM32_SigningTool_CLI.exe" -bin "%BIN_FILE%" -nk -t ssbl -hv 2.3 -align -o "%SIGNED_BIN%"
if errorlevel 1 (
    echo [ERREUR] Echec de la signature.
    exit /b 1
)

echo.
echo === Programmation de la flash externe ^(0x70000000^) ===
"%CUBEPROG_BIN%\STM32_Programmer_CLI.exe" -c port=SWD mode=HOTPLUG -el "%LOADER%" -hardRst -w "%SIGNED_BIN%" 0x70000000
if errorlevel 1 (
    echo [ERREUR] Echec de la programmation. La carte est-elle branchee et en mode developpement ?
    exit /b 1
)

echo.
echo === OK : bascule le switch BOOT sur "Boot from flash" puis redemarre la carte. ===
