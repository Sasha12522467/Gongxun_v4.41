param([string]$GccPath = 'C:\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$rawWindowRoot = Split-Path $PSScriptRoot -Parent
$rawWindowSource = [IO.File]::ReadAllText((Join-Path $rawWindowRoot 'bsp\scr\action.c'))
$rawWindowHeader = [IO.File]::ReadAllText((Join-Path $rawWindowRoot 'bsp\inc\action.h'))
$rawWindowHarness = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'raw_material_grab_window_test.c'))
$rawWindowInterface = [regex]::Match($rawWindowHeader, '(?s)typedef enum\s*\{\s*RAW_MATERIAL_GRAB_WAITING.*?(?=void GrabMaterialsInOrder)').Value
$rawWindowBody = [regex]::Match($rawWindowSource, '(?ms)^RawMaterial_GrabState_t g_raw_grab;.*?(?=^void task_judge)').Value
if (-not $rawWindowInterface -or -not $rawWindowBody) { throw 'Cannot locate actual production grab-window code' }

$rawWindowMacroNames = @('RAW_MATERIAL_GRAB_TARGET_X_PX','RAW_MATERIAL_GRAB_TARGET_Y_PX',
    'RAW_MATERIAL_GRAB_TOLERANCE_PX','RAW_MATERIAL_GRAB_FRAME_TIMEOUT_MS','RAW_MATERIAL_GRAB_POLL_MS')
$rawWindowMacros = foreach ($rawWindowMacro in $rawWindowMacroNames) {
    $rawWindowLine = [regex]::Match($rawWindowHeader, ('(?m)^#define ' + $rawWindowMacro + '[^\r\n]+')).Value
    if (-not $rawWindowLine) { throw ('Missing production parameter ' + $rawWindowMacro) }
    $rawWindowLine
}
# Check current settings plus independent target coordinates/exact and loose windows.
$rawWindowProfiles = @($null, @{X=90;Y=80;Tolerance=0}, @{X=120;Y=130;Tolerance=10})
$rawWindowTestIndex = 0
foreach ($rawWindowProfile in $rawWindowProfiles) {
    $rawWindowDefines = $rawWindowMacros
    if ($null -ne $rawWindowProfile) {
        $rawWindowDefines = @(
            ('#define RAW_MATERIAL_GRAB_TARGET_X_PX ' + $rawWindowProfile.X + 'U'),
            ('#define RAW_MATERIAL_GRAB_TARGET_Y_PX ' + $rawWindowProfile.Y + 'U'),
            ('#define RAW_MATERIAL_GRAB_TOLERANCE_PX ' + $rawWindowProfile.Tolerance + 'U')
        ) + @($rawWindowMacros | Where-Object { $_ -notmatch '^#define RAW_MATERIAL_GRAB_(TARGET_X_PX|TARGET_Y_PX|TOLERANCE_PX)\s' })
    }
    $rawWindowInput = (@('#include <assert.h>','#include <stdint.h>','#include <stdio.h>','#include <string.h>') +
        $rawWindowDefines + @($rawWindowInterface,$rawWindowHarness,$rawWindowBody)) -join "`n"
    $rawWindowExe = Join-Path $env:TEMP ('gongxun_raw_grab_window_' + $rawWindowTestIndex + '.exe')
    $rawWindowInput | & $GccPath -std=c99 -Wall -Wextra -Werror -x c - -o $rawWindowExe
    if ($LASTEXITCODE -ne 0) { throw 'Grab-window test compilation failed' }
    & $rawWindowExe
    if ($LASTEXITCODE -ne 0) { throw 'Grab-window test failed' }
    $rawWindowTestIndex++
}
