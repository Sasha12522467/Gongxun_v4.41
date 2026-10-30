param([string]$GccPath = 'C:\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$visualHeadingRoot = Split-Path $PSScriptRoot -Parent
$visualHeadingAction = [IO.File]::ReadAllText((Join-Path $visualHeadingRoot 'bsp\scr\action.c'))
$visualHeadingHeader = [IO.File]::ReadAllText((Join-Path $visualHeadingRoot 'bsp\inc\action.h'))
$visualHeadingImu = [IO.File]::ReadAllText((Join-Path $visualHeadingRoot 'bsp\scr\hwt101.c'))
$visualHeadingHarness = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'visual_heading_test.c'))
function Get-VisualHeadingChunk([string]$source, [string]$pattern) {
    $visualHeadingFound = [regex]::Match($source, $pattern)
    if (!$visualHeadingFound.Success) { throw ('Missing actual source: ' + $pattern) }
    return $visualHeadingFound.Value
}
$visualHeadingBody = Get-VisualHeadingChunk $visualHeadingHeader '(?s)typedef enum\s*\{\s*VISUAL_POSITION_OK.*?VisualPosition_Result_t;'
$visualHeadingBody += "`n" + (Get-VisualHeadingChunk $visualHeadingHeader '(?m)^#define VISUAL_POSITION_IMU_TIMEOUT_MS[^\r\n]+')
$visualHeadingBody += "`n" + (Get-VisualHeadingChunk $visualHeadingHeader '(?m)^#define RAW_GRAB_POSITION_MAX_RPM[^\r\n]+')
$visualHeadingBody += "`n" + (([regex]::Matches($visualHeadingAction, '(?m)^#define VISUAL_POSITION_[^\r\n]+') | ForEach-Object { $_.Value }) -join "`n")
# Locate each current function independently; old compatibility functions may be removed.
foreach ($visualHeadingFunction in @('VisualPosition_WrapDeg', 'VisualPosition_MapToRawYaw',
    'VisualPosition_LimitSpeed', 'VisualPosition_RunWithMax', 'VisualPosition_Run',
    'VisualPosition_RunRawMaterial')) {
    $visualHeadingBody += "`n" + (Get-VisualHeadingChunk $visualHeadingAction ('(?ms)^(?:static )?(?:float|uint8_t|int|VisualPosition_Result_t) ' + $visualHeadingFunction + '\([^)]*\)\s*\{.*?^\}'))
}
foreach ($visualHeadingFunction in @('my_abs_1', 'Limit', 'imu_pid')) {
    $visualHeadingBody += "`n" + (Get-VisualHeadingChunk $visualHeadingImu ('(?ms)^(?:float|void) ' + $visualHeadingFunction + '\([^)]*\)\s*\{.*?^\}'))
}
$visualHeadingInput = $visualHeadingHarness.Replace('/* ACTUAL_VISUAL_AND_PID */', $visualHeadingBody)
$visualHeadingExe = Join-Path $env:TEMP ('visual_heading_' + [guid]::NewGuid().ToString('N') + '.exe')
Push-Location $visualHeadingRoot
try {
    $visualHeadingInput | & $GccPath -std=c99 -Wall -Wextra -Werror -Ibsp/inc -x c - bsp/scr/chassis_nav.c -lm -o $visualHeadingExe
    if ($LASTEXITCODE -ne 0) { throw 'Visual heading test compilation failed' }
    & $visualHeadingExe
    if ($LASTEXITCODE -ne 0) { throw 'Visual heading test failed' }
} finally {
    Pop-Location
}
