param([string]$GccPath = 'C:\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$mappingRoot = Split-Path $PSScriptRoot -Parent
$mappingRuntime = [IO.File]::ReadAllText((Join-Path $mappingRoot 'bsp\scr\chassis_runtime.c'))
$mappingLegacy = [IO.File]::ReadAllText((Join-Path $mappingRoot 'bsp\scr\chassic.c'))
$mappingTest = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'chassis_mapping_test.c'))
$mappingParts = $mappingTest -split '/\* ACTUAL_PRODUCTION \*/', 2
if ($mappingParts.Count -ne 2) { throw 'Missing test injection marker' }

function Get-MappingChunk([string]$source, [string]$pattern) {
    $mappingFound = [regex]::Match($source, $pattern)
    if (-not $mappingFound.Success) { throw ('Production chunk missing: ' + $pattern) }
    return $mappingFound.Value
}

# Compile unchanged production helpers with UART/actuator mocks, never operate hardware.
$mappingChunks = @($mappingParts[0])
$mappingChunks += [regex]::Matches($mappingRuntime, '(?m)^#define CHASSIS_[^\r\n]+') | ForEach-Object { $_.Value }
$mappingChunks += Get-MappingChunk $mappingRuntime '(?m)^static const uint8_t s_motor_address[^\r\n]+'
foreach ($mappingName in @('Clamp', 'ReadAllPositions', 'SendOneRpm', 'SendWheelRpm', 'SendCoarseTurn')) {
    $mappingChunks += Get-MappingChunk $mappingRuntime ('(?ms)^static (?:float|uint8_t|void) ChassisRuntime_' + $mappingName + '\([^)]*\)\s*\{.*?^\}')
}
foreach ($mappingName in @('MotorSetVel', 'MotorSetPulse', 'my_abs', 'motor_Omnidirectional_set')) {
    $mappingChunks += Get-MappingChunk $mappingLegacy ('(?ms)^(?:void|int) ' + $mappingName + '\([^)]*\)\s*\{.*?^\}')
}
$mappingChunks += $mappingParts[1]
$mappingTemp = Join-Path $env:TEMP ('gongxun_mapping_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $mappingTemp | Out-Null
Push-Location $mappingRoot
try {
    foreach ($mappingMode in @(0, 1)) {
        $mappingExe = Join-Path $mappingTemp ('mapping_' + $mappingMode + '.exe')
        ($mappingChunks -join "`n") | & $GccPath -std=c99 -Wall -Wextra -Werror "-DCHASSIS_DIAGONAL_SWAP=$mappingMode" -Ibsp/inc -x c - bsp/scr/chassis_nav.c -lm -o $mappingExe
        if ($LASTEXITCODE -ne 0) { throw "Mapping $mappingMode compilation failed" }
        & $mappingExe
        if ($LASTEXITCODE -ne 0) { throw "Mapping $mappingMode test failed" }
    }
} finally {
    Pop-Location
}
