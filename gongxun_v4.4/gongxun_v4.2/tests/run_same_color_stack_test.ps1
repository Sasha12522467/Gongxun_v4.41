param([string]$GccPath = 'C:\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$stackTestRoot = Split-Path $PSScriptRoot -Parent
$stackAction = [IO.File]::ReadAllText((Join-Path $stackTestRoot 'bsp\scr\action.c'))
$stackHeader = [IO.File]::ReadAllText((Join-Path $stackTestRoot 'bsp\inc\action.h'))
$stackMain = [IO.File]::ReadAllText((Join-Path $stackTestRoot 'Core\Src\freertos.c'))
$stackCamera = [IO.File]::ReadAllText((Join-Path $stackTestRoot 'bsp\scr\maixcam.c'))
$stackPlan = [IO.File]::ReadAllText((Join-Path $stackTestRoot 'bsp\inc\path_plan.h'))

function Get-StackTestChunk([string]$source, [string]$pattern) {
    $found = [regex]::Match($source, $pattern)
    if (-not $found.Success) { throw ('Cannot locate actual production chunk: ' + $pattern) }
    return $found.Value
}

# Compile unchanged production functions in memory with HAL/RTOS/actuator mocks.
# Never rewrite production sources or substitute copied algorithms.
$stackChunks = @(
    '#include <assert.h>', '#include <stdbool.h>', '#include <stdint.h>',
    '#include <stdio.h>', '#include <string.h>', '#include <math.h>',
    (Get-StackTestChunk $stackMain '(?m)^#define STAGE_DOCK_TEST_HOLD_MS[^\r\n]+'),
    (Get-StackTestChunk $stackPlan '(?m)^#define PP_MAX_STAGES[^\r\n]+'),
    (Get-StackTestChunk $stackHeader '(?s)typedef enum\s*\{\s*VISUAL_POSITION_OK.*?VisualPosition_Result_t;'),
    (Get-StackTestChunk $stackHeader '(?s)typedef enum\s*\{\s*STORAGE_STACK_FAULT_NONE.*?(?=void positioning_angle)'),
    (Get-StackTestChunk $stackMain '(?ms)^static uint8_t s_stage_action_order.*?^\};'),
    [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'same_color_stack_test.c')),
    (Get-StackTestChunk $stackAction '(?ms)^uint8_t ColorToCameraMode\([^)]*\).*?^\}'),
    (Get-StackTestChunk $stackAction '(?ms)^StorageStack_State_t g_storage_stack;.*?(?=^//限幅函数)'),
    (Get-StackTestChunk $stackAction '(?ms)^#define VISUAL_POSITION_FRAME_TIMEOUT_MS.*?(?=^uint8_t FindThirdMaterial)'),
    (Get-StackTestChunk $stackCamera '(?s)uint8_t mode1\[3\].*?;'),
    (Get-StackTestChunk $stackCamera '(?ms)^void Maixcam_setmode\([^)]*\).*?^\}'),
    (Get-StackTestChunk $stackMain '(?ms)^static uint8_t MainTask_RunStageAction\([^)]*\)\s*\{.*?^\}')
)
$stackTestExe = Join-Path $env:TEMP 'gongxun_same_color_stack_test.exe'
($stackChunks -join "`n") | & $GccPath -std=c99 -Wall -Wextra -Werror -x c - -lm -o $stackTestExe
if ($LASTEXITCODE -ne 0) { throw 'Same-color stack test compilation failed' }
& $stackTestExe
if ($LASTEXITCODE -ne 0) { throw 'Same-color stack test failed' }
