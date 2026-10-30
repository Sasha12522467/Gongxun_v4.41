param(
    [string]$GccPath = 'C:\mingw64\bin\gcc.exe'
)
$ErrorActionPreference = 'Stop'
$rawRoot = Split-Path $PSScriptRoot -Parent
$rawAction = [IO.File]::ReadAllText((Join-Path $rawRoot 'bsp\scr\action.c'))
$rawHeader = [IO.File]::ReadAllText((Join-Path $rawRoot 'bsp\inc\action.h'))
function Get-RawChunk([string]$source, [string]$pattern) {
    $found = [regex]::Match($source, $pattern)
    if (!$found.Success) { throw ('Missing production chunk: ' + $pattern) }
    return $found.Value
}
function Get-RawFunction([string]$source, [string]$name) {
    Get-RawChunk $source ('(?ms)^(?:static )?(?:uint8_t|void|float|int|VisualPosition_Result_t) ' + $name + '\([^)]*\)\s*\{.*?^\}')
}
$rawTemp = Join-Path $env:TEMP ('raw_occlusion_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $rawTemp | Out-Null
$rawDefines = Get-RawChunk $rawHeader '(?ms)^#define RAW_GRAB_LEFT_X_PX.*?^extern volatile uint16_t raw_grab_test_layout;'
$rawEnum = Get-RawChunk $rawHeader '(?s)typedef enum\s*\{\s*VISUAL_POSITION_OK.*?VisualPosition_Result_t;'
$rawFunctions = Get-RawFunction $rawAction 'ColorToCameraMode'
$rawFunctions += "`nvolatile uint16_t raw_grab_test_layout;`n"
foreach ($name in @('RawGrab_WaitColorAt', 'RawGrab_Run')) {
    $rawFunctions += (Get-RawFunction $rawAction $name) + "`n"
}
$rawHarness = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'raw_material_single_test.c'))
$rawInput = "#include <stdint.h>`n" + $rawDefines + "`n" + $rawEnum + "`n" +
    $rawHarness.Replace('/* INSERT_ACTUAL_PRODUCTION_FUNCTIONS */', $rawFunctions)
foreach ($step in @(1,2)) {
    $rawExe = Join-Path $rawTemp ('logic_' + $step + '.exe')
    $rawInput | & $GccPath -std=c99 -Wall -Wextra -Werror "-DRAW_GRAB_ROTATION_STEP=$step" -x c - -o $rawExe
    if ($LASTEXITCODE -ne 0) { throw 'Raw logic compilation failed' }
    & $rawExe
    if ($LASTEXITCODE -ne 0) { throw 'Raw logic test failed' }
}

# Verify the production dispatch calls the existing user grab_* exactly once.
    $rawMechanical = @'
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
static unsigned calls, actual_position, actual_slot;
void grab_left(int slot) { ++calls; actual_position=0; actual_slot=slot; }
void grab_middle(int slot) { ++calls; actual_position=1; actual_slot=slot; }
void grab_right(int slot) { ++calls; actual_position=2; actual_slot=slot; }
'@
    $rawMechanical += "`n" + $rawDefines + "`n"
    $rawMechanical += (Get-RawFunction $rawAction 'RawGrab_GrabPosition') + "`n"
    $rawMechanical += @'
int main(void) {
    unsigned position,slot;
    for(position=0;position<3;++position) for(slot=1;slot<=3;++slot) {
        calls=0; RawGrab_GrabPosition((RawGrab_Position_t)position,slot);
        assert(calls==1 && actual_position==position && actual_slot==slot);
    }
    calls=0; RawGrab_GrabPosition((RawGrab_Position_t)99,1);
    assert(calls==0);
    puts("PASS: original grab_left/middle/right dispatch, 3 positions x 3 slots.");
    return 0;
}
'@
    $rawExe = Join-Path $rawTemp 'mechanical.exe'
    $rawMechanical | & $GccPath -std=c99 -Wall -Wextra -Werror -x c - -o $rawExe
    if ($LASTEXITCODE -ne 0) { throw 'Mechanical compilation failed' }
    & $rawExe
    if ($LASTEXITCODE -ne 0) { throw 'Mechanical comparison failed' }
# Compare the actual actuator command sequence with/without the final-grab flag.
# Only the original four-command return-to-middle tail may disappear.
$rawRobot = [IO.File]::ReadAllText((Join-Path $rawRoot 'bsp\scr\robot_task.c'))
$rawTailTest = @'
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef struct { int kind,a,b; } Event;
static Event events[128], baseline[128];
static unsigned count,total_delay;
static void Record(int kind,int a,int b) {
    assert(count<128U);
    events[count].kind=kind; events[count].a=a; events[count].b=b; ++count;
}
void ZDT_set(int id,int pulse) { Record(1,id,pulse); }
void osDelay(unsigned ms) { Record(2,(int)ms,0); total_delay+=ms; }
void ser_set(int angle,int time) { Record(3,angle,time); }
void Plate(int slot) { Record(4,slot,0); }
void servo_set(int state) { Record(5,state,0); }
'@
$rawTailTest += "`n" + (Get-RawChunk $rawRobot '(?m)^uint8_t material_last[^;]*;') + "`n"
foreach ($name in @('grab_left','grab_middle','grab_right')) {
    $rawTailTest += (Get-RawFunction $rawRobot $name) + "`n"
}
$rawTailTest += @'
int main(void) {
    void (*grab[3])(int)={grab_left,grab_middle,grab_right};
    unsigned p,slot,i,n,normal_delay;
    assert(material_last==0U);
    for(p=0;p<3;++p) for(slot=1;slot<=3;++slot) {
        count=total_delay=0U; material_last=0U; grab[p]((int)slot);
        n=count; normal_delay=total_delay; assert(n>=4U);
        for(i=0;i<n;++i) baseline[i]=events[i];
        assert(baseline[n-4].kind==1 && baseline[n-4].a==6 && baseline[n-4].b==5500);
        assert(baseline[n-3].kind==2 && baseline[n-3].a==300);
        assert(baseline[n-2].kind==3 && baseline[n-2].a==-69 && baseline[n-2].b==800);
        assert(baseline[n-1].kind==2 && baseline[n-1].a==810);
        count=total_delay=0U; material_last=1U; grab[p]((int)slot);
        assert(count==n-4U && total_delay+1110U==normal_delay);
        for(i=0;i<count;++i) {
            assert(events[i].kind==baseline[i].kind && events[i].a==baseline[i].a && events[i].b==baseline[i].b);
        }
    }
    puts("PASS: actual left/middle/right grabs x3 slots; final flag skips only return-to-middle tail and 1110ms; all earlier lift/claw/storage commands unchanged.");
    return 0;
}
'@
$rawExe=Join-Path $rawTemp 'last_grab_tail.exe'
$rawTailTest | & $GccPath -std=c99 -Wall -Wextra -Werror -x c - -o $rawExe
if ($LASTEXITCODE -ne 0) { throw 'Final-grab actuator compilation failed' }
& $rawExe
if ($LASTEXITCODE -ne 0) { throw 'Final-grab actuator test failed' }
Write-Output 'Simulation only; no firmware flashed and no motors driven.'
