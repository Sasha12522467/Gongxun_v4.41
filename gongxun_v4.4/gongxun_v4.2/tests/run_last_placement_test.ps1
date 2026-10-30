param([string]$GccPath='C:\mingw64\bin\gcc.exe')
$ErrorActionPreference='Stop'
$placementRoot=Split-Path $PSScriptRoot -Parent
$placementSource=[IO.File]::ReadAllText((Join-Path $placementRoot 'bsp\scr\action.c'))
$placementRobot=[IO.File]::ReadAllText((Join-Path $placementRoot 'bsp\scr\robot_task.c'))
$placementFlag=[regex]::Match($placementRobot,'(?m)^uint8_t material_last[^;]*;').Value
if(-not $placementFlag) { throw 'Missing shared flag definition' }
$placementInput="#include <stdint.h>`n"+$placementFlag+"`n"+[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'last_placement_test.c'))+"`n"
foreach($name in @('PutMaterialFromSlot','PutStorageMaterialFromSlot','PutMaterial2FromSlot','GrabMaterialToSlot','Processing_First','Processing_Second','Storage_First','Storage_Second')) {
    $function=[regex]::Match($placementSource,('(?ms)^void '+$name+'\([^)]*\)\s*\{.*?^\}')).Value
    if(-not $function) { throw ('Missing production function: '+$name) }
    $placementInput+=$function+"`n"
}
$placementExe=Join-Path $env:TEMP ('last_placement_'+[guid]::NewGuid().ToString('N')+'.exe')
$placementInput | & $GccPath -std=c99 -Wall -Wextra -Werror -x c - -o $placementExe
if($LASTEXITCODE -ne 0) { throw 'Placement host compilation failed' }
& $placementExe
if($LASTEXITCODE -ne 0) { throw 'Placement host simulation failed' }
Write-Output 'Simulation only; no motors driven.'
$storageEntry=[regex]::Match($placementSource,'(?ms)^void Storage_First\([^)]*\)\s*\{.*?^\}').Value
if([regex]::Matches($storageEntry,'\bPutStorageMaterialFromSlot\(').Count -ne 3 -or
   $storageEntry -match '\bPutMaterialFromSlot\(') { throw 'First-layer storage must use only its independent placement function' }
foreach($name in @('Processing_First','Processing_Second')) {
    $entry=[regex]::Match($placementSource,('(?ms)^void '+$name+'\([^)]*\)\s*\{.*?^\}')).Value
    if([regex]::Matches($entry,'\bPutMaterialFromSlot\(').Count -ne 3 -or $entry -match '\bPutStorageMaterialFromSlot\(') { throw ('Processing routing changed: '+$name) }
}
Write-Output 'PASS: storage first-layer and both processing rounds call separate placement implementations.'

# Exercise the actual motor-profile implementations against a protocol recorder.
$profileSource=[IO.File]::ReadAllText((Join-Path $placementRoot 'bsp\scr\actionInit.c'))
$profileInput=@'
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
static unsigned calls,expected_id,expected_direction,expected_speed,expected_pulse;
void Emm_V5_Pos_Control(uint8_t id,uint8_t direction,uint16_t speed,uint8_t acceleration,uint32_t pulse,bool absolute,bool sync) {
    assert(id==expected_id && direction==expected_direction);
    assert(speed==expected_speed && acceleration==240U && pulse==expected_pulse);
    assert(absolute && !sync); ++calls;
}
'@
foreach($name in @('ZDT_set','ZDT_set2')) {
    $function=[regex]::Match($profileSource,('(?ms)^void '+$name+'\([^)]*\)\s*\{.*?^\}')).Value
    if(-not $function) { throw ('Missing production motor profile: '+$name) }
    $profileInput+="`n"+$function+"`n"
}
$profileInput+=@'
int main(void) {
    static const uint16_t pulses[]={0U,1300U,8800U};
    for(unsigned fast=0;fast<2;++fast) for(unsigned i=0;i<3;++i)
    for(unsigned motor=0;motor<2;++motor) {
        expected_id=motor?6U:1U; expected_direction=motor?0U:1U;
        expected_speed=fast?550U:230U; expected_pulse=pulses[i]; calls=0U;
        if(fast) ZDT_set2((uint8_t)expected_id,pulses[i]);
        else ZDT_set((uint8_t)expected_id,pulses[i]);
        assert(calls==1U);
    }
    calls=0U; ZDT_set2(2U,100U); assert(calls==0U);
    puts("PASS: actual original ZDT_set=230 and separate ZDT_set2=550 for motors1/6; acceleration240, directions, pulse, absolute/sync flags preserved.");
    return 0;
}
'@
$profileExe=Join-Path $env:TEMP ('processing_profile_'+[guid]::NewGuid().ToString('N')+'.exe')
$profileInput | & $GccPath -std=c99 -Wall -Wextra -Werror -x c - -o $profileExe
if($LASTEXITCODE -ne 0) { throw 'Motor profile host compilation failed' }
& $profileExe
if($LASTEXITCODE -ne 0) { throw 'Motor profile simulation failed' }
