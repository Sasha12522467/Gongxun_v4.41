param([string]$GccPath = 'C:\mingw64\bin\gcc.exe')

$ErrorActionPreference = 'Stop'
# Compatibility entry: the time-based policy was replaced by color/coordinate gating.
# Historical raw_material_grab_delay_test.c is not compiled anymore.
& (Join-Path $PSScriptRoot 'run_raw_material_grab_window_test.ps1') -GccPath $GccPath
