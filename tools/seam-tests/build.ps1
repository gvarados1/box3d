# Builds the headless seam/contact test programs against an existing box3d build.
#
#   .\build.ps1                       # configure + build box3d, then the tests, then run them
#   .\build.ps1 -NoRun                # build only
#   .\build.ps1 -RulesOff             # build a comparison engine with the seam rules disabled
#
# -RulesOff compiles box3d with B3_GRAZING_FACE_ALIGNMENT=-1 and B3_CONVEX_REST_OFFSET=0, which
# is the behaviour before the conveyor seam fix. Build both and diff the output to see what the
# rules actually change.
#
# Build output goes to a directory outside the repo by default so nothing here gets dirtied.

param(
    [string]$BuildRoot = "$env:TEMP\box3d-seam-tests",
    [switch]$RulesOff,
    [switch]$GhostCullOff,
    [switch]$NoRun
)

$ErrorActionPreference = 'Stop'

$repo = Resolve-Path "$PSScriptRoot\..\.."
$flavor = if ($RulesOff) { 'rules-off' } elseif ($GhostCullOff) { 'ghost-cull-off' } else { 'rules-on' }
$engineDir = Join-Path $BuildRoot "engine-$flavor"
$outDir = Join-Path $BuildRoot $flavor

# Find a Visual Studio developer environment. cl and cmake are not on PATH by default.
$vcCandidates = @(
    "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
)
$vc = $vcCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $vc) { throw "No vcvars64.bat found. Edit the candidate list in build.ps1." }

function Invoke-Dev([string]$cmdline) {
    cmd /c "`"$vc`" >nul 2>nul && $cmdline"
    if ($LASTEXITCODE -ne 0) { throw "command failed: $cmdline" }
}

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

Write-Host "== configuring box3d ($flavor) ==" -ForegroundColor Cyan
$cflags = if ($RulesOff) { ' "-DCMAKE_C_FLAGS=/DB3_GRAZING_FACE_ALIGNMENT=-1.0f /DB3_CONVEX_REST_OFFSET=0.0f"' } elseif ($GhostCullOff) { ' "-DCMAKE_C_FLAGS=/DB3_CULL_CONVEX_EDGE_GHOSTS=0"' } else { '' }
Invoke-Dev "cmake -S `"$repo`" -B `"$engineDir`" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SHARED_LIBS=ON -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=OFF$cflags"

Write-Host "== building box3d ==" -ForegroundColor Cyan
Invoke-Dev "cmake --build `"$engineDir`""

Copy-Item "$engineDir\bin\box3d.dll" $outDir -Force

$programs = @('seam_ghost', 'regress', 'regress_mesh', 'lifecycle', 'rest_offset_latch', 'toi_probe', 'tunneling', 'tunnel_trace', 'container', 'ledge_launch')
Write-Host "== building tests ==" -ForegroundColor Cyan
Push-Location $outDir
try {
    foreach ($p in $programs) {
        Invoke-Dev "cl /nologo /O2 /I`"$repo\include`" `"$PSScriptRoot\$p.c`" /Fe:$p.exe /link `"$engineDir\src\box3d.lib`""
    }
} finally {
    Pop-Location
}

Write-Host "built to $outDir" -ForegroundColor Green

if (-not $NoRun) {
    foreach ($p in $programs) {
        Write-Host "`n========== $p ==========" -ForegroundColor Cyan
        & "$outDir\$p.exe"
    }
}
