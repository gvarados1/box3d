# A/B the contact/CCD changes against a MineMogul-shaped workload.
#
#   .\perf-ab.ps1                    # all four variants, alternating
#   .\perf-ab.ps1 -Reps 5 -Scene belt
#   .\perf-ab.ps1 -Workers 4 -NoBuild
#
# Method: build ONE perf.exe, then swap box3d.dll between runs. The changes add no exports, so the
# same binary measures every variant with identical benchmark code - the benchmark itself cannot be
# a source of difference. Runs alternate across variants so thermal drift shows up as spread rather
# than as a fake win for whichever variant ran first.
#
# Variants, so each change is attributed rather than guessed:
#   base      both changes reverted to HEAD
#   contact   only the isFast gate on the grazing drop (src/contact.c)
#   shape     only the continuous-collision fallback (src/shape.c)
#   fixed     both
#
# TWO THINGS THIS SCRIPT IS CAREFUL ABOUT, BOTH LEARNED THE HARD WAY:
#
# 1. Copy-Item PRESERVES the source timestamp. Restoring a file that way leaves it OLDER than the
#    object built from the other version, so ninja decides it is up to date and silently builds
#    nothing - and you end up benchmarking two identical DLLs while believing they differ. Every
#    source swap here is followed by an explicit LastWriteTime stamp, and each variant gets its own
#    build tree so no stale object can leak across variants.
#
# 2. A hash is not proof. A PE file embeds a link timestamp, so two builds of identical source hash
#    differently, and a build that did nothing hashes the SAME as the previous variant. So each
#    variant is fingerprinted by BEHAVIOUR instead: tunneling.exe returns a distinct tunnel count
#    per variant, and the script aborts if a variant does not report its expected number.
#
# The working tree is restored in a finally block. Nothing is committed and nothing is stashed.

param(
    [string]$BuildRoot = "$env:TEMP\box3d-perf",
    [int]$Reps = 3,
    [string]$Scene = 'all',
    [int]$Workers = 1,
    [switch]$NoBuild,
    [switch]$SkipFingerprint
)

$ErrorActionPreference = 'Stop'

$repo = Resolve-Path "$PSScriptRoot\..\.."
$changed = @('src/shape.c', 'src/contact.c')

# Expected tunneled-drop count from tools\seam-tests\tunneling.c. These four numbers are distinct,
# so they identify which sources actually went into each DLL.
$fingerprint = @{ base = 597; contact = 20; shape = 78; fixed = 0 }
$variants = @('base', 'contact', 'shape', 'fixed')

$vcCandidates = @(
    "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
)
$vc = $vcCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $vc) { throw "No vcvars64.bat found. Edit the candidate list in perf-ab.ps1." }

function Invoke-Dev([string]$cmdline) {
    cmd /c "`"$vc`" >nul 2>nul && $cmdline" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "command failed: $cmdline" }
}

# Restore a saved file and make it unambiguously newer than anything built from it.
function Restore-Source([string]$savePath, [string]$repoRelative) {
    $dest = Join-Path $repo $repoRelative
    Copy-Item $savePath $dest -Force
    (Get-Item $dest).LastWriteTime = Get-Date
}

function Revert-Source([string[]]$repoRelative) {
    & git -C $repo checkout -- $repoRelative
    if ($LASTEXITCODE -ne 0) { throw "git checkout failed" }
    foreach ($f in $repoRelative) { (Get-Item (Join-Path $repo $f)).LastWriteTime = Get-Date }
}

$outDir = Join-Path $BuildRoot 'run'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

function Build-Variant([string]$name) {
    # Its own tree, so a missed rebuild cannot inherit another variant's objects.
    $engineDir = Join-Path $BuildRoot "engine-$name"
    Write-Host "== building engine ($name) ==" -ForegroundColor Cyan
    Invoke-Dev "cmake -S `"$repo`" -B `"$engineDir`" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=OFF"
    Invoke-Dev "cmake --build `"$engineDir`""
    Copy-Item "$engineDir\bin\box3d.dll" (Join-Path $outDir "box3d-$name.dll") -Force
}

if (-not $NoBuild) {
    $save = Join-Path $BuildRoot 'save'
    New-Item -ItemType Directory -Force -Path $save | Out-Null
    foreach ($f in $changed) {
        Copy-Item (Join-Path $repo $f) (Join-Path $save (Split-Path $f -Leaf)) -Force
    }
    foreach ($f in $changed) {
        if (-not (Test-Path (Join-Path $save (Split-Path $f -Leaf)))) { throw "failed to save $f - refusing to modify the tree" }
    }

    try {
        Build-Variant 'fixed'

        Revert-Source $changed
        Build-Variant 'base'

        Restore-Source (Join-Path $save 'contact.c') 'src/contact.c'
        Build-Variant 'contact'

        Revert-Source @('src/contact.c')
        Restore-Source (Join-Path $save 'shape.c') 'src/shape.c'
        Build-Variant 'shape'
    }
    finally {
        Write-Host "== restoring the working tree ==" -ForegroundColor Yellow
        foreach ($f in $changed) {
            Restore-Source (Join-Path $save (Split-Path $f -Leaf)) $f
        }
        & git -C $repo status --short
    }

    # Both programs link against the import library, which is identical across variants.
    # /DNDEBUG because a Release box3d does not export b3InternalAssert.
    Write-Host "== building perf.exe and tunneling.exe ==" -ForegroundColor Cyan
    $lib = Join-Path $BuildRoot 'engine-fixed\src\box3d.lib'
    Push-Location $outDir
    try {
        Invoke-Dev "cl /nologo /O2 /DNDEBUG /I`"$repo\include`" `"$PSScriptRoot\perf.c`" /Fe:perf.exe /link `"$lib`""
        Invoke-Dev "cl /nologo /O2 /DNDEBUG /I`"$repo\include`" `"$PSScriptRoot\tunneling.c`" /Fe:tunneling.exe /link `"$lib`""
    }
    finally {
        Pop-Location
    }
}

function Use-Variant([string]$name) {
    Copy-Item (Join-Path $outDir "box3d-$name.dll") (Join-Path $outDir 'box3d.dll') -Force
}

# Prove each DLL is the build it claims to be, by behaviour, before timing anything.
if (-not $SkipFingerprint) {
    Write-Host "`n== fingerprinting each variant (tunneled drops out of 3528) ==" -ForegroundColor Cyan
    $bad = @()
    foreach ($v in $variants) {
        Use-Variant $v
        $out = & (Join-Path $outDir 'tunneling.exe') 2>&1
        $count = -1
        foreach ($line in $out) {
            if ($line -match '^\s*(\d+) of \d+ drops tunneled') { $count = [int]$Matches[1] }
        }
        $want = $fingerprint[$v]
        $ok = ($count -eq $want)
        "{0,-9} tunneled {1,4}   expected {2,4}   {3}" -f $v, $count, $want, $(if ($ok) { 'ok' } else { 'MISMATCH' }) | Write-Host
        if (-not $ok) { $bad += $v }
    }
    if ($bad.Count -gt 0) {
        throw "variant(s) $($bad -join ', ') do not match their expected behaviour - the builds are not what they claim, so any timing would be meaningless"
    }
}

$order = @()
for ($i = 0; $i -lt $Reps; $i++) { $order += $variants }

$results = @{}
foreach ($v in $variants) { $results[$v] = @() }

foreach ($variant in $order) {
    Use-Variant $variant
    Write-Host "---------- $variant ----------" -ForegroundColor Green
    $output = & (Join-Path $outDir 'perf.exe') $Scene 1 $Workers
    $output | Write-Host
    foreach ($line in $output) {
        if ($line -match 'RESULT\s+(\S+)\s+best-of-reps median\s+([0-9.]+)') {
            $results[$variant] += [pscustomobject]@{ Scene = $Matches[1]; Ms = [double]$Matches[2] }
        }
    }
}

Write-Host "`n================ A/B summary (best median per variant) ================" -ForegroundColor Cyan
$scenes = $results.fixed | ForEach-Object { $_.Scene } | Sort-Object -Unique
foreach ($s in $scenes) {
    $b = ($results.base | Where-Object { $_.Scene -eq $s } | ForEach-Object { $_.Ms } | Measure-Object -Minimum).Minimum
    Write-Host ""
    foreach ($v in $variants) {
        $m = ($results[$v] | Where-Object { $_.Scene -eq $s } | ForEach-Object { $_.Ms } | Measure-Object -Minimum).Minimum
        $delta = if ($b -gt 0) { 100.0 * ($m - $b) / $b } else { 0 }
        "{0,-8} {1,-9} {2,7:N4} ms   vs base {3,7:N2} %" -f $s, $v, $m, $delta | Write-Host
    }
}
