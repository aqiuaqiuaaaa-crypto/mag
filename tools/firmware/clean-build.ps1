# Promoted from artifacts/firmware-clean-build-20261006/clean-build.ps1.
# Original-project Rebuild only. Never download, stage Git files, or publish staging HEX.
[CmdletBinding()]
param(
    [string]$ArtifactDirectory,
    [string]$UV4Path,
    [string]$FromElfPath,
    [string]$Python = 'python',
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'

function Assert-SafePath([string]$Path, [string]$Root) {
    $full = [IO.Path]::GetFullPath($Path)
    $base = [IO.Path]::GetFullPath($Root).TrimEnd('\')
    if (-not $full.StartsWith($base + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Path outside intended directory: $full"
    }
    $ancestor = $full
    while ($ancestor.Length -ge $base.Length) {
        if ((Test-Path -LiteralPath $ancestor) -and
            ((Get-Item -LiteralPath $ancestor -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw "Reparse point is not allowed: $ancestor"
        }
        $ancestor = Split-Path -Parent $ancestor
    }
    return $full
}

function Get-CanonicalProject([string]$Workspace, [string]$Project) {
    $expected = Join-Path $Workspace 'pwm_double20260926DC6output\MDK-ARM\pwm_02.uvprojx'
    $projectPath = Assert-SafePath $Project $Workspace
    if ($projectPath -ne [IO.Path]::GetFullPath($expected)) {
        throw 'Official build input must be the ORIGINAL canonical project, never staging'
    }
    [xml]$xml = Get-Content -LiteralPath $projectPath -Raw -Encoding UTF8
    $targets = @($xml.Project.Targets.Target)
    if ($targets.Count -ne 1 -or $targets[0].TargetName -ne 'pwm_02') { throw 'Unexpected target' }
    $common = $targets[0].TargetOption.TargetCommonOption
    if ($common.Device -ne 'STM32F407IGTx' -or $common.CreateHexFile -ne '1' -or
        $common.CreateExecutable -ne '1') { throw 'MCU/executable/HEX settings changed' }
    foreach ($hook in @('BeforeCompile', 'BeforeMake', 'AfterMake')) {
        if ($common.$hook.RunUserProg1 -ne '0' -or $common.$hook.RunUserProg2 -ne '0') {
            throw "Enabled custom build hook must be reviewed: $hook"
        }
    }
    $mdk = Split-Path -Parent $projectPath
    $output = Assert-SafePath (Join-Path $mdk ([string]$common.OutputDirectory)) $mdk
    if ($output.TrimEnd('\') -ne (Join-Path $mdk 'pwm_02') -or $common.OutputName -ne 'pwm_02') {
        throw 'Canonical output configuration changed'
    }
    $ioc = Get-Content -LiteralPath (Join-Path (Split-Path -Parent $mdk) 'pwm_02.ioc') -Raw
    if ($ioc -notmatch '(?m)^Mcu.CPN=STM32F407IGT6\r?$') { throw 'Unexpected MCU CPN' }
    return [ordered]@{
        project = $projectPath; project_sha256 = (Get-FileHash -LiteralPath $projectPath -Algorithm SHA256).Hash.ToLowerInvariant()
        target = 'pwm_02'; mcu = 'STM32F407IGT6'; device = [string]$common.Device
        output = $output; mdk = $mdk
        images = @('hex', 'axf', 'map') | ForEach-Object { Join-Path $output "pwm_02.$_" }
    }
}

function Get-GitText([string]$Workspace, [string[]]$Arguments) {
    $ErrorActionPreference = 'Continue' # Native stderr must not hide a successful Git exit.
    $text = & git -C $Workspace -c core.quotepath=false @Arguments 2>$null
    if ($LASTEXITCODE -ne 0) { throw "Git command failed: $Arguments" }
    return ($text -join "`n")
}

function Get-FileMetadata([string]$Path) {
    $item = Get-Item -LiteralPath $Path
    return [ordered]@{ path = $item.FullName; size = $item.Length
        mtime_utc = $item.LastWriteTimeUtc.ToString('o')
        sha256 = (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
}

function Assert-FirmwareClean([string]$TrackedStatus) {
    if ($TrackedStatus) { throw 'Tracked firmware is dirty; refusing official release (no automatic commit/discard)' }
}

function Get-CleanupFiles($Info, [string]$Workspace, [string[]]$Tracked) {
    $candidates = @()
    foreach ($directory in @($Info.output, (Join-Path $Info.mdk 'Objects'), (Join-Path $Info.mdk 'Listings'))) {
        Assert-SafePath $directory $Info.mdk | Out-Null
        if (Test-Path -LiteralPath $directory) {
            foreach ($item in @(Get-ChildItem -LiteralPath $directory -Recurse -Force)) {
                Assert-SafePath $item.FullName $Info.mdk | Out-Null
                if (-not $item.PSIsContainer) { $candidates += $item.FullName }
            }
        }
    }
    $candidates += @(Get-ChildItem -LiteralPath $Info.mdk -Filter '*.lst' -File | ForEach-Object { $_.FullName })
    $allowed = @('.o', '.d', '.crf', '.axf', '.hex', '.bin', '.map', '.lnp', '.lst', '.htm', '.dep', '.iex', '.tmp')
    foreach ($path in $candidates) {
        $relative = $path.Substring($Workspace.TrimEnd('\').Length + 1).Replace('\', '/')
        if ($relative -in $Tracked) {
            if ($path -in $Info.images) { throw "Canonical image unexpectedly tracked: $path" }
            continue # In particular, preserve the tracked scatter input.
        }
        if ([IO.Path]::GetExtension($path).ToLowerInvariant() -notin $allowed) {
            throw "Unknown output file; refusing cleanup: $path"
        }
        Assert-SafePath $path $Info.mdk
    }
}

function Clear-GeneratedOutput($Info, [string[]]$Files) {
    # Validate the complete plan before deleting anything; use native LiteralPath only.
    foreach ($path in $Files) { Assert-SafePath $path $Info.mdk | Out-Null }
    foreach ($path in $Files) { Remove-Item -LiteralPath $path -Force }
    foreach ($path in $Info.images) {
        if (Test-Path -LiteralPath $path) { throw "Stale canonical output remains: $path" }
    }
    return $true
}

function Assert-FreshOutputs($Info, [bool]$AbsentBefore, [DateTimeOffset]$Started, [DateTimeOffset]$Finished) {
    if (-not $AbsentBefore) { throw 'Missing proof that canonical images were absent before Rebuild' }
    foreach ($path in $Info.images) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing new canonical image: $path" }
        $item = Get-Item -LiteralPath $path
        if ($item.Length -le 0 -or $item.LastWriteTimeUtc -lt $Started.UtcDateTime -or
            $item.LastWriteTimeUtc -gt $Finished.UtcDateTime) { throw "Empty/stale canonical image: $path" }
    }
}

function Assert-SameHash([string]$Canonical, [string]$Frozen, [string]$VerifiedHash) {
    $canonicalHash = (Get-FileMetadata $Canonical).sha256
    $artifactHash = (Get-FileMetadata $Frozen).sha256
    if ($canonicalHash -ne $artifactHash -or $canonicalHash -ne $VerifiedHash) {
        throw "Canonical/artifact/verified SHA mismatch: $Canonical"
    }
    return $artifactHash
}

function Get-BuildResult([string]$Log, [int]$ExitCode) {
    $matches = [regex]::Matches($Log, '(\d+) Error\(s\),\s*(\d+) Warning\(s\)')
    if ($matches.Count -eq 0) { throw 'Missing Keil errors/warnings summary' }
    $last = $matches[$matches.Count - 1]
    if ($ExitCode -notin @(0, 1) -or [int]$last.Groups[1].Value -ne 0 -or $Log -notmatch 'Rebuild target') {
        throw 'Canonical Rebuild failed'
    }
    return [ordered]@{ errors = 0; warnings = [int]$last.Groups[2].Value; exit_code = $ExitCode
        warning_lines = @($Log -split "`n" | Where-Object { $_ -match '(?i)warning' }) }
}

if ($MyInvocation.InvocationName -eq '.') { return } # Pure helpers are testable without building.

$workspaceDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..')).TrimEnd('\')
$firmwareName = 'pwm_double20260926DC6output'
if ((Get-GitText $workspaceDir @('rev-parse', '--show-toplevel')).Replace('/', '\').TrimEnd('\') -ne $workspaceDir) {
    throw 'Script must run from the original Git workspace, not a staged/delivery copy'
}
$info = Get-CanonicalProject $workspaceDir (Join-Path $workspaceDir "$firmwareName\MDK-ARM\pwm_02.uvprojx")
$trackedStatus = Get-GitText $workspaceDir @('status', '--porcelain', '--untracked-files=no')
Assert-FirmwareClean (Get-GitText $workspaceDir @('status', '--porcelain', '--untracked-files=no', '--', $firmwareName))
$head = Get-GitText $workspaceDir @('rev-parse', 'HEAD')
$tracked = (Get-GitText $workspaceDir @('ls-files')) -split "`n"
$inputs = @{}
foreach ($name in $tracked | Where-Object { $_.StartsWith($firmwareName + '/') }) {
    $inputs[$name] = (Get-FileMetadata (Join-Path $workspaceDir $name)).sha256
}
$cleanup = @(Get-CleanupFiles $info $workspaceDir $tracked)
if (-not $ArtifactDirectory) { $ArtifactDirectory = Join-Path $workspaceDir ('artifacts\firmware-release-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff')) }
$artifactDir = Assert-SafePath ([IO.Path]::GetFullPath($ArtifactDirectory)) (Join-Path $workspaceDir 'artifacts')
if (Test-Path -LiteralPath $artifactDir) { throw 'Artifact directory already exists; do not overwrite evidence' }
$preflight = [ordered]@{ git_head = $head; tracked_status = $trackedStatus; canonical = $info
    preflight_utc = [DateTimeOffset]::UtcNow.ToString('o'); input_sha256 = $inputs
    old_images = @($info.images | Where-Object { Test-Path -LiteralPath $_ } | ForEach-Object { Get-FileMetadata $_ }) }
if ($DryRun) {
    [ordered]@{ operation = 'DRY_RUN_NOT_A_RELEASE'; preflight = $preflight; cleanup_files = $cleanup
        artifacts = $artifactDir; build_source = $info.project } | ConvertTo-Json -Depth 8
    return # No cleanup, Keil invocation, conversion, copies, or Git mutation.
}
if (-not $UV4Path) {
    $installation = Join-Path $env:LOCALAPPDATA 'Keil_v5'
    $found = @(Get-ChildItem -LiteralPath $installation -Filter UV4.exe -Recurse -File)
    if ($found.Count -ne 1) { throw 'Specify a discovered installed UV4.exe with -UV4Path' }
    $UV4Path = $found[0].FullName
}
$UV4Path = (Resolve-Path -LiteralPath $UV4Path).Path
if (-not $FromElfPath) {
    $compilerDir = Join-Path (Split-Path -Parent (Split-Path -Parent $UV4Path)) 'ARM\ARMCC'
    $found = @(Get-ChildItem -LiteralPath $compilerDir -Filter fromelf.exe -Recurse -File)
    if ($found.Count -ne 1) { throw 'Specify the matching installed fromelf.exe with -FromElfPath' }
    $FromElfPath = $found[0].FullName
}
$FromElfPath = (Resolve-Path -LiteralPath $FromElfPath).Path
& $Python -B -c 'import sys; print(sys.executable)' | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Python unavailable; refuse cleanup' }
New-Item -ItemType Directory -Path $artifactDir | Out-Null
$manifest = [ordered]@{ status = 'FAIL'; build_kind = 'official_original_project'; git_head = $head
    project = $info.project; project_sha256 = $info.project_sha256; target = $info.target; mcu = $info.mcu
    uv4 = $UV4Path; fromelf = $FromElfPath; tracked_status = $trackedStatus; canonical_hex = $info.images[0] }
$logPath = Join-Path $info.output 'pwm_02.release-build.tmp'
try {
    $preflight | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $artifactDir 'preflight.json') -Encoding UTF8
    foreach ($path in $cleanup) {
        $relative = $path.Substring($info.mdk.Length + 1)
        $backup = Join-Path (Join-Path $artifactDir 'old-output') $relative
        New-Item -ItemType Directory -Path (Split-Path -Parent $backup) -Force | Out-Null
        Copy-Item -LiteralPath $path -Destination $backup
    }
    $absent = Clear-GeneratedOutput $info $cleanup
    $manifest.absent_before_rebuild = $absent
    $started = [DateTimeOffset]::UtcNow
    $manifest.started_utc = $started.ToString('o')
    $processArgs = @('-r', ('"' + $info.project + '"'), '-t', '"pwm_02"', '-j0', '-sg', '-o', ('"' + $logPath + '"'))
    $manifest.command = @($UV4Path) + $processArgs
    $process = Start-Process -FilePath $UV4Path -ArgumentList $processArgs -WorkingDirectory $info.mdk -WindowStyle Hidden -Wait -PassThru
    $finished = [DateTimeOffset]::UtcNow
    $manifest.finished_utc = $finished.ToString('o')
    $manifest.build = Get-BuildResult (Get-Content -LiteralPath $logPath -Raw) $process.ExitCode
    Assert-FreshOutputs $info $absent $started $finished
    $manifest.freshness_pass = $true
    $verificationPath = Join-Path $artifactDir 'static-verification.json'
    & $Python -B -X utf8 (Join-Path $PSScriptRoot 'verify_image.py') --output $info.output --fromelf $FromElfPath --report $verificationPath
    if ($LASTEXITCODE -ne 0) { throw 'Canonical static verification failed' }
    $verification = Get-Content -LiteralPath $verificationPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($verification.status -ne 'PASS') { throw 'Static verification is not PASS' }
    if ((Get-GitText $workspaceDir @('rev-parse', 'HEAD')) -ne $head) { throw 'Git HEAD changed during build' }
    foreach ($name in $inputs.Keys) {
        if ((Get-FileMetadata (Join-Path $workspaceDir $name)).sha256 -ne $inputs[$name]) { throw "Firmware input changed: $name" }
    }
    # Freeze only after canonical verification. Never copy staging/artifacts into canonical.
    Copy-Item -LiteralPath $logPath -Destination (Join-Path $artifactDir 'build.log')
    $frozen = @()
    foreach ($path in $info.images) {
        $expectedHash = ($verification.images | Where-Object { $_.path -eq $path }).sha256
        $destination = Join-Path $artifactDir (Split-Path -Leaf $path)
        Copy-Item -LiteralPath $path -Destination $destination
        Assert-SameHash $path $destination $expectedHash | Out-Null
        $frozen += [ordered]@{ canonical = Get-FileMetadata $path; artifact = Get-FileMetadata $destination }
    }
    $manifest.images = $frozen
    $manifest.verification_status = 'PASS'
    $manifest.sha_equality_pass = $true
    $manifest.status = 'PASS'
    $frozen | ForEach-Object { $_.artifact.sha256 + '  ' + (Split-Path -Leaf $_.artifact.path) } |
        Set-Content -LiteralPath (Join-Path $artifactDir 'SHA256.txt') -Encoding ASCII
} catch {
    $manifest.failure = $_.Exception.Message
    $manifest.status = 'FAIL'
    if (Test-Path -LiteralPath $logPath) {
        Copy-Item -LiteralPath $logPath -Destination (Join-Path $artifactDir 'build.log')
    }
    throw
} finally {
    $manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $artifactDir 'manifest.json') -Encoding UTF8
}
Write-Output "Canonical release PASS: $($info.images[0]); manifest: $(Join-Path $artifactDir 'manifest.json')"
