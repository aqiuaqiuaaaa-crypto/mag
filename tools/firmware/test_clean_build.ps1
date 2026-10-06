# No Keil build, flashing, or canonical-output mutation. Fixtures stay in artifacts.
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'clean-build.ps1')
$workspaceDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..')).TrimEnd('\')
$fixtureRoot = Join-Path $workspaceDir ('artifacts\canonical-release-sop-tests-' + [Guid]::NewGuid().ToString('N'))
$mdk = Join-Path $fixtureRoot 'pwm_double20260926DC6output\MDK-ARM'
$output = Join-Path $mdk 'pwm_02'
New-Item -ItemType Directory -Path $output -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $workspaceDir 'pwm_double20260926DC6output\MDK-ARM\pwm_02.uvprojx') -Destination $mdk
Copy-Item -LiteralPath (Join-Path $workspaceDir 'pwm_double20260926DC6output\pwm_02.ioc') -Destination (Split-Path -Parent $mdk)
$info = Get-CanonicalProject $fixtureRoot (Join-Path $mdk 'pwm_02.uvprojx')
$script:passed = 0
function Check([bool]$Condition, [string]$Message) { if (-not $Condition) { throw $Message } }
function Expect-Failure([scriptblock]$Action) {
    $failed = $false
    try { & $Action | Out-Null } catch { $failed = $true }
    Check $failed 'Expected rejection did not occur'
}
function Test-Case([string]$Name, [scriptblock]$Action) {
    & $Action
    $script:passed++
    Write-Output "PASS $Name"
}

Test-Case 'original project and canonical HEX/AXF/MAP resolution' {
    Check ($info.project -eq (Join-Path $mdk 'pwm_02.uvprojx')) 'Wrong original project'
    Check ($info.images.Count -eq 3 -and $info.images[0] -eq (Join-Path $output 'pwm_02.hex')) 'Wrong canonical images'
    Check ($info.images[1].EndsWith('pwm_02.axf') -and $info.images[2].EndsWith('pwm_02.map')) 'Wrong AXF/MAP'
}
Test-Case 'staging cannot be an official build source' {
    Expect-Failure { Get-CanonicalProject $fixtureRoot (Join-Path $fixtureRoot 'staging\pwm_02.uvprojx') }
}
Test-Case 'staged or unstaged firmware edits block release' {
    Assert-FirmwareClean ''
    Expect-Failure { Assert-FirmwareClean ' M firmware/main.c' }
    Expect-Failure { Assert-FirmwareClean 'M  firmware/main.c' }
}
Test-Case 'unsafe cleanup path is rejected' {
    Expect-Failure { Clear-GeneratedOutput $info @((Join-Path $fixtureRoot 'outside-output.hex')) }
}
$scatter = Join-Path $output 'pwm_02.sct'
Set-Content -LiteralPath $scatter 'preserved tracked input' -Encoding ASCII
foreach ($path in $info.images) { Set-Content -LiteralPath $path 'stale output' -Encoding ASCII }
$trackedScatter = 'pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.sct'
Test-Case 'stale outputs are removed and tracked scatter is preserved' {
    $plan = @(Get-CleanupFiles $info $fixtureRoot @($trackedScatter))
    Check ($plan.Count -eq 3) 'Unexpected cleanup plan'
    Check (Clear-GeneratedOutput $info $plan) 'No absence proof'
    Check (Test-Path -LiteralPath $scatter) 'Tracked scatter deleted'
    Check (-not (Test-Path -LiteralPath $info.images[0])) 'Stale HEX remained'
}
Test-Case 'missing HEX after build fails' {
    Expect-Failure { Assert-FreshOutputs $info $true ([DateTimeOffset]::UtcNow.AddMinutes(-1)) ([DateTimeOffset]::UtcNow) }
}
foreach ($path in $info.images) { Set-Content -LiteralPath $path 'new fixture output' -Encoding ASCII }
$started = [DateTimeOffset]::UtcNow.AddMinutes(-1)
$finished = [DateTimeOffset]::UtcNow.AddMinutes(1)
Test-Case 'existing image without pre-build absence proof fails' {
    Expect-Failure { Assert-FreshOutputs $info $false $started $finished }
}
Test-Case 'fresh images within the build window pass' { Assert-FreshOutputs $info $true $started $finished }
Test-Case 'old mtime fails even if file exists and has content' {
    (Get-Item -LiteralPath $info.images[0]).LastWriteTimeUtc = [DateTime]::UtcNow.AddDays(-1)
    Expect-Failure { Assert-FreshOutputs $info $true $started $finished }
    (Get-Item -LiteralPath $info.images[0]).LastWriteTimeUtc = [DateTime]::UtcNow
}
Test-Case 'zero-size new HEX fails' {
    [IO.File]::WriteAllBytes($info.images[0], [byte[]]@())
    Expect-Failure { Assert-FreshOutputs $info $true $started $finished }
    Set-Content -LiteralPath $info.images[0] 'new fixture output' -Encoding ASCII
}
$frozen = Join-Path $fixtureRoot 'frozen.hex'
$verifiedHash = (Get-FileMetadata $info.images[0]).sha256
Test-Case 'different canonical/artifact SHA fails' {
    Set-Content -LiteralPath $frozen 'different artifact' -Encoding ASCII
    Expect-Failure { Assert-SameHash $info.images[0] $frozen $verifiedHash }
}
Test-Case 'equal SHA passes' {
    Copy-Item -LiteralPath $info.images[0] -Destination $frozen
    Check ((Assert-SameHash $info.images[0] $frozen $verifiedHash) -eq $verifiedHash) 'Equal hash rejected'
}
Test-Case 'identical copies still fail if changed after static verification' {
    Expect-Failure { Assert-SameHash $info.images[0] $frozen ('0' * 64) }
}
Test-Case 'unknown output sources are not deleted' {
    $unexpected = Join-Path $output 'unexpected.c'
    Set-Content -LiteralPath $unexpected 'keep me' -Encoding ASCII
    Expect-Failure { Get-CleanupFiles $info $fixtureRoot @($trackedScatter) }
    Check (Test-Path -LiteralPath $unexpected) 'Unknown source was deleted'
}
Test-Case 'warnings recorded but errors and non-Rebuild logs fail' {
    $result = Get-BuildResult 'Rebuild target pwm_02: 0 Error(s), 1 Warning(s)' 1
    Check ($result.warnings -eq 1 -and $result.errors -eq 0) 'Bad warning count'
    Expect-Failure { Get-BuildResult 'Rebuild target: 1 Error(s), 0 Warning(s)' 2 }
    Expect-Failure { Get-BuildResult 'Build target: 0 Error(s), 0 Warning(s)' 0 }
}
Test-Case 'entrypoint has no Git stage/commit/push or download command' {
    $source = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'clean-build.ps1') -Raw
    Check ($source -notmatch '\bgit\s+(add|commit|push)\b') 'Git mutation found'
    Check ($source -notmatch '["'']-f["'']') 'Keil download flag found'
}
Test-Case 'original workspace dry-run preserves canonical images and staged diff' {
    $canonical = Get-CanonicalProject $workspaceDir (Join-Path $workspaceDir 'pwm_double20260926DC6output\MDK-ARM\pwm_02.uvprojx')
    $before = @($canonical.images | ForEach-Object {
        if (Test-Path -LiteralPath $_) { Get-FileMetadata $_ } else { @{ path = $_; absent = $true } }
    })
    $indexBefore = Get-GitText $workspaceDir @('diff', '--cached')
    $json = & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'clean-build.ps1') -DryRun
    Check ($LASTEXITCODE -eq 0) 'Dry-run failed'
    $plan = $json -join "`n" | ConvertFrom-Json
    Check ($plan.operation -eq 'DRY_RUN_NOT_A_RELEASE' -and $plan.build_source -eq $canonical.project) 'Invalid dry-run provenance'
    Check (-not (Test-Path -LiteralPath $plan.artifacts)) 'Dry-run wrote release artifacts'
    for ($i = 0; $i -lt 3; $i++) {
        if ($before[$i].absent) {
            Check (-not (Test-Path -LiteralPath $canonical.images[$i])) 'Dry-run created canonical output'
        } else {
            $after = Get-FileMetadata $canonical.images[$i]
            Check ($after.sha256 -eq $before[$i].sha256 -and $after.mtime_utc -eq $before[$i].mtime_utc) 'Canonical image changed'
        }
    }
    Check ((Get-GitText $workspaceDir @('diff', '--cached')) -eq $indexBefore) 'Git staging changed'
}
Write-Output "$script:passed tests PASS; no Keil build or hardware operations; fixtures: $fixtureRoot"
