param(
    [string]$BinDir = '.\build\Release',
    [string]$OutputDir = '',
    [int]$DronePort = 14550,
    [int]$GroundPort = 14551
)

$ErrorActionPreference = 'Stop'
$binPath = (Resolve-Path -LiteralPath $BinDir).Path
if ([string]::IsNullOrWhiteSpace($OutputDir)) {
    $OutputDir = Join-Path 'demo-output' (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
}
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
$outputPath = (Resolve-Path -LiteralPath $OutputDir).Path
$csvPath = Join-Path $outputPath 'flight.csv'

function Start-DemoProcess([string]$Name, [string]$Arguments) {
    $info = New-Object System.Diagnostics.ProcessStartInfo
    $info.FileName = Join-Path $binPath ($Name + '.exe')
    $info.Arguments = $Arguments
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.RedirectStandardInput = $true
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $info
    if (-not $process.Start()) { throw "Cannot start $Name" }
    # Drain both output streams immediately to avoid filling a redirected pipe.
    $stdout = $process.StandardOutput.ReadToEndAsync()
    $stderr = $process.StandardError.ReadToEndAsync()
    return [pscustomobject]@{ Name = $Name; Process = $process; Stdout = $stdout; Stderr = $stderr }
}

function Wait-Telemetry([scriptblock]$Condition, [string]$Description) {
    $deadline = [DateTime]::UtcNow.AddSeconds(5)
    while ([DateTime]::UtcNow -lt $deadline) {
        if ($ground.Process.HasExited) { throw 'GCS exited before the scenario completed' }
        if (Test-Path -LiteralPath $csvPath) {
            $rows = @(Import-Csv -LiteralPath $csvPath)
            if ($rows.Count -gt 0) {
                $last = $rows[-1]
                if (& $Condition $last) { return }
            }
        }
        Start-Sleep -Milliseconds 100
    }
    throw "Timed out: $Description"
}

function Send-Command([string]$Command) {
    Write-Host "> $Command"
    $ground.Process.StandardInput.WriteLine($Command)
    $ground.Process.StandardInput.Flush()
}

function Save-ProcessOutput($App) {
    if ($null -eq $App) { return }
    if (-not $App.Process.HasExited) {
        $App.Process.Kill()
        $App.Process.WaitForExit()
    }
    $App.Stdout.GetAwaiter().GetResult() | Set-Content -LiteralPath (Join-Path $outputPath ($App.Name.ToLower() + '.txt')) -Encoding UTF8
    $App.Stderr.GetAwaiter().GetResult() | Set-Content -LiteralPath (Join-Path $outputPath ($App.Name.ToLower() + '-errors.txt')) -Encoding UTF8
    $App.Process.Dispose()
}

$drone = $null
$ground = $null
try {
    $drone = Start-DemoProcess 'DroneSim' "--listen-port $DronePort --peer-port $GroundPort --duration 12"
    $ground = Start-DemoProcess 'GCS' "--listen-port $GroundPort --peer-port $DronePort --log `"$csvPath`""
    Wait-Telemetry { param($row) $row.armed -eq '0' } 'initial telemetry'
    Send-Command 'arm'
    Wait-Telemetry { param($row) $row.armed -eq '1' } 'ARM'
    Send-Command 'mode 1'
    Wait-Telemetry { param($row) $row.mode -eq '1' } 'GUIDED'
    Send-Command 'goto 55.751000 37.610000 15'
    Wait-Telemetry {
        param($row)
        $latitude = [double]::Parse($row.lat, [Globalization.CultureInfo]::InvariantCulture)
        $altitude = [double]::Parse($row.alt, [Globalization.CultureInfo]::InvariantCulture)
        $latitude -gt 55.7502 -and $altitude -gt 0
    } 'movement towards target'
    Send-Command 'status'
    Send-Command 'disarm'
    Wait-Telemetry { param($row) $row.armed -eq '0' } 'DISARM'
    Send-Command 'status'
    Send-Command 'quit'
    if (-not $ground.Process.WaitForExit(3000)) { throw 'GCS did not stop after quit' }
    if ($ground.Process.ExitCode -ne 0) { throw 'GCS failed' }
    if (-not $drone.Process.WaitForExit(15000)) { throw 'DroneSim did not stop after its duration' }
    if ($drone.Process.ExitCode -ne 0) { throw 'DroneSim failed' }
    $transcript = $ground.Stdout.GetAwaiter().GetResult()
    foreach ($expected in @('ACK ARM', 'ACK SET_MODE', 'ACK GOTO', 'ACK DISARM')) {
        if (-not $transcript.Contains($expected)) { throw "Missing $expected" }
    }
    Write-Host "PASS: ARM -> GUIDED -> GOTO -> movement -> DISARM. Results: $outputPath"
} finally {
    Save-ProcessOutput $ground
    Save-ProcessOutput $drone
}
