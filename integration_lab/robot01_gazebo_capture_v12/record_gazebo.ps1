# Windows PowerShell 5+; capture REAL visible Gazebo Sim window, not offline data.
# Requires Windows ffmpeg.exe (separate from WSL ffmpeg). Never controls the robot.
param(
  [ValidateRange(5,7200)][int]$Seconds=180,
  [ValidateRange(5,60)][int]$Fps=15,
  [string]$OutputPath='',
  [switch]$Desktop
)
$ErrorActionPreference='Stop'
$cmd=Get-Command ffmpeg.exe -ErrorAction SilentlyContinue
if (-not $cmd) {
  Write-Host 'WINDOWS_FFMPEG_MISSING: WSL ffmpeg is not Windows ffmpeg.exe.'
  Write-Host 'Windows PowerShell: winget install --id Gyan.FFmpeg -e'
  Write-Host 'Then OPEN A NEW PowerShell/WSL terminal. Or use Win+Alt+R (Xbox Game Bar).'
  exit 2
}
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
  $dest=Join-Path $env:USERPROFILE 'Videos'
  New-Item -ItemType Directory -Force -Path $dest | Out-Null
  $OutputPath=Join-Path $dest ('Robot01_Gazebo_REAL_'+(Get-Date -Format yyyyMMdd_HHmmss)+'.mp4')
}
if (Test-Path -LiteralPath $OutputPath) {throw ('File already exists: '+$OutputPath)}
$inputSource=if ($Desktop) {'desktop'} else {'title=Gazebo Sim'}
Write-Host "INPUT: $inputSource"
Write-Host "RECORDING: $Seconds seconds -> $OutputPath"
Write-Host 'Show the Gazebo Sim window while capturing. Do NOT cover or minimize it.'
& $cmd.Source -hide_banner -loglevel warning -f gdigrab -framerate $Fps -draw_mouse 0 -i $inputSource -t $Seconds -an -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p -y $OutputPath
if ($LASTEXITCODE -ne 0) {
  if (Test-Path -LiteralPath $OutputPath) {Remove-Item -LiteralPath $OutputPath -Force}
  Write-Host 'Capture failed. For Qt/WSLg title mismatch retry with -Desktop to capture visible full desktop.'
  exit $LASTEXITCODE
}
Write-Host "VIDEO_SAVED: $OutputPath"
