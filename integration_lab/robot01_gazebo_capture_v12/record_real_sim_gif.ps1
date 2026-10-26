# Records live, visible Gazebo Sim or RViz2 GUI into an animated GIF.
# Read-only with respect to ROS/Gazebo. Requires Windows ffmpeg.exe.
param(
  [ValidateSet('Gazebo','RViz','Desktop')][string]$Target='Gazebo',
  [ValidateRange(5,600)][int]$Seconds=30,
  [ValidateRange(2,30)][int]$Fps=8,
  [ValidateRange(320,1920)][int]$Width=960,
  [string]$WindowTitle='',
  [string]$OutputPath=''
)
$ErrorActionPreference='Stop'
$ffmpeg=Get-Command ffmpeg.exe -ErrorAction Stop
$source=switch($Target) {
  'Gazebo' {'title=Gazebo Sim'}
  'RViz' {'title=RViz'}
  'Desktop' {'desktop'}
}
if (-not [string]::IsNullOrWhiteSpace($WindowTitle)) { $source="title=$WindowTitle" }
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
  $dir=Join-Path $env:USERPROFILE 'Videos\Robot01'
  New-Item -ItemType Directory -Path $dir -Force | Out-Null
  $OutputPath=Join-Path $dir ('Robot01_'+$Target+'_'+(Get-Date -Format 'yyyyMMdd_HHmmss')+'.gif')
}
if ([IO.Path]::GetExtension($OutputPath) -ne '.gif') {throw 'OutputPath must end with .gif'}
if (Test-Path -LiteralPath $OutputPath) {throw "Output already exists: $OutputPath"}
# One-pass palette-generation + paletteuse (with split) keeps the GIF output direct.
$filter="fps=$Fps,scale=$Width`:-2:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=sierra2_4a"
Write-Host "SOURCE: $source"
Write-Host "START IN 5 SECONDS: $Seconds seconds -> $OutputPath"
Write-Host 'Keep the selected GUI visible and unminimized throughout capture.'
Start-Sleep -Seconds 5
& $ffmpeg.Source -hide_banner -loglevel warning -f gdigrab -framerate $Fps -draw_mouse 0 -i $source -t $Seconds -filter_complex $filter -loop 0 -an -y $OutputPath
if ($LASTEXITCODE -ne 0) {
  if (Test-Path -LiteralPath $OutputPath) {Remove-Item -LiteralPath $OutputPath -Force}
  throw "Capture failed (exit $LASTEXITCODE). Check window title or use -Target Desktop."
}
Write-Host "GIF_SAVED: $OutputPath"
