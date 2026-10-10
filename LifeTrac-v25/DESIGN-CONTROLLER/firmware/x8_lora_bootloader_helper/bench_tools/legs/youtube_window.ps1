# youtube_window.ps1 [-Open] [-Front] [-Close] [-Url <url>] [-ProfileDir <dir>]
#                    [-FirefoxExe <path>] [-NoPlayerFullscreen]
#
# Moving content for camera legs WITHOUT taking the screen away from the operator.
#   -Open   start a separate Firefox instance (its own profile, audio scaled to 0,
#           autoplay allowed) as a NORMAL window on -Url, bring it to the front
#           and press "f" = the YouTube player's own fullscreen button. The
#           operator leaves it any time with Esc; the window stays a normal
#           window with its own close button. Never --kiosk.
#   -Front  bring that window back to the front (it drops behind any app the
#           operator clicks) and re-enter the player's fullscreen only if the
#           window shows YouTube and is not already fullscreen.
#   -Close  close that instance only (the operator's own Firefox is untouched:
#           the bench instance is found by its -ProfileDir on the command line).
#
# Parameters (each falls back to the environment that lib/bench_env.sh exports,
# then to a built-in default):
#   -Url         $env:YOUTUBE_URL, else the railroad reference video
#                https://www.youtube.com/watch?v=B1yUQwpNhJA. A still for the
#                step-1 landscape passes works too (a file:/// or https URL of a
#                picture); the "f" key is only sent to a YouTube window.
#   -ProfileDir  $env:FIREFOX_PROFILE_DIR, else %TEMP%\lifetrac-bench\ff_bench_youtube
#   -FirefoxExe  $env:FIREFOX_EXE, else Program Files\Mozilla Firefox, else PATH
#   -NoPlayerFullscreen  never send "f" (leave the page as a normal window)
#
# Touches no board and no radio. Screen rule (BENCH_RUNBOOK prep 7): never a kiosk
# and nothing fullscreen the operator cannot leave with Esc -- the 2026-09-27
# kiosk locked the operator out of the PC for a whole round.
# Origin: bench-evidence/RS_13_vector_scene_2026-09-26/scripts/youtube_window.ps1
#         (historical copy, unchanged; it hardcoded the profile dir and the URL).
param(
  [switch]$Open,
  [switch]$Front,
  [switch]$Close,
  [string]$Url = "",
  [string]$ProfileDir = "",
  [string]$FirefoxExe = "",
  [switch]$NoPlayerFullscreen
)

if (-not ($Open -or $Front -or $Close)) {
  "usage: youtube_window.ps1 -Open | -Front | -Close [-Url <url>] [-ProfileDir <dir>] [-FirefoxExe <path>] [-NoPlayerFullscreen]"
  exit 1
}
if (-not $Url) { $Url = $env:YOUTUBE_URL }
if (-not $Url) { $Url = "https://www.youtube.com/watch?v=B1yUQwpNhJA" }
if (-not $ProfileDir) { $ProfileDir = $env:FIREFOX_PROFILE_DIR }
if (-not $ProfileDir) { $ProfileDir = Join-Path $env:TEMP "lifetrac-bench\ff_bench_youtube" }
# one canonical spelling (backslashes, absolute) so the command-line match below is exact
$prof = [System.IO.Path]::GetFullPath(($ProfileDir -replace '/', '\')).TrimEnd('\')

function Find-Firefox {
  if ($FirefoxExe) { return $FirefoxExe }
  if ($env:FIREFOX_EXE) { return $env:FIREFOX_EXE }
  foreach ($p in @("$env:ProgramFiles\Mozilla Firefox\firefox.exe", "${env:ProgramFiles(x86)}\Mozilla Firefox\firefox.exe")) {
    if ($p -and (Test-Path $p)) { return $p }
  }
  $c = Get-Command firefox.exe -ErrorAction SilentlyContinue
  if ($c) { return $c.Source }
  return $null
}

Add-Type @"
using System; using System.Runtime.InteropServices;
public class BenchWin {
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool BringWindowToTop(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  public struct RECT { public int Left, Top, Right, Bottom; }
}
"@
function Get-BenchProcs {
  Get-CimInstance Win32_Process -Filter "Name='firefox.exe'" | Where-Object {
    $_.CommandLine -and $_.CommandLine.IndexOf($prof, [StringComparison]::OrdinalIgnoreCase) -ge 0
  }
}
function Get-BenchWindow {
  $ids = Get-BenchProcs | ForEach-Object { $_.ProcessId }
  if (-not $ids) { return $null }
  Get-Process -Id $ids -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
}
function Raise($w) {
  [BenchWin]::ShowWindow($w.MainWindowHandle, 9) | Out-Null        # SW_RESTORE: a normal window, never kiosk
  [BenchWin]::BringWindowToTop($w.MainWindowHandle) | Out-Null
  [BenchWin]::SetForegroundWindow($w.MainWindowHandle) | Out-Null
  (New-Object -ComObject WScript.Shell).AppActivate($w.Id) | Out-Null
}
function Is-PlayerFullscreen($w) {
  $r = New-Object BenchWin+RECT; [BenchWin]::GetWindowRect($w.MainWindowHandle, [ref]$r) | Out-Null
  Add-Type -AssemblyName System.Windows.Forms
  $s = [System.Windows.Forms.Screen]::FromHandle($w.MainWindowHandle).Bounds
  return ($r.Left -le $s.Left -and $r.Top -le $s.Top -and $r.Right -ge $s.Right -and $r.Bottom -ge $s.Bottom)
}

if ($Close) {
  Get-BenchProcs | ForEach-Object { try { Stop-Process -Id $_.ProcessId -ErrorAction Stop } catch {} }
  "bench video window closed"; exit 0
}
if ($Open) {
  if (-not (Get-BenchWindow)) {
    $ff = Find-Firefox
    if (-not $ff) { "Firefox not found: pass -FirefoxExe or set FIREFOX_EXE"; exit 3 }
    New-Item -ItemType Directory -Force $prof | Out-Null
    @(
      'user_pref("browser.aboutwelcome.enabled", false);',
      'user_pref("browser.shell.checkDefaultBrowser", false);',
      'user_pref("browser.startup.homepage_override.mstone", "ignore");',
      'user_pref("datareporting.policy.dataSubmissionPolicyBypassNotification", true);',
      'user_pref("toolkit.telemetry.reportingpolicy.firstRun", false);',
      'user_pref("trailhead.firstrun.didSeeAboutWelcome", true);',
      'user_pref("browser.sessionstore.resume_from_crash", false);',
      'user_pref("media.autoplay.default", 0);',
      'user_pref("media.autoplay.blocking_policy", 0);',
      'user_pref("media.volume_scale", "0.0");',
      'user_pref("full-screen-api.warning.timeout", 1500);'
    ) | Set-Content -Path (Join-Path $prof "user.js") -Encoding ascii
    Start-Process -FilePath $ff -ArgumentList @("--no-remote", "-profile", "`"$prof`"", "-new-window", "`"$Url`"") | Out-Null
    $t0 = Get-Date
    do { Start-Sleep -Milliseconds 500; $w = Get-BenchWindow } until ($w -or ((Get-Date) - $t0).TotalSeconds -gt 20)
    Start-Sleep -Seconds 6                                          # page + player load
  }
  $Front = $true
}
if ($Front) {
  $w = Get-BenchWindow
  if (-not $w) { "no bench video window (run -Open)"; exit 2 }
  Raise $w
  Start-Sleep -Milliseconds 600
  $isYouTube = ($w.MainWindowTitle -match 'YouTube')
  if ((-not $NoPlayerFullscreen) -and $isYouTube -and -not (Is-PlayerFullscreen $w)) {
    (New-Object -ComObject WScript.Shell).SendKeys("f")               # the player's fullscreen button; Esc leaves it
    Start-Sleep -Milliseconds 800
  }
  $w = Get-BenchWindow
  "window pid $($w.Id) '$($w.MainWindowTitle)' player fullscreen: $(Is-PlayerFullscreen $w)"
}
