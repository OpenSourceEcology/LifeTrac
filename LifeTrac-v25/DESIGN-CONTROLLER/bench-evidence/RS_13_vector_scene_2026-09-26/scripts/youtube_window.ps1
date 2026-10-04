# youtube_window.ps1 [-Open] [-Front] [-Close]
# Moving content for camera legs WITHOUT taking the screen away from the operator:
#   -Open   start a separate Firefox instance (own profile, audio scaled to 0,
#           autoplay allowed) as a NORMAL window on the railroad reference video,
#           bring it to the front and press "f" = the player's own fullscreen
#           button. The operator leaves it any time with Esc; the window stays a
#           normal window with its own close button. Never --kiosk.
#   -Front  bring that window back to the front (it drops behind the Claude app
#           whenever the operator clicks into the app) and re-enter the player's
#           fullscreen only if it is not already fullscreen.
#   -Close  close that instance only (the operator's own Firefox is untouched).
param([switch]$Open, [switch]$Front, [switch]$Close)
$sp   = "C:\Users\dorkm\AppData\Local\Temp\claude\C--Users-dorkm-Documents-GitHub-LifeTrac\5eaec8c2-12ac-4272-80af-b19d1a563f48\scratchpad"
$prof = "$sp\ff_bench_youtube"
$url  = "https://www.youtube.com/watch?v=B1yUQwpNhJA"
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
function Get-BenchProcs { Get-CimInstance Win32_Process -Filter "Name='firefox.exe'" | Where-Object { $_.CommandLine -like "*ff_bench_youtube*" } }
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
  "bench YouTube window closed"; exit 0
}
if ($Open) {
  if (-not (Get-BenchWindow)) {
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
    ) | Set-Content -Path "$prof\user.js" -Encoding ascii
    Start-Process -FilePath "C:\Program Files\Mozilla Firefox\firefox.exe" -ArgumentList @("--no-remote", "-profile", "`"$prof`"", "-new-window", $url) | Out-Null
    $t0 = Get-Date
    do { Start-Sleep -Milliseconds 500; $w = Get-BenchWindow } until ($w -or ((Get-Date) - $t0).TotalSeconds -gt 20)
    Start-Sleep -Seconds 6                                          # page + player load
  }
  $Front = $true
}
if ($Front) {
  $w = Get-BenchWindow
  if (-not $w) { "no bench YouTube window (run -Open)"; exit 2 }
  Raise $w
  Start-Sleep -Milliseconds 600
  if (-not (Is-PlayerFullscreen $w)) {
    (New-Object -ComObject WScript.Shell).SendKeys("f")               # the player's fullscreen button; Esc leaves it
    Start-Sleep -Milliseconds 800
  }
  $w = Get-BenchWindow
  "window pid $($w.Id) '$($w.MainWindowTitle)' player fullscreen: $(Is-PlayerFullscreen $w)"
}
