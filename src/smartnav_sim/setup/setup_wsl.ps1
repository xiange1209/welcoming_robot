# 在 Windows 端建立 WSL2 的 Ubuntu 24.04（裝在 D 槽，不佔 C 槽）。
#
# ★ 這個檔案必須是「UTF-8 含 BOM」：Windows PowerShell 5.1 讀沒有 BOM 的 UTF-8 檔時
#   會用系統碼頁（繁中是 CP950）解碼，中文訊息會變亂碼，嚴重時連語法都壞掉。
#
# 用法（一般權限的 PowerShell 即可，不需要系統管理員）：
#   powershell -ExecutionPolicy Bypass -File "$env:USERPROFILE\Desktop\專題\車子\src\smartnav_sim\setup\setup_wsl.ps1"
#
# 前提：已經做過第 0 步（系統管理員 PowerShell 執行 wsl.exe --install --no-distribution 並重開機）。
#       這支會先檢查，沒做的話會告訴你怎麼做，不會往下跑。

$ErrorActionPreference = "Stop"
$Distro   = "Ubuntu-24.04"
$Location = "D:\WSL\Ubuntu-24.04"
$MinFreeGB = 15

function Say($m)  { Write-Host ""; Write-Host "▶ $m" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "  ✓ $m" -ForegroundColor Green }
function Warn($m) { Write-Host "  ⚠ $m" -ForegroundColor Yellow }
function Fail($m) { Write-Host "  ✗ $m" -ForegroundColor Red; exit 1 }

# wsl.exe 的輸出是 UTF-16LE，直接當字串比對會夾著 \0。統一去掉再用。
function WslOut([string[]]$wslArgs) {
    $raw = & wsl.exe @wslArgs 2>&1 | Out-String
    return ($raw -replace "`0", "")
}

# ── 1. WSL2 能不能用 ────────────────────────────────────────
Say "1/4 檢查 WSL2 是否已啟用"
# 用 HypervisorPresent 判斷，不去比對 wsl --status 的中文字串（語系一換就失效）
$hv = (Get-CimInstance Win32_ComputerSystem).HypervisorPresent
if (-not $hv) {
    Write-Host ""
    Write-Host "  WSL2 還不能用：Windows 的「虛擬機器平台」沒有啟用。" -ForegroundColor Yellow
    Write-Host "  請做第 0 步（只要做一次）：" -ForegroundColor Yellow
    Write-Host "    1. 開始功能表搜尋 PowerShell → 右鍵「以系統管理員身分執行」"
    Write-Host "    2. 貼上這一行：  wsl.exe --install --no-distribution"
    Write-Host "    3. 跑完後重新開機"
    Write-Host "    4. 重開機後再執行這支腳本"
    exit 1
}
Ok "虛擬化已啟用（HypervisorPresent = True）"

# ── 2. D 槽空間 ─────────────────────────────────────────────
Say "2/4 檢查 D 槽"
$d = Get-PSDrive D -ErrorAction SilentlyContinue
if ($null -eq $d) { Fail "找不到 D 槽。若要改裝到別處，編輯本檔開頭的 `$Location" }
$freeGB = [math]::Round($d.Free / 1GB, 1)
if ($freeGB -lt $MinFreeGB) { Fail "D 槽只剩 $freeGB GB，至少要 $MinFreeGB GB" }
Ok "D 槽剩 $freeGB GB（預估會用 5~7 GB）"

# ── 3. 安裝 Ubuntu 24.04 ────────────────────────────────────
Say "3/4 安裝 $Distro 到 $Location"
$existing = (WslOut @("--list", "--quiet")) -split "`r?`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne "" }
if ($existing -contains $Distro) {
    Ok "$Distro 已經裝過，跳過（要重裝請先自己執行 wsl --unregister $Distro，會刪掉裡面所有東西）"
} else {
    New-Item -ItemType Directory -Force -Path $Location | Out-Null
    Write-Host "  下載與安裝約需數分鐘。裝完會開一個 Ubuntu 視窗要你設定 Linux 使用者名稱與密碼，"
    Write-Host "  設定完輸入 exit 回到這裡。"
    # WSL 2.5.9 的 --install 支援 --location（本機 wsl --help 確認過）
    & wsl.exe --install $Distro --location $Location
    $rc = $LASTEXITCODE
    # ★ 2026-09-26：不能只看結束碼。安裝完會直接進 Ubuntu 讓你設帳號，輸入 exit 離開時，
    #   wsl.exe 回傳的是「Ubuntu 裡最後一個指令」的結束碼（打錯一個指令就是 127），不是安裝結果。
    #   實際發生過：明明裝好了，這裡卻說失敗。所以改成直接看清單裡有沒有它。
    $after = (WslOut @("--list", "--quiet")) -split "`r?`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne "" }
    if ($after -contains $Distro) {
        if ($rc -ne 0) { Ok "（wsl 回傳 $rc，但 $Distro 已經在清單裡 —— 那是離開 Ubuntu 前最後一個指令的結束碼，不影響）" }
    } else {
        Warn "wsl --install 回傳 $LASTEXITCODE。若訊息提到 --location 不支援，可改用："
        Warn "  wsl --install $Distro   （先裝到預設位置）"
        Warn "  wsl --export $Distro D:\WSL\ubuntu.tar"
        Warn "  wsl --unregister $Distro"
        Warn "  wsl --import $Distro $Location D:\WSL\ubuntu.tar"
        exit 1
    }
    Ok "$Distro 安裝完成"
}

# ── 4. .wslconfig ───────────────────────────────────────────
# 限制 WSL 最多用多少記憶體與核心，swap 放 D 槽。已經有這個檔就不覆蓋，只提示。
Say "4/4 WSL 資源設定（%USERPROFILE%\.wslconfig）"
$cfg = Join-Path $env:USERPROFILE ".wslconfig"
$want = @"
[wsl2]
memory=12GB
processors=8
swapFile=D:\\WSL\\swap.vhdx
"@
if (Test-Path $cfg) {
    $cur = Get-Content $cfg -Raw
    if ($cur -match "(?m)^\s*memory\s*=") {
        Ok "已經有 .wslconfig 而且設過 memory，不動它"
    } else {
        Copy-Item $cfg "$cfg.bak_$(Get-Date -Format yyyyMMdd_HHmmss)"
        Warn ".wslconfig 已存在但沒設記憶體上限。已備份，建議手動加入："
        Write-Host $want
    }
} else {
    # .wslconfig 是 WSL 讀的，用 UTF-8（無 BOM）寫
    [System.IO.File]::WriteAllText($cfg, $want, (New-Object System.Text.UTF8Encoding($false)))
    Ok "已建立 .wslconfig（記憶體 12 GB、8 核心、swap 在 D 槽）"
    & wsl.exe --shutdown
    Ok "已重啟 WSL 讓設定生效"
}

Write-Host ""
Write-Host "完成。下一步：開始功能表開「Ubuntu 24.04」，在裡面執行：" -ForegroundColor Cyan
# 本檔所在資料夾換成 WSL 路徑（C:\Users\... -> /mnt/c/Users/...）。
# 不寫死：路徑裡有 Windows 使用者名稱，這個檔案會進 GitHub
$wslDir = "/mnt/" + $PSScriptRoot.Substring(0, 1).ToLower() + ($PSScriptRoot.Substring(2) -replace '\\', '/')
Write-Host "  bash `"$wslDir/setup_ubuntu.sh`""
