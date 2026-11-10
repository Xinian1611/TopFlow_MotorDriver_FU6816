# hex2bin.ps1 - build hex into a right-sized bin (ASCII only, no encoding issue)
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

function Convert-HexToBin {
    param([string]$HexFile, [string]$BinFile, [int]$Base, [int]$MaxSize)

    if (-not (Test-Path $HexFile)) { Write-Host ("  [hex2bin] hex not found: " + $HexFile); return }

    $buf = New-Object byte[] $MaxSize
    for ($i = 0; $i -lt $MaxSize; $i++) { $buf[$i] = 0xFF }

    $cnt = 0
    foreach ($line in Get-Content $HexFile) {
        if ($line -notmatch '^:([0-9A-Fa-f]{2})([0-9A-Fa-f]{4})([0-9A-Fa-f]{2})([0-9A-Fa-f]*)[0-9A-Fa-f]{2}$') { continue }
        $len  = [Convert]::ToInt32($matches[1], 16)
        $addr = [Convert]::ToInt32($matches[2], 16)
        $type = [Convert]::ToInt32($matches[3], 16)
        if ($type -ne 0) { continue }
        $data = $matches[4]
        for ($i = 0; $i -lt $len; $i++) {
            $off = $addr - $Base + $i
            if ($off -ge 0 -and $off -lt $MaxSize) {
                $buf[$off] = [Convert]::ToByte($data.Substring($i * 2, 2), 16)
                $cnt++
            }
        }
    }

    $last = $MaxSize - 1
    while ($last -ge 0 -and $buf[$last] -eq 0xFF) { $last-- }
    if ($last -lt 0) { Write-Host "  [hex2bin] empty image"; return }
    $size = $last + 1
    while (($size % 4) -ne 0) { $size++ }

    $out = New-Object byte[] $size
    [Array]::Copy($buf, $out, $size)
    [IO.File]::WriteAllBytes($BinFile, $out)

    Write-Host ("  [hex2bin] " + (Split-Path $HexFile -Leaf) + " -> " + (Split-Path $BinFile -Leaf) + "  size=" + $out.Length + " (0x" + $out.Length.ToString("X") + ")  first=0x" + $out[0].ToString("X2") + "  from_hex=" + $cnt)
}

# APP project: Output\Fortior_FOC_FU68x6.hex -> Output\App_0x1800.bin
$appHex = Join-Path $root "Output\Fortior_FOC_FU68x6.hex"
if (Test-Path $appHex) {
    Convert-HexToBin $appHex (Join-Path $root "Output\App_0x1800.bin") 0x1800 0x6800
    exit 0
}

# Boot project: Objects\Boot_FU68x6.hex -> Objects\Boot_0x0000.bin
$bootHex = Join-Path $root "Objects\Boot_FU68x6.hex"
if (Test-Path $bootHex) {
    Convert-HexToBin $bootHex (Join-Path $root "Objects\Boot_0x0000.bin") 0x0000 0x1700
    exit 0
}

Write-Host ("  [hex2bin] no known hex found, root=" + $root)