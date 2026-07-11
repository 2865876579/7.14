[CmdletBinding()]
param(
    [string]$Message
)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

function Invoke-Git {
    param(
        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$GitArgs
    )

    & git @GitArgs
    if ($LASTEXITCODE -ne 0) {
        throw "git $($GitArgs -join ' ') failed with exit code $LASTEXITCODE"
    }
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "Git was not found in PATH."
}

if (-not (Test-Path -LiteralPath ".git")) {
    throw "This folder is not a Git repository."
}

Invoke-Git add -A

& git diff --cached --quiet
if ($LASTEXITCODE -eq 1) {
    if ([string]::IsNullOrWhiteSpace($Message)) {
        $Message = "Update $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    }
    Invoke-Git commit -m $Message
} elseif ($LASTEXITCODE -ne 0) {
    throw "Unable to inspect staged changes."
} else {
    Write-Host "No local changes to commit."
}

$remoteMain = & git ls-remote --heads origin main
if ($LASTEXITCODE -ne 0) {
    throw "Unable to contact the GitHub repository."
}

if (-not [string]::IsNullOrWhiteSpace(($remoteMain | Out-String))) {
    Invoke-Git pull --rebase origin main
}

Invoke-Git push --set-upstream origin main
Write-Host "Push completed successfully." -ForegroundColor Green

