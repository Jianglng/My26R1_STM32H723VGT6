$ErrorActionPreference = 'Stop'
$testProjectRoot = Split-Path -Parent $PSScriptRoot
$testOutputDir = Join-Path ([System.IO.Path]::GetTempPath()) ('chassis-host-tests-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testOutputDir | Out-Null
$testExecutable = Join-Path $testOutputDir 'localization_tests.exe'

Push-Location $testProjectRoot
try {
    $testIncludeDirs = @(
        'tests/support', 'common', 'bsp', 'localization', 'protocol/lidar',
        'protocol/remote', 'robot_config', 'control/auto', 'control/manual',
        'algorithm/math', 'algorithm/pid', 'algorithm/trajectory'
    )
    $testSources = @(
        'tests/localization_tests.cpp', 'bsp/bsp_usb.cpp',
        'protocol/lidar/lidar_pose_protocol.cpp', 'localization/lidar_localization.cpp',
        'localization/pose_source.cpp', 'localization/chassis_odometry.cpp',
        'control/auto/auto_move.cpp', 'control/auto/auto_mode_controller.cpp',
        'control/manual/manual_mode_controller.cpp', 'algorithm/trajectory/trapezoidal_profile.cpp'
    )
    $testCompilerArgs = @('-std=c++11', '-Wall', '-Wextra', '-Werror')
    $testCompilerArgs += $testIncludeDirs | ForEach-Object { '-I' + $_ }
    $testCompilerArgs += $testSources
    $testCompilerArgs += @('-o', $testExecutable)
    & g++ @testCompilerArgs
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
}
finally {
    Pop-Location
}
