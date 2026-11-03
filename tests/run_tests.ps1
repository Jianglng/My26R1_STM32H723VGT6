param([string]$Compiler = "g++")

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path $PSScriptRoot -Parent
$testOutput = Join-Path $PSScriptRoot "build"
New-Item -ItemType Directory -Force -Path $testOutput | Out-Null
$includeDirs = @("tests/stubs", "algorithm/math", "algorithm/pid", "algorithm/kinematics",
    "algorithm/trajectory", "control/auto", "localization", "common", "robot_config",
    "protocol/lidar", "protocol/dji", "protocol/vesc", "bsp", "chassis")
$sources = @("tests/motion_tests.cpp", "algorithm/pid/pid.cpp",
    "algorithm/kinematics/chassis_kinematics.cpp", "algorithm/trajectory/trapezoidal_profile.cpp",
    "control/auto/auto_move.cpp", "control/auto/auto_mode_controller.cpp",
    "localization/chassis_odometry.cpp", "localization/lidar_localization.cpp",
    "protocol/lidar/lidar_pose_protocol.cpp", "protocol/dji/dji_6020.cpp",
    "protocol/vesc/vesc_motor.cpp", "chassis/chassis.cpp")
$arguments = @("-std=c++14", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2")
foreach ($directory in $includeDirs) { $arguments += "-I" + (Join-Path $projectRoot $directory) }
foreach ($source in $sources) { $arguments += Join-Path $projectRoot $source }
$executable = Join-Path $testOutput "motion_tests.exe"
$arguments += @("-o", $executable)
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw "Host compilation failed" }
& $executable
if ($LASTEXITCODE -ne 0) { throw "Behavior tests failed" }
