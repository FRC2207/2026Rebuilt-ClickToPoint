@echo off
REM Run the desktop GUI with native JNI libraries from build\jni\release in PATH.
REM Usage: run-desktop.bat
SETLOCAL ENABLEDELAYEDEXPANSION
SET ROOT_DIR=%~dp0
SET JNI_DIR=%ROOT_DIR%build\jni\release
IF EXIST "%JNI_DIR%\ntcorejni.dll" (
  SET PATH=%JNI_DIR%;%PATH%
  ECHO Using JNI dir: %JNI_DIR%
) ELSE (
  ECHO Warning: JNI dir not found: %JNI_DIR% (you may need to run a Gradle build to generate it)
)

CALL gradlew.bat jar
java -cp "%ROOT_DIR%build\libs\GoToBall.jar" frc.robot.GoToBall %*
