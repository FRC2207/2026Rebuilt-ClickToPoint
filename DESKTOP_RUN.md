Run the desktop GUI (Linux / Windows)

This project uses native JNI libraries (ntcorejni etc.) that live in `build/jni/release`.

If you run the main class directly (for example from an IDE), the JVM may not have
`java.library.path` or the OS library search path set to include that folder and you'll
see an error like:

  java.io.IOException: ntcorejni could not be loaded from path.

Use the provided helper scripts which set the correct environment before launching:

Linux / macOS:

```bash
./run-desktop.sh
```

Windows (cmd.exe):

```
run-desktop.bat
```

What these scripts do:
- Ensure `build/jni/release` (where the native libs are placed during Gradle/native build)
  is available on `LD_LIBRARY_PATH` (Unix) or `PATH` (Windows).
- Build the fat jar (`./gradlew jar`) and run the main class from it.

If you prefer to run with Gradle directly, the `simulateJava` Gradle task already
configures the JVM to find the native libraries when run via Gradle:

```bash
./gradlew simulateJava
```

If you want IDE integration instead of the scripts, configure your run configuration to:
- Add JVM arg: `-Djava.library.path=/path/to/your/project/build/jni/release`
- Or add environment variable:
  - `LD_LIBRARY_PATH=/path/to/your/project/build/jni/release` (Linux/macOS)
  - `PATH=C:\path\to\project\build\jni\release;%PATH%` (Windows)

Notes:
- I also added a small preload attempt in `GoToBall.main()` which tries to load the
  platform-native library from `build/jni/release` at startup to help in some cases,
  but the reliable cross-platform solution is to run via the scripts or set the
  environment / JVM arg in your run configuration.
