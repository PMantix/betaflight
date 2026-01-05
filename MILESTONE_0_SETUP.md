# Milestone 0: Setup and Baseline Validation

## Step 1: Install Required Tools

### Option A: Using Chocolatey (Recommended for Windows)

If you don't have Chocolatey, install it first from: https://chocolatey.org/install

Then open PowerShell as Administrator and run:

```powershell
# Install ARM toolchain
choco install gcc-arm-embedded

# Install Make
choco install make

# Install Python (if not already installed)
choco install python

# Refresh environment variables
refreshenv
```

### Option B: Manual Installation

1. **ARM Toolchain**
   - Download from: https://developer.arm.com/downloads/-/gnu-rm
   - Install to default location
   - Add to PATH: `C:\Program Files (x86)\GNU Arm Embedded Toolchain\<version>\bin`

2. **Make for Windows**
   - Download from: http://gnuwin32.sourceforge.net/packages/make.htm
   - Or install MSYS2: https://www.msys2.org/

3. **Python 3**
   - Download from: https://www.python.org/downloads/
   - Make sure to check "Add Python to PATH" during installation

## Step 2: Verify Installation

Open a **new** PowerShell window and verify:

```powershell
arm-none-eabi-gcc --version
make --version
python --version
```

Expected output should show version numbers for all three tools.

## Step 3: Identify Your Target Board

Before building, you need to know which target to build for. Common targets:

| Board/FC | Target | MCU |
|----------|--------|-----|
| Speedybee F405 | SPEEDYBEEF405V3 | STM32F405 |
| Matek F722 | MATEKF722 | STM32F722 |
| Kakute F7 | KAKUTEF7 | STM32F745 |
| Generic F405 | STM32F405 | STM32F405 |

To see all available targets:
```powershell
cd C:\Users\pmant\source\repos\betaflight\betaflight
make targets
```

## Step 4: Build Firmware

Once you know your target (replace `MATEKF722` with your target):

```powershell
cd C:\Users\pmant\source\repos\betaflight\betaflight
make TARGET=MATEKF722
```

Build output will be in: `obj/betaflight_<VERSION>_<TARGET>.hex`

## Step 5: Flash Firmware

1. Connect flight controller via USB
2. Open Betaflight Configurator
3. Go to "Firmware Flasher" tab
4. Click "Load Firmware [Local]"
5. Select your built `.hex` file from `obj/` folder
6. Click "Flash Firmware"

## Step 6: Flight Test

1. Configure your drone in Betaflight Configurator
2. Verify motors, RX, sensors
3. Enable blackbox logging
4. Perform test hover flight
5. Download blackbox log for baseline comparison

## Troubleshooting

### "make: command not found"
- Make sure tools are installed and PATH is updated
- Close and reopen PowerShell after installation
- Try running `refreshenv` if using Chocolatey

### "arm-none-eabi-gcc: command not found"
- Verify installation path is in System PATH
- Restart PowerShell/Command Prompt

### Build errors about missing files
- Make sure you're in the correct directory
- Run `git submodule update --init --recursive` to fetch dependencies

### Flash size error
- Some targets have specific variants, check with `make targets`
- Use exact target name from the list

## Current Progress (2026-01-04)

### ✅ Milestone 0 - COMPLETE!

#### Completed Tasks
- ✅ ARM toolchain installed: v13.3.1
- ✅ Make installed: v4.4.1
- ✅ Python installed: v3.14.2
- ✅ Git Bash installed: v2.52.0
- ✅ Repository cloned and configs hydrated
- ✅ Target identified: **BETAFPVG473** (STM32G47X) - BETAFPV Air G4 5in1
- ✅ Firmware built successfully: `betaflight_2026.6.0-alpha_STM32G47X_BETAFPVG473.hex`
- ✅ DFU drivers installed (ImpulseRC Driver Fixer)
- ✅ Firmware flashed to drone
- ✅ All sensors verified working
- ✅ Blackbox logging enabled
- ✅ Test flight completed successfully
- ✅ Baseline blackbox log captured

#### Known Issues
- Slower boot time observed on alpha firmware (expected for development builds)

### 🎯 Ready For
- **Milestone 1**: Infrastructure & State Machine implementation

## Next Steps

Once you have:
- ✅ Built firmware successfully
- ✅ Flashed to your drone
- ✅ Verified stable flight
- ✅ Captured baseline blackbox log

You're ready to proceed to **Milestone 1: Infrastructure & State Machine**
