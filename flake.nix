{
  description = "GroundFlight - RC Car Gyro Stabilizer for RadioMaster Nexus";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    
    # STM32 HAL/CMSIS
    stm32cubef7 = {
      url = "git+https://github.com/STMicroelectronics/STM32CubeF7?submodules=1";
      flake = false;
    };
  };

  outputs = inputs@{ flake-parts, nixpkgs, stm32cubef7, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" "aarch64-linux" ];

      perSystem = { system, pkgs, ... }:
      let
        # ARM bare-metal toolchain
        armToolchain = pkgs.pkgsCross.arm-embedded.buildPackages.gcc;
        armBinutils = pkgs.pkgsCross.arm-embedded.buildPackages.binutils;
        armNewlib = pkgs.pkgsCross.arm-embedded.newlib;

        # Helper: Flash via OpenOCD + ST-Link
        gf-flash = pkgs.writeShellScriptBin "gf-flash" ''
          firmware="''${1:-build/groundflight.elf}"
          
          if [ ! -f "$firmware" ]; then
            echo "Error: Firmware not found: $firmware"
            echo "Run 'cmake --build build' first"
            exit 1
          fi

          echo "Flashing $firmware via ST-Link..."
          ${pkgs.openocd}/bin/openocd \
            -f interface/stlink.cfg \
            -f target/stm32f7x.cfg \
            -c "program $firmware verify reset exit"
        '';

        # Helper: One-shot upload - send dfu command, wait, flash
        gf-upload = pkgs.writeShellScriptBin "gf-upload" ''
          firmware="''${1:-build/groundflight.bin}"
          port="''${2:-/dev/ttyACM0}"
          
          if [ ! -f "$firmware" ]; then
            echo "Error: Firmware not found: $firmware"
            echo "Run 'gf-build' first"
            exit 1
          fi

          # If already in DFU mode, just flash
          if ${pkgs.dfu-util}/bin/dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
            echo "Already in DFU mode, flashing..."
          elif [ -e "$port" ]; then
            echo "Sending 'dfu' command to $port..."
            ${pkgs.coreutils}/bin/stty -F "$port" 115200 raw -echo
            echo "dfu" > "$port"
            sleep 0.5
            
            # Wait for DFU device (up to 3 seconds)
            echo "Waiting for DFU mode..."
            for i in $(seq 1 30); do
              if ${pkgs.dfu-util}/bin/dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
                break
              fi
              sleep 0.1
            done
            
            if ! ${pkgs.dfu-util}/bin/dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
              echo "Error: Device did not enter DFU mode"
              exit 1
            fi
          else
            echo "Error: No device found at $port and not in DFU mode"
            exit 1
          fi

          echo "Flashing $firmware..."
          ${pkgs.dfu-util}/bin/dfu-util -a 0 -s 0x08000000:leave -D "$firmware"
          
          echo ""
          echo "Done!"
        '';

        # Helper: Flash via USB DFU (no ST-Link needed)
        gf-dfu = pkgs.writeShellScriptBin "gf-dfu" ''
          firmware="''${1:-build/groundflight.bin}"
          
          if [ ! -f "$firmware" ]; then
            echo "Error: Firmware not found: $firmware"
            echo "Run 'gf-build' first"
            exit 1
          fi

          # Check if already in DFU mode
          if ${pkgs.dfu-util}/bin/dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
            echo "DFU device found, flashing $firmware..."
          else
            echo "========================================"
            echo "  USB DFU Flash"
            echo "========================================"
            echo ""
            echo "Enter DFU mode using ONE of these methods:"
            echo ""
            echo "  A) From running firmware:"
            echo "     Type 'dfu' in the CLI"
            echo ""
            echo "  B) Hardware method:"
            echo "     1. Disconnect USB"
            echo "     2. Hold BOOT/BIND button"
            echo "     3. Connect USB while holding"
            echo "     4. Release after 1 second"
            echo ""
            echo "Press Enter when Nexus is in DFU mode..."
            read -r

            echo "Checking for DFU device..."
            if ! ${pkgs.dfu-util}/bin/dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
              echo "Error: No STM32 DFU device found"
              echo ""
              echo "Expected: [0483:df11] (STMicroelectronics STM32 BOOTLOADER)"
              echo ""
              echo "Troubleshooting:"
              echo "  - Ensure you're in DFU mode (try 'dfu' command or button)"
              echo "  - Try a different USB cable"
              echo "  - Check 'dmesg' for USB errors"
              echo "  - Verify udev rules for plugdev group"
              exit 1
            fi
            echo "Found DFU device, flashing $firmware..."
          fi

          ${pkgs.dfu-util}/bin/dfu-util -a 0 -s 0x08000000:leave -D "$firmware"
          
          echo ""
          echo "Done! Device should reboot automatically."
        '';

        # Helper: List DFU devices
        gf-dfu-list = pkgs.writeShellScriptBin "gf-dfu-list" ''
          echo "DFU devices:"
          ${pkgs.dfu-util}/bin/dfu-util -l
        '';

        # Helper: Start OpenOCD GDB server
        gf-debug = pkgs.writeShellScriptBin "gf-debug" ''
          echo "Starting OpenOCD GDB server on :3333..."
          echo "Connect with: arm-none-eabi-gdb -ex 'target remote :3333' build/groundflight.elf"
          ${pkgs.openocd}/bin/openocd \
            -f interface/stlink.cfg \
            -f target/stm32f7x.cfg
        '';

        # Helper: Serial monitor (Ctrl-C to exit)
        gf-monitor = pkgs.writeShellScriptBin "gf-monitor" ''
          port="''${1:-/dev/ttyACM0}"
          baud="''${2:-115200}"
          
          if [ ! -e "$port" ]; then
            echo "Waiting for $port..."
            while [ ! -e "$port" ]; do sleep 0.1; done
          fi
          
          echo "Connecting to $port at $baud baud (Ctrl-C to exit)"
          echo "---"
          
          # Use stty to configure port, then cat with trap for clean exit
          ${pkgs.coreutils}/bin/stty -F "$port" "$baud" raw -echo -echoe -echok
          
          # Trap Ctrl-C to exit cleanly
          trap 'echo ""; echo "Disconnected."; exit 0' INT
          
          # Background reader
          cat "$port" &
          reader_pid=$!
          
          # Writer - send stdin to port
          cat > "$port"
          
          # Cleanup
          kill $reader_pid 2>/dev/null
        '';

        # Helper: Serial console to CLI (UART6 / PORT-B)
        gf-cli = pkgs.writeShellScriptBin "gf-cli" ''
          port="''${1:-/dev/ttyUSB0}"
          echo "Connecting to GroundFlight CLI on $port at 115200 baud..."
          echo "Press Ctrl-A Ctrl-X to exit"
          ${pkgs.picocom}/bin/picocom -b 115200 "$port"
        '';

        # Helper: Reset via ST-Link
        gf-reset = pkgs.writeShellScriptBin "gf-reset" ''
          echo "Resetting target via ST-Link..."
          ${pkgs.openocd}/bin/openocd \
            -f interface/stlink.cfg \
            -f target/stm32f7x.cfg \
            -c "init; reset; exit"
        '';

        # Helper: Build (convenience wrapper)
        gf-build = pkgs.writeShellScriptBin "gf-build" ''
          if [ ! -d "build" ]; then
            echo "Configuring CMake..."
            cmake -B build -DCMAKE_BUILD_TYPE=''${1:-Debug}
          fi
          cmake --build build -j$(nproc)
        '';

        # Helper: Clean build
        gf-clean = pkgs.writeShellScriptBin "gf-clean" ''
          rm -rf build
          echo "Build directory cleaned"
        '';

        # Helper: Run host-side tests
        gf-test = pkgs.writeShellScriptBin "gf-test" ''
          cd test && cmake -B build -G Ninja && ninja -C build && ctest --test-dir build --output-on-failure
        '';

      in {
        devShells.default = pkgs.mkShell {
          packages = [
            # ARM toolchain
            armToolchain
            armBinutils
            pkgs.gcc-arm-embedded  # Provides arm-none-eabi-gdb

            # Build system
            pkgs.cmake
            pkgs.ninja
            pkgs.gnumake

            # Flashing/debugging
            pkgs.openocd
            pkgs.stlink
            pkgs.dfu-util

            # Serial/debug
            pkgs.picocom
            pkgs.minicom

            # Project tools
            gf-build
            gf-upload
            gf-flash
            gf-dfu
            gf-dfu-list
            gf-debug
            gf-monitor
            gf-cli
            gf-reset
            gf-clean
            gf-test

            # Logic analyzer (helpful for CRSF/SRXL2 debugging)
            pkgs.sigrok-cli
            pkgs.pulseview

            # General utilities
            pkgs.python3  # For various scripts
          ];

          # Set up cross-compilation environment
          shellHook = ''
            export ARM_TOOLCHAIN_PATH="${pkgs.gcc-arm-embedded}/bin"
            export STM32CUBE_PATH="${stm32cubef7}"

            echo ""
            echo "══════════════════════════════════════════════════════════════"
            echo "  GroundFlight - RC Car Gyro Stabilizer"
            echo "  STM32F722 (RadioMaster Nexus) + ICM-42688 + CRSF + SRXL2"
            echo "══════════════════════════════════════════════════════════════"
            echo ""
            echo "  Build & Flash"
            echo "    gf-build [Release|Debug]    Configure & build"
            echo "    gf-upload [file.bin]        Build, reboot to DFU, flash (one shot)"
            echo "    gf-clean                    Clean build directory"
            echo "    gf-test                     Run host-side unit tests"
            echo ""
            echo "  Flash (manual)"
            echo "    gf-dfu [file.bin]           Flash via USB DFU"
            echo "    gf-flash [file.elf]         Flash via ST-Link"
            echo ""
            echo "  Debug"
            echo "    gf-debug                    Start GDB server (:3333)"
            echo "    gf-reset                    Reset target via ST-Link"
            echo "    gf-dfu-list                 List USB DFU devices"
            echo ""
            echo "  Serial"
            echo "    gf-monitor [port] [baud]    Serial monitor (Ctrl-C to exit)"
            echo "    gf-cli [/dev/ttyUSB0]       Connect via picocom (Ctrl-A X)"
            echo ""
            echo "  DFU mode: Hold BOOT button while connecting USB"
            echo ""
            echo "══════════════════════════════════════════════════════════════"
            echo ""
          '';
        };
      };
    };
}
