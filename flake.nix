{
  description = "plant-trace — time- and frequency-domain characterisation of a steam-turbine plant";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      rust-overlay,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        # ── shared modules (nix/) ─────────────────────────────────────────────
        pkgs = import ./nix/pkgs.nix { inherit nixpkgs system rust-overlay; };

        inherit
          (import ./nix/rust.nix {
            inherit pkgs;
            toolchainFile = ./rust-toolchain.toml;
          })
          rustToolchain
          rustPlatform
          ;

        embedded = import ./nix/embedded.nix pkgs;

        # FHS sandbox for the Xtensa toolchain. `targetPkgs` is everything the
        # esp-rs binaries link against plus the tools used alongside them; the
        # profile puts the installed toolchain on PATH and sources the export
        # file `espup` writes (LIBCLANG_PATH and the xtensa-esp-elf binaries).
        esp32Fhs = pkgs.buildFHSEnv {
          name = "plant-trace-esp32";
          targetPkgs =
            p:
            (with p; [
              # runtime of the prebuilt rustc/LLVM
              zlib
              libxml2
              openssl
              stdenv.cc.cc.lib
              # the tooling around it
              espup
              espflash
              ldproxy
              pkg-config
              udev
              # espup's own fetch-and-unpack path
              cacert
              curl
              gnutar
              xz
              git
              gnumake
            ])
            ++ embedded.host;
          profile = ''
            export ESP_TOOLCHAIN="$HOME/.rustup/toolchains/esp"
            if [ -d "$ESP_TOOLCHAIN/bin" ]; then
              export PATH="$ESP_TOOLCHAIN/bin:$PATH"
              [ -f "$PWD/.esp-env.sh" ] && . "$PWD/.esp-env.sh"
              echo "Xtensa toolchain: $(rustc --version 2>/dev/null || echo 'not runnable')"
            else
              echo "Xtensa toolchain not installed yet. Run once, inside this shell:"
              echo "  espup install --targets esp32 --export-file \"$PWD/.esp-env.sh\""
            fi
            echo "  flash: cd firmware/esp32-sig && cargo run --release"
          '';
          runScript = "bash";
        };

        # Shared by every shell: the pinned toolchain plus the crates the host
        # CLI links against.
        commonNativeBuildInputs = [
          rustToolchain
          pkgs.cargo-nextest
          pkgs.taplo
          pkgs.nixfmt
          pkgs.ngspice
        ]
        ++ embedded.nativeBuildInputs;

        # The GUI (`plant-trace gui`): eframe links X11/Wayland/GL at run time,
        # and egui_shadcn's build script fetches its icon font over native-tls,
        # which needs OpenSSL through pkg-config at build time.
        guiLibs = with pkgs; [
          libxkbcommon
          libGL
          wayland
          libx11
          libxcursor
          libxrandr
          libxi
          fontconfig
        ];
        guiBuildInputs = guiLibs ++ [ pkgs.openssl ];

        rcFilterPython = pkgs.python3.withPackages (
          ps: with ps; [
            numpy
            matplotlib
          ]
        );

        rcFilterApp = pkgs.writeShellApplication {
          name = "rc_filter";
          runtimeInputs = [
            pkgs.ngspice
            rcFilterPython
          ];
          text = ''
            exec python3 ${./sim/rc_filter/run_filter_tests.py} "$@"
          '';
        };
      in
      {
        # ── packages ──────────────────────────────────────────────────────────
        #
        # Only the host CLI is packaged: firmware images are flashed from the
        # dev shell against a board that is physically present, so a derivation
        # for them would be a build no one consumes.
        packages.default = self.packages.${system}.plant-trace;

        packages.plant-trace = rustPlatform.buildRustPackage {
          pname = "plant-trace";
          version = "0.1.0";
          src = ./.;

          cargoLock.lockFile = ./Cargo.lock;
          cargoBuildFlags = [
            "-p"
            "plant-trace"
          ];

          nativeBuildInputs = embedded.nativeBuildInputs;
          buildInputs = embedded.buildInputs;

          meta = {
            description = "Acquisition, orchestration and identification CLI for the plant-trace rig";
            mainProgram = "plant-trace";
          };
        };

        packages.rc_filter = rcFilterApp;

        # ── dev shells ────────────────────────────────────────────────────────
        #
        # `default` covers the host CLI and the nRF52840 firmware: one pinned
        # stable toolchain, thumbv7em target included, probe-rs for flash+RTT.
        devShells.default = pkgs.mkShell {
          nativeBuildInputs = commonNativeBuildInputs ++ embedded.nrf ++ embedded.host;
          buildInputs = embedded.buildInputs ++ guiBuildInputs;
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath guiLibs;

          shellHook = ''
            export PKG_CONFIG_PATH="${pkgs.udev.dev}/lib/pkgconfig''${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
            echo "plant-trace dev shell"
            echo "  host:   cargo run -p plant-trace -- --help"
            echo "  gui:    cargo run -p plant-trace -- gui"
            echo "  rig:    cd firmware/nrf-daq && cargo run --release"
            echo "  filter: nix run .#rc_filter"
            echo "  ESP32:  nix develop .#esp32   (spare generator, Xtensa toolchain)"
          '';
        };

        # `esp32` is a second shell because the Xtensa target needs the
        # esp-rs fork of rustc, which is not in nixpkgs and cannot be built
        # purely here. `espup` fetches it into ~/.rustup as generic Linux
        # binaries — and NixOS has no `/lib64/ld-linux-x86-64.so.2`, so those
        # binaries only run inside an FHS sandbox. That sandbox is this shell.
        #
        # The ESP32 no longer drives the plant — the nRF's PWM pairs do — but
        # `firmware/esp32-sig` is kept building, and this is what builds it.
        devShells.esp32 = esp32Fhs.env;

        # ── apps ──────────────────────────────────────────────────────────────
        # Same sandbox, as a one-shot command: `nix run .#esp32` drops into it
        # even where `nix develop` cannot set one up.
        apps.esp32 = {
          type = "app";
          program = "${esp32Fhs}/bin/plant-trace-esp32";
        };

        apps.rc_filter = {
          type = "app";
          program = "${rcFilterApp}/bin/rc_filter";
        };

        apps.fmt = {
          type = "app";
          program = "${
            pkgs.writeShellApplication {
              name = "fmt";
              runtimeInputs = [
                rustToolchain
                pkgs.nixfmt
                pkgs.taplo
              ];
              text = ''
                set -euo pipefail
                nixfmt .
                taplo fmt
                cargo fmt --all
                for ws in firmware/nrf-daq firmware/esp32-sig; do
                  ( cd "$ws" && cargo fmt --all )
                done
              '';
            }
          }/bin/fmt";
        };

        formatter = pkgs.nixfmt;
      }
    );
}
