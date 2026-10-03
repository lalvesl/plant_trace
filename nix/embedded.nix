# Tooling for the two microcontrollers and for talking to them from the host.
#
# Split by board because the Xtensa half is installed impurely (see
# `devShells.esp32` in flake.nix) and only the `nrf` half is needed for the
# day-to-day host + nRF52840 work.
pkgs: rec {
  # nRF52840 Supermini: the board has no debug probe, so flashing is UF2 —
  # `rust-objcopy` from cargo-binutils plus `tools/uf2.py`, driven by
  # `tools/flash-uf2.sh`. probe-rs is still here because it is what you reach
  # for the moment you solder to the SWD pads and want the defmt log back.
  nrf = with pkgs; [
    probe-rs-tools # cargo-embed / probe-rs run / probe-rs attach
    flip-link # stack-overflow protection on Cortex-M
    cargo-binutils # size / objdump / objcopy through llvm-tools
    python3 # tools/uf2.py — no packaged nRF UF2 converter exists
  ];
  # Deliberately absent: `adafruit-nrfutil`, the serial-DFU half of the
  # bootloader's interface. It is marked unfree in nixpkgs (it carries Nordic's
  # licensed pieces), and pulling it in would force `allowUnfree` on anyone
  # entering this shell. `tools/flash.sh` uses it when it happens to be on PATH
  # and falls back to UF2, which needs nothing, when it is not.

  # ESP32-WROOM-32 (Xtensa LX6). `espup` fetches the esp-rs rustc fork into
  # ~/.rustup — it cannot be a pure derivation, so the dev shell only provides
  # the installer and the flasher; see docs/HARDWARE.md.
  esp = with pkgs; [
    espup # installs the Xtensa Rust toolchain
    espflash # flash + serial monitor over the CP2102
    ldproxy # linker shim used by esp-idf-sys builds
  ];

  # Host-side: the serial link, plotting for the report, scratch tools for
  # poking at the wire when something looks wrong.
  host = with pkgs; [
    gnuplot # report figures (phase 6)
    picocom # quick manual look at either serial port
    socat # fake/forward serial links while debugging
    usbutils # lsusb, when a board does not enumerate
  ];

  # `serialport` links libudev for port enumeration.
  buildInputs = with pkgs; [ udev ];
  nativeBuildInputs = with pkgs; [ pkg-config ];
}
