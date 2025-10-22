{
  description = "PlatformIO development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            # PlatformIO
            platformio
            platformio-core

            # Python for PlatformIO
            python3
            python3Packages.pip
            python3Packages.virtualenv

            # Build tools
            gcc
            gnumake
            cmake

            # Serial communication
            picocom
            minicom

            # USB access (for programming devices)
            libusb1
            pkg-config
          ];

          shellHook = ''
            # Set PlatformIO core directory to project-local directory
            export PLATFORMIO_CORE_DIR=$PWD/.nix-platformio

            echo "PlatformIO development environment loaded"
            echo "PlatformIO version: $(pio --version)"
            echo ""
            echo "Available commands:"
            echo "  pio init      - Initialize a new PlatformIO project"
            echo "  pio run       - Build the project"
            echo "  pio run -t upload  - Upload to device"
            echo "  pio device monitor - Open serial monitor"
            echo ""
          '';
        };
      }
    );
}
