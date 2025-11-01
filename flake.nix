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

            # Python for PlatformIO with needed packages
            python3
            python3Packages.pip
            python3Packages.virtualenv

            # Pre-install Python packages that need compilation
            python3Packages.jsonschema
            python3Packages.rpds-py
            python3Packages.attrs
            python3Packages.referencing

            # Rust toolchain (in case compilation is needed)
            rustc
            cargo

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

            # Create and activate Python virtual environment
            if [ ! -d .venv ]; then
              echo "Creating Python virtual environment..."
              python3 -m venv .venv --system-site-packages
            fi
            source .venv/bin/activate

            # Prefer binary wheels over building from source
            export PIP_PREFER_BINARY=1

            echo "PlatformIO development environment loaded"
            echo "Python virtual environment activated: .venv"
            echo "PlatformIO version: $(pio --version)"
            echo "Python version: $(python --version)"
            echo ""
            echo "Available commands:"
            echo "  pio init      - Initialize a new PlatformIO project"
            echo "  pio run       - Build the project"
            echo "  pio run -t upload  - Upload to device"
            echo "  pio device monitor - Open serial monitor"
            echo "  pip install <package>  - Install Python packages in venv"
            echo ""
          '';
        };
      }
    );
}
