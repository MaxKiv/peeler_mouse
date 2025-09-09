{
    description = "ESP-32 Rust dev environment";

    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
        flake-utils.url = "github:numtide/flake-utils";
    };

    outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
        let
            pkgs = import nixpkgs { inherit system; };
        in {
            devShells.default = pkgs.mkShell {
                # packages to install
                buildInputs = with pkgs; [
                    bashInteractive # fixes console in vscode

                    cargo-generate # generate rust projects from github templates
                    cargo-udeps # find unused dependencies in Cargo.toml

                    # required for esp development
                    espup       # tool for installing esp-rs toolchain
                    rustup      # rust installer, required by espup
                    espflash    # flash binary to esp
                    python3
                ];

                # execute some commands before environment is accessible
                shellHook = ''
                    echo -e "\e[1mInstalling toolchains for esp"
                    echo -e "-----------------------------\e[0m"
                    espup install
                    . ~/export-esp.sh

                    echo
                    echo -e "\e[1mInstalling ldproxy"
                    echo -e "------------------\e[0m"
                    cargo install ldproxy
                '';

                # LIBCLANG_PATH="$HOME/.espup/esp-clang";

                NIX_LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
                  pkgs.stdenv.cc.cc.lib   # libstdc++.so.6, libgcc_s.so.1
                  pkgs.zlib               # libz.so.1
                  pkgs.libxml2            # libxml2.so.2
                  pkgs.ncurses            # often needed by esp-idf tools
                ];
                NIX_LD =with pkgs; lib.fileContents "${stdenv.cc}/nix-support/dynamic-linker";
                # NIX_LD = (pkgs.runCommand "ld.so" {} ''
                #     ln -s "$(cat '${pkgs.stdenv.cc}/nix-support/dynamic-linker')" $out
                # '');
            };
        }
    );

    # use prebuilt binaries
    nixConfig = {
        extra-substituters = [
            "https://nix-community.cachix.org"
            "https://cache.nixos.org/"
        ];
        extra-trusted-public-keys = [
            "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
        ];
    };
}
