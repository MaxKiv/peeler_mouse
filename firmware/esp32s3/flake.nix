{
  description = "ESP32-S3 Rust dev environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          # git
          # cmake
          # ninja
          # python3
          # python3Packages.virtualenv
          # llvm
          # clang
          rustup
          ldproxy
          cargo-espflash
          # gcc
          # gperf
          # ccache
          dfu-util
          minicom
          # for flashing
          esptool
          espup
        ];

       # for debugging with lsp in neovim
        CODE_LLDB_PATH = "${pkgs.vscode-extensions.vadimcn.vscode-lldb}/share/vscode/extensions/vadimcn.vscode-lldb/adapter/codelldb";
        LIB_LLDB_PATH = "${pkgs.vscode-extensions.vadimcn.vscode-lldb}/share/vscode/extensions/vadimcn.vscode-lldb/lldb/lib/liblldb";


        shellHook = ''
          echo "Setting up ESP environment"
          # to prevent infinite recursion of activating rust environment
          # see .envrc for activation condition
          export INSIDE_RUST_ENV=1

          # To fix the issue with libiconv not found
          # export LIBRARY_PATH=$LIBRARY_PATH:$(brew --prefix)/opt/libiconv/lib

          set -e # stops this on error of any command below

          # run espup to install the esp toolchain for this command to work
          . ~/export-esp.sh

          # ln -sf ~/.rustup/toolchains/nightly-aarch64-apple-darwin/bin/rust-analyzer ~/.rustup/toolchains/esp/bin/rust-analyzer
        '';

      };
    };
}

