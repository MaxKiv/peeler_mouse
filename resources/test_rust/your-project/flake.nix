{
  description = "Ledka";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    esp-dev = {
      url = "github:mirrexagon/nixpkgs-esp-dev";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs@{ flake-parts, esp-dev, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" "aarch64-darwin" ];
      perSystem = { config, self', inputs', pkgs, system, ... }:
        {
          devShells.build = pkgs.mkShell {
            IDF_CCACHE_ENABLE = 0;
            hardeningDisable = [ "all" ];
            buildInputs = [
              esp-dev.packages.${system}.esp-idf-full
              pkgs.esptool-ck
              # pkgs.ccache
            ];
          };
          devShells.default = pkgs.mkShell {
            hardeningDisable = [ "all" ];
            buildInputs = [
              # C
              pkgs.bear
              pkgs.clang-tools
              pkgs.cmake-format

              # Tools
              pkgs.picocom
            ];
          };
        };
    };
}
