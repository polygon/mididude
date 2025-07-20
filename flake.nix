{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, rust-overlay, nixpkgs }:
    let
      system = "x86_64-linux";
      overlays = [ (import rust-overlay) ];
      pkgs = import nixpkgs { inherit overlays system; };
    in {
      devShells.${system} = {
        board = pkgs.mkShell {
          buildInputs = [
            pkgs.kicad-small
          ];
        };
        native = pkgs.mkShell {
          buildInputs = [
            pkgs.rust-bin.nightly.latest.default
            pkgs.rustfmt
          ];
        };
        rp-pico = pkgs.mkShell {
          buildInputs = [
            (pkgs.rust-bin.selectLatestNightlyWith (toolchain:
              toolchain.default.override {
                targets = [ "thumbv6m-none-eabi" ];
                extensions = [ "rust-src" ];
              }))
            pkgs.rust-analyzer
            pkgs.flip-link
            pkgs.probe-rs
            pkgs.elf2uf2-rs
            pkgs.rustfmt
            pkgs.pkg-config
            pkgs.openssl
            pkgs.kicad-small
          ];
        };
        ch32 = let
          rust-bin = (pkgs.rust-bin.selectLatestNightlyWith (toolchain:
            toolchain.default.override {
              targets = [ ];
              extensions = [ "rust-src" ];
            }));
          rust-platform = pkgs.makeRustPlatform {
            cargo = rust-bin;
            rustc = rust-bin;
          };
          deps = [
            pkgs.cargo-bloat
            pkgs.rust-analyzer
            pkgs.gdb
            pkgs.minicom
            #pkgs.flip-link
            #pkgs.probe-rs
            pkgs.rustfmt
            #pkgs.pkg-config
            #pkgs.openssl
            (pkgs.python3.withPackages
              (ps: [ ps.ipython ps.numpy ps.matplotlib ps.jupyter ]))
          ];

        in pkgs.mkShell { buildInputs = deps ++ [ rust-bin ]; };
      };
    };
}
