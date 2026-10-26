{
  description = "probehive: an Arty A7 as a six-target CMSIS-DAP probe for CI, over Ethernet";

  inputs = {
    openxc7.url = "github:openXC7/toolchain-nix";
    nixpkgs.follows = "openxc7/nixpkgs";
    litex = { url = "github:enjoy-digital/litex"; flake = false; };
    liteeth = { url = "github:enjoy-digital/liteeth"; flake = false; };
    litex-boards = { url = "github:litex-hub/litex-boards"; flake = false; };
    # LiteEth's PHY package imports LiteICLink for its transceiver PHYs.
    liteiclink = { url = "github:enjoy-digital/liteiclink"; flake = false; };
    # The CMSIS-DAP interpreter (Amaranth) and the SWD/JTAG engine (Verilog)
    # of the ORBTrace probe, BSD-3-Clause.
    # VexRiscv, the slots' control CPU, as LiteX's pregenerated Verilog
    # (MIT).  The firmware is ours, so no LiteX BIOS, picolibc or
    # compiler-rt data is needed.
    pythondata-cpu-vexriscv = { url = "github:litex-hub/pythondata-cpu-vexriscv"; flake = false; };
    # OpenOCD after 0.12.0, for its CMSIS-DAP backend over TCP.
    openocd-src = { url = "git+https://github.com/openocd-org/openocd?submodules=1"; flake = false; };
  };

  outputs = { self, nixpkgs, openxc7, litex, liteeth, litex-boards, liteiclink,
              pythondata-cpu-vexriscv, openocd-src }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
      toolchain = openxc7.packages.${system};
      # The openXC7 shell's fasm and prjxray tools are built for Python 3.12,
      # so LiteX runs on the same interpreter.
      python = pkgs.python312.withPackages (p: with p; [ migen pyserial requests packaging pyyaml ]);
      # Yosys 0.67 maps a chain of enabled FDREs to an SRL with its enable
      # tied high, so the shift register shifts on every clock: LiteEth's
      # ARP and IP header registers then hold payload bytes and the core
      # answers nothing.  Fixed upstream in 0.68; drop this once the
      # openXC7 shell's Yosys is 0.68 or later.  sim/srl_enable_check.py shows it.
      yosys = pkgs.yosys.overrideAttrs (old: {
        patches = (old.patches or [ ]) ++ [
          (pkgs.fetchpatch {
            name = "xilinx_srl-keep-fdre-clock-enable.patch";
            url = "https://github.com/YosysHQ/yosys/commit/3006280d709c901ff108d94b423b4f7549422edc.patch";
            hash = "sha256-/lXMBpet5QG47UKm4WyF/sQYreQ4tTAh+LEoqWu0jEc=";
          })
        ];
        # Yosys's own suite takes most of an hour; sim/srl_enable_check.py
        # checks the one pass the patch changes.
        doCheck = false;
      });
      # nextpnr routes data nets through an empty BUFGCTRL and leaves that
      # buffer unconfigured, so the loads behind it see a constant: a block
      # RAM byte lane that never wrote, CSR reads with fixed bits set, by
      # placement.  Fixed in openXC7/nextpnr four days after the revision
      # the openXC7 toolchain pins; drop this once its pin includes
      # a17bd29c.  scripts/bufg_check.py finds the defect in a FASM.
      nextpnr = toolchain.nextpnr.overrideAttrs (old: {
        patches = (old.patches or [ ]) ++ [
          (pkgs.fetchpatch {
            name = "xilinx-never-route-through-an-unbound-BUFGCTRL.patch";
            url = "https://github.com/openXC7/nextpnr/commit/a17bd29c1270d2144e1916fced34c2cca55cd782.patch";
            hash = "sha256-YHefEe8AyzlZvmHvFfpp9kjOjfN2ZsNMo/pVWAwFcbc=";
          })
        ];
      });
      # The firmware's compiler: bare metal, for the CPU variant's ISA (RV32IM, RV32I for
      # VexRiscv minimal, where the firmware divides by itself).
      riscv = pkgs.pkgsCross.riscv32-embedded.buildPackages.gcc;
      openocd = pkgs.openocd.overrideAttrs (old: {
        version = "0.12.0-unstable-${openocd-src.lastModifiedDate}";
        src = openocd-src;
        nativeBuildInputs = old.nativeBuildInputs ++ (with pkgs; [ autoconf automake libtool texinfo which ]);
        preConfigure = (old.preConfigure or "") + ''
          SKIP_SUBMODULE=1 ./bootstrap
        '';
      });
      # Vivado, the bitstream's toolchain, is AMD's and is installed by hand (README, "Vivado"):
      # this is the FHS environment it needs on NixOS, and `vivado` on the shell's PATH runs it.
      vivado-fhs = pkgs.buildFHSEnv {
        name = "vivado-fhs";
        targetPkgs = p: with p; [
          bash coreutils gnugrep gnused gawk findutils procps which file
          gnumake gcc binutils unzip zip gzip xz bzip2 gnutar perl python3
          stdenv.cc.cc.lib zlib ncurses5 ncurses libxcrypt-legacy libuuid
          expat dbus glib gtk2 gtk3 cairo pango atk gdk-pixbuf nss nspr alsa-lib
          freetype fontconfig dejavu_fonts pixman libpng
          xorg.libX11 xorg.libXext xorg.libXrender xorg.libXtst xorg.libXi
          xorg.libXft xorg.libXrandr xorg.libXcursor xorg.libXau xorg.libXdmcp
          xorg.libxcb xorg.libXcomposite xorg.libXdamage xorg.libXfixes xorg.libSM xorg.libICE
          libxkbcommon libdrm mesa libGL lsb-release
        ];
        # AMD's FlexLM tools (lmutil, the license check) ask for the LSB loader by name
        extraBuildCommands = ''
          ln -s /lib64/ld-linux-x86-64.so.2 $out/usr/lib64/ld-lsb-x86-64.so.3
        '';
        # ld.so.cache knows libraries by soname, and libncurses.so.5 here is the ABI 5 compat
        # library whose soname is libncursesw.so.5: only a search by file name finds it.
        profile = ''
          export LC_ALL=C.UTF-8
          export LD_LIBRARY_PATH=/usr/lib64''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
        '';
        runScript = "bash";
      };
      vivado = pkgs.writeShellScriptBin "vivado" ''
        root=''${VIVADO_ROOT:-$HOME/opt/Xilinx/2026.1/Vivado}
        lic=''${XILINXD_LICENSE_FILE:-$HOME/.Xilinx/Xilinx.lic}
        [ -f "$root/settings64.sh" ] || { echo "vivado: no Vivado 2026.1 at $root (set VIVADO_ROOT; README, Vivado)" >&2; exit 2; }
        [ -f "$lic" ] || { echo "vivado: no license at $lic (scripts/basic_license.py; README, Vivado)" >&2; exit 2; }
        export XILINXD_LICENSE_FILE=$lic
        exec ${vivado-fhs}/bin/vivado-fhs -c 'source "$1/settings64.sh" >/dev/null && cd "$2" && shift 2 && exec vivado "$@"' \
          vivado-fhs "$root" "$PWD" "$@"
      '';
    in {
      devShells.${system}.default = pkgs.mkShell {
        inputsFrom = [ openxc7.devShell.${system} ];
        # First, so the patched Yosys and nextpnr shadow the openXC7 shell's own.
        packages = [ yosys nextpnr python openocd pkgs.openfpgaloader pkgs.gnumake pkgs.iproute2 pkgs.iverilog riscv vivado vivado-fhs ];
        shellHook = openxc7.devShell.${system}.shellHook + ''

          export CHIPDB=${toolchain.nextpnr-xilinx-chipdb.artix7}
          export PYTHONPATH=${litex}:${liteeth}:${liteiclink}:${litex-boards}:${pythondata-cpu-vexriscv}:$PYTHONPATH
        '';
      };
    };
}
