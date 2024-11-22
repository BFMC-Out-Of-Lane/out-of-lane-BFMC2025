{ pkgs ? import <nixpkgs> {}}:
let
  fhs = pkgs.buildFHSUserEnv {
    name = "bosch-future-24-fhs";

    targetPkgs = _: [
      pkgs.gcc
      pkgs.gcc-arm-embedded-10
      pkgs.micromamba
    ];

    profile = ''
      set -e
      eval "$(micromamba shell hook --shell=posix)"
      export MAMBA_ROOT_PREFIX=${builtins.getEnv "PWD"}/.mamba
      if ! test -d $MAMBA_ROOT_PREFIX/envs/bosch-future-24-mamba; then
          micromamba create --yes -q -n bosch-future-24-mamba
          micromamba activate bosch-future-24-mamba
          micromamba install --yes conda-forge::pip
          micromamba install --yes conda-forge::cmake
      fi
      micromamba activate bosch-future-24-mamba
      pip install -r requirements.txt
      set +e
    '';
  };
in fhs.env