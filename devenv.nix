{ pkgs, ... }: {
    
    packages = [
        pkgs.gcc
        pkgs.gcc-arm-embedded-10 # cross-compiler for the embedded platform
        pkgs.cmake # build system for the embedded platform
    ];

    enterShell = ''
    	conda-shell
    '';
}