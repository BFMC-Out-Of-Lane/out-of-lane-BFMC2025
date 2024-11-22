# Out of Lane BFMC 2025
Repository for our team's project for the Bosch Future Mobility Challenge 2025.

## Usage
### NixOS
1. Install micromamba on your NixOS machine
2. Execute `nix-shell` in order to create the enviroment declared in ./shell.nix
3. 

1. Install conda on your NixOS machine
2. Initialize conda in shell
3. Create enviroment
4. Activate it
5. Run:
- `conda install conda-forge::libglib`
- `conda install conda-forge::numpy`
- `conda install pytorch torchvision torchaudio cpuonly -c pytorch`
6. Run `pip install -r requirements.txt`


