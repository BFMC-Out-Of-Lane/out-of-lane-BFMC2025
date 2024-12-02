CONDA_ENV_NAME='out-of-lane-bfmc2025'

alias c='clear'

if test $(conda env list | grep -q $CONDA_ENV_NAME); then
    echo "Creating conda environment $CONDA_ENV_NAME"
    conda create -n $CONDA_ENV_NAME python=3.9
    conda activate $CONDA_ENV_NAME
    conda install --yes conda-forge::libglib
    conda install --yes pytorch torchvision torchaudio cpuonly -c pytorch
    pip install -r requirements.txt
fi

conda activate $CONDA_ENV_NAME