# Geometries

A collection of python modules for handling pose (position/orientation), transformations and point clouds

## Installation
In order for the code to work properly, the following steps are required
1. Install correct version of python
2. Install geometries using Poetry

### 1. Setup Python environment

#### Deadsnakes PPA (requires sudo access)
1. On ubuntu systems, start by adding the deadsnakes PPA to add the required version of python.
```
sudo add-apt-repository ppa:deadsnakes/ppa
```

2. Update the package list
```
sudo apt update
```

3. Install python 3.10 along with the required development dependencies
```
sudo apt install python3.10 python3.10-dev
```

The following resources may be helpful [Deadsnakes PPA description](https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa), [Tutorial on Deadsnakes on Ubuntu](https://preocts.github.io/python/20221230-deadsnakes/)

#### Conda (Backup)
1. If conda isn't already installed, follow the [Conda Install Instructions](https://conda.io/projects/conda/en/stable/user-guide/install/index.html) to install conda
2. Use the following command to download the conda installation (for linux)
```
wget https://repo.anaconda.com/archive/Anaconda3-2023.09-0-Linux-x86_64.sh
```
3. Run the conda installation script (-b for auto accepting the license)
```
bash Anaconda3-2023.09-0-Linux-x86_64.sh -b
```
3. Once conda is installed, create a new conda environment with the correct version of python
```
conda create -n geometries python=3.10
```

### 2. Clone geometries
```
git clone https://github.com/davidmhunt/geometries.git
```
### 3. Install geometries using Poetry

#### Installing Poetry:
 
1. Check to see if Python Poetry is installed. If the below command is successful, poetry is installed move on to setting up the conda environment

```
    poetry --version
```
2. If Python Poetry is not installed, follow the [Poetry Install Instructions](https://python-poetry.org/docs/#installing-with-the-official-installer). On linux, Poetry can be installed using the following command:
```
curl -sSL https://install.python-poetry.org | python3 -
```

If you are using poetry over an ssh connection or get an error in the following steps, try running the following command first and then continuing with the remainder fo the installation.
```
export PYTHON_KEYRING_BACKEND=keyring.backends.null.Keyring
```

This project requires at least python 3.10 to run. In order to ensure that poetry uses the correct version of python, execute the following command in the terminal
```
poetry env use python3.10
```
### Installing geometries
Navigate to the odometry foler (this folder) and execute the following command

```
cd odometries
poetry install
```

#### Updating geometries
If the pyproject.toml file is updated, the poetry installation must also be updated. Use the following commands to update the version of poetry
```
poetry lock --no-update
poetry install
```