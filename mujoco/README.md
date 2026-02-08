## Dependencies
**unitree_sdk2_python**

```sh
# first cd to your workspace and create virtual environment

sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
reference: https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#python-simulator-simulate_python

## Run Mujoco

```sh
cd mujoco
python ik_mujoco_test.py
```

## Troubleshooting
### Mujoco Python Binding Issue
https://github.com/google-deepmind/mujoco/issues/1292

### Could not locate cyclonedds

If `pip3 install -e .` fails with `Could not locate cyclonedds`, follow the
official instructions from `unitree_sdk2_python`:

```sh
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

Then:

```sh
cd ~/unitree_sdk2_python
export CYCLONEDDS_HOME="~/cyclonedds/install"
pip3 install -e .
```

For details, see: https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries

