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

### Import "unitree_sdk2py.core.channel" could not be resolved 

```zsh
uv pip install . # don't install in editable mode
```
https://github.com/unitreerobotics/unitree_mujoco/issues/110

