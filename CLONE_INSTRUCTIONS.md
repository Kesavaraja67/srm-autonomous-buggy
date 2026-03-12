# ⚠️ READ THIS BEFORE CLONING

## Correct Clone Steps — copy paste exactly:
```bash
mkdir -p ~/srm_buggy_ws/src
cd ~/srm_buggy_ws/src
git clone https://github.com/Kesavaraja67/srm-autonomous-buggy.git .
cd ~/srm_buggy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## ⚠️ The dot at the end of git clone is NOT a typo
It clones contents directly into src/ so colcon can find all packages.
