# Tic-Tac-Toe: Human vs Robot

## Usage

```bash
# Git
git clone git@github.com:AbdulShahzeb/mtrn4231-project.git
cd mtrn4231-project

# (Optional) Training Tic-Tac-Toe agents
python3 scripts/train_menace.py --num-games 100000 --output-dir models/ # Adjust num-games as desired

# Bulding
colcon build --symlink-install
source install/setup.bash

# Running
ros2 launch brain tictactoe.launch.py                               # Default: Play human vs. robot as X
ros2 launch brain tictactoe.launch.py player:=o                     # Play human vs. robot as O
ros2 launch brain tictactoe.launch.py player:=x gamemode:=robot     # Play robot vs. robot as X
ros2 launch brain tictactoe.launch.py player:=x gamemode:=human     # Play human vs. robot as X (same as default)
```

## Requirements
1. ROS 2 Humble
2. Python 3.x
3. `pip install -r requirements.txt`