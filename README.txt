create venv

python -m venv <name>


add this to bashrc

```
alias src-ros2='cd && source ~/isopod-explorer/envs/isopod/bin/activate && source /opt/ros/humble/setup.bash && cd ~/isopod-explorer/ros2_ws && source install/setup.bash && source /usr/share/colcon_cd/function/colcon_cd.sh && export _colcon_cd_root=/opt/ros/humble/ && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && export QT_QPA_PLATFORMTHEME=qt5ct'
```
