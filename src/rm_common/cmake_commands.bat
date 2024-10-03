sudo /usr/bin/cmake /home/stardust/Code/rm_control_fix/src/rm_common -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=/home/stardust/Code/rm_control_fix/install/rm_common
sudo /usr/bin/cmake --build /home/stardust/Code/rm_control_fix/build/rm_common -- -j8 -l8
sudo /usr/bin/cmake --install /home/stardust/Code/rm_control_fix/build/rm_common