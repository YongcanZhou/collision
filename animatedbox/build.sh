rm -r *

cmake ..

make -j

gazebo ../animated_box.world
