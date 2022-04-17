rm -r *

cmake ..

make -j

gazebo ../model_move.world
