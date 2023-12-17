#!/bin/zsh
echo "Buidling full package"

echo "Building Bootloader"
cd ./bootloader
make clean && make

echo "Building Main Application"
cd ../app
make clean && make

echo "Finished"
cd ..



