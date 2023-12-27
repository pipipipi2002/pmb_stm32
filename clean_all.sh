#!/bin/zsh
echo "Cleaning all make artifacts"

cd ./bootloader
make clean
cd ../app
make clean

echo "Finished"
cd ..



