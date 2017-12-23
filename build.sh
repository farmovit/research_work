!#bin/bash

path2build="build"
[ ! -d $path2build ] && mkdir $path2build

cd $path2build

cmake -G "MSYS Makefiles" ..

make