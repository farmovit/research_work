@echo off

set path2build=build

if not exist %path2build% (
	mkdir %path2build%
)

cd %path2build%

cmake -G "MSYS Makefiles" ..
mingw32-make -j 8