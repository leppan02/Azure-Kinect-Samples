### Linux

For building with Linux, CMake is used. Build from a `git clone`, we do not support building from a ZIP file.

Install the prerequisites
```
apt install libxi-dev
```

From the root of the git project
```
mkdir build
cd build
cmake .. -GNinja
ninja
```
'
