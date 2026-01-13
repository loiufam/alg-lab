This repository implements algorithms for solving the exact cover problem efficiently. The key components include:

Incrementally decompose matrix;

Parallel search;

ETT structures for computing graph connectivity

Together, these modules accelerate the solving of exact cover instances by exploiting structural decompositions and concurrency.

## Building

### Requirements

This code uses CMake version 3.22.1 and g++ version 11.4.0 with openmp library on Linux.

### Commands

Run the following shell commands:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug .. # or `cmake -DCMAKE_BUILD_TYPE=Release ..` 
make -j

./main mdxd ../data/runset/Aarnet.txt 3 ett # run a benchmark
```