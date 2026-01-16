This repository implements algorithms for solving the exact cover problem efficiently. The key components include: Dynamic generating connected components based on SplayETT and Parallel search.

## Build Instructions

System requirements: `Ubuntu22.04`

Environment requirements: `GCC 11.4 or higher, G++ 11.4 or higher, CMake 3.22.1 or higher, OpenMP 4.0 or higher`

To check whether OpenMP is available and verify its version, run:
```bash
echo | g++ -fopenmp -dM -E - | grep _OPENMP
```

The output is an integer macro indicating the supported OpenMP version.
For example: _OPENMP 201511 → OpenMP 4.5

To install all required dependencies, run:
```bash
sudo apt update
sudo apt install -y build-essential cmake
```

Install dxd from https://github.com/loiufam/alg-lab


### Compile and Run the code with the following command:

To compile the solver:
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug .. # or `cmake -DCMAKE_BUILD_TYPE=Release ..` 
make -j
```

To run the solver, you can use the script "main" in this
directory with the following arguments:
```bash
./main <alg_name> <test_case_path> <read_mode> [use_ett_or_not] [thread_num]
```

## Arguments

- **`alg_name`**  
  Specifies the algorithm to be used. Supported options include:
  - `dxd`: the single-threaded DXD algorithm (single-thread execution by default).
  - `mdxd`: the multi-threaded DXD algorithm (parallel execution by default).

- **`test_case_path`**  
  The path to the input test case file.

- **`read_mode`**  
  Specifies the input format or reading mode.
  - Default value: `3`for test cases located in the `run_set` directory.
  - Use `1` for test cases located in the `exact_cover_benchmark` directory.

- **`use_ett_or_not`** *(optional)*  
  Indicates whether Euler Tour Trees (ETT) are used for connected component generation.
  - `ett`: enable ETT-based connected component maintenance.
  - `no-ett` or omitted: disable ETT and use the default method.

- **`thread_num`** *(optional)*  
  Specifies the number of threads to be used during execution.  
  This parameter is effective only for multi-threaded configurations (e.g., `mdxd`).


For example:
```bash
./main mdxd ../data/runset/Aarnet.txt 3 ett 8 # run a benchmark
```

## Benchmarks

We use two types of exact cover instance datasets.
One is the exact cover benchmark dataset in the `exact_cover_benchmark` directory, and the other is the graph benchmark dataset in the `run_set` directory.

All datasets are stored in the `benchmark` folder.
