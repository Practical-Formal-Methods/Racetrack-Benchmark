# Racetrack Benchmark
We created this repository to present benchmarks that are used to evaluate `Neuro-Aware Program Analysis` introduced in our CAV 2021 paper - "Automated Safety Verification of Programs Invoking Neural Networks". For a complete working environment and reproducing exact results in the paper please refer to [Neuro-Aware Program Analysis](https://github.com/Practical-Formal-Methods/clam-racetrack) repository.

## Project Structure

### Agents

This folder contains neural networks trained to operate on various maps. A neural network takes the current state of the agent as an input and predicts the direction of acceleration towards the goal states. We use _Tensorflow_ framework for training. Naming scheme of the neural network models describe the map that the network trained on. In our paper, we present neural networks of various quality, however, here we provide only `GOOD` agents for inspection purposes. For all other agents, plesae refer to [Neuro-Aware Program Analysis](https://github.com/Practical-Formal-Methods/clam-racetrack) repository.

### Source
We present the `C` program used in evalaution in this folder. This program invokes a neural network to obtain predicted acceleration. Later, the accelaration is processed and the agent is moved over the map. Note that, the noise definitions in the paper are actualized in outside of these source files. Also, in the paper, we consider several parameters for `Lookahead` functionality, this program refer to `LA=3` variant. Noise implementations and other `Lookahead` variants can be seen in [Neuro-Aware Program Analysis](https://github.com/Practical-Formal-Methods/clam-racetrack) repository.

### Include

This folder contains header files.

### Map

This folder involves visuals of maps with changing difficulty that the neural networks are trained on.

## Running
In `include/maps.h` file, uncomment the map array that you would like to use and update the path to the neural network model in `main` function in `src/main.c`.

### Build
```shell
$ make
```

Note that, building this project requires a proper installation of `Tensorflow C API`. For a proper usage, either it has to be installed in `/usr/local` or the path to the installation should be added to environment variables `LIBRARY_PATH`, `LD_LIBRARY_PATH` and `C_INCLUDE_PATH`.

### Run
```shell
$ ./bin/racetrack-controllers
```
