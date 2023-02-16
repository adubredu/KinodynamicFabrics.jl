# KinodynamicFabrics.jl
Julia implementation of the Kinodynamic Fabrics Whole-Body Control framework.

Project webpage: [adubredu.github.io/kinofabs](http://adubredu.github.io/kinofabs)


![](media/trailer.gif)


## Installation
1. Clone this repository using the command `git clone -b sim https://github.com/adubredu/KinodynamcFabrics.jl`
2. Navigate to the parent directory of this repo and type  `julia` in your terminal to launch the Julia REPL.
3. Press `]` on your keyboard to enter the package manager 
4. Enter command `activate .` then enter command `instantiate` to download all package dependencies
5. Once installation is complete, press the `Backspace` key on your keyboard to return to the REPL

## Usage
To run the examples in the [examples](examples) folder, run the following command
```
include("examples/dodge_cannonball.jl")
```

or

```
include("examples/dodge_cannonball_maintain_ee.jl")
```
**Note:** The initial run of either example may have a delayed startup of the *mujoco-python-viewer*. If the interface asks, click `Wait`, and it may take up to 30 seconds to run. After the first run you should not longer have this delay. 

## Citing
```
@article{adubredugibson2023,
    title={Kinodynamic Fabrics for Reactive Whole-Body Control of Bipedal Humanoid robots},
    author={Adu-Bredu, Alphonsus and Gibson, Grant and Jenkins, Odest Chadwicke and Grizzle, Jessy},
    journal={},
    url={},
    year={2023}
}
```
