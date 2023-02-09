# KinodynamicFabrics.jl
Julia implementation of the Kinodynamic Fabrics Whole-Body Control framework.

## Installation
1. Clone this repository using the command `git clone -b sim https://github.com/adubredu/KinodynamcFabrics.jl`
2. Navigate to the parent directory of this repo and type  `julia` in your terminal to launch the Julia REPL.
3. Press `]` on your keyboard to enter the package manager 
4. Enter command `instantiate` to download all package dependencies
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