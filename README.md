Pathfinder
====

[![Azure Status](https://dev.azure.com/grapplerobotics/Pathfinder/_apis/build/status/GrappleRobotics.Pathfinder?branchName=master)](https://dev.azure.com/grapplerobotics/Pathfinder/_build/latest?definitionId=1)


Pathfinder is a robotic motion library, with logic for motion profiling, path planning and more. 

Pathfinder is still under heavy development, and the structure is still very volatile, please see the issues tab if you'd like to contribute to making Pathfinder great.

## Cloning (IMPORTANT!)
If you wish to clone pathfinder, ensure you do so with `--recurse-submodules`. Under `libs/`, we have a number of submodules (including Eigen and GoogleTest) which are used in the library.

Eigen is the only runtime dependency. All other dependencies are used exclusively during unit testing.

## Building Pathfinder
The requirements to build Pathfinder are as follows:
- A C++ Compiler (C++14 or later). Clang is preferred over GCC as our testing system includes the use of the Clang address sanitizer  
- A Java Compiler (JDK 9+ recommended).  
- GNUPlot Recommended. After unit testing, gnuplot is automatically run to generate graphs of the simulated test cases.  

If you wish, you can run a build with benchmarks by passing `-PwithBench` to your gradle command.

## A note on formatting
For C++ files, we include a .clang-format file in the project root. Please ensure all files are formatted using clang-format (or through your IDE if it supports the use of .clang-format) before committing.
