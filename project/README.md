# dronerace2018
Drone Race 2018

## Overview 
```
|-- doc:            Some diagrams
|-- innerloop:      low level control loop of the drone
|-- outerloop:      high level control loop of the drone
|-- lib:            Libraries like opencv, eigen
|-- target:         the project files depending on where the code shall run
   |-- test:        Projects to run the code in a terminal with handcrafted/file input
      |-- examples: Example projects for testing
   |-- simulator:   Projects to run the code in the simulator
      |-- examples: Example projects for simulation
   |-- jevois:      Projects to run the code on the jevois
```
## Installation

###  Tools
We use [cmake](https://cmake.org/files/v3.11/cmake-3.11.0-rc4-win64-x64.msi) as build tool. It can be integrated to almost every IDE so it shouldn't matter if you use Visual Studio/CLion/Codeblocks/Make etc. However, so far its only tested with VisualStudio and that one is also needed if you wanna create environments for the simulator.

### Submodules
To make sure you have all submodules run ```git submodule update  --init --recursive```

### Build the libraries:
- OpenCV: 
  * Create the folder "build" in lib/opencv/
  * Execute ```cmake -G "Visual Studio 15 2017 Win64" ..``` 
  (If you use another compiler look for the generator you need. You can also type ```cmake ..``` and hope it will detect         it automatically.)
  * Execute ```cmake --build .```
  * Get a coffee
  * If you are on Windows adapt your PATH variable. Extend it to the path lib/opencv/build/bin/Debug
- AirSim:
  * see [here](target/simulator/README.md)
  
### Verfiy installation:
  - Go to target/test/loop
  - create a folder "build"
  - Execute ```cmake -G "Visual Studio 15 2017 Win64" .. ```(If you use another compiler look for the generator you need)
  - Execute ```cmake --build .```
  - Go to target/test/loop/bin/Debug
  - Run loop_test.exe
  
  
  ## Example Workflow
  Assuming you want to add an average filter to the filter module:
  1. Create a new branch called "average_filter" ```git checkout -b branch average_filter``` 
  2. Create a new project that includes the other sources by copying the folder "loop" in target/test and renaming it to "AverageFilter".
  3. Open the cmake file with Visual Studio or another IDE.
  4. Rename the project() in the cmake file.
  5. In the folder outerloop/src/filtering add a class called "AverageFilter" and add it to the cmake file in outerloop/CMakeList.txt
  6. Adapt the main function of your project such that it creates an object of your class and runs the code you implement.
  7. Throughout the process add and commit your changes regularly. 
  ```git add outerloop/src/AwesomeAverageFilter.cpp outerloop/src/AwesomeAverageFilter.h target/test/AverageFilter/CMakeLists.txt target/test/AverageFilter/src/main.cpp```
  ```git commit -m "added average filter"```
  8. Implement your code and test whether it works.
  9. Potentially also create a project in target/simulator by copying the "loop" folder. If step 5 succeeded you should see your new source files as well.
  10. Test your code in the simulation.
  11. Integrate your code to the rest of the outerloop.
  12. Make sure everything works by compiling and running target/test/loop and target/simulator/loop.
  13. Commit everything you added ```git commit -m "average filter integrated"``` 
  14. Push your branch ```git push origin average_filter```
  15. Go to Github and submit a pull request with your changes.
  16. See what other features we need and start again with 1. :).
  
