# Installation
## Windows

- Install UnrealEngine via [Epic Games Launcher (v > 4.16)](https://www.unrealengine.com/download)
- Install [cmake](https://cmake.org/files/v3.11/cmake-3.11.0-rc4-win64-x64.msi)
- Install [Visual Studio](https://www.visualstudio.com/downloads/)
- Install [Windows 8.1 SDK](https://go.microsoft.com/fwlink/p/?LinkId=323507)

- Clone this repo including submodules with:
 ```
 git clone --recursive https://github.com/tudelft/dronerace2018.git 
 ```
  Sometimes the submodules are not cloned correctly. You can tell that if the folder in target/simulator/AirSim is empty. In that case get the submodules with:
```
 git submodule update --init --recursive
```

- Open the solution file in Visual Studio. If it wants to upgrade the projects select *yes*.

- Open developers command prompt and execute AirSim/build.cmd. If prompted with: 
```
Starting cmake to build rpclib...
CMake Error at CMakeLists.txt:2 (project):
  Failed to run MSBuild command:

    MSBuild.exe

  to get the value of VCTargetsPath:

    The system cannot find the file specified
```
the script is looking for the wrong version of visual studio. 
	- Open AirSim/build.bat and change line 60 to your version (e.g. "Visual Studio 15 2017 Win64"). 
	- Remove the directory AirSim/external/rpclib/rpclib-*-*/build and rerun AirSim/build.cmd.

- To test whether everything is properly build you can test the simulator in the Blocks environment:
	- Go to AirSim/Unreal/Environments/Blocks and run "update_from_git.cmd"
	- Sometimes the UnrealEngine is not registered properly. In that case the command above fails complaining about a key not found in the registry. To fix that go to the folder of your *Epic Games Launcher* and look for "Epic Games\Launcher\Engine\Binaries\Win64\VersionSelector.exe" copy that file to the installation path of the *UnrealEngine*, paste it in "Epic Games\UE_4.19\Engine\Binaries\Win64" and run it. It should conclude with "Registration successfull". After that the above should work.
	- Open the created .sln file, hit "run" and pray.
	- The UnrealEditor should show up and "Play" should start the simulation. The drone can only be flewn with a proper controller. However the car can be tested with arrow keys.

# Custom Environments
To run a custom environment make sure you copied the "AirSim/Unreal/Plugins" to the respective project folder. 
