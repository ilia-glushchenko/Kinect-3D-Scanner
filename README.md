# What is it?
It's a small room scanner that can use first generation Kinect. Written with PCL and Qt in C++.

PS:
This is my reanimated bachelor project. Don't judge the code quality :)

# What do I need to build the scanner?
It's not that hard I promise. 

1) You'll need Windows 10 PC (It might work on Linux and macOS in principle, cause CMake and VCPKG are cross-platform. But I did not try it yet. Sorry).
2) You'll need to install [VCPKG](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg). Just follow the official guide, it's pretty good. It's a Microsoft C++ package manager. This is a lot better for complex libraries such as PCL, Qt, etc, since you don't have to worry too much about building them. (Been there, done that, don't want to do it ever again). `vcpkg` contains all of the dependencies for the scanner. 
3) You'll also need to install [CMake](https://cmake.org/). Which is a build automation system for the project.
4) You also might want to install OpenNI2 SDK and Kinect1.8 SDK. They are supposed to be installed automatically by the `vcpkg` but just in case. Here is [OpenNI2](https://structure.io/openni) and here is [Kinect1.8](https://www.microsoft.com/en-us/download/details.aspx?id=40278).
5) You'll need Visual Studio 2019. Since that's the one I tested it on.
6) The project depends on `qt5` `qt5 serial port` `vtk` `pcl` `openni2` `boost` `eigen` `flann` `qhull` `opencv-2.4.10` `aruco-1.2.5` `cpu_tsdf`. My point here is, that it's a lot of libraries. Most of them are quite heavy. Be prepared to spend some quality time building the project. 
7) **First generation Kinect**. This is the hardware this project is written to work with. 

# How to build it?
I use git bash and gonna write all of the commands in it.
1) `git clone https://github.com/ilia-glushchenko/Kinect-3D-Scanner.git --recursive`
2) `cd Kinect-3D-Scanner`
3) `mkdir build && cd build`
4) `cmake ..`
5) `explorer .`
6) Open `RoomScanner.sln`
7) Build as usual in Visual Studio.
8) Set `RoomScanner` as Stratup Project.

# How to use it?
1) First let's connect your Kinect to the PC. (If you don't want to just keep reading, you can replay and record streams from virtual Kinect, cool yeah, I know. I am also gonna attach some sample streams, if there is no way you can get your hands on one of those Kinects anymore.)

2) The next step is to create a new project by clicking `Make Project`. It will create a new project directory.


![image](https://user-images.githubusercontent.com/14330873/110510455-401e7900-8114-11eb-8d2d-6eb8f31a4276.png)


3) Let's open it by clicking `Open Project` and selecting `project.ini` inside our new project folder.

4) You'll be presented with 2 windows. It's a point cloud viewer and control ui window. It might look like there is a lot of stuff. But generally what you are interested `Start Stream` button, `Record stream`, `Replay recorded stream` checkboxes. You can start streaming from your Kinect by pressing `Start Stream`. You can record it if you checked `Record stream`. You can **replay** streams if you uncheck `Record stream` and check `Replay recorded stream` (Be careful there are no clever "foolproof" checkbox checks inside the app). 
![image](https://user-images.githubusercontent.com/14330873/110510899-b6bb7680-8114-11eb-8273-f58885620cb0.png)

5) Once you start the stream you'll be presented with two more windows. That contains the RGB and depth data. Notice how `Start Stream` button became `Stop Stream`. That's the one you want to click to properly shut down the device and save your streams. 
![image](https://user-images.githubusercontent.com/14330873/110533842-761d2680-812f-11eb-8a3b-a790fdc8910b.png)

6) To perform reconstruction you'll need to convert native OpenNI \*.ONI stream file into PCL compatible \*.PCD files. To do that check `Save Stream as PCD` checkbox, before starting the stream. You can do that from the replayed stream, there is no difference for the application.

7) When you have saved your point clouds. You can click the final button `Perform Reconstruction`. To register (align) point clouds and then triangulate them using the TSDF algorithm. 

To be continued...






