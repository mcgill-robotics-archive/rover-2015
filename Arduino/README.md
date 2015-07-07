# Arduino

This is the root folder for all arduino sketches and projects. Normally, all sketches should be in a folder with the same name as the main arduino file. That is the one containing the `void setup()` and `void loop()` functions.

## CMake

With emphasis on scalability, we decided to implement a CMake build system to build the important arduino projects. This allows us more control on the building and linking options of the programs. It is also easier to maintain large scale applications. However, there are several subtleties regarding environment setup that you have to be aware of to build the project.

### ros_lib

Before you are ready to build the arduino projects, you will need to create the ros libraries. These library are build using a script in the `rosserial_arduino` package. As such, they can only be generated using a system on which ros is installed. However, once they are built, they can be copied and moved around as they are compiled by the Arduino compiler, which is multi-platform.

So to generate the libraries, the easiest way for us is to navigate to the root of the catkin workspace. If you used the compsys script, simply use 

```$ roscd``` 

and you should be at the right location. If you did not, you will have to find the root of the rover repository and enter the `catkin_ws` folder. Then simply call 

```$ catkin_make``` 

to build the entire workspace. This will generate the ros libraries for arduino as well.

Once you generated the ros_lib you will need to edit the base `CMakeLists.txt` file. __We ask that you do not commit the changes you make to that file.__

1. Find the line `set(ROS_LIB_DIR /home/david/rover/catkin_ws/build/arduino_test/ros_lib)`, currently line 6.
2. Change the path to the generated ros_lib folder. Only the part before `rover` should change. 
3. You should be ready for the next step.

### Arduino core

You need to have the Arduino IDE installed on your computer before using any this. Depending on which platform you are running, this step might be more or less complicated. If the documentation in this file is insufficient, refer yourself to the [arduino-cmake documentation](https://github.com/queezythegreat/arduino-cmake)

If you are using Ubuntu and installed arduino using apt-get, you are done with pre-requisites. If you are running on other platforms and installed Arduino in the recommended location, you should be done. If not, [see the doc](https://github.com/queezythegreat/arduino-cmake). 

## CMake

Once your pre-requisites are met, you can generate the project and build it. Here is the traditional command line only approach:
```bash
$ mkdir build
$ cd build
$ cmake ..
$ make 
$ make drive-upload
```

Under each project folder, you should find an additional CMakeLists.txt file which describes the executables present in that folder. For example, under the OTS_Drive folder, we describe the executable `drive`, hence the `drive-upload` in the example above. Note the four (4) components of the description:
1. SRCS These are all the source (not header) files in the project that should be built in order for this project to work. Additionally, you need to include __${ROS_LIB_DIR}/time.cpp__ if you hope to build against the ros_lib.
2. HRDS These are all the header files in the project folder linked for this executable.
3. BOARD This is the board you will want to upload to. The processor chances for each board so pick it carefully.
4. PORT This is the terminal device port used for upload. If you do not upload the code, then you don't care. 

Voila, for any question contact [David Lavoie-Boutin](mailto:david.lavoie-boutin@mail.mcgill.ca)

## Note 

The Arduino sketches created for the custom motor controllers and the RS485 serial bus are under deprecated folder as they are now really irrelevant. 

