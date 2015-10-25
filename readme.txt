====================================================
           Graph Search Library(C), 1999-2003
====================================================

This file contains a brief description of what
a GSL download bunch consists of and how to
compile and execute samples.

====================================================
*****************FILES LOCATIONS********************
====================================================

1. All C++ library headers are included in LIB
directory. The library interfaces and implemen
tation are located in .h files.

----------------------------------------------------

2. The documentation consists of a tutorial
which could be found in DOCS directory.

----------------------------------------------------

3. Illustrating samples are located in SAMPLES
subdirectories. Following samples are presented:

===================================*grid* sample====
a sample is based on a triangular grid world model

================================*epsilon* sample====
world (grid) model is inherited from *grid* sample
and modified cost type is used to implement an
epsilon transfromation to reduce a number of
expanded nodes in a search tree.

==============================*waypoints* sample====
finding a minimal length path between two points.
The route should be based on a set of predefined
waypoints and each lag length is limited by a
constant value. To make a search more fast a
heuristic function is used. This sample
demonstrates a use of a g+h typical heuristic
function based on smallest-distance metric between
current and goal waypoints.

==============================*obstacles* sample====
major functionality is taken from *waypoints*
sample, but each lag in a route should not cross any
obstacle from predefined ones. This feature is
encapsulated in expand function class.

==============================*hexagonal* sample====
tile-based world model. The main restriction is a map
of obstacles.

====================================================
********************PORTABILITY*********************
====================================================

Linux
-----

Samples' sources were successfully compiled using
gcc 2.96 and 3.2.1 under RedHat Linux v.8.1.

----------------------------------------------------

Windows
-------

Samples' sources were successfully compiled using
Visual C++ 6.0 under Windows'98.

====================================================
********************COMPILATION*********************
====================================================

Linux
-----

1. Go to GSL root directory. Check gmake and
g++ are available on your computer.

----------------------------------------------------

2. To compile all samples call

gmake compile

----------------------------------------------------

3. To run all samples call

gmake run

This gmake target initially compiles all samples
and execute them one by one, so you can call
this target without previous one.

----------------------------------------------------

4. If you want to execute samples separately from
each other just call

gmake run SAMPLES=<sample_name>

where sample_name is one of the following

grid
epsilon
waypoints
obstacles
hexagonal

If you want to execute a pair of samples just say

gmake run SAMPLES="grid waypoints"

----------------------------------------------------

5. After compilation succeeds sample binaries are
located under ./bin/<sample_name> names. If you
want to execute a sample by hands then do the
following

cd bin/
./<sample_name>

----------------------------------------------------

Windows
-------

1. Add the path to <GSL Location>\lib to "Directories"
item in Build menu.

2. Create a new Win32 Console application and add one
of files from <GSL Location>\lib\<Sample Name>\ to
this project.

3. Build the project.

====================================================
