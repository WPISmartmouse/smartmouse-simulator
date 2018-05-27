# Smartmouse Simulator


This project currently only supports Ubuntu Linux 16.04 and 18.04

## Installing dependencies

Our build system is CMake

We recommend we using the CLion IDE.

# Licensing and Attribution

I'm using font-awesome icons with no changes, which are under CCA4: https://fontawesome.com/license
I'm using source from gazebo and ignition projects, which are under Apache: http://www.apache.org/licenses/LICENSE-2.0


# Generating dependency graph

    cd .sim_build
    cmake --graphviz=test.dot .
    dot -Tpng test.dot -o out.png
    eog out.png

