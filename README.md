# Motion-Strategy-Library

## Installation

Install dependencies
* `PQP` is requiered for proximity queries;
* `FOX`, `OpenGL`, and `GLUT` are optional to build the motion planning application.

Build project
``` shell
mkdir build
pushd build
cmake ../
make [-j4]
popd
```

The build binaries are in build folder, for example, `build/src/msl/`.
