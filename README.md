#About PTAM-Plus

This implementation is inspired by the work described in
[PTAMM-Plus: Refactoring and Extending PTAMM](http://www.icg.tugraz.at/Members/thanh/publications/ptamm-plus-refactoring-and-extending-ptamm-1).
It is a refactored code of PTAM from the paper
[Parallel Tracking and Mapping for Small AR Workspaces](http://www.robots.ox.ac.uk/~gk/PTAM/).
The architect is redesigned a bit in order to overcome monolithic issue of original PTAM.
For instance,  UI code is completely separated from core slam code.

# Dependencies
1. [CVD library](http://www.edwardrosten.com/cvd)
2. [TooN](http://www.edwardrosten.com/cvd/toon.html)
3. [GVars3](http://www.edwardrosten.com/cvd/gvars3.html)
4. [Boost](http://www.boost.org): for threading
5. [OpenCV](http://www.opencv.org) : for `apps/slam` and `apps/calibration` to catpure video)
6. [CMake](http://www.cmake.org): for compilation

# Todo
- support Qt based ui
- Map IO
- better recovery (features based).

# COPY stuffs!
You're welcome to use the code for 'non-commercial' purposes.
However, you need to be aware of PTAM's license (LICENSE.txt in root folder)
if you want to use for other purposes.

# Need help!
contact: thanhnguyen dot cs at gmail dot com
