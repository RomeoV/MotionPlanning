# Path planner
After reading up on [Sampling based motion planning](http://ompl.kavrakilab.org/OMPL_Primer.pdf) I wanted to try it out myself, so I wrote a little library that does simple sample based motion planning using graph optimization (with Boost::Graph).
This was only a single afternoon project, so the functionality isn't too crazy yet. Still, it achieves fairly good results, avoiding obstacles in a 3D state-space (x, y, rot) and is about 50x faster than the python implementation.

This was originally a python implementation, which I then ported to C++. In addition to learning about motion planning, I also wanted to get some more experience with some new libraries that caught my attention recently, as well as writing in a good and modern style.
I have made heavy use of the [range-v3 library](https://github.com/ericniebler/range-v3), which will be available in the C++20 out-of-the-box, as well as the [cppitertools library](https://github.com/ericniebler/range-v3), both of which turned out to be great to use.
[Eigen](http://eigen.tuxfamily.org) is used for some basic Vector arithmetic and [Boost.Graph](https://www.boost.org/doc/libs/1_71_0/libs/graph/doc/) is used for the graph creation and optimization (where most of the runtime is).
Finally, [matplotlib-cpp](https://github.com/lava/matplotlib-cpp) has been used to plot the results using Python+Matplotlib bindings and [cxxopts](https://github.com/jarro2783/cxxopts) for some console argument handling.


## Todo
If I ever am motivated to work on this more, the following things should be added:
- tests
- clang-format
- read obstacles from a file
- generate smoother trajectories
