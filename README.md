# particlefilter_slam

To start the packet go to the root and type catkin_make
Then go to /devel and type source setup.bash
After that go to src/my_package/src and you can do the steps below to make the cython module

!!YOU NEED C COMPILER AND A PYTHON VERSION HERE I PUT 2.7!!

To build the Cython module go to src/my_package/src and type in terminal python2.7 setup.py build_ext --inplace
a new module file will appear(.so)

If you want to check if the Cython module is adequately close to c run cython -a MapBuilder.pyx and then check the new html file


Then to run the program (hopefully you know how to start a gazebo in ros)

Press rosrun my_package bot.py

More to come :)
