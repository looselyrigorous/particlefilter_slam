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

bot.py now work for turtlebot in gazebo with scanner in /scan (if you want kobuki
you must manually change it to /kobuki/laser/scan that is also a commented line)


More to come :)


Best mean time for

('Merge Map Maker:', 0.004237684700059595)!!
('Error Calc:', 0.00029190216005218697)
('Propability Map Maker:', 0.004237684700059595)!!
('Grid Maker:', 0.00035273702988713427)
