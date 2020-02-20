on mac:
If Python.h not found: `export CPLUS_INCLUDE_PATH=/System/Library/Frameworks/Python.framework/Versions/2.7/include/python2.7/`

If 'numpy/arrayobject.h' not found: CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/System/Library/Frameworks/Python.framework/Versions/2.7/Extras/lib/python/numpy/core/include

g++ -std=c++14 test_pf.cpp particle_filter.cpp idm.cpp types.cpp kinematics.cpp -I/usr/include/python2.7 -lpython2.7

Also: https://stackoverflow.com/questions/21784641/installation-issue-with-matplotlib-python?noredirect=1&lq=1