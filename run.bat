call call C:\opt\ros\foxy\x64\local_setup.bat
: call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
colcon test --merge-install
colcon test-result
call install\local_setup.bat
ros2 run demo_nodes_cpp talker

call install\local_setup.bat
ros2 run demo_nodes_py listener


choco install -y peazip

choco install -y svn hg