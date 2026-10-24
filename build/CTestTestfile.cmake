# CMake generated Testfile for 
# Source directory: D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab
# Build directory: D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[TorqueLabNativeValidationSuite]=] "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/build/Debug/TorqueLabNativeTests.exe")
  set_tests_properties([=[TorqueLabNativeValidationSuite]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;220;add_test;D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[TorqueLabNativeValidationSuite]=] "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/build/Release/TorqueLabNativeTests.exe")
  set_tests_properties([=[TorqueLabNativeValidationSuite]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;220;add_test;D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[TorqueLabNativeValidationSuite]=] "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/build/MinSizeRel/TorqueLabNativeTests.exe")
  set_tests_properties([=[TorqueLabNativeValidationSuite]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;220;add_test;D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[TorqueLabNativeValidationSuite]=] "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/build/RelWithDebInfo/TorqueLabNativeTests.exe")
  set_tests_properties([=[TorqueLabNativeValidationSuite]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;220;add_test;D:/# My Folder/My Work/Visual Studio script/WebApplications/TorqueLab/CMakeLists.txt;0;")
else()
  add_test([=[TorqueLabNativeValidationSuite]=] NOT_AVAILABLE)
endif()
subdirs("_deps/glfw-build")
subdirs("_deps/bullet3-build")
