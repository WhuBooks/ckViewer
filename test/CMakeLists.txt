project(ckViewer)

add_executable(MultiThreadTest MultiThreadTest.cpp)
target_link_libraries(MultiThreadTest ViewerLib)

add_executable(LcmTest LcmTest.cpp)
target_link_libraries(LcmTest XmlLib ${LCM_LIB})
