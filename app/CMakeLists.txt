project(ckViewer)

add_executable(PCLViewer PCLViewer.h PCLViewer.cpp)
target_link_libraries(PCLViewer ViewerLib XmlLib ${LCM_LIB})

