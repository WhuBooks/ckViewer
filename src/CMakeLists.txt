project(ckViewer)

add_library(XmlLib STATIC tinyxml2.h tinyxml2.cpp)

add_library(ViewerLib STATIC CloudHandler.h CloudHandler.cpp)

