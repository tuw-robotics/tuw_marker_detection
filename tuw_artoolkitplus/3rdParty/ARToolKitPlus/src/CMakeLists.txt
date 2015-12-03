
SET(SOURCES
  MemoryManager.cpp
  DLL.cpp
  librpp/rpp.cpp
  librpp/rpp_quintic.cpp
  librpp/rpp_vecmat.cpp
  librpp/rpp_svd.cpp
  librpp/librpp.cpp
  extra/Profiler.cpp
  extra/FixedPoint.cpp)
	

add_library(ARToolKitPlus SHARED ${SOURCES})

