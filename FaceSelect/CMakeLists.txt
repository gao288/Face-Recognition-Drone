# Local libraries
include_directories(${LandmarkDetector_SOURCE_DIR}/include)
	
add_executable(FaceSelect FaceSelect.cpp)
target_link_libraries(FaceSelect LandmarkDetector)
target_link_libraries(FaceSelect FaceAnalyser)
target_link_libraries(FaceSelect GazeAnalyser)
target_link_libraries(FaceSelect Utilities)

install (TARGETS FaceSelect DESTINATION bin)
