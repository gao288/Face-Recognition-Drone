# Local libraries
include_directories(${LandmarkDetector_SOURCE_DIR}/include)
	
add_executable(FaceSelect FaceSelect.cpp GimbalUtil.cpp GimbalUtil.h)
target_link_libraries(FaceSelect LandmarkDetector)
target_link_libraries(FaceSelect FaceAnalyser)
target_link_libraries(FaceSelect GazeAnalyser)
target_link_libraries(FaceSelect Utilities)

install (TARGETS FaceSelect DESTINATION bin)

#add_executable(tt FaceLandmarkVid.cpp)
#target_link_libraries(tt LandmarkDetector)
#target_link_libraries(tt FaceAnalyser)
#target_link_libraries(tt GazeAnalyser)
#target_link_libraries(tt Utilities)
#
#install (TARGETS tt DESTINATION bin)
