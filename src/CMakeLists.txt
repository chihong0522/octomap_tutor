# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
list(APPEND CMAKE_PREFIX_PATH "~/octomap")

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# octomap
FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )

include_directories(/home/chihung/octomap_tutor/include/pugixml-1.11/src)

ADD_EXECUTABLE( pcd2octomap pcd2octomap.cpp )
TARGET_LINK_LIBRARIES( pcd2octomap
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES})

ADD_EXECUTABLE( pcd2colorOctomap pcd2colorOctomap.cpp )
TARGET_LINK_LIBRARIES( pcd2colorOctomap
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES})

FIND_PACKAGE(OpenCV REQUIRED)

ADD_EXECUTABLE( joinmap joinMap.cpp )
TARGET_LINK_LIBRARIES( joinmap
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBS})

