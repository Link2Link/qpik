cmake_minimum_required(VERSION 3.22)
project(qpik VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})
link_libraries(${PINOCCHIO_LIBRARIES})

find_package(osqp)
find_package(OsqpEigen)



file(GLOB_RECURSE SRC_FILES src/*.cpp)
include_directories(include)


add_library(qpik SHARED ${SRC_FILES})

# ============================================================================
# 目标属性配置（关键：使其他项目可以使用 qpik::qpik）
# ============================================================================

# 设置公共头文件路径（必须！否则其他项目找不到头文件）
target_include_directories(qpik
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 设置 C++ 标准（传递给依赖项目）
set_target_properties(qpik PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# 链接依赖库（PUBLIC 确保依赖会传递给使用 qpik::qpik 的项目）
target_link_libraries(qpik PUBLIC   
    osqp::osqp         
    OsqpEigen::OsqpEigen
    urdf::urdf
)

# ============================================================================
# 安装配置
# ============================================================================

# 安装库文件
install(TARGETS qpik
    EXPORT qpikTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# 安装头文件
install(DIRECTORY include/qpik
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# 安装导出文件（用于其他项目使用 find_package）
install(EXPORT qpikTargets
    FILE qpikTargets.cmake
    NAMESPACE qpik::
    DESTINATION lib/cmake/qpik
)
# 这样，可以让其他工程利用 CMake 的 find_package 自动找到 qpik，并通过 target_link_libraries 使用 qpik::qpik

# 创建配置文件供 find_package 使用
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/qpikConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/qpikConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/qpikConfig.cmake"
    INSTALL_DESTINATION lib/cmake/qpik
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/qpikConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/qpikConfigVersion.cmake"
    DESTINATION lib/cmake/qpik
)

# ============================================================================
# CPack 打包配置
# ============================================================================
set(CPACK_PACKAGE_NAME "qpik")
set(CPACK_PACKAGE_VENDOR "qpik")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "QP-based Inverse Kinematics Solver")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
# 生成 TGZ 和 DEB 包
set(CPACK_GENERATOR "TGZ;DEB")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "qpik")

# 设置使用 DESTDIR，这样打包时不需要 root 权限
# CPack 会将文件安装到临时目录，然后打包
set(CPACK_SET_DESTDIR ON)

# 可选：如果存在 LICENSE 和 README.md 文件，则包含它们
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
    set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
endif()

# DEB 包配置
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "qpik")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")

include(CPack)