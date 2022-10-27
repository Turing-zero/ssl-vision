unset(HIKMVCAM_FOUND)
unset(HIKMVCAM_INCLUDE_DIRS)
unset(HIKMVCAM_LIBRARIES)

find_path(HIKMVCAM_INCLUDE_DIRS
    NAMES
        MvCameraControl.h
    PATHS
        $ENV{MVCAM_SDK_PATH}/include
        /opt/MVS/include
)

find_library(HIKMVCAM_LIBRARIES
    NAMES
        MvCameraControl
    PATHS
        $ENV{MVCAM_SDK_PATH}/lib/64
        $ENV{MVCAM_COMMON_RUNENV}/64
        /opt/MVS/lib/64
)

if (HIKMVCAM_INCLUDE_DIRS AND HIKMVCAM_LIBRARIES)
    set(HIKMVCAM_FOUND TRUE)
    message("Found Hikvision SDK: ${HIKMVCAM_LIBRARIES}")
else()
    set(HIKMVCAM_FOUND FALSE)
    message("Hikvision SDK not found")
endif()