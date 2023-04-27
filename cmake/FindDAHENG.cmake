unset(DAHENG_FOUND)
unset(DAHENG_INCLUDE_DIRS)
unset(DAHENG_LIBS)

find_path(DAHENG_INCLUDE_DIRS
    NAMES
        GxIAPI.h
    PATHS
        $ENV{DAHENG_SDK_PATH}/inc
        /opt/Galaxy_camera/inc
)

find_library(DAHENG_LIBS
    NAMES
        gxiapi
    PATHS
        $ENV{DAHENG_SDK_PATH}/lib/x86_64
        /opt/Galaxy_camera/lib/x86_64
)

if (DAHENG_INCLUDE_DIRS AND DAHENG_LIBS)
    set(DAHENG_FOUND TRUE)
    message("Found DAHENG SDK: ${DAHENG_LIBS}")
else()
    set(DAHENG_FOUND FALSE)
    message("DAHENG SDK not found")
endif()