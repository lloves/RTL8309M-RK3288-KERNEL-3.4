LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := test.c \
        mdcmdio.c \
        rtk_api.c \
        rtl8309n_asicdrv.c        

LOCAL_SHARED_LIBRARIES := libc libcutils
LOCAL_MODULE = rtl8309m
include $(BUILD_EXECUTABLE)
