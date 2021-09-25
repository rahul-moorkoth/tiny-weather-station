# Edit following two lines to set component requirements (see docs)
#set(COMPONENT_REQUIRES )
#set(COMPONENT_PRIV_REQUIRES )

if (CONFIG_USE_SH1106)
    set(COMPONENT_SRCS "sh1106.c")
endif()
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
