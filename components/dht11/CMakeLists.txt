if (CONFIG_USE_DHT11)
    set(srcs "dht11.c")
endif()
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")
              
