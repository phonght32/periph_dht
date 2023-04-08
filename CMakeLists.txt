set(srcs
    "periph_dht.c"
    "dht/dht.c")

set(includes 
    "."
    "dht")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS ${includes}
                       REQUIRES driver periph_esp32)
