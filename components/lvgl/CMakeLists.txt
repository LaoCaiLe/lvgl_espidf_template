file(GLOB_RECURSE SOURCES src/*.c)
file(GLOB_RECURSE LV_DEV_SOURCES lvgl_driver/*.c)
file(GLOB_RECURSE DEMOS_SOURCES demos/*.c)
file(GLOB_RECURSE EXAMPLES_SOURCES examples/*.c)
idf_component_register(	SRCS ${SOURCES}
						SRCS ${LV_DEV_SOURCES}
						SRCS ${DEMOS_SOURCES}
						SRCS ${EXAMPLES_SOURCES}
						INCLUDE_DIRS . src ../ ./lvgl_driver demos examples
                       	PRIV_REQUIRES driver lcd) 