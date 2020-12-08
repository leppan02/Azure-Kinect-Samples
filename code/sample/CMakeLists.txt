
add_executable(sample main.c)


# Dependencies of this library
target_link_libraries(sample PRIVATE 
    k4a
    k4abt
    )


