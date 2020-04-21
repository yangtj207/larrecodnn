
if( DEFINED ENV{TENSORFLOW_DIR} )
  add_subdirectory(Tensorflow)
endif ()
add_subdirectory(job)
add_subdirectory(Keras)
