
# tensorflow and trtis_clients are artificially linked
# at this time, trtis_clients is only available for python 3
# therefore, we need to check to see if BOTH tesorflow and trtis_clients are available
if(( DEFINED ENV{TENSORFLOW_DIR} ) AND ( DEFINED ENV{TRTIS_CLIENTS_DIR} ))
  add_subdirectory(Tensorflow)
endif ()
add_subdirectory(job)
add_subdirectory(Keras)
