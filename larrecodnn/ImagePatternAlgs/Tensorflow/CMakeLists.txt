include_directories ( $ENV{TENSORFLOW_INC}/eigen )

add_subdirectory(PointIdAlg)
if( DEFINED ENV{TRTIS_CLIENTS_DIR} )
  add_subdirectory(PointIdAlgTools)
  add_subdirectory(WaveformRecogTools)
endif ()
add_subdirectory(TF)
add_subdirectory(Modules)
