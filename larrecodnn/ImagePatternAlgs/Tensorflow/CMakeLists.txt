include_directories ( $ENV{TENSORFLOW_INC}/eigen )

add_subdirectory(PointIdAlg)
add_subdirectory(PointIdAlgTools)
add_subdirectory(WaveformRecogTools)
add_subdirectory(TF)
add_subdirectory(Modules)
