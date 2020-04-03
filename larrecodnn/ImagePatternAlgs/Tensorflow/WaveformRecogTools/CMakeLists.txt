include_directories( $ENV{TENSORFLOW_INC}/absl )
include_directories($ENV{TRTIS_CLIENTS_INC})
cet_find_library(TRTIS_CLIENTS_LIBRARY NAMES request PATHS $ENV{TRTIS_CLIENTS_LIB})
#cet_enable_asserts()

art_make(
         TOOL_LIBRARIES
         larrecodnn_ImagePatternAlgs_Tensorflow_TF
         art_Utilities
         canvas
         ${MF_MESSAGELOGGER}
         ${FHICLCPP}
         cetlib cetlib_except
	 ${Boost_FILESYSTEM_LIBRARY}
	 ${TRTIS_CLIENTS_LIBRARY}
        )

install_headers()
install_fhicl()
install_source()
