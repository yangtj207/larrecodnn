include_directories($ENV{TRTIS_CLIENTS_INC})
cet_find_library(TRTIS_CLIENTS_LIBRARY NAMES request PATHS $ENV{TRTIS_CLIENTS_LIB})

art_make(
          TOOL_LIBRARIES
          larreco_RecoAlg_ImagePatternAlgs_DataProvider
          ROOT::Core
          canvas
          ${MF_MESSAGELOGGER}
	  ${TRTIS_CLIENTS_LIBRARY}
        )

install_source()
