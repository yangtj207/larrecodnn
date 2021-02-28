art_make( MODULE_LIBRARIES
	lardata_ArtDataHelper
        lardataobj_RawData
	larrecodnn_ImagePatternAlgs_Tensorflow_PointIdAlg
        art_Framework_Services_System_TriggerNamesService_service
        larsim_MCCheater_ParticleInventoryService_service
        nurandom_RandomUtils_NuRandomService_service
)

install_headers()
install_fhicl()
install_source()
