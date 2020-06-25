#include "art/Utilities/ToolMacros.h"
#include "messagefacility/MessageLogger/MessageLogger.h"
#include "tensorflow/core/public/session.h"
#include "larrecodnn/ImagePatternAlgs/Tensorflow/TF/tf_graph.h"
#include "larrecodnn/ImagePatternAlgs/Tensorflow/WaveformRecogTools/IWaveformRecog.h"

#include <sys/stat.h>

namespace wavrec_tool
{

  class WaveformRecogTf : public IWaveformRecog {
  public:
    explicit WaveformRecogTf(const fhicl::ParameterSet& pset);

    std::vector< std::vector<float> > predictWaveformType( const std::vector< std::vector<float> >& ) const override;

  private:
    std::unique_ptr<tf::Graph> g; // network graph
    std::string fNNetModelFilePath;
    std::vector< std::string > fNNetOutputPattern;

  };

  // ------------------------------------------------------
  WaveformRecogTf::WaveformRecogTf(const fhicl::ParameterSet & pset)
  {
    fNNetModelFilePath = pset.get<std::string>("NNetModelFile", "mymodel.pb");
    fNNetOutputPattern = pset.get<std::vector<std::string> >("NNetOutputPattern", {"cnn_output", "dense_3"});
    if ((fNNetModelFilePath.length() > 3) &&
        (fNNetModelFilePath.compare(fNNetModelFilePath.length() - 3, 3, ".pb") == 0)) {
      g = tf::Graph::create(findFile(fNNetModelFilePath.c_str()).c_str(), fNNetOutputPattern);
      if (!g) { throw art::Exception(art::errors::Unknown) << "TF model failed."; }
      mf::LogInfo("WaveformRecogTf") << "TF model loaded.";
    } else {
      mf::LogError("WaveformRecogTf") << "File name extension not supported.";
    }

    fCnnInputMean=pset.get<float>("CnnInputMean",0.);
    fCnnInputScale=pset.get<float>("CnnInputScale",1.);
    fWaveformSize=pset.get<unsigned int>("WaveformSize", 0); // 6000
    fWindowSize=pset.get<unsigned int>("ScanWindowSize", 0); // 200
    fStrideLength=pset.get<unsigned int>("StrideLength", 0); // 150

    if(fWaveformSize>0 && fWindowSize>0){ 
      float dmn = fWaveformSize - fWindowSize;	// dist between trail edge of 1st win & last data point
      fNumStrides = std::ceil(dmn/float(fStrideLength));	// # strides to scan entire waveform
      unsigned int overshoot = fNumStrides*fStrideLength+fWindowSize - fWaveformSize;
      fLastWindowSize = fWindowSize - overshoot;
      unsigned int numwindows = fNumStrides + 1;
      std::cout << " !!!!! WaveformRoiFinder: WindowSize = " << fWindowSize << ", StrideLength = "
                << fStrideLength << ", dmn/StrideLength = " << dmn/fStrideLength << std::endl;
      std::cout << "       dmn = " << dmn << ", NumStrides = " << fNumStrides << ", overshoot = " << overshoot
                << ", LastWindowSize = " << fLastWindowSize << ", numwindows = " << numwindows << std::endl;
    }

  }

  // ------------------------------------------------------
  std::vector< std::vector<float> > WaveformRecogTf::predictWaveformType( const std::vector< std::vector<float> >& waveforms) const
  {
    if (waveforms.empty() || waveforms.front().empty()){
      return std::vector< std::vector<float> >();
    }

    long long int samples = waveforms.size(), numtcks = waveforms.front().size();

    tensorflow::Tensor _x(tensorflow::DT_FLOAT, tensorflow::TensorShape({ samples, numtcks, 1 }));
    auto input_map = _x.tensor<float, 3>();
    for (long long int s = 0; s < samples; ++s) {
      const auto & wvfrm = waveforms[s];
      for (long long int t = 0; t < numtcks; ++t) {
        input_map(s, t, 0) = wvfrm[t];
      }
    }

    return g->run(_x);
  }

}
DEFINE_ART_CLASS_TOOL(wavrec_tool::WaveformRecogTf)
