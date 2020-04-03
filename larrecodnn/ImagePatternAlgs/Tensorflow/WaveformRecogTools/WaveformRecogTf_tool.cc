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
    ~WaveformRecogTf();

    void configure(const fhicl::ParameterSet& pset) override;
    std::vector< std::vector<float> > predictWaveformType( const std::vector< std::vector<float> >& ) const override;

  protected:
    std::string findFile(const char* fileName) const;
    
  private:
    mutable std::unique_ptr<tf::Graph> g; // network graph
    std::string fNNetModelFilePath;
    std::vector< std::string > fNNetOutputPattern;

  };

  // ------------------------------------------------------
  WaveformRecogTf::WaveformRecogTf(const fhicl::ParameterSet & pset)
  {
    configure(pset);
    if ((fNNetModelFilePath.length() > 3) &&
        (fNNetModelFilePath.compare(fNNetModelFilePath.length() - 3, 3, ".pb") == 0)) {
      g = tf::Graph::create(findFile(fNNetModelFilePath.c_str()).c_str(), fNNetOutputPattern);
      if (!g) { throw art::Exception(art::errors::Unknown) << "TF model failed."; }
      mf::LogInfo("WaveformRecogTf") << "TF model loaded.";
    } else {
      mf::LogError("WaveformRecogTf") << "File name extension not supported.";
    }
  }

  // ------------------------------------------------------
  WaveformRecogTf::~WaveformRecogTf()
  {
  }

  // ------------------------------------------------------
  void WaveformRecogTf::configure(const fhicl::ParameterSet& pset)
  {
    //get parameters
    fNNetModelFilePath = pset.get<std::string>("NNetModelFile", "mymodel.pb");
    fNNetOutputPattern = pset.get<std::vector<std::string> >("NNetOutputPattern", {"cnn_output", "dense_3"});

    return;
  }

  // ------------------------------------------------------
  std::string WaveformRecogTf::findFile(const char* fileName) const
  {
    std::string fname_out;
    cet::search_path sp("FW_SEARCH_PATH");
    if (!sp.find_file(fileName, fname_out)){
      struct stat buffer;
      if (stat(fileName, &buffer) == 0) {
        fname_out = fileName;
      } else {
        throw art::Exception(art::errors::NotFound)
          << "Could not find the model file " << fileName;
      }
    }
    return fname_out;
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
