#include "art/Utilities/ToolMacros.h"
#include "messagefacility/MessageLogger/MessageLogger.h"
#include "larrecodnn/ImagePatternAlgs/Tensorflow/WaveformRecogTools/IWaveformRecog.h"

// Nvidia TensorRT inference server client includes
#include "trtis_clients/request_grpc.h"
#include "trtis_clients/request_http.h"
#include "trtis_clients/model_config.pb.h"

namespace ni = nvidia::inferenceserver;
namespace nic = nvidia::inferenceserver::client;

namespace wavrec_tool
{

  class WaveformRecogTrtis : public IWaveformRecog {
  public:
    explicit WaveformRecogTrtis(const fhicl::ParameterSet& pset);

    std::vector< std::vector<float> > predictWaveformType( const std::vector< std::vector<float> >& ) const override;

  private:
    std::string fTrtisModelName;
    std::string fTrtisURL;
    bool fTrtisVerbose;
    int64_t fTrtisModelVersion;

    std::unique_ptr<nic::InferContext> ctx; // tRTis context
    std::shared_ptr<nic::InferContext::Input> model_input;

  };

  // ------------------------------------------------------
  WaveformRecogTrtis::WaveformRecogTrtis(const fhicl::ParameterSet & pset)
  {
    fTrtisModelName    = pset.get<std::string>("TrtisModelName","mymodel.pb");
    fTrtisURL          = pset.get<std::string>("TrtisURL","localhost:8001");
    fTrtisVerbose      = pset.get<bool>("TrtisVerbose",false);
    fTrtisModelVersion = pset.get<int64_t>("TrtisModelVersion",-1);

    // ... Create the inference context for the specified model.
    auto err = nic::InferGrpcContext::Create(&ctx, fTrtisURL, fTrtisModelName, fTrtisModelVersion, fTrtisVerbose);
    if (!err.IsOk()) {
      throw cet::exception("WaveformRecogTrtis") << "unable to create tRTis inference context: " << err << std::endl;
    }

    // ... Get the specified model input
    err = ctx->GetInput("input_3", &model_input);
    if (!err.IsOk()) {
      throw cet::exception("WaveformRecogTrtis") << "unable to get tRTis input: " << err << std::endl;
    }

    mf::LogInfo("WaveformRecogTrtis") << "url: " << fTrtisURL;
    mf::LogInfo("WaveformRecogTrtis") << "model name: " << fTrtisModelName;
    mf::LogInfo("WaveformRecogTrtis") << "model version: " << fTrtisModelVersion;
    mf::LogInfo("WaveformRecogTrtis") << "verbose: " << fTrtisVerbose;

    mf::LogInfo("WaveformRecogTrtis") << "tensorRT inference context created.";

    fCnnPredCut=pset.get<float>("CnnPredCut", 0.5);
    fWaveformSize=pset.get<unsigned int>("WaveformSize", 0); // 6000
    std::string fMeanFilename=pset.get< std::string >("MeanFilename","");
    std::string fScaleFilename=pset.get< std::string >("ScaleFilename","");

    // ... load the mean and scale (std) vectors
    if(!fMeanFilename.empty() && !fScaleFilename.empty()){
      float val;
      std::ifstream meanfile(findFile(fMeanFilename.c_str()).c_str());
      if (meanfile.is_open()){
        while(meanfile >> val)meanvec.push_back(val);
        meanfile.close();
        if (meanvec.size()!=fWaveformSize){
          throw cet::exception("WaveformRecogTrtis_tool") << "vector of mean values does not match waveform size, exiting" << std::endl;
        }
      } else {
        throw cet::exception("WaveformRecogTrtis_tool") << "failed opening StdScaler mean file, exiting" << std::endl;
      }
      std::ifstream scalefile(findFile(fScaleFilename.c_str()).c_str());
      if (scalefile.is_open()){
        while(scalefile >> val)scalevec.push_back(val);
        scalefile.close();
        if (scalevec.size()!=fWaveformSize){
          throw cet::exception("WaveformRecogTrtis_tool") << "vector of scale values does not match waveform size, exiting" << std::endl;
        }
      } else {
        throw cet::exception("WaveformRecogTrtis_tool") << "failed opening StdScaler scale file, exiting" << std::endl;
      }
    }

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
  std::vector< std::vector<float> > WaveformRecogTrtis::predictWaveformType( const std::vector< std::vector<float> >& waveforms) const
  {
    if (waveforms.empty() || waveforms.front().empty()){
      return std::vector< std::vector<float> >();
    }

    size_t usamples = waveforms.size(), numtcks = waveforms.front().size();

    // ~~~~ Configure context options

    std::unique_ptr<nic::InferContext::Options> options;
    auto err = nic::InferContext::Options::Create(&options);
    if (!err.IsOk()) {
      throw cet::exception("WaveformRecogTrtis") << "failed initializing tRTis infer options: " << err << std::endl;
    }

    options->SetBatchSize(usamples);	// set batch size
    for (const auto& output : ctx->Outputs()) {	// request all output tensors
      options->AddRawResult(output);
    }

    err = ctx->SetRunOptions(*options);
    if (!err.IsOk()) {
      throw cet::exception("WaveformRecogTrtis") << "unable to set tRTis inference options: " << err << std::endl;
    }

    // ~~~~ For each sample, register the mem address of 1st byte of image and #bytes in image

    err = model_input->Reset();
    if (!err.IsOk()) {
      throw cet::exception("WaveformRecogTrtis") << "failed resetting tRTis model input: " << err << std::endl;
    }

    size_t sbuff_byte_size = numtcks*sizeof(float);

    for (size_t idx = 0; idx < usamples; ++idx) {
      err = model_input->SetRaw(reinterpret_cast<const uint8_t*>(waveforms[idx].data()),sbuff_byte_size);
      if (!err.IsOk()) {
        throw cet::exception("WaveformRecogTrtis") << "failed setting tRTis input: " << err << std::endl;
      }
    }

    // ~~~~ Send inference request

    std::map<std::string, std::unique_ptr<nic::InferContext::Result>> results;

    err = ctx->Run(&results);
    if (!err.IsOk()) {
      throw cet::exception("WaveformRecogTrtis") << "failed sending tRTis synchronous infer request: " << err << std::endl;
    }

    // ~~~~ Retrieve inference results

    std::vector< std::vector<float> > out;

    for(unsigned int i = 0; i < usamples; i++) {
      std::map<std::string, std::unique_ptr<nic::InferContext::Result>>::iterator itRes = results.begin();

      // .. loop over the outputs
      std::vector<float> vprb;
      while(itRes != results.end()){
        const std::unique_ptr<nic::InferContext::Result>& result = itRes->second;
        const uint8_t* rbuff;	  // pointer to buffer holding result bytes
        size_t rbuff_byte_size;	  // size of result buffer in bytes
        result->GetRaw(i, &rbuff,&rbuff_byte_size);
        const float *prb = reinterpret_cast<const float*>(rbuff);

        // .. loop over each class in output
        size_t ncat = rbuff_byte_size/sizeof(float);
        for(unsigned int j = 0; j < ncat; j++){
          vprb.push_back(prb[j]);
        }
        itRes++;
      }
      out.push_back(vprb);
    }

    return out;
  }

}
DEFINE_ART_CLASS_TOOL(wavrec_tool::WaveformRecogTrtis)
