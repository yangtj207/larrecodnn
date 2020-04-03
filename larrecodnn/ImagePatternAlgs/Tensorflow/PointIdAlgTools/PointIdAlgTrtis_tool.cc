////////////////////////////////////////////////////////////////////////////////////////////////////
// Class:       PointIdAlgTrtis_tool
// Authors:     M.Wang,                                   from DUNE, FNAL, 2020: tensorRT inf client
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "art/Utilities/ToolMacros.h"
#include "larrecodnn/ImagePatternAlgs/Tensorflow/PointIdAlgTools/IPointIdAlg.h"

// Nvidia TensorRT inference server client includes
#include "trtis_clients/request_grpc.h"
#include "trtis_clients/request_http.h"
#include "trtis_clients/model_config.pb.h"

namespace ni = nvidia::inferenceserver;
namespace nic = nvidia::inferenceserver::client;

namespace PointIdAlgTools
{

  class PointIdAlgTrtis : public IPointIdAlg {
  public:
    explicit PointIdAlgTrtis(const fhicl::ParameterSet& pset) :
             PointIdAlgTrtis(fhicl::Table<Config>(pset, {})())
	     {}
    explicit PointIdAlgTrtis(const Config& config);
    ~PointIdAlgTrtis();

    std::vector<float> Run(std::vector< std::vector<float> > const & inp2d) const override;
    std::vector< std::vector<float> > Run(std::vector< std::vector< std::vector<float> > > const & inps, int samples = -1) const override;

  private:
    std::string fTrtisModelName;
    std::string fTrtisURL;
    bool fTrtisVerbose;
    int64_t fTrtisModelVersion;

    std::unique_ptr<nic::InferContext> ctx; // tRTis context
    mutable nic::Error err;
    std::shared_ptr<nic::InferContext::Input> model_input;

  };

  // ------------------------------------------------------
  PointIdAlgTrtis::PointIdAlgTrtis(const Config& config) : img::DataProviderAlg(config)
  {
    // ... Get common config vars
    fNNetOutputs = config.NNetOutputs();
    fPatchSizeW = config.PatchSizeW();
    fPatchSizeD = config.PatchSizeD();
    fCurrentWireIdx = 99999;
    fCurrentScaledDrift = 99999;

    // ... Get "optional" config vars specific to tRTis interface
    std::string s_cfgvr;
    int64_t i_cfgvr;
    bool b_cfgvr;
    if ( config.TrtisModelName(s_cfgvr) ) {
      fTrtisModelName = s_cfgvr;
    } else {
      fTrtisModelName = "mycnn";
    }
    if ( config.TrtisURL(s_cfgvr) ) {
      fTrtisURL = s_cfgvr;
    } else {
      fTrtisURL = "localhost:8001";
    }
    if ( config.TrtisVerbose(b_cfgvr) ) {
      fTrtisVerbose = b_cfgvr;
    } else {
      fTrtisVerbose = false;
    }
    if ( config.TrtisModelVersion(i_cfgvr) ) {
      fTrtisModelVersion = i_cfgvr;
    } else {
      fTrtisModelVersion = -1;
    }
     
    // ... Create the inference context for the specified model.
    err = nic::InferGrpcContext::Create(&ctx, fTrtisURL, fTrtisModelName, fTrtisModelVersion, fTrtisVerbose);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "unable to create tRTis inference context: " << err << std::endl;
    }

    // ... Get the specified model input
    err = ctx->GetInput("main_input", &model_input);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "unable to get tRTis input: " << err << std::endl;
    }

    mf::LogInfo("PointIdAlgTrtis") << "url: " << fTrtisURL;
    mf::LogInfo("PointIdAlgTrtis") << "model name: " << fTrtisModelName;
    mf::LogInfo("PointIdAlgTrtis") << "model version: " << fTrtisModelVersion;
    mf::LogInfo("PointIdAlgTrtis") << "verbose: " << fTrtisVerbose;

    mf::LogInfo("PointIdAlgTrtis") << "tensorRT inference context created.";

    resizePatch();
  }

  // ------------------------------------------------------
  PointIdAlgTrtis::~PointIdAlgTrtis()
  {
  }

  // ------------------------------------------------------
  std::vector<float> PointIdAlgTrtis::Run(std::vector< std::vector<float> > const & inp2d) const
  {
    size_t nrows = inp2d.size(), ncols = inp2d.front().size();

    // ~~~~ Configure context options

    std::unique_ptr<nic::InferContext::Options> options;
    err = nic::InferContext::Options::Create(&options);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed initializing tRTis infer options: " << err << std::endl;
    }

    options->SetBatchSize(1);			// set batch size
    for (const auto& output : ctx->Outputs()) {	// request all output tensors
      options->AddRawResult(output);
    }

    err = ctx->SetRunOptions(*options);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "unable to set tRTis infer options: " << err << std::endl;
    }

    // ~~~~ Register the mem address of 1st byte of image and #bytes in image

    err = model_input->Reset();
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed resetting tRTis model input: " << err << std::endl;
    }

    size_t sbuff_byte_size = (nrows*ncols)*sizeof(float);
    std::vector<float> fa(sbuff_byte_size);

    // ..flatten the 2d array into contiguous 1d block
    for (size_t ir = 0; ir < nrows; ++ir){
      std::copy(inp2d[ir].begin(), inp2d[ir].end(), fa.begin()+(ir*ncols)); 
    }
    err = model_input->SetRaw(reinterpret_cast<uint8_t*>(fa.data()),sbuff_byte_size);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed setting tRTis input: " << err << std::endl;
    }

    // ~~~~ Send inference request

    std::map<std::string, std::unique_ptr<nic::InferContext::Result>> results;

    err = ctx->Run(&results);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed sending tRTis synchronous infer request: " << err << std::endl;
    }

    // ~~~~ Retrieve inference results

    std::vector<float> out;
    std::map<std::string, std::unique_ptr<nic::InferContext::Result>>::iterator itRes = results.begin();

    // .. loop over the outputs
    while(itRes != results.end()){
      const std::unique_ptr<nic::InferContext::Result>& result = itRes->second;
      const uint8_t* rbuff;	// pointer to buffer holding result bytes
      size_t rbuff_byte_size;	// size of result buffer in bytes
      result->GetRaw(0, &rbuff,&rbuff_byte_size);
      const float *prb = reinterpret_cast<const float*>(rbuff);

      // .. loop over each class in output
      size_t ncat = rbuff_byte_size/sizeof(float);
      for(unsigned int j = 0; j < ncat; j++){
        out.push_back(prb[j]);
      }
      itRes++;
    }

    return out;
  }

  // ------------------------------------------------------
  std::vector< std::vector<float> > PointIdAlgTrtis::Run(std::vector< std::vector< std::vector<float> > > const & inps, int samples) const
  {
    if ((samples == 0) || inps.empty() || inps.front().empty() || inps.front().front().empty()) {
      return std::vector< std::vector<float> >();
    }

    if ((samples == -1) || (samples > (long long int)inps.size())) { samples = inps.size(); }

    size_t usamples = samples;
    size_t nrows = inps.front().size(), ncols = inps.front().front().size();

    // ~~~~ Configure context options
 
    std::unique_ptr<nic::InferContext::Options> options;
    err = nic::InferContext::Options::Create(&options);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed initializing tRTis infer options: " << err << std::endl;
    }

    options->SetBatchSize(usamples);	// set batch size
    for (const auto& output : ctx->Outputs()) {	// request all output tensors
      options->AddRawResult(output);
    }

    err = ctx->SetRunOptions(*options);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "unable to set tRTis inference options: " << err << std::endl;
    }

    // ~~~~ For each sample, register the mem address of 1st byte of image and #bytes in image

    err = model_input->Reset();
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed resetting tRTis model input: " << err << std::endl;
    }

    size_t sbuff_byte_size = (nrows*ncols)*sizeof(float);
    std::vector< std::vector<float> > fa(usamples, std::vector<float>(sbuff_byte_size));
  
    for (size_t idx = 0; idx < usamples; ++idx) {
      // ..first flatten the 2d array into contiguous 1d block
      for (size_t ir = 0; ir < nrows; ++ir){
        std::copy(inps[idx][ir].begin(), inps[idx][ir].end(), fa[idx].begin()+(ir*ncols)); 
      }
      err = model_input->SetRaw(reinterpret_cast<uint8_t*>(fa[idx].data()),sbuff_byte_size);
      if (!err.IsOk()) {
        throw cet::exception("PointIdAlgTrtis") << "failed setting tRTis input: " << err << std::endl;
      }
    }

    // ~~~~ Send inference request

    std::map<std::string, std::unique_ptr<nic::InferContext::Result>> results;

    err = ctx->Run(&results);
    if (!err.IsOk()) {
      throw cet::exception("PointIdAlgTrtis") << "failed sending tRTis synchronous infer request: " << err << std::endl;
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
DEFINE_ART_CLASS_TOOL(PointIdAlgTools::PointIdAlgTrtis)
