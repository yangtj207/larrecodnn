////////////////////////////////////////////////////////////////////////
// Class:       WaveformRoiFinder
// Plugin Type: producer (art v3_05_00)
// File:        WaveformRoiFinder_module.cc
//
// Authors:     Mike Wang (mwang@fnal.gov)
//              Lorenzo Uboldi (uboldi.lorenzo@gmail.com)
//              Tingjun Yang (tjyang@fnal.gov)
//
// Generated at Fri Apr 10 23:30:12 2020 by Tingjun Yang using cetskelgen
// from cetlib version v3_10_00.
// Based on the Analyzer module written by Mike Wang.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDProducer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "art/Utilities/make_tool.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"
#include "canvas/Utilities/InputTag.h"

#include "lardataobj/RecoBase/Wire.h"
#include "lardataobj/RawData/RawDigit.h"
#include "lardataobj/RawData/raw.h"
#include "larcore/Geometry/Geometry.h"
#include "larrecodnn/ImagePatternAlgs/Tensorflow/WaveformRecogTools/IWaveformRecog.h"

#include <memory>

namespace nnet {
  class WaveformRoiFinder;
}


class nnet::WaveformRoiFinder : public art::EDProducer {
public:
  explicit WaveformRoiFinder(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  WaveformRoiFinder(WaveformRoiFinder const&) = delete;
  WaveformRoiFinder(WaveformRoiFinder&&) = delete;
  WaveformRoiFinder& operator=(WaveformRoiFinder const&) = delete;
  WaveformRoiFinder& operator=(WaveformRoiFinder&&) = delete;

  // Required functions.
  void produce(art::Event& e) override;

private:


  std::string fMeanFilename;	// StandardScaler mean file;
  std::string fMeanPathname;
  std::string fScaleFilename;	// StandardScaler scale (std) file;
  std::string fScalePathname;

  std::vector<float>scalevec;
  std::vector<float>meanvec;
  unsigned int fWaveformSize;		// Full waveform size
  unsigned int fWindowSize;		// Scan window size
  unsigned int fStrideLength;		// Offset (in #time ticks) between scan windows
  unsigned int fNumStrides;
  unsigned int fLastWindowSize;

  art::InputTag fRawProducerLabel;
  art::InputTag fWireProducerLabel;
  std::unique_ptr<wavrec_tool::IWaveformRecog> fWaveformRecogTool;

};


nnet::WaveformRoiFinder::WaveformRoiFinder(fhicl::ParameterSet const& p)
  : EDProducer{p}  ,
  fMeanFilename(p.get< std::string >("MeanFilename")),
  fScaleFilename(p.get< std::string >("ScaleFilename")),
  fWaveformSize(p.get<unsigned int>("WaveformSize", 6000)),
  fWindowSize(p.get<unsigned int>("ScanWindowSize", 200)),
  fStrideLength(p.get<unsigned int>("StrideLength", 150)),
  fRawProducerLabel(p.get<art::InputTag>("RawProducerLabel","")),
  fWireProducerLabel(p.get<art::InputTag>("WireProducerLabel",""))
{

  if (fRawProducerLabel.empty() && fWireProducerLabel.empty()){
    throw cet::exception("WaveformRoiFinder") << "Both RawProducerLabel and WireProducerLabel are empty";
  }

  if ((!fRawProducerLabel.empty()) && (!fWireProducerLabel.empty())){
    throw cet::exception("WaveformRoiFinder") << "Only one of RawProducerLabel and WireProducerLabel should be set";
  }

  cet::search_path sp("FW_SEARCH_PATH");
  if( !sp.find_file(fMeanFilename, fMeanPathname) ) {
    throw cet::exception("WaveformRoiFinder") << "cannot find the StdScaler mean file, exiting." << std::endl;
  }
  if( !sp.find_file(fScaleFilename, fScalePathname) ) {
    throw cet::exception("WaveformRoiFinder") << "cannot find the StdScaler scale file, exiting." << std::endl;
  }

  // ... load the mean and scale (std) vectors
  float val;
  std::ifstream meanfile(fMeanPathname.c_str());
  if (meanfile.is_open()){
    while(meanfile >> val)meanvec.push_back(val);
    meanfile.close();
    if (meanvec.size()!=fWaveformSize){
      throw cet::exception("WaveformRoiFinder") << "vector of mean values does not match waveform size, exiting" << std::endl;
    }
  } else {
    throw cet::exception("WaveformRoiFinder") << "failed opening StdScaler mean file, exiting" << std::endl;
  }
  std::ifstream scalefile(fScalePathname.c_str());
  if (scalefile.is_open()){
    while(scalefile >> val)scalevec.push_back(val);
    scalefile.close();
    if (scalevec.size()!=fWaveformSize){
      throw cet::exception("WaveformRoiFinder") << "vector of scale values does not match waveform size, exiting" << std::endl;
    }
  } else {
    throw cet::exception("WaveformRoiFinder") << "failed opening StdScaler scale file, exiting" << std::endl;
  }

  float dmn = fWaveformSize - fWindowSize;	// dist between trail edge of 1st win & last data point
  fNumStrides = std::ceil(dmn/float(fStrideLength));	// # strides to scan entire waveform
  unsigned int overshoot = fNumStrides*fStrideLength+fWindowSize - fWaveformSize;
  fLastWindowSize = fWindowSize - overshoot;
  unsigned int numwindows = fNumStrides + 1;
  std::cout << " !!!!! WaveformRoiFinder: WindowSize = " << fWindowSize << ", StrideLength = "
            << fStrideLength << ", dmn/StrideLength = " << dmn/fStrideLength << std::endl;
  std::cout << "       dmn = " << dmn << ", NumStrides = " << fNumStrides << ", overshoot = " << overshoot
            << ", LastWindowSize = " << fLastWindowSize << ", numwindows = " << numwindows << std::endl;

  // Signal/Noise waveform recognition tool
  fWaveformRecogTool = art::make_tool<wavrec_tool::IWaveformRecog>(p.get<fhicl::ParameterSet>("WaveformRecog"));

  produces< std::vector<recob::Wire> >();

}

void nnet::WaveformRoiFinder::produce(art::Event& e)
{

  std::cout<<fRawProducerLabel<<std::endl;

  art::Handle< std::vector<raw::RawDigit> > rawListHandle;
  std::vector<art::Ptr<raw::RawDigit> > rawlist;
  if (e.getByLabel(fRawProducerLabel, rawListHandle))
    art::fill_ptr_vector(rawlist, rawListHandle);

  //std::cout<<rawlist.size()<<std::endl;

  art::Handle< std::vector<recob::Wire> > wireListHandle;
  std::vector<art::Ptr<recob::Wire> > wirelist;
  if (e.getByLabel(fWireProducerLabel, wireListHandle))
    art::fill_ptr_vector(wirelist, wireListHandle);

  //auto wireHandle = e.getValidHandle< std::vector<recob::Wire> >(fWireProducerLabel);

  std::unique_ptr<std::vector<recob::Wire> > outwires(new std::vector<recob::Wire>);

  auto const* geo = lar::providerFrom<geo::Geometry>();

  //##############################
  //### Looping over the wires ###
  //##############################
  //for(size_t wireIter = 0; wireIter < wireHandle->size(); wireIter++){
  //std::cout<<"size = "<<(rawlist.empty()?wirelist.size():rawlist.size())<<std::endl;
  for (unsigned int ich = 0; ich < (rawlist.empty()?wirelist.size():rawlist.size()); ++ich){
    
    std::vector<float> inputsignal(fWaveformSize);
    std::vector<float> adc(fWaveformSize);

    if (!wirelist.empty()){
      const auto & wire = wirelist[ich];
      //art::Ptr<recob::Wire>   wire(wireHandle, wireIter);
      const auto & signal = wire->Signal();

      for (size_t itck = 0; itck < adc.size(); ++itck){
        inputsignal[itck] = signal[itck];
        adc[itck] = (inputsignal[itck] - meanvec[itck])/scalevec[itck];
      }
    }
    else if (!rawlist.empty()){
      //std::cout<<ich<<std::endl;
      const auto & digitVec = rawlist[ich];
      std::vector<short> rawadc(fWaveformSize);
      raw::Uncompress(digitVec->ADCs(), rawadc, digitVec->GetPedestal(), digitVec->Compression());
      for (size_t itck = 0; itck < rawadc.size(); ++itck){
        inputsignal[itck] = rawadc[itck] - digitVec->GetPedestal();
        //std::cout<<rawadc[itck]<<" "<<digitVec->GetPedestal()<<std::endl;
        if (geo->View(rawlist[ich]->Channel()) == 2){
          inputsignal[itck] -= 900;
        }
        else{
          inputsignal[itck] -= 2350;
        }
        adc[itck] = (inputsignal[itck] - meanvec[itck])/scalevec[itck];
      }
    }
    // .. create a vector of windows
    std::vector<std::vector<float>>wwv(fNumStrides+1, std::vector<float>(fWindowSize,0.));
    std::vector<std::vector<float>>predv(fNumStrides+1, std::vector<float>(1,0.));

    // .. fill each window with adc values
    unsigned int j1, j2, k;
    for (unsigned int i=0; i<fNumStrides; i++){
      j1 = i*fStrideLength;
      j2 = j1 + fWindowSize;
      k = 0;
      for (unsigned int j=j1; j<j2; j++){
	wwv[i][k]=adc[j];
	k++;
      }
    }
    // .. last window is a special case
    j1 = fNumStrides*fStrideLength;
    j2 = j1 + fLastWindowSize;
    k=0;
    for (unsigned int j=j1; j<j2; j++){
      wwv[fNumStrides][k]=adc[j];
      k++;
    }

    // ... use waveform recognition CNN to perform inference on each window
    predv = fWaveformRecogTool->predictWaveformType(wwv);

    std::vector<float> sigs;
    int lastsignaltick = -1;
    int roistart = -1;

    recob::Wire::RegionsOfInterest_t rois(fWaveformSize);

    //for (size_t i = 0; i<predv.size(); ++i){
    for (size_t i = 0; i<fWaveformSize; ++i){
      bool isroi = false;
      // For ProtoDUNE, shape of predv is (40,1) for 40 windows and a single output/categor.
      // The window size is 200 ticks. The algorithm scans across the entire 6000 tick waveform in strides or step sizes of 150 ticks
      for (size_t j = 0; j<predv.size(); ++j){
        if (i >= j*fStrideLength &&
            i < j*fStrideLength + fWindowSize){
          if (predv[j][0]>0.5){
            isroi = true;
            break;
          }
        }
      }
      if (isroi){
        if (sigs.empty()){
          sigs.push_back(inputsignal[i]);
          lastsignaltick = i;
          roistart = i;
        }
        else{
          if (int(i)!=lastsignaltick+1){
            rois.add_range(roistart, std::move(sigs));
            sigs.clear();
            sigs.push_back(inputsignal[i]);
            lastsignaltick = i;
            roistart = i;
          }
          else {
            sigs.push_back(inputsignal[i]);
            lastsignaltick = i;
          }
        }
      }
    }
    if (!sigs.empty()){
      rois.add_range(roistart, std::move(sigs));
    }
    if (!wirelist.empty()){
      outwires->emplace_back(recob::Wire(rois, wirelist[ich]->Channel(), wirelist[ich]->View()));
    }
    else if (!rawlist.empty()){
      outwires->emplace_back(recob::Wire(rois, rawlist[ich]->Channel(), geo->View(rawlist[ich]->Channel())));
    }
  }

  e.put(std::move(outwires));

}

DEFINE_ART_MODULE(nnet::WaveformRoiFinder)
