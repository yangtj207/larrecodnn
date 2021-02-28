////////////////////////////////////////////////////////////////////////
// Class:       CheckCNNScore
// Plugin Type: analyzer (art v3_02_06)
// File:        CheckCNNScore_module.cc
//
// Generated at Tue Oct  1 11:26:39 2019 by Tingjun Yang using cetskelgen
// from cetlib version v3_07_02.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "art_root_io/TFileService.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "lardata/ArtDataHelper/MVAReader.h"
#include "lardataobj/RecoBase/Hit.h"
#include "messagefacility/MessageLogger/MessageLogger.h"

#include "TTree.h"

namespace pdsp {
  class CheckCNNScore;
}

class pdsp::CheckCNNScore : public art::EDAnalyzer {
public:
  explicit CheckCNNScore(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  CheckCNNScore(CheckCNNScore const&) = delete;
  CheckCNNScore(CheckCNNScore&&) = delete;
  CheckCNNScore& operator=(CheckCNNScore const&) = delete;
  CheckCNNScore& operator=(CheckCNNScore&&) = delete;

  // Required functions.
  void analyze(art::Event const& e) override;

  // Selected optional functions.
  void beginJob() override;

private:
  // Declare member data here.
  // Input parameters
  art::InputTag fNNetModuleLabel; // label of the module used for CNN tagging
  art::InputTag fHitsModuleLabel; // label of hit finder module
  std::vector<std::string> fNNOutputs;  // label of network outputs

  TTree* ftree;
  int run;
  int subrun;
  int event;
  std::vector<short> channel;
  std::vector<short> tpc;
  std::vector<short> plane;
  std::vector<short> wire;
  std::vector<double> charge;
  std::vector<double> peakt;
  std::vector<double> score_0;
  std::vector<double> score_1;
  std::vector<double> score_2;
  std::vector<double> score_3;
};

pdsp::CheckCNNScore::CheckCNNScore(fhicl::ParameterSet const& p)
  : EDAnalyzer{p}
  , fNNetModuleLabel(p.get<art::InputTag>("NNetModuleLabel"))
  , fHitsModuleLabel(p.get<art::InputTag>("HitsModuleLabel"))
  , fNNOutputs(p.get<std::vector<std::string>>("NNOutputs"))
{}

void
pdsp::CheckCNNScore::analyze(art::Event const& e)
{

  run = e.run();
  subrun = e.subRun();
  event = e.id().event();
  channel.clear();
  tpc.clear();
  plane.clear();
  wire.clear();
  charge.clear();
  peakt.clear();
  score_0.clear();
  score_1.clear();
  score_2.clear();
  score_3.clear();

  anab::MVAReader<recob::Hit, 4> hitResults(e, fNNetModuleLabel);

  art::Handle<std::vector<recob::Hit>> hitListHandle;
  std::vector<art::Ptr<recob::Hit>> hitlist;
  if (e.getByLabel(fHitsModuleLabel, hitListHandle)) art::fill_ptr_vector(hitlist, hitListHandle);

  // loop over hits
  //  for (size_t h = 0; h < hitResults.size(); ++h) {
  for (auto& hit : hitlist) {

    // Get cnn output for hit h
    std::array<float, 4> cnn_out = hitResults.getOutput(hit);

    if (hit->WireID().Plane == 2) {
      channel.push_back(hit->Channel());
      tpc.push_back(hit->WireID().TPC);
      plane.push_back(hit->WireID().Plane);
      wire.push_back(hit->WireID().Wire);
      charge.push_back(hit->Integral());
      peakt.push_back(hit->PeakTime());
      for (size_t i = 0; i<fNNOutputs.size(); ++i){
        if (i==0) score_0.push_back(cnn_out[hitResults.getIndex(fNNOutputs[0])]);
        if (i==1) score_1.push_back(cnn_out[hitResults.getIndex(fNNOutputs[1])]);
        if (i==2) score_2.push_back(cnn_out[hitResults.getIndex(fNNOutputs[2])]);
        if (i==3) score_3.push_back(cnn_out[hitResults.getIndex(fNNOutputs[3])]);
      }
      //      std::cout<<hit->WireID().TPC<<" "
      //               <<hit->WireID().Wire<<" "
      //               <<hit->PeakTime()<<" "
      //               <<cnn_out[hitResults.getIndex("el")]<<" "
      //               <<cnn_out[hitResults.getIndex("inel")]<<" "
      //               <<cnn_out[hitResults.getIndex("none")]<<std::endl;
    }
  }
  if (!channel.empty()) ftree->Fill();
}

void
pdsp::CheckCNNScore::beginJob()
{
  art::ServiceHandle<art::TFileService> fileServiceHandle;
  ftree = fileServiceHandle->make<TTree>("ftree", "hit info");
  ftree->Branch("run", &run, "run/I");
  ftree->Branch("event", &event, "event/I");
  ftree->Branch("channel", &channel);
  ftree->Branch("tpc", &tpc);
  ftree->Branch("plane", &plane);
  ftree->Branch("wire", &wire);
  ftree->Branch("charge", &charge);
  ftree->Branch("peakt", &peakt);
  ftree->Branch("score_0", &score_0);
  ftree->Branch("score_1", &score_1);
  ftree->Branch("score_2", &score_2);
  ftree->Branch("score_3", &score_3);
 
}

DEFINE_ART_MODULE(pdsp::CheckCNNScore)
