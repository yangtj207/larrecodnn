////////////////////////////////////////////////////////////////////////
// Class:       EvaluateROIEff
// Plugin Type: analyzer (art v3_05_00)
// File:        EvaluateROIEff_module.cc
//
// Generated at Sun May  3 23:16:14 2020 by Tingjun Yang using cetskelgen
// from cetlib version v3_10_00.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art_root_io/TFileService.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"

#include "larcore/Geometry/Geometry.h"
#include "lardata/DetectorInfoServices/DetectorClocksService.h"
#include "lardata/DetectorInfoServices/DetectorPropertiesService.h"
#include "lardataobj/RecoBase/Wire.h"
#include "lardataobj/Simulation/SimChannel.h"
#include "larevt/CalibrationDBI/Interface/ChannelStatusProvider.h"
#include "larevt/CalibrationDBI/Interface/ChannelStatusService.h"

#include "TEfficiency.h"
#include "TH1D.h"

namespace nnet {
  class EvaluateROIEff;
}

class nnet::EvaluateROIEff : public art::EDAnalyzer {
public:
  explicit EvaluateROIEff(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  EvaluateROIEff(EvaluateROIEff const&) = delete;
  EvaluateROIEff(EvaluateROIEff&&) = delete;
  EvaluateROIEff& operator=(EvaluateROIEff const&) = delete;
  EvaluateROIEff& operator=(EvaluateROIEff&&) = delete;

  // Required functions.
  void analyze(art::Event const& e) override;

private:
  void beginJob() override;
  void endJob() override;
  // Declare member data here.
  TH1D* h_energy[3];
  TH1D* h_energy_roi[3];
  TEfficiency* eff_energy[3];

  art::InputTag fWireProducerLabel;
};

nnet::EvaluateROIEff::EvaluateROIEff(fhicl::ParameterSet const& p)
  : EDAnalyzer{p}, fWireProducerLabel(p.get<art::InputTag>("WireProducerLabel", ""))
// More initializers here.
{
  // Call appropriate consumes<>() for any products to be retrieved by this module.
}

void
nnet::EvaluateROIEff::analyze(art::Event const& e)
{

  auto const* geo = lar::providerFrom<geo::Geometry>();
  auto const* detprop = lar::providerFrom<detinfo::DetectorPropertiesService>();
  auto const* tclk = lar::providerFrom<detinfo::DetectorClocksService>();
  auto const& chStatus = art::ServiceHandle<lariov::ChannelStatusService>()->GetProvider();

  art::Handle<std::vector<recob::Wire>> wireListHandle;
  std::vector<art::Ptr<recob::Wire>> wires;
  if (e.getByLabel(fWireProducerLabel, wireListHandle)) {
    art::fill_ptr_vector(wires, wireListHandle);
  }

  auto simChannelHandle = e.getValidHandle<std::vector<sim::SimChannel>>("tpcrawdecoder:simpleSC");

  // ... Loop over simChannels
  for (auto const& channel : (*simChannelHandle)) {

    // .. get simChannel channel number
    const raw::ChannelID_t ch1 = channel.Channel();
    if (chStatus.IsBad(ch1)) continue;
    if (ch1 % 1000 == 0) mf::LogInfo("EvaluateROIEFF") << ch1;
    int view = geo->View(ch1);
    auto const& timeSlices = channel.TDCIDEMap();
    for (auto const& timeSlice : timeSlices) {

      auto const& energyDeposits = timeSlice.second;
      auto const tpctime = timeSlice.first;
      int tdctick = static_cast<int>(tclk->TPCTDC2Tick(double(tpctime)));
      //if(tdctick!=tpctime)std::cout << "tpctime: " << tpctime << ", tdctick: " << tdctick << std::endl;
      if (tdctick < 0 || tdctick > int(detprop->ReadOutWindowSize()) - 1) continue;
      double totalE = 0;
      for (auto const& energyDeposit : energyDeposits) {
        totalE += energyDeposit.energy;
      }
      //std::cout<<totalE<<std::endl;
      h_energy[view]->Fill(totalE);
      for (auto& wire : wires) {
        if (wire->Channel() != ch1) continue;
        const recob::Wire::RegionsOfInterest_t& signalROI = wire->SignalROI();
        for (const auto& range : signalROI.get_ranges()) {
          const auto& waveform = range.data();
          raw::TDCtick_t roiFirstBinTick = range.begin_index();
          if (tdctick >= roiFirstBinTick && tdctick < int(roiFirstBinTick + waveform.size())) {
            h_energy_roi[view]->Fill(totalE);
            break;
          }
        }
      }
    }
  }
}

void
nnet::EvaluateROIEff::beginJob()
{

  art::ServiceHandle<art::TFileService const> tfs;

  for (int i = 0; i < 3; ++i) {
    h_energy[i] =
      tfs->make<TH1D>(Form("h_energy_%d", i), Form("Plane %d; Energy (MeV);", i), 100, 0, 1);
    h_energy_roi[i] =
      tfs->make<TH1D>(Form("h_energy_roi_%d", i), Form("Plane %d; Energy (MeV);", i), 100, 0, 1);
  }
}

void
nnet::EvaluateROIEff::endJob()
{

  art::ServiceHandle<art::TFileService const> tfs;

  for (int i = 0; i < 3; ++i) {
    std::cout << i << " " << h_energy[i]->GetNbinsX() << " " << h_energy_roi[i]->GetNbinsX()
              << std::endl;
    if (TEfficiency::CheckConsistency(*(h_energy[i]), *(h_energy_roi[i]))) {
      eff_energy[i] = tfs->make<TEfficiency>(*(h_energy_roi[i]), *(h_energy[i]));
      eff_energy[i]->Write(Form("eff_energy_%d", i));
    }
  }
}

DEFINE_ART_MODULE(nnet::EvaluateROIEff)
