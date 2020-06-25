#ifndef IWaveformRecog_H
#define IWaveformRecog_H

#include <sys/stat.h>
#include "fhiclcpp/ParameterSet.h"
#include "canvas/Utilities/Exception.h"

namespace wavrec_tool
{
    class IWaveformRecog
    {
    public:
        virtual ~IWaveformRecog() noexcept = default;

        // Calculate multi-class probabilities for waveform
        virtual std::vector< std::vector<float> > predictWaveformType( const std::vector< std::vector<float> >& ) const = 0;

        std::vector<bool> findROI(const std::vector<float>& adcin) const
        {
          std::vector<bool>bvec(fWaveformSize,false);
          if(adcin.size()!=fWaveformSize){
            return bvec;
          }

          // .. rescale input waveform for CNN
          std::vector<float>adc(fWaveformSize);
          for (size_t itck = 0; itck < fWaveformSize; ++itck){
            adc[itck] = (adcin[itck] - meanvec[itck])/scalevec[itck];
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
          predv = predictWaveformType(wwv);

          // .. set all bins in the result vector, corresponding to windows identified as signals, to true
          for (unsigned int i=0; i<fNumStrides; i++){
            j1 = i*fStrideLength;
            if(predv[i][0]>0.5){
              std::fill_n(bvec.begin()+j1,fWindowSize,true);
            }
          }
          // .. last window is a special case
          if(predv[fNumStrides][0]>0.5){
            j1 = fNumStrides*fStrideLength;
            std::fill_n(bvec.begin()+j1,fLastWindowSize,true);
          }
          return bvec;
        }

    protected:
        std::vector<float>scalevec;
        std::vector<float>meanvec;
        unsigned int fWaveformSize;	    // Full waveform size
        unsigned int fWindowSize;	    // Scan window size
        unsigned int fStrideLength;	    // Offset (in #time ticks) between scan windows
        unsigned int fNumStrides;
        unsigned int fLastWindowSize;

        std::string findFile(const char* fileName) const
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
    };
}

#endif
