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

        // ---------------------------------------------------------------------
        // Return a vector of booleans of the same size as the input  waveform.
	// The value of each element of the vector represents whether the 
	// corresponding time bin of the waveform is in an ROI or not.
        // ---------------------------------------------------------------------
        std::vector<bool> findROI(const std::vector<float>& adcin) const
        {
          std::vector<bool>bvec(fWaveformSize,false);
          if(adcin.size()!=fWaveformSize){
            return bvec;
          }

          std::vector<std::vector<float>> predv = scanWaveform(adcin);

          // .. set to true all bins in the output vector that are in windows identified as signals
          int j1;
	  for (unsigned int i=0; i<fNumStrides; i++){
            j1 = i*fStrideLength;
            if(predv[i][0]>fCnnPredCut){
              std::fill_n(bvec.begin()+j1,fWindowSize,true);
            }
          }
          // .. last window is a special case
          if(predv[fNumStrides][0]>fCnnPredCut){
            j1 = fNumStrides*fStrideLength;
            std::fill_n(bvec.begin()+j1,fLastWindowSize,true);
          }
          return bvec;
        }

        // -------------------------------------------------------------
        // Return a vector of floats of the same size as the input 
	// waveform. The value in each bin represents the probability
	// whether that bin is in an ROI or not
        // -------------------------------------------------------------
	std::vector<float> predROI(const std::vector<float>& adcin) const
        {
          std::vector<float>fvec(fWaveformSize,0.);
          if(adcin.size()!=fWaveformSize){
            return fvec;
          }

          std::vector<std::vector<float>> predv = scanWaveform(adcin);

          // .. set value in each bin of output vector to the prediction for the window it is in
          int j1;
	  for (unsigned int i=0; i<fNumStrides; i++){
            j1 = i*fStrideLength;
            std::fill_n(fvec.begin()+j1,fWindowSize,predv[i][0]);
          }
          // .. last window is a special case
          j1 = fNumStrides*fStrideLength;
          std::fill_n(fvec.begin()+j1,fLastWindowSize,predv[fNumStrides][0]);
          return fvec;
        }

    protected:
        std::vector<float>scalevec;
        std::vector<float>meanvec;
	float fCnnPredCut;
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

    private:
        std::vector<std::vector<float>> scanWaveform(const std::vector<float>& adcin) const
	{
          // .. rescale input waveform for CNN
          std::vector<float>adc(fWaveformSize);
          for (size_t itck = 0; itck < fWaveformSize; ++itck){
            adc[itck] = (adcin[itck] - meanvec[itck])/scalevec[itck];
          }

          // .. create a vector of windows
          std::vector<std::vector<float>>wwv(fNumStrides+1, std::vector<float>(fWindowSize,0.));

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
          return predictWaveformType(wwv);
	}

    };
}

#endif
