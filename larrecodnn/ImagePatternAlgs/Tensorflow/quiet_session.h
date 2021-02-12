#ifndef larrecodnn_ImagePatternAlgs_Tensorflow_quiet_session_h
#define larrecodnn_ImagePatternAlgs_Tensorflow_quiet_session_h

// TensorFlow has sign-comparison issues in their session header.
// This file should be included to disable such warnings.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "tensorflow/core/public/session.h"
#pragma GCC diagnostic pop

#endif /* larrecodnn_ImagePatternAlgs_Tensorflow_quiet_session_h */
