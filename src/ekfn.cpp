
#include <XmlRpcException.h>

#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include "robot_localization/ekfn.h"
#include "robot_localization/filter_common.h"

namespace RobotLocalization
{
    Ekfn::Ekfn(std::vector<double>) : FilterBase()  // Must initialize filter base!
    {
        id = -1;
        p_ekf = se_init();
    }

    Ekfn::~Ekfn()
    {
        se_deinit();
    }
    
    void Ekfn::predict(const double referenceTime, const double delta)
    {
        id += 1; 
        dumpState("k-1+", delta);

	// TODO Add controls
	// prepareControl(referenceTime, delta);
       
        se_predict(delta, id);
        
        dumpState("k-", delta);
    }

    void Ekfn::correct(const Measurement &measurement)
    {
        dumpState("k-", -1);

        int num_msmt = 0;
        int updateIndices[measurement.updateVector_.size()];
        for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
        {
            if (measurement.updateVector_[i])
            {
                // Handle nan and inf values in measurements
                if (!std::isnan(measurement.measurement_(i)) && !std::isinf(measurement.measurement_(i)))
                     updateIndices[num_msmt++] = i;
            }
        }
      
        std::vector<size_t> updateIndicesVec;
        updateIndicesVec.assign(updateIndices, updateIndices+num_msmt);       
        dumpMsmt("k-", measurement, updateIndicesVec);

        // populate msmt
        int count = 0;
        int msmt_size = measurement.measurement_.size();
        double msmt[msmt_size];
        for (size_t i = 0; i < msmt_size; ++i)
        {
	    msmt[count++] = measurement.measurement_[i];
	}

        // populate msmt covariance
        count=0; 
        double msmt_covariance[msmt_size*msmt_size];
        for (size_t i = 0; i < msmt_size; ++i)
        {
	    for (size_t j=0 ; j<msmt_size ; j++)
	    {
   	        msmt_covariance[count++] = measurement.covariance_(i, j);
 	    }
	}        
        se_update(msmt, updateIndices, num_msmt, msmt_covariance, id);

	// copy state
        for (int i=0; i<N_STATE; i++)
	    state_(i) = p_ekf->x[i];

	// copy covariance
	for (int i=0; i<N_STATE; i++)
	{
	  for (int j=0 ; j<N_STATE; j++)
	        estimateErrorCovariance_(i, j) = p_ekf->P[i][j];
	}    

	dumpState("k+", -1);
        dumpMsmt("k+", measurement, updateIndicesVec);
    }

  void Ekfn::dumpState(const std::string& suffix, const double & delta)
  {
    FB_DEBUG("EKFN_s_" << suffix << "_" << id << "=" << state_);
    FB_DEBUG("EKFN_p_" << suffix << "_" << id  << "=" << estimateErrorCovariance_);
    FB_DEBUG("EKFN_q_" << suffix << "_" << id << "=" << processNoiseCovariance_);
    if (delta > 0)
        FB_DEBUG("EKFN_d_" << suffix << "_" << id << "=" << delta << "\n");
  }

  void Ekfn::dumpMsmt(const std::string& suffix, const Measurement& msmt, const std::vector<size_t>& updateIndices)
  {
    FB_DEBUG("EKFN_m_" << suffix << "_" << id <<  "=" <<  msmt.measurement_);
    FB_DEBUG("EKFN_ms_" << suffix << "_" << id <<  "=" <<  updateIndices);
    FB_DEBUG("EKFN_r_" << suffix << "_" << id << "=" <<  msmt.covariance_);
  }
  


}  // namespace RobotLocalization
