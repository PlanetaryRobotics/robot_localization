
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
        idx = -1;
        se_init(p_ekf);

        int i, j;
        for (i=0; i<N_STATE; ++i)
            for (j=0; i<N_STATE; ++i)
                p_ekf->F[i][j] = 0;

        for (i=0; i<N_STATE; ++i)
            p_ekf->A[i][i] = 1;    
    }

    Ekfn::~Ekfn()
    {
    }
    
    void Ekfn::predict(const double referenceTime, const double delta)
    {
        se_predict(delta, p_ekf);
    }

    void Ekfn::correct(const Measurement &measurement)
    {
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
       
        int count=0; 
        int msmt_size = measurement.measurement_.size();
        double msmt[msmt_size];
        double msmt_covariance[msmt_size*msmt_size];
        for (size_t i = 0; i < msmt_size; ++i)
        {
	    for (size_t j=0 ; j<msmt_size ; j++)
	    {
   	        msmt_covariance[count++] = measurement.covariance_(i, j);
		if (msmt_covariance[count-1] < 0)
		    msmt_covariance[count-1] = ::fabs(msmt_covariance[count-1] );
                if (msmt_covariance[count-1]  < 1e-9)
		    msmt_covariance[count-1]  = 1e-9;
 	    }
	}        
        se_update(msmt, updateIndices, num_msmt, msmt_covariance, p_ekf, idx);
    }

}  // namespace RobotLocalization
