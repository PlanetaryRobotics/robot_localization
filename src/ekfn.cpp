
#include <XmlRpcException.h>

#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include "robot_localization/ekfn.h"
#include "robot_localization/filter_common.h"

#include "ekf_native/tiny_ekf.h"

extern "C" 
{
    void rl_init();
    void rl_predict(double delta);
    void rl_update(int* msmt_idx, int num_msmt, double* msmt, double** msmt_covariance);
}

namespace RobotLocalization
{
    Ekfn::Ekfn(std::vector<double>) : FilterBase()  // Must initialize filter base!
    {
        rl_init();
    }

    Ekfn::~Ekfn()
    {
    }
    
    void Ekfn::predict(const double referenceTime, const double delta)
    {
        rl_predict(delta);
    }

    void Ekfn::correct(const Measurement &measurement)
    {
        std::vector<size_t> updateIndices;
        for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
        {
            if (measurement.updateVector_[i])
            {
                // Handle nan and inf values in measurements
                if (!std::isnan(measurement.measurement_(i)) && !std::isinf(measurement.measurement_(i)))
                {
                     updateIndices.push_back(i);
                }
            }
        }
        
        int num_msmt = updateIndices.size();
        double msmt[num_msmt];
        double msmt_covariance[num_msmt][num_msmt];
        for (size_t i = 0; i < num_msmt; ++i)
        {
            msmt[i] = measurement.measurement_(updateIndices[i]);
            for (size_t j = 0; j < num_msmt; ++j)
            {
                msmt_covariance[i][j] = measurement.covariance_(updateIndices[i], updateIndices[j]);

		// Handle negative (read: bad) covariances in the measurement. Rather
		// than exclude the measurement or make up a covariance, just take
		// the absolute value.
		if (msmt_covariance[i][j] < 0)
		{
		    msmt_covariance[i][j] = ::fabs(msmt_covariance[i][j] );
		}

		// If the measurement variance for a given variable is very
                // near 0 (as in e-50 or so) and the variance for that
                // variable in the covariance matrix is also near zero, then
                // the Kalman gain computation will blow up. Really, no
                // measurement can be completely without error, so add a small
                // amount in that case.
                if (msmt_covariance[i][j]  < 1e-9)
                {
		    msmt_covariance[i][j]  = 1e-9;
		}
	    }
	}        
    }

}  // namespace RobotLocalization
