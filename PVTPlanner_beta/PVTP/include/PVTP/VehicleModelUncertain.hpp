#ifndef PVTP_VEHICLE_MODEL_UNCERTAIN_H
#define PVTP_VEHICLE_MODEL_UNCERTAIN_H

#define BUILDBOOST1 1

#ifdef BUILDBOOST1
#if BUILDBOOST1

#include <PVTP/VehicleModel.hpp>

using namespace PVTP;

namespace Scenario {
	
	/**
	 * Uncertain Vehicle Model class. This class maintains two estimates of
	 * of vehicle state corresponding to the min and max of some confidence
	 * interval.
	 *
	 * NOTE: The Boost geometry library is required for this class.
	 */
	class VehicleModelUncertain {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const VehicleModelUncertain& vm);
		
	public:
		
		/**
		 * Constructor
		 */
		VehicleModelUncertain ( const VehicleModel& vm, double velocity_spread = 0., double acc_spread = 0. );
		
		/**
		 * Copy constructor
		 */
		VehicleModelUncertain ( const VehicleModelUncertain& vm );
		
		/**
		 * Destructor
		 */
		~VehicleModelUncertain ( );
		
		/**
		 * Accessor for minimum estimate model
		 */
		VehicleModel& getMinEstimateModel( void ) {
			return *this->min_estimate_model;
		}
		
		/**
		 * Accessor for maximum estimate model
		 */
		VehicleModel& getMaxEstimateModel( void ) {
			return *this->max_estimate_model;
		}
		
		/**
		 * Move the vehicles along the path according to its current state.
		 *
		 * Return true if either vehicle has path left for movement; false otherwise.
		 */
		bool translateInTime( double t ) {
			bool min = this->getMinEstimateModel().translateInTime( t );
			bool max = this->getMaxEstimateModel().translateInTime( t );
			return min || max;
		}
		
		/**
		 * Build and return a vehicle model corresponding to the swept
		 * region between the min and max vehicle estimates
		 */
		static VehicleModel * getSweptRegion( VehicleModel& min_estimate_model, VehicleModel& max_estimate_model );
		
		/**
		 * Build and return a vehicle model corresponding to the swept
		 * region between the min and max vehicle estimates
		 */
		static VehicleModel * getSweptRegion( VehicleModelUncertain& min_estimate_model, VehicleModelUncertain& max_estimate_model );
		
		/**
		 * Object-specific version of above
		 */
		VehicleModel * getSweptRegion( void );
		
	private:
		
		/**
		 * The vehicle model for the minimum state
		 */
		VehicleModel * min_estimate_model;
		
		/**
		 * The vehicle model for the maximum state
		 */
		VehicleModel * max_estimate_model;
		
		/**
		 * The initial spread on velocity
		 */
		double velocity_spread;
		
		/**
		 * The spread on acceleration
		 */
		double acc_spread;

	};
	
} // end Scenario namespace

#endif
#endif

#endif