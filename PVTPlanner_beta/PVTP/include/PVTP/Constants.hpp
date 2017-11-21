#ifndef PVTP_CONSTANTS_H
#define PVTP_CONSTANTS_H

#ifdef BUILDGSL
#if BUILDGSL
#include <gsl/gsl_sf_ellint.h>
#endif
#endif

namespace PVTP {
	
	/**
	 * This namespace contains constants specific to the library.
	 */
	namespace Constants {
		
		/**
		 * Used by specialUnionOfIntervals to mark a number as an interval min
		 */
		static const int INTERVAL_MIN_MARKER = -1;
		
		/**
		 * Used by specialUnionOfIntervals to mark a number an an interval max
		 */
		static const int INTERVAL_MAX_MARKER = 1;
		
		/**
		 * Used by PVT_ObstaclePoint to indicate that a trajectory must pass
		 * above a vertex
		 */
		static const char H_CLASS_AFTER = 1;

		/**
		 * Used by PVT_ObstaclePoint to indicate that a trajectory must pass
		 * below a vertex
		 */
		static const char H_CLASS_BEFORE = 0;
		
		/**
		 * Used by PVT_ObstaclePoint to indicate that a vertex lies on a
		 * trajectory
		 */
		static const char H_CLASS_ON = -1;
		
		/**
		 * The origin needs a homotopic classification; by convention, set it here.
		 */
		static const char H_CLASS_ORIGIN = H_CLASS_BEFORE;
		
		/**
		 * Exception codes: Constraints
		 */
		static const int ACC_VIOLATION = __LINE__;
		static const int VEL_VIOLATION = __LINE__;
		static const int PATH_VIOLATION = __LINE__;
		static const int TIME_VIOLATION = __LINE__;
		static const int EPSILON_VIOLATION = __LINE__;
		
		/**
		 * Exception codes: PVT Obstacles
		 */
		static const int INSUFFICIENT_VERTICES = __LINE__;
		
		/**
		 * Exception codes: Vehicle
		 */
		static const int BAD_ORIENTATION = __LINE__;
		static const int BAD_VERTEX = __LINE__;
		
		/**
		 * Exception codes: Path Segment
		 */
		static const int ZERO_LENGTH = __LINE__;
		static const int BAD_POLY_ORDER = __LINE__;
		static const int NO_END_POINT = __LINE__;
		
		/**
		 * Exception codes: Path Model
		 */
		static const int ZERO_SEGMENTS = __LINE__;
		static const int DISCONNECTED_PATH_SEGMENT = __LINE__;
		
		/**
		 * A default value for epsilon: I've found this value to be reasonably
		 * good in practice; making the value too small exacerbates problems due
		 * to discretization error, and making it too big makes it more likely
		 * that correctness assumptions that the algorithm relies on for proper
		 * operation are violated.
		 */
		static const double DEFAULT_EPSILON = 0.000000005;
		
		/**
		 * For debugging purposes a really high output precision is needed,
		 * I've found this value to be good
		 */
		static const double DEBUGGING_OUTPUT_PRECISION = 30;
		
#ifdef BUILDGSL
#if BUILDGSL
		/**
		 * Accuracy mode for GSL functions
		 */
		static const gsl_mode_t GSL_MODE = GSL_PREC_APPROX;
#endif
#endif

		/**
		 * For trajectories to be able to be executed, the durations of their
		 * segments must be integer multiples of a given time step. This constant
		 * defines a minimum remainder a segment must have to be considered 
		 * a multiple.
		 */
		static const double DURATION_SAFE_REMAINDER = 0.01;
		
	} // end Constants namespace

} // end PVTP namespace

#endif