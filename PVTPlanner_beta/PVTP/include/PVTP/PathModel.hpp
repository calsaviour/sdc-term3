#ifndef PVTP_PATH_MODEL_H
#define PVTP_PATH_MODEL_H

#include <PVTP/PathSegment.hpp>

using namespace PVTP;

namespace Scenario {
	
	/**
	 * User Path class. This class is for contructing and handling User paths,
	 * which are defined as piecewise range-bounded polynomials in X.
	 */
	class PathModel {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PathModel& path);
		
	public:
		
		/**
		 * Constructor
		 */
		PathModel( std::vector<PathSegment>& segments, const Constraints& c );
		
		/**
		 * Copy constructor
		 */
		PathModel( const PathModel& path );
		
		/**
		 * The set of user path segments
		 */
		std::vector<PathSegment> segments;
		
		/**
		 * Compute the arc length from the beginning of the path to a specified point.
		 */
		double getArcLengthToPoint( const XY_Point& target_point, const Constraints& c ) const;
		
		/**
		 * Compute the arc length between two points on this path.
		 */
		double getArcLengthBetweenPoints( const XY_Point& start_point, const XY_Point& end_point, const Constraints& c ) const;
		
		/**
		 * Given an arc length, determine X and Y coordinate. Returns the path
		 * segment index corresponding to the arc length, or -1 on failure.
		 */
		ssize_t getPositionVectorAtArcLength( double& x, double& y, double arc_length, const Constraints& c ) const;
		
		/**
		 * Given an arc length, determine X and Y coordinate. Returns the path
		 * segment index corresponding to the arc length, or -1 on failure.
		 */
		ssize_t getArcLengthAtPositionVector( double& arc_length, const XY_Point& p, const Constraints& c ) const;
		
		/**
		 * Given an arc length, determine X and Y coordinate. Returns the path
		 * segment index corresponding to the arc length, or -1 on failure.
		 */
		ssize_t getArcLengthAtPositionVector( double& arc_length, double x, double y, const Constraints& c ) const;
		
		/**
		 * Get the orientation (clockwise angle of deviation from x-axis in radians) at a specified arc length
		 */
		bool getSlopeAtArcLength( double& slope, double arc_length, const Constraints& c ) const;
		
		/**
		 * Get the orientation (clockwise angle of deviation from x-axis in radians) at a specified arc length
		 */
		bool getOrientationAtArcLength( double& orientation, double arc_length, const Constraints& c, bool orient_to_prev = false ) const;
		
		/**
		 * Get the initial coordinate of this path
		 */
		const XY_Point& getInitialPoint( void ) const {
			return this->segments.front().getOrigin();
		}
		
		/**
		 * Get the final coordinate of this path
		 */
		const XY_Point& getFinalPoint( void ) const {
			return this->segments.back().getEndPoint();
		}
		
		/**
		 * Accessor for path length
		 */
		double getLength( void ) const {
			return this->length;
		}
		
		/**
		 * Translate this path
		 */
		void translate( double x, double y, const Constraints& c ) {
			for ( size_t i=0; i<this->segments.size(); ++i ) {
				this->segments[i].translate( x, y, c );
			}
		}
		
		/**
		 * Message handler for exceptions thrown during construction
		 */
		static std::string exceptionMessage( int e );
		
	private:
		
		/**
		 * Store the length of this path
		 */
		double length;
		
	};
	
} // end Scenario namespace

#endif