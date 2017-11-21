#ifndef PVTP_OBSTACLE_POINT_H
#define PVTP_OBSTACLE_POINT_H

#include <PVTP/PVT_Point.hpp>

namespace PVTP {
	
	class PVT_ObstacleEdge;
	
	/**
	 * Path-Time Obstacle Point class. This class contains PVT_Points that are
	 * obstacle vertices. These are PVT_Points that contain an extra bit flag
	 * denoting whether a trajectory must pass above them (0) or below them (1)
	 * in order to remain collision-free and respect monotonicity constraints.
	 */
	class PVT_ObstaclePoint : public PVT_Point {

		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_ObstaclePoint& obs_p);
		
	public:
		
		/**
		 * PVT Obstacle Point constructor: unspecified interval.
		 */
		PVT_ObstaclePoint ( double p, double t, const Constraints& c );
		
		/**
		 * PVT Obstacle Point constructor: specified interval.
		 */
		PVT_ObstaclePoint ( double p, double t, const Interval& i, const Constraints& c );
		
		/**
		 * PVT Obstacle Point constructor: specified interval.
		 */
		PVT_ObstaclePoint ( double p, double t, double min, double max, const Constraints& c );
		
		/**
		 * PVT Obstacle Point constructor: copy
		 */
		PVT_ObstaclePoint ( const PVT_Point& p, const Constraints& c );

		/**
		 * PVT Obstacle Point constructor: copy
		 */
		PVT_ObstaclePoint ( const PVT_ObstaclePoint& p, const Constraints& c );
		
		/**
		 * If the admissable velocities at this point prevent it from being
		 * feasibly reached at all, return false.
		 */
		bool isReachable( const Constraints& c ) const;
		
		/**
		 * The incoming edge to this point on the obstacle
		 */
		PVT_ObstacleEdge * incoming_edge;
		
		/**
		 * The outgoing edge to this point on the obstacle
		 */
		PVT_ObstacleEdge * outgoing_edge;
		
	private:
		
		/**
		 * Called on initialization to adjust reachable interval based on
		 * whether point coordinates are valid, and set type
		 */
		void init( const Constraints& c );
		
	};

	/**
	 * A function object used as a comparator to sort points by time coordinate.
	 *
	 * Sort order: Ascending.
	 */
	struct PVT_ObstaclePointComparator {
		double epsilon;
		bool operator() (PVT_ObstaclePoint * A, PVT_ObstaclePoint * B) {
			if ( Maths::approxGt(B->getTimeCoord(), A->getTimeCoord(), epsilon) ) {
				return true;
			} else if ( Maths::approxEq(B->getTimeCoord(), A->getTimeCoord(), epsilon) ) {
				if ( B->getPathCoord() > A->getPathCoord() ) {
					return true;
				}
			}
			return false;
		}
	};
	
} // end PVTP namespace

#endif
