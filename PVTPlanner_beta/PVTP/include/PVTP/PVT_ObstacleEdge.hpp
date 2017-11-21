#ifndef PVTP_OBSTACLEEDGE_H
#define PVTP_OBSTACLEEDGE_H

#include <iostream>
#include <PVTP/Constraints.hpp>
#include <PVTP/PVT_ObstaclePt.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time Obstacle Edge class. This class is for contructing and
	 * handling PT obstacle edges.
	 */
	class PVT_ObstacleEdge {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_ObstacleEdge& oe);
		
		public:
		
		/**
		 * PVT Obstacle Edge constructor
		 */
		PVT_ObstacleEdge ( const PVT_ObstaclePoint& p1, const PVT_ObstaclePoint& p2, const Constraints& c );
		
		/**
		 * PVT Obstacle Edge copy constructor
		 */
		PVT_ObstacleEdge ( const PVT_ObstacleEdge& edge );
		
		/**
		 * Destructor
		 */
		~PVT_ObstacleEdge ();
		
		/**
		 * Accessor for segment intial point
		 */
		const PVT_ObstaclePoint& getInitialPoint( void ) const;
		
		/**
		 * Accessor for segment final point
		 */
		const PVT_ObstaclePoint& getFinalPoint( void ) const;
		
		/**
		 * Get the slope of this segment
		 *
		 * NOTE: By convention in this library, slopes are always given as delta path / delta time
		 */
		double getSlope( void ) const;
		
		/**
		 * Get the path intercept of the line this segment belongs to
		 */
		double getPathIntercept( void ) const;
		
		/**
		 * Get the change in the path coordinate represented by this edge
		 */
		double getPathDelta( void ) const;
		
		/**
		 * Get the change in the time coordinate represented by this edge
		 */
		double getTimeDelta( void ) const;
		
		/**
		 * Check whether a given point lies on this segment
		 */
		bool contains( const PVT_Point& p, const Constraints& c ) const;
		
		/**
		 * Check whether a given set of coordinates lies on this segment
		 */
		bool contains( double p, double t, const Constraints& c ) const;
		
		/**
		 * Translate the edge
		 */
		void translate( double path_offset, double time_offset, const Constraints& c );
		
		/**
		 * Test for equality
		 */
		bool equals( const PVT_ObstacleEdge& edge, const Constraints& c ) {
			return this->p1->equals( edge.getInitialPoint(), c )
			&& this->p2->equals( edge.getFinalPoint(), c );
		}

		private:
		
		/**
		 * Convenience function for initialize slope and intercept of the edge
		 */
		void init( const Constraints& c );
		
		/**
		 * The initial point on the segment
		 */
		PVT_ObstaclePoint * p1;
		
		/**
		 * The final point on the segment
		 */
		PVT_ObstaclePoint * p2;
		
		/**
		 * Store the slope of this segment
		 */
		double slope;
		
		/**
		 * Store the path intercept of the line the segment belongs to
		 */
		double path_intercept;
		
		/**
		 * Store the path delta for this segment
		 */
		double path_delta;
		
		/**
		 * Store the time delta for this segment
		 */
		double time_delta;

	};

} // end PVTP namespace

#endif
