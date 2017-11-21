#ifndef PVTP_STATE_H
#define PVTP_STATE_H

#include <iostream>
#include <PVTP/PVT_Point.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time State class. Stores a point in PVT space, which is
	 * regarded as a system state by the planner. There are also some utility
	 * functions associated with states.
	 */
	class PVT_State {
		
		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_State& pvt_s);
		
	public:
		
		/**
		 * PVT State constructor: Default.
		 */
		PVT_State ( );

		/**
		 * PVT State constructor: Copy.
		 */
		PVT_State ( const PVT_State& s );
		
		/**
		 * PVT State constructor: Specified.
		 */
		PVT_State ( double p, double t, double v );
		
		/**
		 * PVT State constructor: Specified.
		 */
		PVT_State ( const PVT_Point& p, double v );
		
		/**
		 * Set the coordinates of a PVT State object
		 */
		void setCoords( double p, double t, double v ) {
			this->p = p;
			this->t = t;
			this->v = v;
		}
		
		/**
		 * Set the coordinates of a PVT State object
		 */
		void setCoords( const PVT_State& s ) {
			this->setCoords( s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord() );
		}
		
		/**
		 * Accessor for path coordinate.
		 */
		double getPathCoord( void ) const {
			return this->p;
		}
		
		/**
		 * Accessor for time coordinate.
		 */
		double getTimeCoord( void ) const {
			return this->t;
		}
		
		/**
		 * Accessor for velocity coordinate.
		 */
		double getVelocityCoord( void ) const {
			return this->v;
		}
		
		/**
		 * Fast test for reachability from this state to a given point.
		 */
		bool canReach( const PVT_Point& p2, const Constraints& c ) const;
		
		/**
		 * Test for state equality
		 */
		bool equals( const PVT_State& s2, const Constraints& c ) const {
			return Maths::approxEq( this->getPathCoord(), s2.getPathCoord(), c.getEpsilon() )
			&& Maths::approxEq( this->getTimeCoord(), s2.getTimeCoord(), c.getEpsilon() )
			&& Maths::approxEq( this->getVelocityCoord(), s2.getVelocityCoord(), c.getEpsilon() );
		}
		
		/**
		 * Test for state equality
		 *
		 * NOTE: This test ignores the velocity coordinate
		 */
		bool equals( const PVT_Point& p2, const Constraints& c ) const {
			return Maths::approxEq( this->getPathCoord(), p2.getPathCoord(), c.getEpsilon() )
			&& Maths::approxEq( this->getTimeCoord(), p2.getTimeCoord(), c.getEpsilon() );
		}
		
		/**
		 * Test for point component: given PVT_Point
		 */
		bool atPoint( const PVT_Point& p, const Constraints& c ) const {
			return this->atPoint( p.getPathCoord(), p.getTimeCoord(), c );
		}
		
		/**
		 * Test for point component: given doubles
		 */
		bool atPoint( double x, double t, const Constraints& c ) const {
			return Maths::approxEq( this->getPathCoord(), x, c.getEpsilon() )
			&& Maths::approxEq( this->getTimeCoord(), t, c.getEpsilon() );
		}
		
		/**
		 * Translate this state in the path-time plane
		 */
		void translate( double p_offset, double t_offset ) {
			this->p = this->p + p_offset;
			this->t = this->t + t_offset;
		}
		
	private:
		
		/**
		 * The path coordinate in PVT space.
		 */
		double p;
		
		/**
		 * The time coordinate in PVT space.
		 */
		double t;
		
		/**
		 * The velocity coordinate in PVT space.
		 */
		double v;
		
		
	};

} // end PVTP namespace

#endif
