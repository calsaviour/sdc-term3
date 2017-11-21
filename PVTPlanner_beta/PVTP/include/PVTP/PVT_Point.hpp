#ifndef PVTP_POINT_H
#define PVTP_POINT_H

#include <iostream>
#include <PVTP/Interval.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time Point class. This class contains point in PT space
	 * that are velocity constrained by some velocity interval: Trajectories
	 * can only pass through this point if their velocity at this point is
	 * contained on the constraint interval. Points without specified velocity
	 * constraints are to be treated as if the velocity constraint were the
	 * interval [-Inf, +Inf] (note that this interval is implemented as the 
	 * positive and negative limits of double representation).
	 *
	 * The name is a misnomer, since this is actually a line in PVT space. I
	 * call it a point because the algorithm deals with points in PT space.
	 */
	class PVT_Point {
		
		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_Point& pvt_p);
		
	public:
		
		/**
		 * Assignment operator
		 */
		PVT_Point& operator=( const PVT_Point& rhs ) {
			if ( this != &rhs ) {
				this->p = rhs.p;
				this->t = rhs.t;
				this->i = rhs.i;
			}
			return *this;
		}

		/**
		 * PVT Point constructor: unspecified interval.
		 */
		PVT_Point ( double p, double t );
		
		/**
		 * PVT Point constructor: specified interval.
		 */
		PVT_Point ( double p, double t, const Interval& i );
		
		/**
		 * PVT Point constructor: specified interval.
		 */
		PVT_Point ( double p, double t, double min, double max );
		
		/**
		 * PVT Point constructor: copy
		 */
		PVT_Point ( const PVT_Point& p );
		
		/**
		 * Accessor for path coordinate.
		 */
		double getPathCoord() const {
			return this->p;
		}
		
		/**
		 * Accessor for time coordinate.
		 */
		double getTimeCoord() const {
			return this->t;
		}
		
		/**
		 * Test for approximate equality in path and time coordinates of two
		 * PVT_Point objects; this does *not* consider the Interval member.
		 */
		bool equals( const PVT_Point& pvt_p, const Constraints& c ) const {
			return Maths::approxEq( this->getPathCoord(), pvt_p.getPathCoord(), c.getEpsilon() )
			&& Maths::approxEq( this->getTimeCoord(), pvt_p.getTimeCoord(), c.getEpsilon() );
		}
		
		/**
		 * Mutator for PT coordinates of point.
		 */
		void setCoords( double p, double t ) {
			// enforce a positive sign on zero
			if ( p == 0. ) {
				p = 0.;
			}
			if ( t == 0. ) {
				t = 0.;
			}
			this->p = p;
			this->t = t;
		}
		
		/**
		 * Translate the point by the given path and time coordinates
		 */
		void translate( double p, double t ) {
			this->setCoords( this->p + p, this->t + t );
		}
		
		/**
		 * Initialize this point: Explicit
		 */
		void init( double p, double t, const Interval& i ) {
			this->init( p, t, i.getMin(), i.getMax() );
		}
		
		/**
		 * Initialize this point: Explicit
		 */
		void init( double p, double t, double min, double max ) {
			this->setCoords( p, t );
			this->i.setBounds( min, max );
		}
		
		/**
		 * Whether monotonicity constraints are respected from this point to
		 * another; essentially whether the point is viewable, with the added
		 * restriction that time must be strictly greater than, with position
		 * can be greater than or equal.
		 */
		bool canSee( const PVT_Point& p2, const Constraints& c ) const {
			return Maths::approxLt( this->getTimeCoord(), p2.getTimeCoord(), c.getEpsilon() )
			&& Maths::approxLe( this->getPathCoord(), p2.getPathCoord(), c.getEpsilon() );
		}
		
		/**
		 * Whether this point is viewable from the origin according to the same
		 * constraints that apply in the method canSee.
		 */
		bool isViewable( const Constraints& c ) const {
			return Maths::approxLe( 0., this->getTimeCoord(), c.getEpsilon() )
			&& Maths::approxLe( 0., this->getPathCoord(), c.getEpsilon() );
		}
		
		/**
		 * The velocity interval constraint at this point.
		 */
		Interval i;
		
	private:
		
		/**
		 * The path coordinate in PT space.
		 */
		double p;
		
		/**
		 * The time coordinate in PT space.
		 */
		double t;

	};
	
} // end PVTP namespace

#endif