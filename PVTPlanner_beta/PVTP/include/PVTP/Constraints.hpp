#ifndef PVTP_CONSTRAINTS_H
#define PVTP_CONSTRAINTS_H

#include <iostream>
#include <sstream>
#include <string>
#include <PVTP/Constants.hpp>
#include <PVTP/Maths.hpp>

/**
 * This namespace contains the planner library.
 */
namespace PVTP {

	/**
	 *	Constraints class. This class is a container for the constraint constants
	 *	used by the planner.
	 *
	 * Units are considered to be meters and seconds, as appropriate.
	 *
	 *	WARNING: The constructor throws an exception if something in the
	 *	specified constraints is amiss:
	 
		 if ( !((a_min < 0.) && (a_max > 0.)) )
			throw Constraints::ACC_VIOLATION;

		 if ( v_min > v_max )
			throw Constraints::VEL_VIOLATION;

		 if ( x_limit <= 0. )
			throw Constraints::PATH_VIOLATION;

		 if ( t_limit <= 0. )
			throw Constriants::TIME_VIOLATION;

		 if ( epsilon < 0. )
			throw Constraints::EPSILON_VIOLATION;

	 *
	 */

	class Constraints {

		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const Constraints& c);

	public:
		
		/**
		 * Copy constructor with flag for producing constraints for mirrored
		 * problems encountered during backwards propagation.
		 */
		Constraints ( const Constraints& c, bool mirrored = false );
		
		/**
		 * Constraints constructor:
		 *
		 * x_limit: the length of the path being traversed.
		 * t_limit: the maximum duration of time that a trajectory may take.
		 * v_min:	the minimum velocity achievable by the vehicle
		 * v_max:	the maximum velocity achievable by the vehicle
		 * a_min:	the minimum acceleration achievable by the vehicle
		 * a_max:	the maximum acceleration achievable by the vehicle
		 * epsilon:	an approximation factor used when making fuzzy comparisons
		 */
		Constraints ( double x_limit,
					 double t_limit,
					 double v_min,
					 double v_max,
					 double a_min,
					 double a_max,
					 double epsilon = Constants::DEFAULT_EPSILON );
		
		/**
		 * Test for path coordinate
		 */
		 bool validX( double x ) const {
			return Maths::approxGe( x, 0., this->getEpsilon() )
			&& Maths::approxLe( x, this->getXLimit(), this->getEpsilon() );
		}
		
		/**
		 * Test for time coordinate
		 */
		 bool validT( double t ) const {
			return Maths::approxGe( t, 0., this->getEpsilon() )
			&& Maths::approxLe( t, this->getTLimit(), this->getEpsilon() );
		}
		
		/**
		 * Test for velocity
		 */
		 bool validV( double v ) const {
			return Maths::approxGe( v, this->getVMin(), this->getEpsilon() )
			&& Maths::approxLe( v, this->getVMax(), this->getEpsilon() );
		}
		
		/**
		 * Test for acceleration
		 */
		 bool validA( double a ) const {
			return Maths::approxGe( a, this->getAMin(), this->getEpsilon() )
			&& Maths::approxLe( a, this->getAMax(), this->getEpsilon() );
		}
		
		/**
		 * Accessor for x_limit
		 */
		 double getXLimit( void ) const {
			return this->x_limit;
		}
		
		/**
		 * Accessor for t_limit
		 */
		 double getTLimit( void ) const {
			return this->t_limit;
		}
		
		/**
		 * Accessor for v_max
		 */
		double getVMax( void ) const {
			return this->v_max;
		}
		
		/**
		 * Accessor for v_min
		 */
		double getVMin( void ) const {
			return this->v_min;
		}
		
		/**
		 * Accessor for a_max
		 */
		double getAMax( void ) const {
			return this->a_max;
		}
		
		/**
		 * Accessor for a_min
		 */
		double getAMin( void ) const {
			return this->a_min;
		}
		
		/**
		 * Accessor for epsilon
		 */
		double getEpsilon( void ) const {
			return this->epsilon;
		}
		
		/**
		 * Mutator for x_limit
		 */
		void setXLimit( double x_limit ) {
			this->x_limit = x_limit;
		}
		
		/**
		 * Mutator for t_limit
		 */
		void setTLimit( double t_limit ) {
			this->t_limit = t_limit;
		}
		
		/**
		 * Mutator for v_max
		 */
		void setVMax( double v_max ) {
			this->v_max = v_max;
		}
		
		/**
		 * Mutator for v_min
		 */
		void setVMin( double v_min ) {
			this->v_min = v_min;
		}
		
		/**
		 * Mutator for a_max
		 */
		void setAMax( double a_max ) {
			this->a_max = a_max;
		}
		
		/**
		 * Mutator for a_min
		 */
		void setAMin( double a_min ) {
			this->a_min = a_min;
		}
		
		/**
		 * Mutator for epsilon
		 */
		void setEpsilon( double epsilon ) {
			this->epsilon = epsilon;
		}
		
		/**
		 * Message handler for exceptions thrown during construction
		 */
		static std::string exceptionMessage( int e );

	private:
		
		/**
		 * Initialize constraints object
		 */
		void init( double x_limit,
				  double t_limit,
				  double v_min,
				  double v_max,
				  double a_min,
				  double a_max,
				  double epsilon );
		
		/**
		 * The extent of the path
		 */
		double x_limit;
		
		/**
		 * The maximum time a trajectory may take to execute
		 */
		double t_limit;
		
		/**
		 * The maximum velocity available to the planner
		 */
		double v_max;
		
		/**
		 * The minimum velocity available to the planner
		 */
		double v_min;
		
		/**
		 * The maximum acceleration available to the planner
		 */
		double a_max;
		
		/**
		 * The minimum acceleratin available to the planner
		 */
		double a_min;
		
		/**
		 * Numeric comparisons will be done to within this margin of error
		 */
		double epsilon;
	};

} // end PVTP namespace

#endif