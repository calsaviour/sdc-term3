#ifndef PVTP_XY_POINT_H
#define PVTP_XY_POINT_H

#include <PVTP/PVT_ObstaclePt.hpp>

using namespace PVTP;

namespace Scenario {
	
	/**
	 * This class describes a point in the world frame, which is a plane
	 * centered at the initial position of the user-controlled vehicle, and
	 * oriented such that positive Y extends in the direction of forward motion,
	 * and positive X extends in the direction of the driver's forward motion.
	 *
	 * The orientation of "direction of driver's forward motion" is a bit vague,
	 * since the driver's path can curve before settling on a final route
	 * (such as when turning left at an intersection). For now, since we're
	 * currently only considering lane-crossing scenarios, let this direction
	 * be the direction across the lanes.
	 */
	class XY_Point {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const XY_Point& p );
		
	public:
		
		/**
		 * Assignment operator
		 */
		XY_Point& operator=( const XY_Point& rhs ) {
			if ( this != &rhs ) {
				this->setCoords( rhs );
			}
			return *this;
		}
		
		/**
		 * Default constructor
		 */
		XY_Point ( );
		
		/**
		 * Construct a point in the world frame of the scenario.
		 */
		XY_Point ( double x, double y );
		
		/**
		 * Copy constructor
		 */
		XY_Point ( const XY_Point& p );
		
		/**
		 * Accessor for x-coordinate of this point
		 */
		double getX() const {
			return this->x;
		}
		
		/**
		 * Accessor for y-coordinate of this point
		 */
		double getY() const {
			return this->y;
		}
		
		/**
		 * Translate this point in space
		 */
		void translate( double _x, double _y ) {
			this->x += _x;
			this->y += _y;
		}

		/**
		 * Convenience method for rotating 90 degrees counterclockwise about the origin
		 */
		void rotate90CCW( void ) {
			double old_x = this->x;
			double old_y = this->y;
			this->x = -old_y;
			this->y = old_x;
		}
		
		/**
		 * Convenience method for rotating 90 degrees clockwise about the origin
		 */
		void rotate90CW( void ) {
			double old_x = this->x;
			double old_y = this->y;
			this->x = old_y;
			this->y = -old_x;
		}
		
		/**
		 * Rotate this point counter-clockwise about the origin, theta given
		 * in radians.
		 */
		void rotate( double theta ) {
			double sin_theta = sin( theta );
			double cos_theta = cos( theta );
			double new_x = this->x * cos_theta - this->y * sin_theta;
			double new_y = this->x * sin_theta + this->y * cos_theta;
			this->x = new_x;
			this->y = new_y;
		}
		
		/**
		 * Mutator to set this point equal to a given point
		 */
		void setPoint( const XY_Point& p ) {
			this->setCoords( p.getX(), p.getY() );
		}
		
		/**
		 * Mutator for setting the coordinates of this point
		 */
		void setCoords( double x, double y ) {
			this->x = x;
			this->y = y;
		}
		
		/**
		 * Mutator for setting the coordinates of this point
		 */
		void setCoords( const XY_Point& p ) {
			this->setCoords( p.getX(), p.getY() );
		}
		
		/**
		 * Test for approximate equality in X and Y coordinates
		 */
		bool equals( const XY_Point& p, const Constraints& c ) const {
			return Maths::approxEq( this->getX(), p.getX(), c.getEpsilon() )
			&& Maths::approxEq( this->getY(), p.getY(), c.getEpsilon() );
		}
		
	private:
		
		/**
		 * The x-coordinate of the point
		 */
		double x;
		
		/**
		 * The y-coordinate of the point
		 */
		double y;
		
	};

} // end Scenario namespace

#endif
