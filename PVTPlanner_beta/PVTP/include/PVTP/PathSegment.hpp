#ifndef PVTP_PATH_SEGMENT_H
#define PVTP_PATH_SEGMENT_H

#include <PVTP/XY_Point.hpp>

using namespace PVTP;

namespace Scenario {
	
	/**
	 * Supported order of polynomial
	 */
	static unsigned int POLY_ORDER = 2;
	
	/**
	 * User Path Segment class. This class is for contructing and handling User
	 * path segments, which are defined as domain-bounded polynomials in X. The
	 * path segments are implicitly directed as moving from the origin point to
	 * the end point.
	 */
	class PathSegment {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PathSegment& path_segment);
		
	public:
		
		/**
		 * Constructor
		 */
		PathSegment ( const std::vector<double>& coefficients, const XY_Point& origin, double length, const Constraints& c );
		
		/**
		 * Constructor
		 */
		PathSegment ( const XY_Point& begin, const XY_Point& end, const Constraints& c );
		
		/**
		 * Copy constructor
		 */
		PathSegment ( const PathSegment& path_segment );
		
		/**
		 * Given the arc length parameter compute the position vector (x, y)
		 */
		bool getPositionVectorAtArcLength( double& x, double& y, double arc_length, const Constraints& c ) const;
		
		/**
		 * Given the arc length parameter compute the slope at the given arc length
		 */
		bool getSlopeAtArcLength( double& slope, double segment_length, const Constraints& c ) const;
		
		/**
		 * Check whether this segment contains a point or not.
		 */
		bool containsPoint( const XY_Point& p, const Constraints& c ) const;
		
		/**
		 * Check whether this segment contains a point or not.
		 */
		bool containsPoint( double x, double y, const Constraints& c ) const;
		
		/**
		 * Check whether point occurs before segment
		 */
		bool pointBefore( double x, double y ) const;
		
		/**
		 * Check whether point occurs before segment
		 */
		bool pointAfter( double x, double y ) const;
		
		/**
		 * Compute the arc length between two points on this path segment.
		 *
		 * NOTE: Both points must be contained on the segment.
		 */
		double getArcLengthBetweenPoints( const XY_Point& start_point, const XY_Point& end_point, const Constraints& c ) const;
		
		/**
		 * Return the length of this segment.
		 */
		double getLength( void ) const {
			return this->length;
		}
		
		/**
		 * Accessor for the origin of this segment
		 */
		const XY_Point& getOrigin( void ) const {
			return this->origin;
		}
		
		/**
		 * Accessor for the end point of this segment
		 */
		const XY_Point& getEndPoint( void ) const {
			return this->end_point;
		}
		
		/**
		 * Accessor for vertical flag
		 */
		bool isVertical( void ) const {
			return this->is_vertical;
		}
		
		/**
		 * Accessor for horizontal flag
		 */
		bool isHorizontal( void ) const {
			return this->is_horizontal;
		}
		
		/**
		 * Convenience method for getting direction of this segment
		 */
		int getDirection( void ) const {
			return this->direction;
		}
		
		/**
		 * Convenience method for rotating system by 90 degrees clockwise.
		 *
		 * NOTE: This rotates about the world frame origin, *not* the reference
		 * point! Further, this is only to be used with vertical path segments;
		 * others will result in error.
		 */
		bool rotate90CW( void );
		
		/**
		 * Convenience method for rotating system by 90 degrees clockwise.
		 *
		 * NOTE: This rotates about the world frame origin, *not* the reference
		 * point! Further, this is only to be used with horizontal path segments;
		 * others will result in error.
		 */
		bool rotate90CCW( void );
		
		/**
		 * Translate this path segment
		 */
		void translate( double x, double y, const Constraints& c ) {
			origin.translate( x, y );
			end_point.translate( x, y );
			this->setCoefficients( this->coefficients, this->origin, this->end_point, c );
		}
		
		/**
		 * Coefficients for path in plane.
		 */
		std::vector<double> coefficients;
		
		/**
		 * Given a start point and an end point of a segment, determine the sign
		 * to put on the distance. The sign indicates the directedness of the
		 * constructed edge. Direction is positive if increasing on the X axis
		 * (or increasing on Y, if the segment is vertical). Negative otherwise.
		 */
		static int getPathSegmentDirection( const XY_Point& p1, const XY_Point& p2, const Constraints& c );
		
		/**
		 * Message handler for exceptions thrown during construction
		 */
		static std::string exceptionMessage( int e );
		
	private:
		
		/**
		 * Convenience method used by constructors
		 */
		void init( const std::vector<double>& coefficients, const XY_Point& origin, double length, const Constraints& c );
		
		/**
		 * Convenience method for computing the endpoint of this segment.
		 */
		bool setEndPoint( double length, const Constraints& c );
		
		/**
		 * Convenience method for setting coefficients
		 */
		void setCoefficients( std::vector<double>& coefficients, const XY_Point& begin, const XY_Point& end, const Constraints& c );
		
		/**
		 * Domain over which this segment is valid.
		 */
		double length;
		
		/**
		 * Origin of this segment.
		 */
		XY_Point origin;
		
		/**
		 * End point of this segment.
		 */
		XY_Point end_point;
		
		/**
		 * Flag for whether this path segment is vertical in world space
		 */
		bool is_vertical;
		
		/**
		 * Flag for whether this path segment is horizontal in world space
		 */
		bool is_horizontal;
		
		/**
		 * Convenience storage for direction of this vector
		 */
		int direction;
		
	};
	
} // end Scenario namespace

#endif
