#include <PVTP/PathSegment.hpp>

using namespace PVTP;

namespace Scenario {
	
	PathSegment::PathSegment ( const XY_Point& begin, const XY_Point& end, const Constraints& c ) {
		std::vector<double> coefficients;
		this->setCoefficients( coefficients, begin, end, c );
		double length = Maths::L2_Distance( begin.getX(), begin.getY(), end.getX(), end.getY(), c.getEpsilon() );
		int direction = getPathSegmentDirection( begin, end, c );
		this->init( coefficients, begin, (double)direction * length, c );
	}
	
	PathSegment::PathSegment ( const std::vector<double>& coefficients, const XY_Point& origin, double length, const Constraints& c ) {
		this->init( coefficients, origin, length, c );
	}
	
	PathSegment::PathSegment ( const PathSegment& path_segment ) {
		this->is_vertical = path_segment.is_vertical;
		this->is_horizontal = path_segment.is_horizontal;
		this->coefficients = path_segment.coefficients;
		this->length = path_segment.getLength();
		this->origin.setCoords( path_segment.origin );
		this->end_point.setCoords( path_segment.end_point );
		this->direction = path_segment.direction;
	}
	
	void PathSegment::setCoefficients( std::vector<double>& coefficients, const XY_Point& begin, const XY_Point& end, const Constraints& c ) {
		coefficients.resize( 3, 0. );
		coefficients[1] = Maths::slope( begin.getX(), begin.getY(), end.getX(), end.getY(), c.getEpsilon() );
		coefficients[2] = Maths::yIntercept( begin.getX(), begin.getY(), coefficients[1] );
	}
	
	void PathSegment::init( const std::vector<double>& coefficients, const XY_Point& origin, double length, const Constraints& c ) {
		if ( Maths::approxEq(length, 0., c.getEpsilon()) ) {
			throw Constants::ZERO_LENGTH;
		}
		if ( coefficients.size() != (POLY_ORDER + 1) ) {
			throw Constants::BAD_POLY_ORDER;
		}
		this->is_vertical = Maths::isNaN( coefficients[1] ) || Maths::isNaN( coefficients[2] );
		this->is_horizontal = Maths::approxEq( coefficients[1], 0., c.getEpsilon() );
		this->coefficients = coefficients;
		this->length = fabs( length );
		this->origin.setCoords( origin );
		
		if ( !this->setEndPoint(length, c) ) {
			throw Constants::NO_END_POINT;
		}
		
		if ( this->isVertical() ) {
			this->direction = (this->getOrigin().getY() > this->getEndPoint().getY()) ? -1 : 1;
		} else {
			this->direction = (this->getOrigin().getX() > this->getEndPoint().getX()) ? -1 : 1;
		}
	}
	
	bool PathSegment::containsPoint( const XY_Point& p, const Constraints& c ) const {
		return this->containsPoint( p.getX(), p.getY(), c );
	}
	
	bool PathSegment::containsPoint( double x, double y, const Constraints& c ) const {
		
		// check for containment in domain
		Interval x_interval;
		x_interval.setBoundsLazy( this->getOrigin().getX(), this->getEndPoint().getX() );
		if ( !x_interval.contains(x, c) ) {
			return false;
		}
		
		// check for containment in range
		Interval y_interval;
		y_interval.setBoundsLazy( this->getOrigin().getY(), this->getEndPoint().getY() );
		if ( !y_interval.contains(y, c) ) {
			return false;
		}
		
		// if the line is vertical, p is contained at this point
		if ( this->isVertical() ) {
			return true;
		}
		
		// check whether point is contained by the segment's polynomial
		double y_test = Maths::Horners( this->coefficients, x );
		if ( !Maths::approxEq(y_test, y, c.getEpsilon()) ) {
			return false;
		}
		
		// at this point, the segment contains the point
		return true;
		
	}
	
	bool PathSegment::pointBefore( double x, double y ) const {
		if ( this->getDirection() < 0 ) {
			if ( this->isVertical() ) {
				return y > this->getOrigin().getY();
			} else {
				return x > this->getOrigin().getX();
			}
		}
		if ( this->isVertical() ) {
			return y < this->getOrigin().getY();
		} else {
			return x < this->getOrigin().getX();
		}
	}
	
	bool PathSegment::pointAfter( double x, double y ) const {
		if ( this->getDirection() < 0 ) {
			if ( this->isVertical() ) {
				return y < this->getEndPoint().getY();
			} else {
				return x < this->getEndPoint().getX();
			}
		}
		if ( this->isVertical() ) {
			return y > this->getEndPoint().getY();
		} else {
			return x > this->getEndPoint().getX();
		}
	}
	
	double PathSegment::getArcLengthBetweenPoints( const XY_Point& start_point, const XY_Point& end_point, const Constraints& c ) const {
		if ( !this->containsPoint(start_point, c) || !this->containsPoint(end_point, c) ) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		
		return Maths::L2_Distance( start_point.getX(), start_point.getY(), end_point.getX(), end_point.getY(), c.getEpsilon() );
	}
	
	bool PathSegment::setEndPoint( double length, const Constraints& c ) {
		
		double x2;
		double y2;
		
		double slope;
		if ( !this->getSlopeAtArcLength(slope, fabs(length), c) ) {
			std::cerr << "ERROR IN PathSegment::setEndPoint: Failed finding slope." << std::endl;
			return false;
		}
		double intercept = this->coefficients.at(2);
		
		// vertical line
		if ( this->isVertical() ) {
			x2 = this->origin.getX();
			y2 = this->origin.getY() + length;
		} else {
		
			double discriminant = slope * slope + 1.;
			if ( discriminant <= 0. ) {
				std::cerr << "ERROR IN PathSegment::setEndPoint: Invalid discriminant of " << discriminant << std::endl;
				return false;
			}
			
			x2 = length / sqrt( discriminant ) + this->origin.getX();
			y2 = Maths::yFromSlopeIntercept( slope, x2, intercept );
			
		}
		
		this->end_point.setCoords( x2, y2 );
		
		return true;
	}
	
	bool PathSegment::getSlopeAtArcLength( double& slope, double segment_length, const Constraints& c ) const {
		slope = std::numeric_limits<double>::quiet_NaN();
		
		if ( Maths::approxLt(segment_length, 0., c.getEpsilon()) || Maths::approxGt(segment_length, this->getLength(), c.getEpsilon()) ) {
			std::cerr << "ERROR IN PathSegment::getSlopeAtArcLength: Invalid arc length of " << segment_length << std::endl;
			return false;
		}
		
		slope = this->coefficients.at( 1 );

		return true;
	}
	
	bool PathSegment::getPositionVectorAtArcLength( double& x, double& y, double arc_length, const Constraints& c ) const {

		if ( Maths::approxLt(arc_length, 0., c.getEpsilon()) || Maths::approxGt(arc_length, this->getLength(), c.getEpsilon()) ) {
			std::cerr << "ERROR IN PathSegment::getPositionVectorAtArcLength: Invalid arc length of " << arc_length << std::endl;
			return false;
		}
		
		double slope;
		if ( !this->getSlopeAtArcLength(slope, arc_length, c) ) {
			std::cerr << "ERROR IN PathSegment::getPositionVectorAtArcLength: Failed finding slope at arc length: " << arc_length << std::endl;
			return false;
		}
		double intercept = this->coefficients.at( 2 );
		
		// vertical line
		if ( this->isVertical() ) {
			x = this->origin.getX();
			y = this->origin.getY() + (double)this->getDirection() * arc_length;
			return true;
		}
		
		double discriminant = slope * slope + 1.;
		if ( discriminant <= 0. ) {
			std::cerr << "ERROR IN PathSegment::setEndPoint: Invalid discriminant of " << discriminant << std::endl;
			return false;
		}
		
		x = (double)this->getDirection() * arc_length / sqrt( discriminant ) + this->origin.getX();
		y = Maths::yFromSlopeIntercept( slope, x, intercept );
		
		return true;
	}
	
	bool PathSegment::rotate90CW( void ) {
		
		if ( !this->isVertical() ) {
			return false;
		}
		this->is_vertical = false;
		this->is_horizontal = true;
		
		// origin
		this->origin.rotate90CW();
		
		// end point
		this->end_point.rotate90CW();
		
		// slope
		this->coefficients[1] = 0.;
		
		// intercept
		this->coefficients[2] = this->origin.getY();
		
		return true;
	}
	
	bool PathSegment::rotate90CCW( void ) {
		
		if ( !this->isHorizontal() ) {
			return false;
		}
		this->is_vertical = true;
		this->is_horizontal = false;
		
		// origin
		this->origin.rotate90CCW();
		
		// end point
		this->end_point.rotate90CCW();
		
		// slope
		this->coefficients[1] = std::numeric_limits<double>::quiet_NaN();
		
		// intercept
		this->coefficients[2] = std::numeric_limits<double>::quiet_NaN();
		
		return true;		
	}
	
	int PathSegment::getPathSegmentDirection( const XY_Point& p1, const XY_Point& p2, const Constraints& c ) {
		double delta_x = Maths::clipToZero( p2.getX() - p1.getX(), c.getEpsilon() );
		if ( delta_x > 0. ) {
			return 1;
		} else if ( delta_x < 0. ) {
			return -1;
		}
		double delta_y = Maths::clipToZero( p2.getY() - p1.getY(), c.getEpsilon() );
		if ( delta_y > 0. ) {
			return 1;
		} else if ( delta_y < 0. ) {
			return -1;
		}
		return 0;
	}
	
	std::string PathSegment::exceptionMessage( int e ) {
		std::stringstream ss;
		ss << "Path segment construction failed: ";
		switch ( e ) {
			case Constants::ZERO_LENGTH:
				ss << "Segment must be of non-zero length.";
				break;
			case Constants::BAD_POLY_ORDER:
				ss << "Segment polynomial does not have required number of coefficients (" << (POLY_ORDER+1) << ")";
				break;
			case Constants::NO_END_POINT:
				ss << "Could not compute segment end point.";
				break;
			default:
				return "";
		}
		return ss.str();
	}
	
	std::ostream& operator<<(std::ostream& out, const PathSegment& path_segment) {
		out << path_segment.getOrigin() << " -> " << path_segment.getEndPoint() << ", length: " << path_segment.getLength() << ", coefficients: [" << path_segment.coefficients[0] << ", " << path_segment.coefficients[1] << ", " << path_segment.coefficients[2] << "], vertical: " << path_segment.isVertical();
		return out;
	}
	
} // end Scenario