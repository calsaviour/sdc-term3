#include <PVTP/PathModel.hpp>
#include <PVTP/PathSegment.hpp>

using namespace PVTP;

namespace Scenario {
	
	PathModel::PathModel ( std::vector<PathSegment>& segments, const Constraints& c ) {
		if ( segments.empty() ) {
			throw Constants::ZERO_SEGMENTS;
		}
		
		// only attempt collapsing segments if there are more than one -- BROKEN, SO IT'S DISABLED
		if ( false && (segments.size() > 1) ) {
			
			size_t prev_seg = segments.size() - 1;
			for ( size_t cur_seg=0; cur_seg<segments.size(); cur_seg++ ) {
				
				// segments must be connected (except for first and last)
				if ( (cur_seg==(prev_seg+1)) && !segments[prev_seg].getEndPoint().equals(segments[cur_seg].getOrigin(), c) ) {
					std::cerr << "ERROR IN PathModel::PathModel: Disconnected path segments: " << segments[prev_seg].getEndPoint() << " != " << segments[cur_seg].getOrigin() << std::endl;
					throw Constants::DISCONNECTED_PATH_SEGMENT;
				}
				
				// test for approximate equality of coefficients
				bool coeffs_equal = true;
				for ( size_t i=0; i<segments[prev_seg].coefficients.size(); i++ ) {
					if ( !Maths::approxEq(segments[prev_seg].coefficients[i], segments[cur_seg].coefficients[i], c.getEpsilon()) ) {
						coeffs_equal = false;
						break;
					}
				}
				
				// if coefficients are equal, collapse them
				if ( coeffs_equal ) {
					this->segments.push_back( PathSegment(segments[prev_seg].coefficients,
														  segments[prev_seg].getOrigin(),
														  (double)segments[prev_seg].getDirection() * (segments[prev_seg].getLength() + segments[cur_seg].getLength()),
														  c) );
					cur_seg++;
				} else {
					this->segments.push_back( PathSegment(segments[cur_seg]) );
				}
			}
			
		} else {
			this->segments = segments;
		}
		
		// compute total length
		double length = 0.;
		for ( size_t i=0; i<this->segments.size(); i++ ) {
			length += this->segments[i].getLength();
		}
		this->length = length;
		
	}
	
	PathModel::PathModel( const PathModel& path ) {
		this->segments = path.segments;
		this->length = path.getLength();
	}
	
	double PathModel::getArcLengthToPoint( const XY_Point& target_point, const Constraints& c ) const {
		return this->getArcLengthBetweenPoints( this->segments.front().getOrigin(), target_point, c );
	}
	
	bool PathModel::getSlopeAtArcLength( double& slope, double arc_length, const Constraints& c ) const {
		
		// find the proper segment
		slope = std::numeric_limits<double>::quiet_NaN();
		double segment_length = arc_length;
		for ( size_t i=0; i<this->segments.size(); i++ ) {
			
			if ( segment_length > this->segments[i].getLength() ) {
				segment_length -= this->segments[i].getLength();
				continue;
			}
			
			// handle vertical lines
			if ( this->segments[i].isVertical() ) {
				slope = std::numeric_limits<double>::quiet_NaN();
				return true;
			}
			
			if ( !this->segments[i].getSlopeAtArcLength(slope, segment_length, c) ) {
				std::cerr << "ERROR IN PathModel::getSlopeAtArcLength: Failed finding segment arc length." << std::endl;
				return false;
			}
			
			break;
		}
		if ( Maths::isNaN(slope) ) {
			std::cerr << "ERROR IN PathModel::getOrientationAtArcLength: Invalid slope." << std::endl;
			return false;
		}
		return true;
	}
	
	bool PathModel::getOrientationAtArcLength( double& orientation, double arc_length, const Constraints& c, bool orient_to_prev ) const {

		// find the proper segment
		orientation = std::numeric_limits<double>::quiet_NaN();
		double segment_length = arc_length;
		for ( size_t i=0; i<this->segments.size(); i++ ) {
			
			if ( Maths::approxGe(segment_length, this->segments[i].getLength(), c.getEpsilon()) ) {
				segment_length -= this->segments[i].getLength();
				continue;
			}
			
			// handle vertical lines
			if ( this->segments[i].isVertical() ) {
				
				// path is defined as going in the negative Y
				if ( this->segments[i].getOrigin().getY() > this->segments[i].getEndPoint().getY() ) {
					
					orientation = -M_PI_2;

				
				// path is defined as going in the positive Y
				} else {
					
					orientation = M_PI_2;
				}
				
				return true;
			}
			
			double slope;
			double _i = ( orient_to_prev && (i > 0) && Maths::approxEq(segment_length, 0., c.getEpsilon()) ) ? i - 1 : i;
			if ( !this->segments[_i].getSlopeAtArcLength(slope, segment_length, c) ) {
				std::cerr << "ERROR IN PathModel::getSlopeAtArcLength: Failed finding segment arc length." << std::endl;
				return false;
			}
			
			int direction = this->segments[_i].getDirection();
			if ( direction > 0 ) {
				orientation = atan( slope );
			} else {
				orientation = M_PI + atan( slope );
			}
			
			break;
		}

		return true;
	}
	
	ssize_t PathModel::getPositionVectorAtArcLength( double& x, double& y, double arc_length, const Constraints& c ) const {
		x = std::numeric_limits<double>::quiet_NaN();
		y = std::numeric_limits<double>::quiet_NaN();
		ssize_t position_index = -1;

		double segment_length = arc_length;
		for ( size_t i=0; i<this->segments.size(); i++ ) {
			
			if ( Maths::approxGt(segment_length, this->segments[i].getLength(), c.getEpsilon()) ) {
				segment_length -= this->segments[i].getLength();
				continue;
			}
			
			if ( !this->segments[i].getPositionVectorAtArcLength(x, y, segment_length, c) ) {
				std::cerr << "ERROR IN PathModel::getPositionVectorAtLength: getPositionVectorAtArcLength failed." << std::endl;
				return position_index;
			}
			return i;

		}
		
		std::cerr << "ERROR IN PathModel::getPositionVectorAtLength: Requested arc length outside path bounds. Leftover segment length: " << segment_length << ", out of: " << arc_length << std::endl;
		return position_index;
	}
	
	ssize_t PathModel::getArcLengthAtPositionVector( double& arc_length, const XY_Point& p, const Constraints& c ) const {
		return this->getArcLengthAtPositionVector( arc_length, p.getX(), p.getY(), c );
	}
	
	ssize_t PathModel::getArcLengthAtPositionVector( double& arc_length, double x, double y, const Constraints& c ) const {
		arc_length = std::numeric_limits<double>::quiet_NaN();
		
		bool found = false;
		double length = 0.;
		ssize_t path_position_index = -1;
		for ( size_t i=0; i<this->segments.size(); i++ ) {
			if ( !this->segments[i].containsPoint(x, y, c) ) {
				length += this->segments[i].getLength();
				continue;
			}
			
			// found the right segment, compute arc length to point
			if ( this->segments[i].isVertical() ) {
				length += y - this->segments[i].getOrigin().getY();
			} else {			
				length += Maths::secondOrderArcLength( this->segments[i].coefficients[0],
													  this->segments[i].coefficients[1],
													  this->segments[i].coefficients[2],
													  this->segments[i].getOrigin().getX(), x, c.getEpsilon() );
			}
			found = true;
			path_position_index = i;
			break;
		}
		if ( !found ) {
			return path_position_index;
		}
		arc_length = fabs( length );
		return path_position_index;
	}
	
	double PathModel::getArcLengthBetweenPoints( const XY_Point& start_point, const XY_Point& end_point, const Constraints& c ) const {
		
		const XY_Point * first_point;
		const XY_Point * second_point;
		
		// find segments containing start and end points
		ssize_t first_segment_index = -1;
		ssize_t second_segment_index = -1;
		for ( size_t i=0; i<this->segments.size(); i++ ) {
			
			bool contains_start_point = this->segments[i].containsPoint( start_point, c );
			bool contains_end_point = this->segments[i].containsPoint( end_point, c );
			
			// entirely contained on segment
			if ( contains_start_point && contains_end_point ) {
				return this->segments[i].getArcLengthBetweenPoints( start_point, end_point, c );
			}
			
			if ( contains_start_point ) {
				first_segment_index = i;
				first_point = &start_point;
			} else if ( contains_end_point ) {
				second_segment_index = i;
				second_point = &end_point;
			}
		}
		
		// make sure both were found
		if ( (first_segment_index < 0) || (second_segment_index < 0) ) {
			std::cerr << "ERROR IN PathModel::getArcLengthBetweenPoints: Points not contained on path." << std::endl;
			std::cerr << "start_point: " << start_point << " : " << first_segment_index << ", end_point: " << end_point << " : " << second_segment_index << std::endl;
			return std::numeric_limits<double>::quiet_NaN();
		}
		
		// put them in the right order
		if ( second_segment_index < first_segment_index ) {
			const XY_Point * point_tmp;
			point_tmp = first_point;
			first_point = second_point;
			second_point = point_tmp;
			
			ssize_t index_tmp = second_segment_index;
			second_segment_index = first_segment_index;
			first_segment_index = index_tmp;
		}
		
		// sum intermediate distances
		double arc_length = 0.;
		for ( size_t i=(size_t)first_segment_index+1; i<(size_t)second_segment_index; i++ ) {
			arc_length += this->segments[i].getLength();
		}
		
		// add in partial lengths of first and second segments
		arc_length += this->segments[first_segment_index].getArcLengthBetweenPoints( *first_point, this->segments[first_segment_index].getEndPoint(), c );
		arc_length += this->segments[second_segment_index].getArcLengthBetweenPoints( this->segments[second_segment_index].getOrigin(), *second_point, c );
		
		return arc_length;
	}
	
	std::string PathModel::exceptionMessage( int e ) {
		std::stringstream ss;
		ss << "Path construction failed: ";
		switch ( e ) {
			case Constants::ZERO_SEGMENTS:
				ss << "Path must contain at least one path segment.";
				break;
			case Constants::DISCONNECTED_PATH_SEGMENT:
				ss << "Path segments must be connected (end point of one must be start point of next).";
				break;
			default:
				return "";
		}
		return ss.str();
	}
	
	std::ostream& operator<<(std::ostream& out, const PathModel& path) {
		size_t i = 0;
		for ( ; i<path.segments.size()-1; i++ ) {
			out << "Path segment " << (i+1) << ": " << path.segments[i] << std::endl;
		}
		out << "Path segment " << (i+1) << ": " << path.segments[i] << std::endl;
		out << "Total path length: " << path.getLength();
		return out;
	}
	
} // end Scenario
