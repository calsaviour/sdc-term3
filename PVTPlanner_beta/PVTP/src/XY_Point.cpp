#include <PVTP/XY_Point.hpp>

namespace Scenario {
	
	XY_Point::XY_Point ( ) {
		this->setCoords( std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN() );
	}
	
	XY_Point::XY_Point ( double x, double y ) {
		this->setCoords( x, y );
	}
	
	XY_Point::XY_Point ( const XY_Point& p ) {
		this->setCoords( p.x, p.y );
	}
	
	std::ostream& operator<<( std::ostream& out, const XY_Point& p ) {
		return out << "[" << p.getX() << ", " << p.getY() << "]";
	}

} // end SCIMP_Scenario namespace