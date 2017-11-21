#include <PVTP/PVT_Point.hpp>

namespace PVTP {

	PVT_Point::PVT_Point ( double p, double t ) {
		this->init( p, t, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max() );
	}
	
	PVT_Point::PVT_Point ( double p, double t, const Interval& i ) {
		this->init( p, t, i );
	}
	
	PVT_Point::PVT_Point ( double p, double t, double min, double max ) {
		this->init( p, t, min, max );
	}
	
	PVT_Point::PVT_Point ( const PVT_Point& p ) {
		this->init( p.getPathCoord(), p.getTimeCoord(), p.i );
	}

	std::ostream& operator<<(std::ostream& out, const PVT_Point& pvt_p) {
		return out << "(" << pvt_p.getPathCoord() << ", " << pvt_p.getTimeCoord() << ", " << pvt_p.i << ")";
	}
	
} // end PVTP namespace