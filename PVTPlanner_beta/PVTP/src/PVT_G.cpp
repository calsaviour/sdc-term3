#include <PVTP/PVT_G.hpp>

namespace PVTP {

	PVT_G::PVT_G ( const PVT_ObstaclePoint& p ) {
		this->p = new PVT_ObstaclePoint( p );
	}
	
	PVT_G::PVT_G ( const Interval& i, const PVT_ObstaclePoint& p ) {
		this->V.push_back( Interval(i) );
		this->p = new PVT_ObstaclePoint( p );
	}
	
	PVT_G::~PVT_G () {
		delete( this->p );
	}
} // end PVTP namespace