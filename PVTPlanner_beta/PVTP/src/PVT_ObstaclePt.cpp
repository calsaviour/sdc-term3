#include <PVTP/PVT_ObstaclePt.hpp>

namespace PVTP {
	
	PVT_ObstaclePoint::PVT_ObstaclePoint ( double p, double t, const Constraints& c ) : PVT_Point( p, t ) {
		this->init( c );
	}

	PVT_ObstaclePoint::PVT_ObstaclePoint ( double p, double t, const Interval& i, const Constraints& c ) : PVT_Point ( p, t, i ) {
		this->incoming_edge = NULL;
		this->outgoing_edge = NULL;
	}
	
	PVT_ObstaclePoint::PVT_ObstaclePoint ( double p, double t, double min, double max, const Constraints& c ) : PVT_Point ( p, t, min, max ) {
		this->init( c );
	}

	PVT_ObstaclePoint::PVT_ObstaclePoint ( const PVT_Point& p, const Constraints& c ) : PVT_Point( p ) {
		this->init( c );
	}
	
	PVT_ObstaclePoint::PVT_ObstaclePoint ( const PVT_ObstaclePoint& p, const Constraints& c ) : PVT_Point( p ) {
		this->init( c );
	}
	
	void PVT_ObstaclePoint::init( const Constraints& c ) {
		
		bool valid_x = c.validX( this->getPathCoord() );
		bool valid_t = c.validT( this->getTimeCoord() );
		if ( !valid_x || !valid_t ) {
			this->i.setEmpty( true );
		}
		this->incoming_edge = NULL;
		this->outgoing_edge = NULL;
	}
	
	bool PVT_ObstaclePoint::isReachable( const Constraints& c ) const {
		if ( !this->isViewable(c) ) {
			return false;
		}
		if ( this->i.isEmpty() ) {
			return false;
		}
		Interval velocities( c.getVMin(), c.getVMax() );
		velocities.intersect( this->i, c );
		if ( velocities.isEmpty() ) {
			return false;
		}
		return true;
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_ObstaclePoint& obs_p) {
		const PVT_ObstaclePoint * obs_p_pointer = &obs_p;
		return out << *((const PVT_Point *)obs_p_pointer);
	}
	
} // end PVTP namespace