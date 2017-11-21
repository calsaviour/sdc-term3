#include <PVTP/Maths.hpp>
#include <PVTP/PVT_ObstacleEdge.hpp>

namespace PVTP {
	
	PVT_ObstacleEdge::PVT_ObstacleEdge (  const PVT_ObstaclePoint& p1, const PVT_ObstaclePoint& p2, const Constraints& c  ) {
		this->p1 = new PVT_ObstaclePoint( p1 );
		this->p2 = new PVT_ObstaclePoint( p2 );
		this->init( c );
	}
	
	PVT_ObstacleEdge::PVT_ObstacleEdge ( const PVT_ObstacleEdge& edge ) {
		this->p1 = new PVT_ObstaclePoint( edge.getInitialPoint() );
		this->p2 = new PVT_ObstaclePoint( edge.getFinalPoint() );
		this->path_delta = edge.path_delta;
		this->time_delta = edge.time_delta;
		this->slope = edge.slope;
		this->path_intercept = edge.path_intercept;
	}
	
	PVT_ObstacleEdge::~PVT_ObstacleEdge( ) {
		delete( this->p1 );
		delete( this->p2 );
	}
	
	void PVT_ObstacleEdge::init( const Constraints& c ) {
		this->path_delta = this->p2->getPathCoord() - this->p1->getPathCoord();
		this->time_delta = this->p2->getTimeCoord() - this->p1->getTimeCoord();
		this->slope = Maths::slope( this->p1->getTimeCoord(), this->p1->getPathCoord(), this->p2->getTimeCoord(), this->p2->getPathCoord(), c.getEpsilon() );
		this->path_intercept = Maths::yIntercept( this->p1->getTimeCoord(), this->p1->getPathCoord(), this->getSlope(), c.getEpsilon() );
	}
	
	const PVT_ObstaclePoint& PVT_ObstacleEdge::getInitialPoint( void ) const {
		return *this->p1;
	}
	
	const PVT_ObstaclePoint& PVT_ObstacleEdge::getFinalPoint( void ) const {
		return *this->p2;
	}
	
	double PVT_ObstacleEdge::getSlope( void ) const {
		return this->slope;
	}
	
	double PVT_ObstacleEdge::getPathIntercept( void ) const {
		return this->path_intercept;
	}
	
	double PVT_ObstacleEdge::getPathDelta( void ) const {
		return this->path_delta;
	}
	
	double PVT_ObstacleEdge::getTimeDelta( void ) const {
		return this->time_delta;
	}

	bool PVT_ObstacleEdge::contains( const PVT_Point& p, const Constraints& c ) const {
		return this->contains( p.getPathCoord(), p.getTimeCoord(), c );
	}
	
	bool PVT_ObstacleEdge::contains( double p, double t, const Constraints& c ) const {
		
		// first check whether it's on the line at all
		int relation = Maths::pointLineRelation( p, t, this->getSlope(), this->getPathIntercept(), c.getEpsilon() );
		if ( relation != Maths::POINT_ON ) {
			return false;
		}
		
		// now check whether it's on the segment
		Interval path_interval;
		Interval time_interval;
		path_interval.setBoundsLazy( this->getInitialPoint().getPathCoord(), this->getFinalPoint().getPathCoord() );
		time_interval.setBoundsLazy( this->getInitialPoint().getTimeCoord(), this->getFinalPoint().getTimeCoord() );
		if ( !path_interval.contains(p, c) || !time_interval.contains(t, c) ) {
			return false;
		}
		
		return true;
	}
	
	void PVT_ObstacleEdge::translate( double path_offset, double time_offset, const Constraints& c ) {
		this->p1->translate( path_offset, time_offset );
		this->p2->translate( path_offset, time_offset );
		this->init( c );
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_ObstacleEdge& oe) {
		return out << oe.getInitialPoint() << " => " << oe.getFinalPoint() << ", slope: " << oe.getSlope();
	}

} // end PVTP namespace