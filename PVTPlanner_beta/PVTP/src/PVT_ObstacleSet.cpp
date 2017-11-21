#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/ExceptionHandler.hpp>

namespace PVTP {
	
	PVT_ObstacleSet::PVT_ObstacleSet () {
	}
	
	PVT_ObstacleSet::PVT_ObstacleSet ( const PVT_ObstacleSet& O, const Constraints& c ) {
		for ( size_t i=0; i<O.obstacles.size(); ++i ) {
			this->obstacles.push_back( new PVT_Obstacle( *O.obstacles[i], c ) );
		}
	}

	PVT_ObstacleSet::PVT_ObstacleSet ( const double boxes[][4], size_t box_count, const Constraints& c ) {
		this->init( boxes, box_count, c );
	}
	
	PVT_ObstacleSet::PVT_ObstacleSet ( const std::vector< std::vector<double> >& boxes, const Constraints& c ) {
		this->init( boxes, c );
	}
	
	PVT_ObstacleSet::PVT_ObstacleSet ( const std::vector< std::vector< std::pair<double, double> > >& polygons, const Constraints& c ) {
		for ( size_t i=0; i<polygons.size(); ++i ) {
			try {
				this->obstacles.push_back( new PVT_Obstacle(polygons[i], c) );
			} catch ( int e ) {
				std::cerr << "WARNING IN PVT_ObstaleSet: " << ExceptionHandler::exceptionMessageString( e ) << " Probably due to projection onto planning horizons." << std::endl;
			}
		}
	}
	
	PVT_ObstacleSet::~PVT_ObstacleSet () {
		this->freeObstacles();
	}
	
	void PVT_ObstacleSet::init( const double boxes[][4], size_t box_count, const Constraints& c ) {
		double epsilon = c.getEpsilon();
		this->freeObstacles();
		for ( size_t i=0; i<box_count; ++i ) {
			if ( Maths::approxGe(boxes[i][0], boxes[i][1], epsilon) ||
				Maths::approxGe(boxes[i][2], boxes[i][3], epsilon) ) {
				continue;
			}
			this->obstacles.push_back( new PVT_Obstacle(boxes[i], c) );
		}
	}

	void PVT_ObstacleSet::init( const std::vector< std::vector<double> >& boxes, const Constraints& c ) {
		double epsilon = c.getEpsilon();
		this->freeObstacles();
		for ( size_t i=0; i<boxes.size(); ++i ) {
			if ( Maths::approxGe(boxes[i][0], boxes[i][1], epsilon) ||
				Maths::approxGe(boxes[i][2], boxes[i][3], epsilon) ) {
				continue;
			}
			double boxes_array[] = { boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3] };
			this->obstacles.push_back( new PVT_Obstacle(boxes_array, c) );
		}
	}
	
	void PVT_ObstacleSet::init( const PVT_ObstacleSet& O, const Constraints& c ) {
		this->freeObstacles();
		for ( size_t i=0; i<O.obstacles.size(); ++i ) {
			this->obstacles.push_back( new PVT_Obstacle( *O.obstacles[i], c ) );
		}
	}
	
	void PVT_ObstacleSet::addRectangles( const std::vector< std::vector<double> >& boxes, const Constraints& c ) {
		this->init( boxes, c );
	}
	
	void PVT_ObstacleSet::addObstacles( const PVT_ObstacleSet& O, const Constraints& c ) {
		for ( size_t i=0; i<O.obstacles.size(); ++i ) {
			this->obstacles.push_back( new PVT_Obstacle( *O.obstacles[i], c ) );
		}
	}
	
	void PVT_ObstacleSet::translateObstacles( double x_offset, double t_offset ) {
		for ( size_t i=0; i<this->obstacles.size(); ++i ) {
			this->obstacles[i]->translateObstacle( x_offset, t_offset );
		}
	}
	
	void PVT_ObstacleSet::freeObstacles() {
		for ( size_t i=0; i<this->obstacles.size(); ++i ) {
			delete( this->obstacles[i] );
		}
		this->obstacles.clear();
	}
	
	void PVT_ObstacleSet::getPotentiallyReachableVertices( std::vector<PVT_ObstaclePoint*>& vertices, const Constraints& c ) const {
		for ( size_t i=0; i<this->obstacles.size(); ++i ) {
			PVT_Obstacle * obs = this->obstacles[i];
			for ( size_t j=0; j<obs->vertices.size(); j++ ) {
				if ( obs->vertices[j]->isReachable(c) ) {
					vertices.push_back( obs->vertices[j] );
				}
			}
		}
	}
	
	bool PVT_ObstacleSet::inCollision( const PVT_Point& p, const Constraints& c ) const {
		
		for ( size_t i=0; i<this->obstacles.size(); ++i ) {
			PVT_Obstacle * o = this->obstacles[i];
			if ( o->containsPoint(p, c) ) {
				return true;
			}
		}
		return false;

	}
	
	bool PVT_ObstacleSet::inCollision( double path_position, double time, const Constraints& c ) const {
		PVT_Point p(path_position, time);
		return this->inCollision( p, c );
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_ObstacleSet& of) {
		if ( of.obstacles.empty() ) {
			out << "Obstacle set is NULL.";
		} else {
			for ( size_t i=0; i<of.obstacles.size(); ++i ) {
				std::cout << "Obstacle:" << std::endl;
				out << *(of.obstacles[i]);
			}
		}
		return out;
	}

} // end PVTP namespace
