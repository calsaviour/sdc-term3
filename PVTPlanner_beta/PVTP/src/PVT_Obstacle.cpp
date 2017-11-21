#include <PVTP/Constants.hpp>
#include <PVTP/PVT_Obstacle.hpp>
#include <PVTP/Collisions.hpp>
#include <PVTP/Utilities.hpp>

namespace PVTP {
	
	PVT_Obstacle::PVT_Obstacle ( const double box[], const Constraints& c ) {
		this->min_path_coord = NULL;
		this->max_path_coord = NULL;
		this->min_time_coord = NULL;
		this->max_time_coord = NULL;
		std::vector<PVT_Point*> box_vector( 4, NULL );
		PVT_Point p1( box[0], box[3] );
		box_vector[0] = &p1;
		PVT_Point p2( box[1], box[3] );
		box_vector[1] = &p2;
		PVT_Point p3( box[1], box[2] );
		box_vector[2] = &p3;
		PVT_Point p4( box[0], box[2] );
		box_vector[3] = &p4;
		this->init( box_vector, c );
	}
	
	PVT_Obstacle::PVT_Obstacle ( const double polygon[][2], size_t num_points, const Constraints& c ) {
		this->min_path_coord = NULL;
		this->max_path_coord = NULL;
		this->min_time_coord = NULL;
		this->max_time_coord = NULL;
		std::vector<PVT_Point*> box_vector( num_points, NULL );
		for ( size_t i=0; i<num_points; ++i ) {
			box_vector[i] = new PVT_Point( polygon[i][0], polygon[i][1] );
		}

		this->init( box_vector, c );

		for ( size_t i=0; i<box_vector.size(); ++i ) {
			delete( box_vector[i] );
		}
	}
	
	PVT_Obstacle::PVT_Obstacle ( const std::vector< std::pair<double, double> >& polygon, const Constraints& c ) {
		this->min_path_coord = NULL;
		this->max_path_coord = NULL;
		this->min_time_coord = NULL;
		this->max_time_coord = NULL;
		std::vector<PVT_Point*> box_vector( polygon.size(), NULL );
		for ( size_t i=0; i<polygon.size(); ++i ) {
			box_vector[i] = new PVT_Point( polygon[i].first, polygon[i].second );
		}
		
		this->init( box_vector, c );
		
		for ( size_t i=0; i<box_vector.size(); ++i ) {
			delete( box_vector[i] );
		}
	}
	
	PVT_Obstacle::PVT_Obstacle ( const std::vector<PVT_Point*>& box, const Constraints& c ) {
		this->min_path_coord = NULL;
		this->max_path_coord = NULL;
		this->min_time_coord = NULL;
		this->max_time_coord = NULL;
		this->init( box, c );
	}
	
	PVT_Obstacle::PVT_Obstacle ( const PVT_Obstacle& obs, const Constraints& c ) {
		this->min_path_coord = NULL;
		this->max_path_coord = NULL;
		this->min_time_coord = NULL;
		this->max_time_coord = NULL;
		this->freeVertices();
		this->vertices.resize( obs.vertices.size(), NULL );
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			this->vertices[i] = new PVT_ObstaclePoint( *obs.vertices[i], c );
		}
		this->findDirectionalPoints( c );

		// run this *after* findDirectionalPoints
		this->findExtents();
		
		// run this *after* vertices are filled
		this->buildEdges( c );
	}

	void PVT_Obstacle::init( const std::vector<PVT_Point*>& polygon, const Constraints& c ) {
		this->freeVertices();
		if ( polygon.size() < 3 ) {
			throw Constants::INSUFFICIENT_VERTICES;
		}
		
		// project initial point, declare holders for iteration
		double prev_p;
		double prev_t;
		double next_p;
		double next_t;
		double cur_p = Utilities::truncateValue( polygon.front()->getPathCoord(), 0., c.getXLimit() );
		double cur_t = Utilities::truncateValue( polygon.front()->getTimeCoord(), 0., c.getTLimit() );
		//double cur_p = polygon.front()->getPathCoord();
		//double cur_t = polygon.back()->getTimeCoord();
		
		// initialize PT obstacle
		this->vertices.push_back( new PVT_ObstaclePoint(cur_p, cur_t, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), c) );
		
		// add non-redundant vertices
		double next_i;
		for ( size_t i=1; i<polygon.size(); ++i ) {
			size_t next_i = i + 1;
			if ( next_i == polygon.size() ) {
				next_i = 0;
			}
			
			//
			// project points to planning horizons
			//

			prev_p = Utilities::truncateValue( polygon[i-1]->getPathCoord(), 0., c.getXLimit() );
			prev_t = Utilities::truncateValue( polygon[i-1]->getTimeCoord(), 0., c.getTLimit() );
			cur_p = Utilities::truncateValue( polygon[i]->getPathCoord(), 0., c.getXLimit() );
			cur_t = Utilities::truncateValue( polygon[i]->getTimeCoord(), 0., c.getTLimit() );
			next_p = Utilities::truncateValue( polygon[next_i]->getPathCoord(), 0., c.getXLimit() );
			next_t = Utilities::truncateValue( polygon[next_i]->getTimeCoord(), 0., c.getTLimit() );
			
			//prev_p = polygon[i-1]->getPathCoord();
			//prev_t = polygon[i-1]->getTimeCoord();
			//cur_p = polygon[i]->getPathCoord();
			//cur_t = polygon[i]->getTimeCoord();
			//next_p = polygon[next_i]->getPathCoord();
			//next_t = polygon[next_i]->getTimeCoord();
			
			//
			// collapse for redundant vertices, i.e., those that are not extrema of polygon
			//
			
			double slope_prev = Maths::slope( cur_p, cur_t, prev_p, prev_t, c.getEpsilon() );
			double slope_next = Maths::slope( next_p, next_t, cur_p, cur_t, c.getEpsilon() );
			
			// conditions for same slope
			bool both_NaN = Maths::isNaN( slope_prev ) && Maths::isNaN( slope_next );
			bool both_equal = Maths::approxEq( slope_prev, slope_next, c.getEpsilon() );
			
			// both lines same slope
			if ( both_NaN || both_equal ) {
				continue;
			}
			
			// this vertex adds new information
			this->vertices.push_back( new PVT_ObstaclePoint(cur_p, cur_t, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), c) );
		}
		
		// polygon must have at least three vertices
		if ( this->vertices.size() < 3 ) {
			throw Constants::INSUFFICIENT_VERTICES;
		}
		
		this->findDirectionalPoints( c );

		// run this *after* findDirectionalPoints
		this->findExtents();
		
		// run this *after* vertices are filled
		this->buildEdges( c );
	}
	
	PVT_Obstacle::~PVT_Obstacle () {
		this->freeVertices();
		this->freeEdges();
	}
	
	void PVT_Obstacle::freeVertices( void ) {
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			delete( this->vertices[i] );
		}
		this->vertices.clear();
	}
	
	void PVT_Obstacle::freeEdges( void ) {
		for ( size_t i=0; i<this->edges.size(); ++i ) {
			delete( this->edges[i] );
		}
		this->edges.clear();
	}
	
	void PVT_Obstacle::buildEdges( const Constraints& c ) {
		this->freeEdges();
		
		size_t i = this->vertices.size() - 1;
		for ( size_t j=0; j<this->vertices.size(); ++j ) {
			PVT_ObstaclePoint * p1 = this->vertices[i];
			PVT_ObstaclePoint * p2 = this->vertices[j];
			this->edges.push_back( new PVT_ObstacleEdge(*p1, *p2, c) );
		}
	}
	
	void PVT_Obstacle::findDirectionalPoints( const Constraints& c ) {
		
		if ( this->vertices.empty() ) {
			return;
		}
		
		size_t i = this->vertices.size() - 1;
		for ( size_t j=0; j<this->vertices.size(); ++j ) {
			size_t k = (j + 1) % this->vertices.size();
			PVT_ObstaclePoint * p1 = this->vertices[i];
			PVT_ObstaclePoint * p2 = this->vertices[j];
			PVT_ObstaclePoint * p3 = this->vertices[k];
			
			// Set min/max feasible velocities: the edge slope will be the
			// max feasible velocity at p1, and the min feasible velocity at p2
			PVT_ObstacleEdge * prev_edge = new PVT_ObstacleEdge( *p1, *p2, c );
			PVT_ObstacleEdge * next_edge = new PVT_ObstacleEdge( *p2, *p3, c );
			
			if ( p2->incoming_edge != NULL ) {
				delete( p2->incoming_edge );
			}
			p2->incoming_edge = prev_edge;
			
			if ( p2->outgoing_edge != NULL ) {
				delete( p2->outgoing_edge );
			}
			p2->outgoing_edge = next_edge;
			
			double prev_edge_slope = Maths::isNaN(prev_edge->getSlope()) ? std::numeric_limits<double>::max() : prev_edge->getSlope();
			double next_edge_slope = Maths::isNaN(next_edge->getSlope()) ? std::numeric_limits<double>::max() : next_edge->getSlope();

			if ( (prev_edge->getPathDelta() >= 0.) && (prev_edge->getTimeDelta() >= 0.) ) {
				
				// 9 o'clock to 12 o'clock -- ok
				if ( (next_edge->getPathDelta() >= 0.) && (next_edge->getTimeDelta() >= 0.) ) {
					p2->i.setMin( prev_edge_slope );
					p2->i.setMax( next_edge_slope );
				}
				
				// 12 o'clock (obtuse) -- ok
				if ( (next_edge->getPathDelta() >= 0.) && (next_edge->getTimeDelta() < 0.) ) {
					if ( Maths::approxEq(next_edge_slope, 0., c.getEpsilon()) ) {
						p2->i.setMin( 0. );
					} else {
						p2->i.setMin( prev_edge_slope );
					}
					p2->i.setMax( std::numeric_limits<double>::max() );
				}
				
				// 12 o'clock (acute) -- ok
				if ( (next_edge->getPathDelta() < 0.) && (next_edge->getTimeDelta() < 0.) ) {
					p2->i.setMin( next_edge_slope );
					p2->i.setMax( std::numeric_limits<double>::max() );
				}
			}
			
			if ( (prev_edge->getPathDelta() >= 0.) && (prev_edge->getTimeDelta() < 0.) ) {
				
				// 12 o'clock to 3 o'clock -- ok
				if ( (next_edge->getPathDelta() >= 0.) && (next_edge->getTimeDelta() < 0.) ) {
					p2->i.setEmpty( true );
				}
				
				// 3 o'clock -- ok
				if ( (next_edge->getPathDelta() < 0.) && (next_edge->getTimeDelta() < 0.) ) {
					p2->i.setMin( prev_edge_slope );
					p2->i.setMax( next_edge_slope );
				}
			}
			
			if ( (prev_edge->getPathDelta() < 0.) && (prev_edge->getTimeDelta() < 0.) ) {
				
				// 3 o'clock to 6 o'clock -- ok
				if ( (next_edge->getPathDelta() < 0.) && (next_edge->getTimeDelta() < 0.) ) {
					p2->i.setMin( prev_edge_slope );
					p2->i.setMax( next_edge_slope );
				}
				
				// 6 o'clock (obtuse) -- ok
				if ( (next_edge->getPathDelta() <= 0.) && (next_edge->getTimeDelta() >= 0.) ) {
					p2->i.setMax( std::numeric_limits<double>::max() );
					if ( Maths::approxEq(next_edge_slope, 0., c.getEpsilon()) ) {
						p2->i.setMin( 0. );
					} else {
						p2->i.setMin( prev_edge_slope );
					}
				}
				
				// 6 o'clock (acute) -- ok
				if ( (next_edge->getPathDelta() > 0.) && (next_edge->getTimeDelta() > 0.) ) {
					p2->i.setMin( next_edge_slope );
					p2->i.setMin( prev_edge_slope );
				}
			}
			
			if ( (prev_edge->getPathDelta() < 0.) && (prev_edge->getTimeDelta() >= 0.) ) {
				
				// 6 o'clock to 9 o'clock
				if ( (next_edge->getPathDelta() < 0.) && (next_edge->getTimeDelta() >= 0.) ) {
					p2->i.setEmpty( true );
				}
				
				// 9 o'clock
				if ( (next_edge->getPathDelta() >= 0.) && (next_edge->getTimeDelta() >= 0.) ) {
					p2->i.setMin( prev_edge_slope );
					p2->i.setMax( next_edge_slope );
				}
			}
			i = j;
		}
		
		
	}
	
	void PVT_Obstacle::findExtents( void ) {
		if ( this->vertices.empty() ) {
			return;
		}

		PVT_ObstaclePoint * min_x = NULL;
		PVT_ObstaclePoint * max_x = NULL;
		PVT_ObstaclePoint * min_t = NULL;
		PVT_ObstaclePoint * max_t = NULL;
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			if ( (min_x == NULL) || (this->vertices[i]->getPathCoord() < min_x->getPathCoord()) ) {
				min_x = this->vertices[i];
			}
			if ( (max_x == NULL) || (this->vertices[i]->getPathCoord() > max_x->getPathCoord()) ) {
				max_x = this->vertices[i];
			}
			if ( (min_t == NULL) || (this->vertices[i]->getTimeCoord() < min_t->getTimeCoord()) ) {
				min_t = this->vertices[i];
			}
			if ( (max_t == NULL) || (this->vertices[i]->getTimeCoord() > max_t->getTimeCoord()) ) {
				max_t = this->vertices[i];
			}
		}

		this->min_path_coord = min_x;
		this->max_path_coord = max_x;
		this->min_time_coord = min_t;
		this->max_time_coord = max_t;
	}
	
	double PVT_Obstacle::getMinPathCoord() const {
		return this->min_path_coord->getPathCoord();
	}
	
	double PVT_Obstacle::getMaxPathCoord() const {
		return this->max_path_coord->getPathCoord();
	}

	double PVT_Obstacle::getMinTimeCoord() const {
		return this->min_time_coord->getTimeCoord();
	}

	double PVT_Obstacle::getMaxTimeCoord() const {
		return this->max_time_coord->getTimeCoord();
	}
	
	const PVT_ObstaclePoint& PVT_Obstacle::getMinPathPoint() const {
		return *this->min_path_coord;
	}
	
	const PVT_ObstaclePoint& PVT_Obstacle::getMaxPathPoint() const {
		return *this->max_path_coord;
	}
	
	const PVT_ObstaclePoint& PVT_Obstacle::getMinTimePoint() const {
		return *this->min_time_coord;
	}
	
	const PVT_ObstaclePoint& PVT_Obstacle::getMaxTimePoint() const {
		return *this->max_time_coord;
	}
	
	void PVT_Obstacle::translateObstacle( double x_offset, double t_offset ) {
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			this->vertices[i]->translate( x_offset, t_offset );
		}
	}
	
	bool PVT_Obstacle::isBoundaryPoint( const PVT_Point& p, const Constraints& c ) const {
		double epsilon = c.getEpsilon();
		
		// obstacles are open sets; so if a point lies directly on a border, it is not contained
		Interval line_path_interval;
		Interval line_time_interval;
		size_t j = this->vertices.size() - 1;
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			PVT_Point * p1 = (PVT_Point *)this->vertices[i];
			PVT_Point * p2 = (PVT_Point *)this->vertices[j];
			
			line_path_interval.setBoundsLazy( p1->getPathCoord(), p2->getPathCoord() );
			line_time_interval.setBoundsLazy( p1->getTimeCoord(), p2->getTimeCoord() );
			
			double slope = Maths::slope( p1->getTimeCoord(), p1->getPathCoord(), p2->getTimeCoord(), p2->getPathCoord(), epsilon );
			
			//
			// Point lies on boundary?
			//
			
			if ( Maths::isNaN(slope) ) {
				if ( Maths::approxEq(p.getTimeCoord(), p1->getTimeCoord(), epsilon) && line_path_interval.contains(p.getPathCoord(), c) ) {
					return true;
				}
			} else {
				double intercept = Maths::yIntercept( p1->getTimeCoord(), p1->getPathCoord(), slope, epsilon );
				double p_test = Maths::yFromSlopeIntercept( slope, p.getTimeCoord(), intercept, epsilon );
				if ( Maths::approxEq(p.getPathCoord(), p_test, epsilon)
					&& line_path_interval.contains(p.getPathCoord(), c)
					&& line_time_interval.contains(p.getTimeCoord(), c) ) {
					return true;
				}
			}
			
			// move to next polygon edge
			j = i;
		}
		
		return false;
	}

	bool PVT_Obstacle::containsPoint( const PVT_Point& p, const Constraints& c ) const {

		double epsilon = c.getEpsilon();
		double x = p.getPathCoord();
		double t = p.getTimeCoord();
		bool oddNodes = false;
		
		// otherwise check interior
		Interval line_path_interval;
		Interval line_time_interval;
		size_t j = this->vertices.size() - 1;
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			PVT_Point * p1 = (PVT_Point *)this->vertices[i];
			PVT_Point * p2 = (PVT_Point *)this->vertices[j];
			
			line_path_interval.setBoundsLazy( p1->getPathCoord(), p2->getPathCoord() );
			line_time_interval.setBoundsLazy( p1->getTimeCoord(), p2->getTimeCoord() );
			
			double slope = Maths::slope( p1->getTimeCoord(), p1->getPathCoord(), p2->getTimeCoord(), p2->getPathCoord(), epsilon );
			
			//
			// Point lies on boundary?
			//

			if ( Maths::isNaN(slope) ) {
				if ( Maths::approxEq(p.getTimeCoord(), p1->getTimeCoord(), epsilon) && line_path_interval.contains(p.getPathCoord(), c) ) {
					return false;
				}
			} else {
				double intercept = Maths::yIntercept( p1->getTimeCoord(), p1->getPathCoord(), slope, epsilon );
				double p_test = Maths::yFromSlopeIntercept( slope, p.getTimeCoord(), intercept, epsilon );
				if ( Maths::approxEq(p.getPathCoord(), p_test, epsilon)
					&& line_path_interval.contains(p.getPathCoord(), c)
					&& line_time_interval.contains(p.getTimeCoord(), c) ) {
					return false;
				}
			}

			//
			// Point lies in interior?
			//
			
			// adapted from: http://alienryderflex.com/polygon/
			if ( ((Maths::approxLt(this->vertices[i]->getTimeCoord(), t, epsilon)) && (Maths::approxGe(this->vertices[j]->getTimeCoord(), t, epsilon)))
				|| ((Maths::approxLt(this->vertices[j]->getTimeCoord(), t, epsilon)) && (Maths::approxGe(this->vertices[i]->getTimeCoord(), t, epsilon)))
				&& ((Maths::approxLe(this->vertices[i]->getPathCoord(), x, epsilon)) || (Maths::approxLe(this->vertices[j]->getPathCoord(), x, epsilon))) ) {
				
				double tmp = this->vertices[i]->getPathCoord() + (t - this->vertices[i]->getTimeCoord()) / (this->vertices[j]->getTimeCoord() - this->vertices[i]->getTimeCoord()) * (this->vertices[j]->getPathCoord() - this->vertices[i]->getPathCoord());
				
				oddNodes ^= Maths::approxLt( tmp, x, epsilon );
				
			}
			
			// move to next polygon edge
			j = i;			
		}
		
		return oddNodes;
	}
	
	bool PVT_Obstacle::equals( const PVT_Obstacle& other_obstacle, const Constraints& c ) const {
		if ( this->vertices.size() != other_obstacle.vertices.size() ) {
			return false;
		}
		for ( size_t i=0; i<this->vertices.size(); ++i ) {
			if ( !this->vertices[i]->equals(*other_obstacle.vertices[i], c) ) {
				return false;
			}
		}
		return true;
	}
	
	std::string PVT_Obstacle::exceptionMessage( int e ) {
		std::stringstream ss;
		ss << "PVT Obstacle construction failed: ";
		switch ( e ) {
			case Constants::INSUFFICIENT_VERTICES:
				ss << "Obstacle must have at least three non-redundant vertices.";
				break;
			default:
				return "";
		}
		//std::cerr << ss.str().c_str() << std::endl;
		return ss.str();
	}

	std::ostream& operator<<(std::ostream& out, const PVT_Obstacle& o) {
		for ( size_t i=0; i<o.vertices.size(); ++i ) {
			out << *(o.vertices[i]) << std::endl;
		}
		out << "extents: [" << o.getMinPathCoord() << ", " << o.getMaxPathCoord() << ", " << o.getMinTimeCoord() << ", " << o.getMaxTimeCoord() << "]" << std::endl;
		return out;
	}

} // end PVTP namespace