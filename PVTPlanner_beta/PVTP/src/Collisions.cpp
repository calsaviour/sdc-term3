#include <stdio.h>
#include <errno.h>
#include <PVTP/Maths.hpp>
#include <PVTP/Collisions.hpp>
#include <PVTP/Utilities.hpp>

namespace PVTP {
	
	namespace Collisions {
		
		bool PointInPolygon( const Scenario::XY_Point& p, const std::vector<Scenario::XY_Point>& vertices, const Constraints& c ) {
			
			double epsilon = c.getEpsilon();
			
			bool oddNodes = false;
			
			size_t j = vertices.size() - 1;
			for ( size_t i=0; i<vertices.size(); ++i ) {
				
				Interval x_interval;
				Interval y_interval;
				x_interval.setBoundsLazy( vertices[j].getX(), vertices[i].getX() );
				y_interval.setBoundsLazy( vertices[j].getY(), vertices[i].getY() );
				
				// if point is on the edge, disregard
				if ( x_interval.contains(p.getX(), c) && y_interval.contains(p.getY(), c) ) {
					double slope = Maths::slope( vertices[j].getX(), vertices[j].getY(), vertices[i].getX(), vertices[i].getY(), epsilon );
					double intercept = Maths::yIntercept( vertices[j].getX(), vertices[j].getY(), slope, epsilon );
					
					// vertical edge
					if ( Maths::isNaN(slope) ) {
						return false;
					}
					
					// sloped edge
					double y_test = Maths::yFromSlopeIntercept( slope, p.getX(), intercept, epsilon );
					if ( Maths::approxEq(p.getY(), y_test, epsilon) ) {
						return false;
					}
					
				}
				
				// adapted from: http://alienryderflex.com/polygon/
				if ( ((Maths::approxLt(vertices[i].getY(), p.getY(), epsilon)) && (Maths::approxGe(vertices[j].getY(), p.getY(), epsilon)))
					|| ((Maths::approxLt(vertices[j].getY(), p.getY(), epsilon)) && (Maths::approxGe(vertices[i].getY(), p.getY(), epsilon)))
					&& ((Maths::approxLe(vertices[i].getX(), p.getX(), epsilon)) || (Maths::approxLe(vertices[j].getX(), p.getX(), epsilon))) ) {
					
					double tmp = vertices[i].getX() + (p.getY() - vertices[i].getY()) / (vertices[j].getY() - vertices[i].getY()) * (vertices[j].getX() - vertices[i].getX());
					
					oddNodes ^= Maths::approxLt( tmp, p.getX(), epsilon );
					
				}
				
				// move to next polygon edge
				j = i;
				
			}
			
			return oddNodes;
			
		}
		
		void checkPoint( std::set<PVT_Obstacle*>& inCollision,
						const PVT_Point& p,
						const PVT_ObstacleSet& O,
						const Constraints& c,
						bool return_obstacles ) {
			for ( size_t i=0; i<O.obstacles.size(); ++i ) {
				PVT_Obstacle* o = O.obstacles[i];
				if ( o->containsPoint(p, c) ) {
					inCollision.insert( o );
					if ( !return_obstacles ) {
						return;
					}
				}
			}
			
		}

		bool checkTrajectory( std::set<PVT_Obstacle*>& inCollision,
							 const std::vector<TrajectorySegment*>& T,
							 const PVT_ObstacleSet& O,
							 const Constraints& c,
							 bool return_obstacles,
							 PVT_ObstacleEdge ** edge,
							 std::vector<Scenario::XY_Point> * points_of_intersection ) {
			if ( T.empty() || O.obstacles.empty() ) {
				return true;
			}
			
			double t_intersect = std::numeric_limits<double>::max();
			double min_edge_intersection_time = std::numeric_limits<double>::max();
			for ( size_t i=0; i<T.size(); ++i ) {
				if ( T[i]->isNullTransition(c) ) {
					continue;
				}
				if ( !checkTrajectorySegment(inCollision, *T[i], O, c, t_intersect, return_obstacles, edge, points_of_intersection) ) {
					std::cerr << "ERROR IN Collisions::checkTrajectory: Problem checking segment." << std::endl;
					return false;
				}
				if ( !return_obstacles && !inCollision.empty() ) {
					return true;
				}
				if ( t_intersect > min_edge_intersection_time ) {
					if ( *edge != NULL ) {
						delete( *edge );
						*edge = NULL;
					}
				} else {
					min_edge_intersection_time = t_intersect;
				}
			}
			return true;
		}
		
		bool checkTrajectorySegment( std::set<PVT_Obstacle*>& inCollision,
									const TrajectorySegment& T,
									const PVT_ObstacleSet& O,
									const Constraints& c,
									double& _t_intersect,
									bool return_obstacles,
									PVT_ObstacleEdge ** edge,
									std::vector<Scenario::XY_Point> * points_of_intersection ) {
			double epsilon = c.getEpsilon();
			
			// it is important to set `edge' to NULL if it is a valid pointer;
			// the caller needs to be able to check whether memory is actually
			// being pointed to or not
			if ( edge != NULL ) {
				*edge = NULL;

				// if we need the first edge of intersection, collision checking
				// cannot be short circuited because there's no guarantee about
				// the order of the obstacles or edges that are checked
				return_obstacles = true;
			}
			
			// if we're gathering all points of intersection, we must check all edges
			if ( points_of_intersection != NULL ) {
				return_obstacles = true;
			}

			// trajectory parameters
			double x1 = Maths::clipToZero( T.getInitialState().getPathCoord(), epsilon );
			double t1 = Maths::clipToZero( T.getInitialState().getTimeCoord(), epsilon );
			double v1 = Maths::clipToZero( T.getInitialState().getVelocityCoord(), epsilon );
			double x2 = Maths::clipToZero( T.getFinalState().getPathCoord(), epsilon );
			double t2 = Maths::clipToZero( T.getFinalState().getTimeCoord(), epsilon );
			double v2 = Maths::clipToZero( T.getFinalState().getVelocityCoord(), epsilon );
			double acc = Maths::clipToZero( T.getAcceleration(), epsilon );

			bool horizontal_trajectory = Maths::approxEq(t1, t2, epsilon);
			if ( horizontal_trajectory && (points_of_intersection == NULL) ) {
				return true;
			}

			PVT_Point initial_point( x1, t1 );
			PVT_Point final_point( x2, t2 );

			Interval traj_path_interval;
			if ( Maths::approxEq(x1, x2, epsilon) ) {
				traj_path_interval.setBounds(x1, x1);
			} else {
				traj_path_interval.setBounds(x1, x2);
			}

			Interval traj_time_interval;
			if ( Maths::approxEq(t1, t2, epsilon) ) {
				traj_time_interval.setBounds( t1, t1 );
			} else {
				traj_time_interval.setBounds( t1, t2 );
			}

			// check this trajectory segment against all obstacles
			double min_edge_intersection_time = std::numeric_limits<double>::max();
			for ( size_t i=0; i<O.obstacles.size(); ++i ) {
				PVT_Obstacle * o = O.obstacles[i];

				double x_o_min = o->getMinPathCoord();
				double t_o_max = o->getMaxTimeCoord();
				double x_o_max = o->getMaxPathCoord();
				double t_o_min = o->getMinTimeCoord();

				// obstacle before (in path dimension) trajectory
				if ( Maths::approxLe(x_o_max, x1, epsilon) ) {
					continue;
				}
				
				// obstacle after (in path dimension) trajectory
				if ( Maths::approxGe(x_o_min, x2, epsilon) ) {
					continue;
				}
				
				// obstacle before (in time dimension) this trajectory
				if ( Maths::approxLe(t_o_max, t1, epsilon) ) {
					continue;
				}
				
				// obstacle after (in time dimension) this trajectory
				if ( Maths::approxGe(t_o_min, t2, epsilon) ) {
					continue;
				}
				
				// obstacle cannot intersect before current min intersection
				if ( Maths::approxGe(t_o_min, min_edge_intersection_time, epsilon) ) {
					continue;
				}

				// otherwise, loop over obstacle sides, checking for intersections with trajectory
				Interval line_path_interval;
				Interval line_time_interval;
				size_t k = o->vertices.size() - 1;
				for ( size_t j=0; j<o->vertices.size(); ++j ) {

					// construct intervals corresponding to path and time portions of line segment
					PVT_ObstaclePoint * p1 = o->vertices[k];
					PVT_ObstaclePoint * p2 = o->vertices[j];
					line_path_interval.setBoundsLazy( p1->getPathCoord(), p2->getPathCoord() );
					line_time_interval.setBoundsLazy( p1->getTimeCoord(), p2->getTimeCoord() );
					k = j;

					// there must be common ranges in both path and time for intersection to occur
					Interval path_overlap;
					Interval time_overlap;
					Interval::intersect( path_overlap, traj_path_interval, line_path_interval, c );
					Interval::intersect( time_overlap, traj_time_interval, line_time_interval, c );
					if ( time_overlap.isEmpty() || path_overlap.isEmpty() ) {
						continue;
					}
					
					if ( horizontal_trajectory && (points_of_intersection != NULL) ) {
						inCollision.insert( o );
						points_of_intersection->push_back( Scenario::XY_Point(path_overlap.getMin(), t1) );
						continue;
					}

					// get slope for this line segment
					double slope = Maths::slope( p1->getTimeCoord(), p1->getPathCoord(), p2->getTimeCoord(), p2->getPathCoord(), epsilon );
					
					//
					// Horizontal side: Determine intersection point
					//
					std::vector<PVT_Point> intersection_points;
					
					// horizontal side: single-valued time interval
					if ( Maths::isNaN(slope) ) {
						double t_intersect = p1->getTimeCoord();
						double p_intersect = x1 + Maths::motionX_FromV1_T1_T2_A( v1, t1, t_intersect, acc, epsilon );
						if ( path_overlap.contains(p_intersect, c) && time_overlap.contains(t_intersect, c) ) {
							//std::cout << "before 1" << std::endl;
							intersection_points.push_back( PVT_Point(p_intersect, t_intersect) );
							//std::cout << "after 1" << std::endl;
						}
					}

					//
					// Sloping side: Determine intersection points
					//

					// get canonical coefficients for trajectory parabola
					double p_a;
					double p_b;
					double p_c;
					if ( !Maths::parabolaCoefficients1(p_a, p_b, p_c, x1, t1, v1, t2, v2, epsilon) ) {
						std::cerr << "ERROR IN Collisions::checkTrajectorySegment: Could not determine canonical trajectory coefficients." << std::endl;
						return false;
					}

					// determine intersection of line with parabola
					double intercept = Maths::yIntercept( p1->getTimeCoord(), p1->getPathCoord(), slope, epsilon );
					double q_a = p_a;
					double q_b = p_b - slope;
					double q_c = p_c - intercept;
					std::pair<double, double> roots;
					Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );

					// test both roots
					double r[] = { roots.first, roots.second };
					for ( size_t l=0; l<2; ++l ) {
						double t_intersect = r[l];
						double p_intersect = Maths::parabolaCanonical( p_a, p_b, p_c, t_intersect, epsilon );
						if ( path_overlap.contains(p_intersect, c) && time_overlap.contains(t_intersect, c) ) {
							//std::cout << "before 2: " << t_intersect << " " << p_intersect << std::endl;
							intersection_points.push_back( PVT_Point(p_intersect, t_intersect) );
							//std::cout << "after 2" << std::endl;
						}
					}
					
					//
					// Test intersection points
					//

					// if trajectory crosses obstacle boundary, record collision
					for ( size_t l=0; l<intersection_points.size(); ++l ) {
						//std::cout << "before 3" << std::endl;
						double p_intersect = intersection_points[l].getPathCoord();
						double t_intersect = intersection_points[l].getTimeCoord();
						//std::cout << "after 3" << std::endl;
						
						// ignore intersections at vertices and trajectory end points
						//std::cout << "before 4" << std::endl;
						if ( !(p1->equals(intersection_points[l], c)
							|| p2->equals(intersection_points[l], c)
							|| intersection_points[l].equals(initial_point, c)
							|| intersection_points[l].equals(final_point, c)) ) {
							inCollision.insert( o );
							if ( !return_obstacles ) {
								return true;
							}
							if ( points_of_intersection != NULL ) {
								points_of_intersection->push_back( Scenario::XY_Point(p_intersect, t_intersect) );
							}
							if ( (edge!=NULL) && (t_intersect<min_edge_intersection_time) ) {
								min_edge_intersection_time = t_intersect;
								delete( *edge );
								*edge = new PVT_ObstacleEdge( *p1, *p2, c );
							}
						}
						//std::cout << "after 4" << std::endl;
						
					} // end crossover test loop

				} // end vertex loop

				// check for trajectories contained by obstacle; this test requires
				// that the initial, final, *and* midpoint be contained, but this
				// check also needs to happen *after* all edges have been checked
				// in order to ensure that collision edges are properly detected first
				// because it is possible that these three points are contained
				// in the obstacle without the entire trajectory being contained
				double t_midpoint = t1 + (t2 - t1) / 2.;
				double p_midpoint = x1 + Maths::motionX_FromV1_T1_T2_A( v1, t1, t_midpoint, acc, epsilon );
				PVT_Point midpoint( p_midpoint, t_midpoint );
				if ( o->containsPoint(initial_point, c) || o->containsPoint(final_point, c) || o->containsPoint(midpoint, c) ) {
					inCollision.insert( o );
					if ( !return_obstacles ) {
						return true;
					}
					continue;
				}
				
			} // end obstacle loop
			
			_t_intersect = min_edge_intersection_time;

			return true;
		}

	} // end Collisions namespace

} // end PVTP namespace
