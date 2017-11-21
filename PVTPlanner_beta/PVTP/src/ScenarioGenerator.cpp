#include <sstream>
#include <fstream>

#include <PVTP/ScenarioGenerator.hpp>
#include <PVTP/Collisions.hpp>
#include <PVTP/Utilities.hpp>

using namespace PVTP;

namespace Scenario {
	
#ifdef BUILDBOOST1
#if BUILDBOOST1
	namespace Obstacles {
		
		/**
		 * Convenience method for checking validity of found intersection point
		 */
		bool checkPoint( double& first_path_position,
						double& last_path_position,
						const XY_Point& intersection_point,
						const XY_Point& reference_test,
						double edge_x1,
						double edge_y1,
						double edge_x2,
						double edge_y2,
						size_t segment_index,
						const PathModel& path,
						const Constraints& c ) {

			// if the point is not on the edge, there is no collision
			Interval edge_x;
			Interval edge_y;
			edge_x.setBoundsLazy( edge_x1, edge_x2 );
			edge_y.setBoundsLazy( edge_y1, edge_y2 );
			if ( !edge_x.contains(intersection_point.getX(), c) || !edge_y.contains(intersection_point.getY(), c) ) {
				return false;
			}
			
			// if the reference point is not on the current path segment
			double arc_length;
			if ( !path.segments[segment_index].containsPoint(reference_test, c) ) {
				
				// if reference point occurs before/after beginning of path segment
				if ( (segment_index == 0) && path.segments[segment_index].pointBefore(reference_test.getX(), reference_test.getY()) ) {
					arc_length = 0.;
				} else if ( (segment_index == (path.segments.size()-1)) && path.segments[segment_index].pointAfter(reference_test.getX(), reference_test.getY()) ) {
					arc_length = path.getLength();
				} else {
					return false;
				}
			} else {
				arc_length = path.getArcLengthToPoint( reference_test, c );
			}

			
			/**
			 * At this point the reference point is on the driver's
			 * segment, and the collision point is on the edge.
			 * Compute the arc length to the collision point and
			 * record the extrema of the collisions as the extents
			 * of the PT obstacle.
			 */
			
			
			bool found = false;
			if ( (arc_length < first_path_position) || Maths::isNaN(first_path_position) ) {
				first_path_position = arc_length;
				found = true;
			}
			
			if ( (arc_length > last_path_position) || Maths::isNaN(last_path_position) ) {
				last_path_position = arc_length;
				found = true;
			}
			return found;
		}
		
		/**
		 * Convenience method for checking point-to-edge collisions
		 */
		int checkPointToEdge( double& first_path_position,
							  double& last_path_position,
							  VehicleModel& dm,
							  VehicleModel& wo,
							  VehicleModel& _dm,
							  VehicleModel& _wo,
							  PathSegment& seg,
							  double slope,
							  bool flipped,
							  size_t i,
							  const Constraints& c ) {
			
			/**
			 * CHECK FOR POINT-TO-EDGE COLLISIONS
			 */
			
			// loop through the vertices, checking for point-to-edge collisions
			bool found = false;
			for ( size_t j=0; j<dm.getVertexOffsets().size(); j++ ) {
				
				double delta_x = dm.getRawVertexOffsets()[j].getX();
				double delta_y = dm.getRawVertexOffsets()[j].getY();
				
				// check against each edge of obstacle vehicle
				size_t prev_obs_vertex = wo.getVertices().size() - 1;
				for ( size_t cur_obs_vertex=0; cur_obs_vertex<wo.getVertices().size(); cur_obs_vertex++ ) {
					
					double edge_x1 = wo.getVertices()[prev_obs_vertex].getX();
					double edge_y1 = wo.getVertices()[prev_obs_vertex].getY();
					double edge_x2 = wo.getVertices()[cur_obs_vertex].getX();
					double edge_y2 = wo.getVertices()[cur_obs_vertex].getY();
					
					double reference_x;
					double reference_y;
					if ( !Maths::computePointToLineReferencePoint(reference_x,
																  reference_y,
																  seg.coefficients[0],
																  seg.coefficients[1],
																  seg.coefficients[2],
																  slope,
																  delta_x,
																  delta_y,
																  edge_x1,
																  edge_y1,
																  edge_x2,
																  edge_y2,
																  c.getEpsilon()) ) {
						// track along edges
						prev_obs_vertex = cur_obs_vertex;
						continue;
					}
					
					// computed reference point
					XY_Point reference_test( reference_x, reference_y );
					
					// undo rotation that might have occurred
					if ( flipped ) {
						seg.rotate90CCW();
						reference_test.rotate90CCW();
						edge_x1 = _wo.getVertices()[prev_obs_vertex].getX();
						edge_y1 = _wo.getVertices()[prev_obs_vertex].getY();
						edge_x2 = _wo.getVertices()[cur_obs_vertex].getX();
						edge_y2 = _wo.getVertices()[cur_obs_vertex].getY();
					}
					
					// move vehicle to reference point to check it
					XY_Point vertex_check( 0., 0. );
					if ( _dm.computeVertexAtPoint(vertex_check, j, seg.getDirection(), reference_test.getX(), reference_test.getY(), seg.coefficients, c) ) {
						
						// check point for valid intersection
						if ( checkPoint(first_path_position, last_path_position, vertex_check, reference_test, edge_x1, edge_y1, edge_x2, edge_y2, i, _dm.getPath(), c) ) {
							found = true;
						}
						
					} else {
						return -1;
					}
					
					if ( flipped ) {
						seg.rotate90CW();
					}
					
					// track along edges
					prev_obs_vertex = cur_obs_vertex;
					
				}
				
			}
			
			return found ? 1 : 0;
			
		}
		
		/**
		 * Convenience method for checking edge-to-point collisions
		 */
		int checkEdgeToPoint( double& first_path_position,
							  double& last_path_position,
							  VehicleModel& dm,
							  VehicleModel& wo,
							  VehicleModel& _dm,
							  VehicleModel& _wo,
							  PathSegment& seg,
							  double slope,
							  bool flipped,
							  size_t i,
							  const Constraints& c ) {
			
			/**
			 * CHECK FOR EDGE-TO-POINT COLLISIONS
			 */
			
			// check against each edge of driver vehicle
			bool found = false;
			size_t prev_driver_vertex = dm.getVertices().size() - 1;
			for ( size_t cur_driver_vertex=0; cur_driver_vertex<dm.getVertices().size(); cur_driver_vertex++ ) {
				
				double delta_x1 = dm.getRawVertexOffsets()[prev_driver_vertex].getX();
				double delta_y1 = dm.getRawVertexOffsets()[prev_driver_vertex].getY();
				double delta_x2 = dm.getRawVertexOffsets()[cur_driver_vertex].getX();
				double delta_y2 = dm.getRawVertexOffsets()[cur_driver_vertex].getY();
				
				// for each edge, iterate over obstacle vertices
				for ( size_t j=0; j<wo.getVertices().size(); j++ ) {
					
					double reference_x;
					double reference_y;
					if ( !Maths::computeLineToPointReferencePoint(reference_x,
																  reference_y,
																  seg.coefficients[0],
																  seg.coefficients[1],
																  seg.coefficients[2],
																  slope,
																  delta_x1,
																  delta_y1,
																  delta_x2,
																  delta_y2,
																  wo.getVertices()[j].getX(),
																  wo.getVertices()[j].getY(),
																  c.getEpsilon()) ) {
						continue;
					}
					
					// computed reference point
					XY_Point reference_test( reference_x, reference_y );
					
					// undo rotation that might have occurred
					if ( flipped ) {
						seg.rotate90CCW();
						reference_test.rotate90CCW();
					}
					
					// move vehicle to reference point to check it
					XY_Point vertex1_check( 0., 0. );
					XY_Point vertex2_check( 0., 0. );
					if ( _dm.computeVertexAtPoint(vertex1_check, prev_driver_vertex, seg.getDirection(), reference_test.getX(), reference_test.getY(), seg.coefficients, c)
						&& _dm.computeVertexAtPoint(vertex2_check, cur_driver_vertex, seg.getDirection(), reference_test.getX(), reference_test.getY(), seg.coefficients, c) ) {

						double edge_x1 = vertex1_check.getX();
						double edge_y1 = vertex1_check.getY();
						double edge_x2 = vertex2_check.getX();
						double edge_y2 = vertex2_check.getY();
						
						// check point for valid intersection
						if ( checkPoint(first_path_position, last_path_position, _wo.getVertices()[j], reference_test, edge_x1, edge_y1, edge_x2, edge_y2, i, _dm.getPath(), c) ) {

							found = true;

						}
					} else {
						return -1;
					}
					
					if ( flipped ) {
						seg.rotate90CW();
					}
				}
				
				// track along edges
				prev_driver_vertex = cur_driver_vertex;
				
			}
			
			return found ? 1 : 0;
			
		}
		
		int ComputePointsOfIntersection( double& first_path_position,
										 double& last_path_position,
										 VehicleModel& _dm,
										 VehicleModel& _wo,
										 const Constraints& c ) {
			first_path_position = std::numeric_limits<double>::quiet_NaN();
			last_path_position = std::numeric_limits<double>::quiet_NaN();
			
			VehicleModel dm( _dm );
			VehicleModel wo( _wo );
			
			/**
			 * Check all vertices of driver for collision with obstacle edges
			 */
			
			// look for the path positions: loop through segments
			bool found = false;
			for ( size_t i=0; i<dm.getPath().segments.size(); i++ ) {
				
				PathSegment seg( dm.getPath().segments[i] );

				// if this segment of the driver path is vertical, transpose
				bool flipped = false;
				if ( seg.isVertical() ) {

					// flip path segment
					if ( !seg.rotate90CW() ) {
						std::cerr << "ERROR IN Obstacles::ComputePointsOfIntersection: Failed to rotate path segment 90 degrees CW " << seg << std::endl;
						return -1;
					}
					
					// flip driver
					dm.rotate90CW();
					
					// flip obstacle
					wo.rotate90CW();
					
					// set flipped flag
					flipped = true;
				}
				
				// make sure raw vertex offsets are specified as expected
				if ( seg.getDirection() < 0 ) {
					for ( size_t j=0; j<dm.getRawVertexOffsets().size(); j++ ) {
						dm.getRawVertexOffsets()[j].rotate90CCW();
						dm.getRawVertexOffsets()[j].rotate90CCW();
					}
				}
				
				//
				// For each segment, loop through driver vertices, record first and last collision positions
				//
				
				/**
				 * Assume for now that paths are piecewise linear
				 */
				
				// orientation of driver for this segment
				double slope;
				if ( !seg.getSlopeAtArcLength(slope, 0., c) ) {
					std::cerr << "ERROR IN Obstacles::ComputePointsOfIntersection: Error computing orientation of driver's vehicle." << std::endl;
					return -1;
				}
				
				// Check point-to-edge collisions
				int _f = checkPointToEdge(first_path_position, last_path_position, dm, wo, _dm, _wo, seg, slope, flipped, i, c);
				if ( _f == 1 ) {
					found = true;
				} else if ( _f == -1 ) {
					return -1;
				}
				
				// Check edge-to-point collisions
				_f = checkEdgeToPoint(first_path_position, last_path_position, dm, wo, _dm, _wo, seg, slope, flipped, i, c);
				if ( _f == 1 ) {
					found = true;
				} else if ( _f == -1 ) {
					return -1;
				}
				
				// undo flipped vehicles
				if ( flipped ) {
					dm.rotate90CCW();
					wo.rotate90CCW();
					seg.rotate90CCW();
					flipped = false;
				}
				
				// undo any raw vertex rotation
				if ( seg.getDirection() < 0 ) {
					for ( size_t j=0; j<dm.getRawVertexOffsets().size(); j++ ) {
						dm.getRawVertexOffsets()[j].rotate90CW();
						dm.getRawVertexOffsets()[j].rotate90CW();
					}
				}
				
				// first_path_position should be the first collision-free position
				// of the driver, so if it is in collision, then the entire
				// segment up to this point is in collision
				bool first_in_collision = false;
				if ( !Maths::isNaN(first_path_position) ) {
					dm.moveToPathPosition( first_path_position, c );
					
					// check for identical position
					bool overlaid = true;
					for ( size_t j=0; j<dm.getVertices().size(); j++ ) {
						bool vertex_found = false;
						for ( size_t k=0; k<dm.getVertices().size(); k++ ) {
							if ( wo.getVertices()[k].equals(dm.getVertices()[j], c) ) {
								vertex_found = true;
								break;
							}
						}
						if ( !vertex_found ) {
							overlaid = false;
							break;
						}
					}
					first_in_collision = overlaid;
					
					if ( !first_in_collision ) {
						
						size_t k = wo.getVertices().size() - 1;
						for ( size_t j=0; j<wo.getVertices().size(); j++ ) {
							
							// check vertex
							if ( Collisions::PointInPolygon(wo.getVertices()[j], dm.getVertices(), c) ) {
								first_in_collision = true;
								break;
							}
							
							// check previous midpoint
							XY_Point midpoint( (wo.getVertices()[j].getX() + wo.getVertices()[k].getX()) / 2.,
											  (wo.getVertices()[j].getY() + wo.getVertices()[k].getY()) / 2. );
							if ( Collisions::PointInPolygon(midpoint, dm.getVertices(), c) ) {
								first_in_collision = true;
								break;
							}
							
							// wrap
							k = j;
						}
						
					}
					
					if ( !first_in_collision ) {
						
						size_t k = dm.getVertices().size() - 1;
						for ( size_t j=0; j<dm.getVertices().size(); j++ ) {
							
							// check vertex
							if ( Collisions::PointInPolygon(dm.getVertices()[j], wo.getVertices(), c) ) {
								first_in_collision = true;
								break;
							}
							
							// check previous midpoint
							XY_Point midpoint( (dm.getVertices()[k].getX() + dm.getVertices()[j].getX()) / 2.,
											  (dm.getVertices()[k].getY() + dm.getVertices()[j].getY()) / 2. );
							if ( Collisions::PointInPolygon(midpoint, wo.getVertices(), c) ) {
								first_in_collision = true;
								break;
							}
							
							// wrap
							k = j;
						}
					}
				}
				
				// if the last path position is in collision, the entire remainder of the segment is in collision
				bool last_in_collision = false;
				if ( !Maths::isNaN(last_path_position) ) {
					dm.moveToPathPosition( last_path_position, c );
					
					// check for identical position
					bool overlaid = true;
					for ( size_t j=0; j<dm.getVertices().size(); j++ ) {
						bool vertex_found = false;
						for ( size_t k=0; k<dm.getVertices().size(); k++ ) {
							if ( wo.getVertices()[k].equals(dm.getVertices()[j], c) ) {
								vertex_found = true;
								break;
							}
						}
						if ( !vertex_found ) {
							overlaid = false;
							break;
						}
					}
					last_in_collision = overlaid;
					
					if ( !last_in_collision ) {
						
						size_t k = wo.getVertices().size() - 1;
						for ( size_t j=0; j<wo.getVertices().size(); j++ ) {
							
							// check vertex
							if ( Collisions::PointInPolygon(wo.getVertices()[j], dm.getVertices(), c) ) {
								last_in_collision = true;
								break;
							}
							
							// check previous midpiont
							XY_Point midpoint( (wo.getVertices()[j].getX() + wo.getVertices()[k].getX()) / 2.,
											  (wo.getVertices()[j].getY() + wo.getVertices()[k].getY()) / 2. );
							if ( Collisions::PointInPolygon(midpoint, dm.getVertices(), c) ) {
								last_in_collision = true;
								break;
							}
							
							// wrap
							k = j;
							
						}
						
					}
					
					if ( !last_in_collision ) {
						
						size_t k = dm.getVertices().size() - 1;
						for ( size_t j=0; j<dm.getVertices().size(); j++ ) {
							
							// check vertex
							if ( Collisions::PointInPolygon(dm.getVertices()[j], wo.getVertices(), c) ) {
								last_in_collision = true;
								break;
							}
							
							// check previous midpoint
							XY_Point midpoint( (dm.getVertices()[j].getX() + dm.getVertices()[k].getX()) / 2.,
											  (dm.getVertices()[j].getY() + dm.getVertices()[k].getY()) / 2. );
							if ( Collisions::PointInPolygon(midpoint, wo.getVertices(), c) ) {
								last_in_collision = true;
								break;
							}
							
							// wrap
							k = j;
							
						}
					}
					
				}
				
				if ( first_in_collision ) {
					first_path_position = std::min( first_path_position, dm.getPath().getArcLengthToPoint( seg.getOrigin(), c ) );
				}
				
				if ( last_in_collision ) {
					last_path_position = std::max( last_path_position, dm.getPath().getArcLengthToPoint( seg.getOrigin(), c ) );
				}

			}
			
			if ( Maths::approxEq(first_path_position, last_path_position, c.getEpsilon()) ) {
				first_path_position = std::numeric_limits<double>::quiet_NaN();
				last_path_position = std::numeric_limits<double>::quiet_NaN();
				found = false;
			}
			
			return found ? 1 : 0;
		}
		
		void BoundingPolygon( std::vector< std::pair<double, double> >& polygon,
							 const std::vector< std::pair<double, Interval> >& discretized_obstacle ) {
			
			// from 6 o'clock to 12 o'clock
			for ( size_t j=0; j<discretized_obstacle.size(); j++ ) {
				polygon.push_back( std::pair<double, double>(discretized_obstacle[j].second.getMin(), discretized_obstacle[j].first) );
			}
			
			// from 12 o'clock to 6 o'clock
			for ( ssize_t j=discretized_obstacle.size()-1; j>=0; j-- ) {
				polygon.push_back( std::pair<double, double>(discretized_obstacle[j].second.getMax(), discretized_obstacle[j].first) );
			}

		}
		
	};
#endif
#endif
	
} // end Scenario namespace
