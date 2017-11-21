#ifndef PVTP_SCENARIO_GENERATOR_H
#define PVTP_SCENARIO_GENERATOR_H

#include <PVTP/Utilities.hpp>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/VehicleModelUncertain.hpp>

//size_t image_count = 0;

using namespace PVTP;

namespace Scenario {
	
#ifdef BUILDBOOST1
#if BUILDBOOST1
	namespace Obstacles {
		
		/**
		 * Convenience method for validity of collision points; separated
		 * out for debugging purposes.
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
						const Constraints& c );
		
		/**
		 * Convenience method for checking point-to-edge collisions; separated
		 * out for debugging purposes.
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
							  const Constraints& c );
		
		/**
		 * Convenience method for checking edge-to-point collisions; separated
		 * out for debugging purposes.
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
							  const Constraints& c );
		
		/**
		 * This method computes the first and last positions of intersection
		 * between two polygons along their specified paths
		 */
		int ComputePointsOfIntersection( double& first_path_position,
										 double& last_path_position,
										 VehicleModel& dm,
										 VehicleModel& wo,
										 const Constraints& c );
		
		/**
		 * Utility function for retrieving a set of vertices from an ordered
		 * set of forbidden path intervals
		 */
		void BoundingPolygon( std::vector< std::pair<double, double> >& polygon,
							 const std::vector< std::pair<double, Interval> >& discretized_obstacle );
	
		/**
		 * This method takes in a set of 2d polygonal world obstacles, their
		 * expected behavior, and their paths. Via simulation,
		 * the obstacles are marched forward in time and PT obstacles constructed
		 * from the path intersections that occur with the user path.
		 *
		 * The option to smooth the obstacle hull puts a smoother edge around
		 * the perimeter of computed PT obstacles and significantly speeds up
		 * planning at the (usually insignificant) cost of slightly more
		 * conservative obstacles.
		 */
		template <class T>
		bool ForwardObstacleSimulator( PVT_ObstacleSet& O,
									  std::vector<T>& WO,
									  VehicleModel& dm,
									  const Constraints& c,
									  double time_delta,
									  double time_max = std::numeric_limits<double>::quiet_NaN(),
									  bool smooth_obstacle_hull = true,
									  bool draw_output = false ) {
			
			if ( Maths::isNaN(time_max) ) {
				time_max = c.getTLimit();
			}
			
			// the set of discretized obstacles will be stored here, where
			// each world obstacle is a set of occupied portions of the path
			std::vector< std::vector< std::pair<double, Interval> > > discretized_obstacles( WO.size() );
			
			// march forward in time
			double next_time;
			double cur_time = 0.;
			while ( cur_time <= time_max ) {
				
				// cycle through each world obstacle
				bool found = false;
				for ( size_t i=0; i<WO.size(); i++ ) {
					
					// copy of vehicle model to do analysis with
					T _wo( WO[i] );
					T cur_wo( WO[i] );
					
					// march forward
					if ( WO[i].translateInTime(time_delta) ) {
						found = true;
					}
					
					// copy of translated vehicle model
					T next_wo( WO[i] );
					
					// get swept region between markers
					VehicleModel * swept_region = VehicleModelUncertain::getSweptRegion( cur_wo, next_wo );
					if ( swept_region == NULL ) {
						std::cerr << "ERROR IN Obstacles::ForwardObstacleSimulator: Error constructing swept region." << std::endl;
						return false;
					}
					
					/**
					 * This is the important part: Here the first and last positions
					 * of the driver's reference point that brings the driver vehicle
					 * into collision with the obstacle are computed.
					 */
					double first_path_position;
					double last_path_position;
					VehicleModel _dm( dm );
					int _f = ComputePointsOfIntersection( first_path_position, last_path_position, _dm, *swept_region, c );
					//int _f = ComputePointsOfIntersection( first_path_position, last_path_position, _dm, _wo, c );
					if ( _f == -1 ) {
						std::cerr << "ERROR IN Obstacles::ForwardObstacleSimulator: Error computing points of intersection." << std::endl;
						return false;
					}
					
					if ( draw_output && (true || (_f == 1)) ) {
						/**
						double old_path_position = dm.getPathPosition();
						
						std::string figure1 = Utilities::GetMatlabStandardFigure( true );
						if ( !dm.moveToPathPosition(first_path_position, c, true) ) {
							std::cerr << "this shouldn't happen (1)" << std::endl;
						}
						std::string world_scene1 = Utilities::GetMatlabWorldSceneString( dm, *swept_region );
						//std::string world_scene1 = Utilities::GetMatlabWorldSceneString( dm, _wo );
						if ( !dm.moveToPathPosition(last_path_position, c, false) )  {
							std::cerr << "this shouldn't happen (2)" << std::endl;
						}
						std::string last_pos = Utilities::GetMatlabVehicleModel( dm, true );
						std::stringstream ss1;
						ss1 << "set(gca,'FontSize',14);" << std::endl;
						//ss1 << "xlim([0 55]);" << std::endl;
						//ss1 << "ylim([-22 42]);" << std::endl;
						ss1 << "ylim([4 11]);" << std::endl;
						ss1 << "xlim([0 13]);" << std::endl;
						//ss1 << "ylim([-20 20]);" << std::endl;
						//ss1 << "xlim([-100 -50]);" << std::endl;
						//ss1 << "ylim([-30 5]);" << std::endl;
						//ss1 << "xlim([-150 -50]);" << std::endl;
						ss1 << "title('Min and Max collison positions of vehicle with obstacle - " << cur_time << "s" << "');" << std::endl;
						ss1 << "xlabel('World X Position (m)');" << std::endl;
						ss1 << "ylabel('World Y Position (m)');" << std::endl;
						ss1 << "h=get(gca,'xlabel');" << std::endl;
						ss1 << "set(h, 'FontSize', 16);" << std::endl;
						ss1 << "h=get(gca,'ylabel');" << std::endl;
						ss1 << "set(h, 'FontSize', 16);" << std::endl;
						ss1 << "h=get(gca,'title');" << std::endl;
						ss1 << "set(h, 'FontSize', 14);" << std::endl;
						ss1 << "print('-dpng',['data/images/world_scene_', num2str(" << (++image_count) << ")]);" << std::endl;
						ss1 << "saveas(gcf, ['data/images/world_scene_', num2str(" << (image_count) << ")], 'fig');" << std::endl;
						std::cout << figure1 << std::endl;
						std::cout << world_scene1 << std::endl;
						std::cout << last_pos << std::endl;
						std::cout << ss1.str() << std::endl;
						std::cout << "hold off;" << std::endl;
						std::cout << "close;" << std::endl;
						
						if ( !dm.moveToPathPosition( old_path_position, c ) )  {
							std::cerr << "this shouldn't happen (2)" << std::endl;
						}
						/**/
					}

					if ( swept_region != NULL ) {
						delete( swept_region );
					}
					swept_region = NULL;

					// if either of this is NaN, there is no intersection
					if ( Maths::isNaN(first_path_position) && Maths::isNaN(last_path_position) ) {
						continue;
					}
					if ( Maths::isNaN(first_path_position) || Maths::isNaN(last_path_position) ) {
						std::cerr << "ERROR IN Obtacles::ForwardObstacleSimulator: Invalid positions found." << std::endl;
						return false;
					}
					
					// store in interval set
					if ( !smooth_obstacle_hull || discretized_obstacles[i].empty() ) {
						discretized_obstacles[i].push_back( std::pair<double, Interval>(cur_time, Interval(first_path_position, last_path_position)) );
					}
					if ( smooth_obstacle_hull ) {
						double _first_path_position = std::min( discretized_obstacles[i].back().second.getMin(), first_path_position );
						double _last_path_position = std::max( discretized_obstacles[i].back().second.getMax(), last_path_position );
						discretized_obstacles[i].back().second.setBounds( _first_path_position, _last_path_position );
					}
					discretized_obstacles[i].push_back( std::pair<double, Interval>(cur_time + time_delta, Interval(first_path_position, last_path_position)) );
				}
				
				// break out if all obstacles have reached the end of their paths
				if ( !found ) {
					break;
				}
				
				// increment time
				cur_time += time_delta;
			}
			
			// for each obstacle, construct a polygon
			std::vector< std::vector< std::pair<double, double> > > polygons;
			for ( size_t i=0; i<discretized_obstacles.size(); i++ ) {
				
				std::vector< std::pair<double, double> > polygon;
				BoundingPolygon( polygon, discretized_obstacles[i] );
				if ( polygon.empty() ) {
					continue;
				}
				
				polygons.push_back( polygon );
			}
			
			// finally, build the PVT obstacle set
			PVT_ObstacleSet _O( polygons, c );
			O.addObstacles ( _O, c );
			
			return true;
			
		}

	} // end Obstacles namespace
#endif
#endif
	
} // end Scenario namespace

#endif
