#ifndef PVTP_COLLISIONS_H
#define PVTP_COLLISIONS_H

#include <set>
#include <PVTP/Constants.hpp>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/TrajectorySegment.hpp>
#include <PVTP/PVT_ObstacleEdge.hpp>
#include <PVTP/XY_Point.hpp>

namespace PVTP {
	
	/**
	 * This namespace contains routines for library-specific collision detection.
	 */
	namespace Collisions {
		
		/**
		 * This method computes true or false depending on whether a point
		 * in contained within a polygon.
		 *
		 * Adapted from: http://alienryderflex.com/polygon/
		 */
		bool PointInPolygon( const Scenario::XY_Point& p, const std::vector<Scenario::XY_Point>& vertices, const Constraints& c );
		
		/**
		 * Check a given trajectory for collision with any obstacle in O, return
		 * a list of obstacles in collision via inCollision.
		 *
		 * If `edge' is a non-null pointer, it will be set with the first detected
		 * edge in collision. If return_obstacles is `true', the edge parameter is ignored.
		 *
		 * NOTE: It is the responsibility of the caller to free the memory pointed
		 * to by `edge' if it is non-NULL!
		 *
		 * NOTE: This routine is not quite correct: If a trajectory segment
		 * intersects an obstacle by passing through it exactly at the vertices,
		 * the intersection will not be recorded!
		 */
		bool checkTrajectory( std::set<PVT_Obstacle*>& inCollision,
							 const std::vector<TrajectorySegment*>& T,
							 const PVT_ObstacleSet& O,
							 const Constraints& c,
							 bool return_obstacles = false,
							 PVT_ObstacleEdge ** edge = NULL,
							 std::vector<Scenario::XY_Point> * points_of_intersection = NULL );
		
		/**
		 * Check a given trajectory segment for collision with any obstacle in O,
		 * return a list of obstacles in collision via inCollision.
		 *
		 * If `edge' is a non-null pointer, it will be set with the first detected
		 * edge in collision. If return_obstacles is `true', the edge parameter is ignored.
		 *
		 * NOTE: It is the responsibility of the caller to free the memory pointed
		 * to by `edge' if it is non-NULL!
		 *
		 * NOTE: This routine is not quite correct: If a trajectory segment
		 * intersects an obstacle by passing through it exactly at the vertices,
		 * the intersection will not be recorded!
		 */
		bool checkTrajectorySegment( std::set<PVT_Obstacle*>& inCollision,
									const TrajectorySegment& T,
									const PVT_ObstacleSet& O,
									const Constraints& c,
									double& _t_intersect,
									bool return_obstacles = false,
									PVT_ObstacleEdge ** edge = NULL,
									std::vector<Scenario::XY_Point> * points_of_intersection = NULL );
		
		/**
		 * Check whether a point is contained within any obstacles
		 *
		 * Currently this procedure assumes rectangular obstacles; future
		 * versions will handle arbitrary polygonal obstacles.
		 */
		void checkPoint( std::set<PVT_Obstacle*>& inCollision,
						const PVT_Point& p,
						const PVT_ObstacleSet& O,
						const Constraints& c,
						bool return_obstacles = false );
				
	} // end Collisions namespace

} // end PVTP namespace

#endif