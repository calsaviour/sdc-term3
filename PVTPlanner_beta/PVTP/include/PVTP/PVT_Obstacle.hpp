#ifndef PVTP_OBSTACLE_H
#define PVTP_OBSTACLE_H

#include <iostream>
#include <vector>
#include <set>
#include <PVTP/PVT_ObstaclePt.hpp>
#include <PVTP/PVT_ObstacleEdge.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time Obstacle class. This class is for contructing and handling PT
	 * obstacles, which are assumed to be convex and polygonal in PT space.
	 */
	class PVT_Obstacle {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_Obstacle& o);
		
	public:
		
		/**
		 * PVT Obstacle constructor: polygonal obstacle.
		 *
		 * WARNING: THE POINTS MUST BE GIVEN IN SEQUENCE SUCH THAT THEY MARCH
		 * ALONG THE CIRCUMFERENCE USING CLOCKWISE ORDERING. The
		 * start point is irrelevant.
		 *
		 * double[] box = { {x, t}, ..., {x, t} }
		 */
		PVT_Obstacle ( const double polygon[][2], size_t num_points, const Constraints& c );
		
		/**
		 * Alternate version of above
		 */
		PVT_Obstacle ( const std::vector< std::pair<double, double> >& polygon, const Constraints& c );
		
		/**
		 * Legacy PVT Obstacle constructor: rectangular obstacle
		 *
		 * double[] box = { min_x, max_x, min_t, max_t }
		 */
		PVT_Obstacle ( const double box[], const Constraints& c );
		
		/**
		 * PVT Obstacle constructor: vector-based
		 */
		PVT_Obstacle ( const std::vector<PVT_Point*>& box, const Constraints& c );
		
		/**
		 * PVT Obstacle copy constructor
		 */
		PVT_Obstacle ( const PVT_Obstacle& obs, const Constraints& c );
		
		/**
		 * Destructor
		 */
		~PVT_Obstacle ();
		
		/**
		 * Translate the obstacle by some offset in path, and some offset in time
		 */
		void translateObstacle( double x_offset, double t_offset );
		
		/**
		 * Get minimum path extent of obstacle
		 */
		double getMinPathCoord() const;
		
		/**
		 * Get maximum path extent of obstacle
		 */
		double getMaxPathCoord() const;
		
		/**
		 * Get minimum time extent of obstacle
		 */
		double getMinTimeCoord() const;
		
		/**
		 * Get maximum time extent of obstacle
		 */
		double getMaxTimeCoord() const;
		
		/**
		 * Get minimum path extent of obstacle
		 */
		const PVT_ObstaclePoint& getMinPathPoint() const;
		
		/**
		 * Get maximum path extent of obstacle
		 */
		const PVT_ObstaclePoint& getMaxPathPoint() const;
		
		/**
		 * Get minimum time extent of obstacle
		 */
		const PVT_ObstaclePoint& getMinTimePoint() const;
		
		/**
		 * Get maximum time extent of obstacle
		 */
		const PVT_ObstaclePoint& getMaxTimePoint() const;
		
		/**
		 * Return true if this point lies on the obstacle boundary
		 */
		bool isBoundaryPoint( const PVT_Point& p, const Constraints& c ) const;
		
		/**
		 * Return true if this obstacle contains a given point; false otherwise.
		 *
		 * NOTE: By convention obstacles are open sets; thus boundary points are
		 * considered *not* to be contained by the obstacle.
		 */
		bool containsPoint( const PVT_Point& p, const Constraints& c ) const;
		
		/**
		 * Test for equality
		 */
		bool equals( const PVT_Obstacle& other_obstacle, const Constraints& c ) const;

		/**
		 * The PT coordinates of the obstacle: These are the vertices of the
		 * obstacle polygon. The ordering of the vertices is clockwise starting
		 * at the vertex with minimum path-coordinate and maximum time-coordinate.
		 */
		std::vector<PVT_ObstaclePoint*> vertices;
		
		/**
		 * Store the set of edges for this obstacle
		 */
		std::vector<PVT_ObstacleEdge*> edges;
		
		/**
		 * Initializer
		 */
		void init( const std::vector<PVT_Point*>& box, const Constraints& c );
		
		/**
		 * Project the bounds of an obstacle to the constraint horizons
		 */
		void projectToBounds( const Constraints& c );
		
		/**
		 * Message handler for exceptions thrown during construction
		 */
		static std::string exceptionMessage( int e );
		
	private:
		
		/**
		 * Store max extents for quick access
		 */
		PVT_ObstaclePoint * min_path_coord;
		PVT_ObstaclePoint * max_path_coord;
		PVT_ObstaclePoint * min_time_coord;
		PVT_ObstaclePoint * max_time_coord;
		
		/**
		 * Convenience function for getting extents
		 */
		void findExtents( void );
		
		/**
		 * Convenience function for ensuring that points are classified
		 * correctly, and for computing feasible velocities at vertices.
		 */
		void findDirectionalPoints( const Constraints& c );
		
		/**
		 * Convenience method for building edges
		 */
		void buildEdges( const Constraints& c );
		
		/**
		 * Convenience method for clearing vertices
		 */
		void freeVertices( void );
		
		/**
		 * Convenience method for clearing edges
		 */
		void freeEdges( void );
		
	};
	
} // end PVTP namespace

#endif