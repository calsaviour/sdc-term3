#ifndef PVTP_OBSTACLE_SET_H
#define PVTP_OBSTACLE_SET_H

#include <PVTP/PVT_Obstacle.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time Obstacle Field class. This class acts as a container for all
	 * obstacles in a given scenario, and has convenience methods for extracting
	 * information from the field.
	 */
	class PVT_ObstacleSet {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_ObstacleSet& of);
		
	public:
		
		/**
		 * Constructor that builds an empty set
		 */
		PVT_ObstacleSet ();
		
		/**
		 * Obstacle field constructor. Rectangular obstacles.
		 */
		PVT_ObstacleSet ( const double boxes[][4], size_t box_count, const Constraints& c );
		
		/**
		 * Obstacle field constructor. Rectangular obstacles.
		 */
		PVT_ObstacleSet ( const std::vector< std::vector<double> >& boxes, const Constraints& c );
		
		/**
		 * Obstacle field constructor. Convex polygonal obstacles.
		 */
		PVT_ObstacleSet ( const std::vector< std::vector< std::pair<double, double> > >& polygons, const Constraints& c );
		
		/**
		 * Obstacle field constructor. Copy constructor
		 */
		PVT_ObstacleSet ( const PVT_ObstacleSet& O, const Constraints& c );
		
		/**
		 * Destructor
		 */
		~PVT_ObstacleSet ();
		
		/**
		 * Retrieve the vertices of all obstacles in this field, store in the
		 * vector pointed to by the passed reference.
		 */
		void getPotentiallyReachableVertices( std::vector<PVT_ObstaclePoint*>& vertices, const Constraints& c ) const;
		
		/**
		 * Translate a set of obstacles by some offset in path and some offset in time
		 */
		void translateObstacles( double x_offset, double t_offset );
		
		/**
		 * Given a current path position and a set of PT obstacles, check for collision
		 */
		bool inCollision( double path_position, double time, const Constraints& c ) const;
		
		/**
		 * Convenience wrapper for inCollision
		 */
		bool inCollision( const PVT_Point& p, const Constraints& c ) const;
		
		/**
		 * Convenience method for adding rectangles
		 */
		void addRectangles( const std::vector< std::vector<double> >& boxes, const Constraints& c );
		
		/**
		 * Convenience method for adding obstacles
		 */
		void addObstacles( const PVT_ObstacleSet& O, const Constraints& c );
		
		/**
		 * Convenience method for adding a single obstacle
		 */
		void addObstacle( const PVT_Obstacle& o, const Constraints& c ) {
			this->raw_obstacles.push_back( new PVT_Obstacle(o, c) );
		}
		
		/**
		 * Vector of obstacles with boundaries projected onto the horizons
		 */
		std::vector<PVT_Obstacle*> obstacles;
		
		/**
		 * Vector obstacles
		 */
		std::vector<PVT_Obstacle*> raw_obstacles;
		
	private:
		
		/**
		 * Initialization routine: double array
		 */
		void init( const double boxes[][4], size_t box_count, const Constraints& c );
		
		/**
		 * Initialization routine: vector of doubles
		 */
		void init( const std::vector< std::vector<double> >& boxes, const Constraints& c );
		
		/**
		 * Initialization routine: obstacle set
		 */
		void init( const PVT_ObstacleSet& O, const Constraints& c );
		
		/**
		 * Free from memory the obstacle set
		 */
		void freeObstacles();
		
	};
	
} // end PVTP namespace

#endif