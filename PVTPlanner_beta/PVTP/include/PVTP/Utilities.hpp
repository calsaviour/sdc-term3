#ifndef PVTP_UTILITIES_H
#define PVTP_UTILITIES_H

#include <string>
#include <vector>
#include <PVTP/TrajectorySegment.hpp>
#include <PVTP/Planner.hpp>
#include <PVTP/VehicleModel.hpp>
#include <PVTP/PathModel.hpp>
#include <PVTP/Controller.hpp>

//#define SHOW_COMMENTS

#ifdef SHOW_COMMENTS
#include <iostream>
#endif

namespace PVTP {
	
	/**
	 * This namespace contains utility functions specific to the library.
	 */
	namespace Utilities {
		
		/**
		 * Perform an intersection of reachable sets on a point-by-point basis
		 * across G1 and G2. Store the result in G_union. This method is used
		 * during backwards propagation to intersect the sets of forward
		 * reachable intervals with backward reachable intervals.
		 *
		 * Assume that G2 contains a subset of the points in G1, and
		 * that they are ordered in the same way
		 */
		bool IntersectionOfReachableSets( std::vector<PVT_G*>& G_union,
										 const std::vector<PVT_G*>& G1,
										 const std::vector<PVT_G*>& G2,
										 const Constraints& c );
		
		/**
		 * Perform a special union of reachable sets on a point-by-point basis
		 * across G1 and G2. Store the result in G1. This method is used
		 * during backwards propagation to accumulate backwards reachable
		 * intervals
		 *
		 * Assume that G2 contains a subset of the points in G1, and
		 * that they are ordered in the same way
		 */
		void SpecialUnionOfReachableSets( std::vector<PVT_G*>& G1,
										 const std::vector<PVT_G*>& G2,
										 const Constraints& c );
		
		/**
		 * Given a set of obstacles and a new origin, translate the set, 
		 * then reflect about both axes. This procedure is useful when 
		 * performing backwards propagation: transforming the obstacle field
		 * this way allows the existing Forward algorithm to be used.
		 */
		void TranslateAndMirrorObstacleSet( PVT_ObstacleSet& O_backward,
										   PVT_ObstacleSet& O_forward,
										   const PVT_Point& p,
										   const Constraints& c );
		
		/**
		 * Given a set of reachable velocities, mirror the set of points and
		 * translate to some other origin. This is used during backwards
		 * propagation to undo the transformation performed by
		 * TranslateAndMirrorObstacleSet
		 */
		void MirrorAndTranslateReachableSets( std::vector<PVT_G*>& G,
											 const PVT_Point& p );
		
		/**
		 * Clean both node and goal-reachable information from forward propagation.
		 * Convenience method that calls both CleanG and CleanGoal.
		 */
		void CleanResults( std::vector<PVT_G*>& G, std::vector<PVT_S*>& Goal );
		
		/**
		 * Clean the node results from forward propagation
		 */
		void CleanG( std::vector<PVT_G*>& G );
		
		/**
		 * Clean the goal-reachable information from forwad propagation
		 */
		void CleanGoal( std::vector<PVT_S*>& Goal );
		
		/**
		 * A function to free memory associated with a computed trajectory
		 */
		void CleanTrajectory( std::vector<TrajectorySegment*>& T );
		
		/**
		 * A function to duplicate a trajectory; returns a pointer to a new'd object
		 */
		std::vector<TrajectorySegment*> * CopyTrajectory( std::vector<TrajectorySegment*>& T );
		
		/**
		 * A function to remove essentially duplicate state transitions from a trajectory
		 */
		void SanitizeTrajectory( std::vector<TrajectorySegment*>& T, const Constraints& c );
		
		/**
		 * Output information from the results of forward propagation.
		 * Convenience method that calls both DescribeG and DescribeGoal.
		 *
		 * NOTE: This alters Goal by sorting it.
		 */
		void DescribeResults( std::vector<PVT_G*>& G,
							 std::vector<PVT_S*>& Goal,
							 bool verbose = false );
		
		/**
		 * Output node information from the results of forward propagation
		 *
		 * NOTE: This alters Goal by sorting it.
		 */
		void DescribeG( const std::vector<PVT_G*>& G, bool verbose = false );
		
		/**
		 * Output goal information from the results of forward propagation.
		 *
		 * NOTE: This alters Goal by sorting it.
		 */
		void DescribeGoal( std::vector<PVT_S*>& Goal, bool verbose = false );
		
		/**
		 * Print out a full trajectory
		 */
		void PrintTrajectory( const std::vector<TrajectorySegment*>& T );
		
		/**
		 * Print a set of collision obstacles.
		 */
		void PrintCollisionSet( const std::set<PVT_Obstacle*>& inCollision );
		
		/**
		 * Print the homotopic signature of a trajectory
		 */
		void PrintSignature( const std::vector<char>& hClass );
		
		/**
		 * Construct a velocity interval based on the given representative
		 * trajectories.
		 */
		void ExtractInitialVelocityInterval( Interval& I,
											const std::vector<TrajectorySegment*>& UB,
											const std::vector<TrajectorySegment*>& LB );
		
		/**
		 * Construct a velocity interval based on the given representative
		 * trajectories.
		 */
		void ExtractFinalVelocityInterval( Interval& I,
										  const std::vector<TrajectorySegment*>& UB,
										  const std::vector<TrajectorySegment*>& LB );
		
		/**
		 * Truncate a given value to be within specified bounds
		 */
		double truncateValue( double value, double min, double max );
		
		/**
		 * Given a trajectory and a time into that trajectory, compute the 
		 * velocity at that time.
		 *
		 * For invalid imputs (like an empty trajectory or negative time) this
		 * method return NaN.
		 */
		double trajectoryVelocityAtTime( const std::vector<TrajectorySegment*>& T, double time, const Constraints& c, bool truncate = true );
		
		/**
		 * Given a trajectory and a time into that trajectory, compute the 
		 * displacement at that time.
		 *
		 * For invalid imputs (like an empty trajectory or negative time) this
		 * method return NaN.
		 */
		double trajectoryDisplacementAtTime( const std::vector<TrajectorySegment*>& T, double time, const Constraints& c, bool truncate = true );

		/**
		 * Translate a given trajectory
		 */
		void translateTrajectory( std::vector<TrajectorySegment*>& T, double p_offset, double t_offset );
		
		/**
		 * Compute an optimal, executable control for a given time step. An executable control
		 * is one with constant acceleration over the time step that minimizes
		 * deviation from the optimal final displacement and velocity. The coefficients C1
		 * and C2 are weights for velocity and displacement, respectively.
		 *
		 * NOTE: This does NOT check whether the control meets constraints
		 * Because the optimization function is linear, truncating the returned
		 * control to the constraint range is the correct way to optimize; therefore,
		 * it is left to the caller (who has access to the constraints object) to do that.
		 */
		double getOptimalExecutableControl( double initial_velocity, double optimal_displacement, double optimal_velocity, double time_step, double C1 = 1., double C2 = 1. );
		
		/**
		 * Overloaded version of above that takes in a trajetory and desired time
		 */
		double getOptimalExecutableControl( const std::vector<TrajectorySegment*>& T,
										   double cur_velocity,
										   double cur_time,
										   double time_step,
										   const Constraints& c );
		
		/**
		 * Check the executability of a trajectory: a trajectory is executable
		 * if its segment durations are within range of being multiples of the
		 * given time step.
		 */
		bool trajectoryIsExecutable( const std::vector<TrajectorySegment*>& T, double time_step, double range );
		
		/**
		 * Verify that a trajectory satisfies dynamic constraints
		 */
		bool ValidTrajectory( std::vector<TrajectorySegment*>& T, const Constraints& c );
		
		/**
		 * Given a trajectory, construct a vehicle controller that follows it.
		 */
		bool ControllerFromTrajectory( Scenario::Controller& controller,
									  std::vector<TrajectorySegment*>& T,
									  double time_step,
									  const Constraints& c,
									  double cur_velocity = 0. );

		/**
		 * Generate matlab code that draws a set of obstacles.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		std::string GetMatlabObstacles( const PVT_ObstacleSet& O, double ttc = 0. );
		
		/**
		 * Generate matlab code that draws a trajectory.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		std::string GetMatlabTrajectory( const std::vector<TrajectorySegment*>& T, std::string color = "", std::string suffix = "" );
		
		/**
		 * Generate matlab code that draws reachable sets
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		std::string GetMatlabReachableSets( const std::vector<PVT_G*> G );
		
		/**
		 * Generate matlab code that draws paths
		 */
		std::string GetMatlabPath( const Scenario::PathModel& path, bool driver_path = false, bool bike_path = false );
		
		/**
		 * Generate matlab code that draws a vehicle model
		 */
		std::string GetMatlabVehicleModel( const Scenario::VehicleModel& vm, bool driver_vehicle = false );
		
		/**
		 * Generate a matlab figure with standard properties. Convenience method.
		 */
		std::string GetMatlabStandardFigure( bool unit_aspect_ratio = false );
		
		/**
		 * Generate a matlab figure with standard properties. Convenience method.
		 */
		std::string GetMatlabWorldSceneString( const Scenario::VehicleModel& dm, Scenario::VehicleModel& wo );
		
		/**
		 * Generate a world scene given a driver vehicle and a set of obstacle vehicles
		 */
		std::string GetMatlabWorldSceneString( const Scenario::VehicleModel& dm, std::vector<Scenario::VehicleModel>& WO );
		
		/**
		 * Generate a world scene given a driver vehicle and a set of obstacle vehicles
		 */
		void GetMatlabWorldScene( std::string output_path, const Scenario::VehicleModel& dm, std::vector<Scenario::VehicleModel>& WO );
		
		/**
		 * Generate a world scene given a driver vehicle and a set of obstacle vehicles.
		 *
		 * Convenience wrapper.
		 */
		void GetMatlabWorldScene( std::string output_path, const Scenario::VehicleModel& dm, Scenario::VehicleModel& wo );
		
		/**
		 * Generate a matlab script for drawing reachable sets with obstacles.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		void GenerateMatlabReachableSets( const std::string output_path, const PVT_ObstacleSet& O, const std::vector<PVT_G*>& G );
		
		/**
		 * Generate a string that is used to draw the scene in matlab
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		std::string GenerateMatlabSceneString( const PVT_ObstacleSet& O,
									   std::vector< std::vector<TrajectorySegment*>* >& T );
		
		/**
		 * Generate matlab script that draws a scene. Save script to `path'.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		void GenerateMatlabScene( std::string output_path,
								 const PVT_ObstacleSet& O,
								 std::vector< std::vector<TrajectorySegment*>* >& T );
		
		/**
		 * Generate matlab script that draws a scene. Save script to `path'.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		void GenerateMatlabScene( std::string output_path,
								 const PVT_ObstacleSet& O,
								 const std::vector<PVT_G*>& G,
								 std::vector< std::vector<TrajectorySegment*>* >& T );

		/**
		 * Generate matlab script that draws a scene. Save script to `path'.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		void GenerateMatlabScene( std::string output_path,
								 const PVT_ObstacleSet& O,
								 std::vector<TrajectorySegment*>& T );
		
		/**
		 * Given a set of world obstacles, a driver, and a trajectory, generate
		 * a sequence of stills that represents the execution of the scenario
		 * in the world space.
		 *
		 * NOTE: This method is intended to be used in conjunction with the
		 * Matlab scripts included in this library.
		 */
		void PlayScenario( std::string output_path,
						  Scenario::VehicleModel& dm,
						  std::vector<Scenario::VehicleModel>& WO,
						  std::vector<TrajectorySegment*>& T,
						  double simulation_time_step,
						  double recording_time_step,
						  const Constraints& c );
		
		/**
		 * An adaptation of PHP's explode function
		 * http://www.infernodevelopment.com/perfect-c-string-explode-split
		 */
		void StringExplode( std::string str, std::string separator, std::vector<std::string>& results );
	
	} // end Utilities namespace

} // end PVTP namespace

#endif