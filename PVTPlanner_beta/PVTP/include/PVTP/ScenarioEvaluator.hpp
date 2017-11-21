#ifndef PVTP_SCENARIO_EVALUATOR_H
#define PVTP_SCENARIO_EVALUATOR_H

#include <set>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/TrajectorySegment.hpp>
#include <PVTP/PVT_G.hpp>

using namespace PVTP;

namespace Scenario {

	/**
	 * The Evaluator namespace. This namespace contains methods for computing
	 * sets of "important" obstacles in a given scenario that contains
	 * uncertainty. The idea is to look for obstacles correlated with the
	 * navigability of a the scenario. This is done by first padding path-time
	 * obstacles with minimum fore- and aft-ttc values. If an optimal trajectory
	 * is thus found, it is returned, and the scenario is considered "safe."
	 *
	 * If an optimal trajectory is not found, the planner is run again without
	 * the ttc padding. If an optimal trajectory is found, it is returned along
	 * with the set of obstacles whose ttc bounds are violated. These obstacles
	 * are considered "important," and the scenario "risky."
	 *
	 * If an optimal trajectory is still not found, the scenario is considered
	 * "unsafe."
	 */
	namespace Evaluator {
		
		/**
		 * Indicator for state for which the predicted trajectory is collision-free
		 * and there exists an S_{safe} trajectory from the end of the predicted
		 * trajectory
		 */
		static const int P_SAFE = 1;
		
		/**
		 * Indicator for state for which the predicted trajectory is collision-free
		 * but there exists no feasible trajecotry from the end of the predicted
		 * trajectory
		 */
		static const int P_ICIS = 2;
		
		/**
		 * Indicator for when there exists no feasible trajectory from the current
		 * state
		 */
		static const int P_CIS = 3;
		
		/**
		 * Indicator for when the predicted trajectory is in collision, but there
		 * exists a feasible trajetory for the current state
		 */
		static const int P_RISKY_STRONG = 4;
		
		/**
		 * Indicator for when the predicted trajectory is collision-free, but the
		 * only feasible trajectory from the end of the predicted trajectory
		 * violates S_{safe} obstacles
		 */
		static const int P_RISKY_WEAK = 5;
		
		/**
		 * Indicator for use when current state is in collision
		 */
		static const int P_IN_OBS = 6;
		
		/**
		 * Label for a state in which a control are available
		 */
		static const int CONTROL_AVAILABLE_LABEL = 7;
		
		/**
		 * Given a set of reachable velocities and a constant control, compute
		 * the last feasible reaction (in time) that deviates from the control.
		 * The reaction is a partial trajectory that connects the predicted
		 * trajectory to a feasible velocity interval at an obstacle vertex and
		 * that admits a feasible solution.
		 *
		 * NOTE: G *must* be the sets of reachable velocities obtained via
		 * back-propagation! In other words, G must contain the sets of velocities
		 * that are not only reachable from the start position, but that also
		 * admit goal connections.
		 */
		bool computeLastFeasibleReaction( std::vector<TrajectorySegment*>& last_feasible_reaction,
										 double initial_velocity,
										 double control,
										 std::vector<PVT_G*>& G,
										 PVT_ObstacleSet& O,
										 const Interval V_f,
										 const Constraints& c );
		
		/**
		 * Given a discretization and predicted constant control trajectory,
		 * label the trajectory for which types of controls (acceleration or braking)
		 * are available at each quantum.
		 */
		bool getLabelling( std::vector<int>& braking_labels,
						  std::vector<int>& acc_labels,
						  std::vector<int>& coasting_labels,
						  double velocity,
						  double control,
						  double t_step,
						  double t_horizon,
						  double reaction_horizon,
						  PVT_ObstacleSet& O,
						  const Constraints& c );
		
		/**
		 * Determine whether a state is feasible after a predicted control
		 */
		bool controlAvailable( int& control_label,
							  double initial_velocity,
							  double control,
							  double control_duration,
							  PVT_ObstacleSet& O,
							  const Constraints& c );
		
		/**
		 * Evaluate the perceptual safety of the space for a given velocity
		 */
		bool evaluatePVT_S_SafeSafety( std::vector< std::pair<PVT_Point*, double>* >& warning_map,
									  double velocity,
									  double control,
									  double ttc_fore,
									  double ttc_left,
									  double t_horizon,
									  const std::vector<PVT_Point*>& space_grid,
									  PVT_ObstacleSet& O,
									  const Constraints& c );
		
		/**
		 * Construct the set of S_{safe} obstacles given the set of true
		 * obstacles, a velocity, and acceptable time-to-collision margins.
		 */
		void constructS_SafeObstacles( PVT_ObstacleSet& S_safe,
									  const PVT_ObstacleSet& O,
									  double v,
									  double ttc_fore,
									  double ttc_left,
									  const Constraints& c );
		
		/**
		 * Given a path-time space discretization and control discretization, compute the
		 * warning map for a given velocity.
		 */
		bool evaluatePT_SpaceSafety( std::vector< std::pair<PVT_Point*, double>* >& warning_map,
								 double velocity,
								 const std::vector<PVT_Point*>& space_grid,
								 const std::vector<double>& control_grid,
								 double time_step,
								 double time_horizon,
								 double control,
								 PVT_ObstacleSet& O,
								 const Constraints& c );
		
		/**
		 * Given a PVT state and a set of constraints, evaluate the safety of
		 * the state, where safety is defined as the percentage of controls
		 * available that result in a goal-reachable state.
		 *
		 * The method returns the percentage of safe controls, or NaN if it
		 * encounters an error.
		 */
		double evaluateStateSafety( const PVT_State s,
								   const std::vector<double>& control_grid,
								   double time_step,
								   double time_horizon,
								   double control,
								   PVT_ObstacleSet& O,
								   const Constraints& c );
		
		/**
		 * Return a discrete grid sampling of the velocity space at the requested
		 * step size.
		 */
		void getVelocityGrid( std::vector<double>& grid, double step, const Constraints& c );
		
		/**
		 * Return a discrete grid sampling of path-time space at the requested
		 * step sizes.
		 */
		void getPT_SpaceGrid( std::vector<PVT_Point*>& grid, double p_step, double t_step, const Constraints& c );
		
		/**
		 * Return a discrete grid sampling of the control space [-1, 1] at the
		 * requested step size.
		 */
		void getControlGrid( std::vector<double>& grid, double step, const Constraints& c );
		
		/**
		 * Perform a naive analysis of the safety of a given scenario. This is
		 * done by assuming a fixed control over some time horizon. This method
		 * fills the set of collision PT obstacles, which is empty if no
		 * collision is detected.
		 */
		void naiveSafetyTest( std::set<PVT_Obstacle*>& inCollision,
							 double cur_vel,
							 double u,
							 double time_horizon,
							 PVT_ObstacleSet& O,
							 const Constraints& c );
		
		/**
		 * Run the planner on a given set of obstacles and constraints, return
		 * the optimal trajectory (or an empty trajectory, if there is none).
		 */
		bool getOptimalTrajectory( std::vector<TrajectorySegment*>& T,
								  const Interval& V_i,
								  const Interval& V_f,
								  PVT_ObstacleSet& O,
								  const Constraints& c );
		
		/**
		 * Detect an impending imminent collision with a set of obstacles, all
		 * of which will be involved in the collision. This is a special case
		 * of a collision imminent state in which the set of obstacles in the
		 * collision path is known exactly. This can happen if the driver is
		 * travelling at non-zero velocity towards one or more obstacles, and
		 * cannot brake in time to avoid hitting 'all' of them, that is, all
		 * trajectory channels are obstructed by exactly the same set of
		 * obstacles. Such a scenario is called a "bottleneck" imminent
		 * collision state.
		 *
		 * inCollision is set to contain the set of bottleneck CIS obstacles.
		 */
		void detectBottleneckCIS( std::set<PVT_Obstacle*>& inCollision,
								 const Interval& V_i,
								 const Interval& V_f,
								 PVT_ObstacleSet& O,
								 const Constraints& c );
		
		/**
		 * Given an upper bounding trajectory and a lower bounding trajectory
		 * between two path-time points, this method computes the set of 
		 * obstacles that both intersect.
		 *
		 * inCollision is set to contain the set of obstacles that both UB and
		 * LB intersect.
		 */
		void getIntersectionOfCollisionObstacles( std::set<PVT_Obstacle*>& inCollision,
												 std::vector<TrajectorySegment*>& UB,
												 std::vector<TrajectorySegment*>& LB,
												 PVT_ObstacleSet& O,
												 const Constraints& c );
		
		/**
		 * This method computes the intersection of the obstacle sets contained
		 * in inCollisionA and inCollisionB.
		 *
		 * inCollision is set to contain the intersection of inCollisionA and
		 * inCollisionB.
		 */
		void intersectCollisionObstacleSets( std::set<PVT_Obstacle*>& inCollision,
											const std::set<PVT_Obstacle*>& inCollisionA,
											const std::set<PVT_Obstacle*>& inCollisionB,
											const Constraints& c );
		
	} // end Evaluator namespace
	
} // end Scenario namespace

#endif
