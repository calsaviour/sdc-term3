#include <stdio.h>
#include <errno.h>
#include <PVTP/Maths.hpp>
#include <PVTP/Constants.hpp>
#include <PVTP/Utilities.hpp>
#include <PVTP/Planner.hpp>

//#define DEBUG
#define SHORTCUT

#ifdef DEBUG
#include <fstream>
#endif

namespace PVTP {
	
	namespace Planner {
		
#ifdef DEBUG
		std::vector< std::vector<TrajectorySegment*>* > matlab_bounding_trajectories;
#endif

		bool Backward( std::vector<PVT_G*>& G_intersect,
					  std::vector<PVT_G*>& G_forward,
					  std::vector<PVT_S*>& Goal_forward,
					  const Interval& V_f,
					  PVT_ObstacleSet& O_forward,
					  const Constraints& c_forward ) {
			
			// constraint set for backwards problem: accelerations are inverted
			Constraints c_backward( c_forward, true );
			
			// ensure that intervals are sorted
			struct PVT_G_Comparator comp;
			comp.epsilon = c_forward.getEpsilon();
			std::sort( G_forward.begin(), G_forward.end(), comp );
			
			struct PVT_S_Comparator goal_comp;
			std::sort( Goal_forward.begin(), Goal_forward.end(), goal_comp );
			
			// this set will contain the special union of the backwards 
			// reachable intervals
			std::vector<PVT_G*> G_union;

			// initialize union and intersection sets with empty intervals
			for ( size_t i=0; i<G_forward.size(); ++i ) {
				G_union.push_back( new PVT_G( *G_forward[i]->p) );
				G_intersect.push_back( new PVT_G( *G_forward[i]->p) );
			}
			
			// for each point with goal-reachable velocities, mirror the
			// problem and run the forward algorithm
			for ( size_t i=0; i<Goal_forward.size(); ++i ) {

				// point about which problem is to be mirrored
				PVT_Point p = *Goal_forward[i]->p;
				
				// translate and mirror the problem for this point
				PVT_ObstacleSet O_backward;
				Utilities::TranslateAndMirrorObstacleSet( O_backward,
														 O_forward,
														 p,
														 c_backward );
				
				// run forward propagation on this obstacle set
				// don't do goal propagation; it isn't necessary for this
				std::vector<PVT_G*> G;
				std::vector<PVT_S*> Goal_dummy;
				Interval V_i_backward;
				Goal_forward[i]->getReachableInterval( V_i_backward );
				if ( !Forward(G, Goal_dummy, V_i_backward, V_f, O_backward, c_backward, false, false) ) {
					std::cerr << "ERROR IN Planner::Backward: Backward propagation failed." << std::endl;
					return false;
				}
				
				// the reachable intervals exist in G, but the points have to
				// be transformed back to the original space
				Utilities::MirrorAndTranslateReachableSets( G, p );
				
				// sort the intervals so that the order matches G_forward
				struct PVT_G_Comparator comp;
				comp.epsilon = c_forward.getEpsilon();
				std::sort( G.begin(), G.end(), comp );
				
				/**
				 * G now contains the backwards reachable intervals for the
				 * i-th point with goal-reachable intervals, ordered as in
				 * G_forward
				 */
				
				// union G with G_union to accumulate a union of backwards
				// reachable intervals
				Utilities::SpecialUnionOfReachableSets(G_union, G, c_forward);
				
				// clean memory for this iteration
				Utilities::CleanResults( G, Goal_dummy );
			}

			/**
			 * G_union now contains the special union of the backwards reachable
			 * intervals. The intersection of this set with that of the forward
			 * set will yield exacty the set of intervals that yield
			 * goal-reachable trajectories
			 */
			if ( !Utilities::IntersectionOfReachableSets(G_intersect, G_union, G_forward, c_forward) ) {
				std::cerr << "ERROR IN Planner::Backward: Intersection of forwards and backwards reachable sets failed." << std::endl;
				return false;
			}
			
			// clean union memory
			Utilities::CleanG( G_union );
			
			// success
			return true;
		}
		
		bool Forward( std::vector<PVT_G*>& G,
					 std::vector<PVT_S*>& Goal,
					 const Interval& _V_i,
					 const Interval& _V_f,
					 PVT_ObstacleSet& O,
					 const Constraints& c,
					 bool do_goal_propagation,
					 bool add_origin ) {
			
			//std::cout << "Please note: Obstacles should not overlap the goal region." << std::endl;
			
			double min_goal_time = std::numeric_limits<double>::max();
			
			// get our own copy of the intervals
			Interval V_i( _V_i );
			Interval V_f( _V_f );
			
			// make sure input is within bounds
			Interval v_feasible( c.getVMin(), c.getVMax() );
			V_i.intersect( v_feasible, c );
			V_f.intersect( v_feasible, c );
			if ( V_i.isEmpty() || V_f.isEmpty() ) {
				std::cerr << "ERROR IN Planner::Forward: Input velocities invalid, V_i: " << V_i << ", V_f: " << V_f << ", v_feasible: " << v_feasible << std::endl;
				return false;
			}
			
			// get points
			std::vector<PVT_ObstaclePoint*> P_t;
			O.getPotentiallyReachableVertices( P_t, c );
			
#ifdef DEBUG
			std::cout << "Analyzing " << P_t.size() << " PT points... ";
#endif
			
			// add origin, if necessary
			PVT_ObstaclePoint p1( 0., 0., c );
			if ( add_origin ) {
				P_t.push_back( &p1 );
			}
			if ( P_t.empty() ) {
				std::cerr << "ERROR IN Planner::Forward: No points for analysis." << std::endl;
				return false;
			}
			
			// sort points by t-coord
			struct PVT_ObstaclePointComparator comp;
			comp.epsilon = c.getEpsilon();
			std::sort( P_t.begin(), P_t.end(), comp );
			
			// allocate storage
			G.resize( P_t.size(), NULL );
			
			// initialize with first point
			G[0] = new PVT_G( V_i, *P_t.front() );
			
			// try to connect points
			for ( size_t j=0; j<P_t.size(); ++j ) {
				
				// destination point
				PVT_ObstaclePoint * p2 = P_t[j];
				
				// entry for this point in G
				if ( G[j] == NULL ) {
					G[j] = new PVT_G( *p2 );
				}

				// storage for reachable velocity intervals
				std::vector<PVT_S*> S;
				std::vector<PVT_S*> S_goal;
				for ( size_t l=0; l<G[j]->V.size(); ++l ) {
					S.push_back( new PVT_S(G[j]->V[l]) );
				}

				// only need to try connecting points before this in time
				for ( size_t i=0; i<j; ++i ) {
					
					// source point
					PVT_ObstaclePoint * p1 = P_t[i];
					
					// initial velocity intervals at point i
					std::vector<Interval> V_i = G[i]->V;

					// for each disjoing interval in V_i
					for ( size_t k=0; k<V_i.size(); ++k ) {
						
						// build set S for this connection
						if ( !Propagate(S, S_goal, *p1, *p2, V_i[k], V_f, O, P_t, c) ) {
							std::cerr << "ERROR IN Planner::Forward: propagation failed" << std::endl;
							return false;
						}
					}
				}

				// store information for this point
				Merge( G[j]->V, S, c );

				// Clean S
				for ( size_t k=0; k<S.size(); ++k ) {
					delete( S[k] );
				}
				
				if ( do_goal_propagation ) {
					for ( size_t k=0; k<G[j]->V.size(); ++k ) {
						if ( !PropagateGoal(S_goal, *G[j]->p, G[j]->V[k], V_f, O, P_t, c) ) {
							std::cerr << "ERROR IN Planner::Forward: PropagateGoal failed" << std::endl;
							return false;
						}
					}

#ifdef SHORTCUT
					// shortcut
					for ( size_t k=0; k<S_goal.size(); ++k ) {
						if ( (S_goal[k]->UB != NULL) && !(S_goal[k]->UB->empty()) ) {
							double tmp = S_goal[k]->UB->back()->getFinalState().getTimeCoord();
							if ( tmp < min_goal_time ) {
								min_goal_time = tmp;
							}
						}
					}
#endif
					
					// merge goal velocities, add to goal intervals
					MergeGoal( Goal, S_goal, c );

					// Clean S_goal
					for ( size_t k=0; k<S_goal.size(); ++k ) {
						delete( S_goal[k] );
					}
					
#ifdef SHORTCUT
					// if we can get to the goal before ever reaching this point, shortcut out
					if ( min_goal_time <= p2->getTimeCoord() ) {
						G.resize( j + 1 );
						break;
					}
#endif
				}

			}

#ifdef DEBUG
			/**
			// sort bounding trajectories by time
			Trajectory_Comparator traj_comp;
			traj_comp.epsilon = c.getEpsilon();
			std::sort( matlab_bounding_trajectories.begin(), matlab_bounding_trajectories.end(), traj_comp );
			std::vector< std::vector<TrajectorySegment*>* > _matlab_bounding_trajectories;
			double thresh = 200.;
			for ( size_t i=0; i<matlab_bounding_trajectories.size(); ++i ) {
				
				double val = matlab_bounding_trajectories[i]->front()->getInitialState().getPathCoord();
				if ( (val < thresh) && ((i%3) == 0) ) {
					_matlab_bounding_trajectories.push_back( matlab_bounding_trajectories[i] );
				} else if ( val >= thresh ) {
					_matlab_bounding_trajectories.push_back( matlab_bounding_trajectories[i] );
				}
			}
			size_t counter = 0;
			std::stringstream ss_out;
			ss_out << "setup;" << std::endl;
			std::vector< std::vector<TrajectorySegment*>* > tmp;
			for ( size_t i=0; i<_matlab_bounding_trajectories.size(); ++i ) {
				tmp.push_back( _matlab_bounding_trajectories[i] );
				
				if ( (i % 20) == 0) {
					ss_out << Utilities::GetMatlabStandardFigure() << std::endl;
					ss_out << Utilities::GenerateMatlabSceneString( O, tmp );
					ss_out << "set(gca, 'FontSize', 14);" << std::endl;
					ss_out << "xlabel('Path Position (m)', 'FontSize', 16);" << std::endl;
					ss_out << "ylabel('Time (s)', 'FontSize', 16);" << std::endl;
					ss_out << "display(['Printing graph ', num2str(" << (counter+1) << ")]);" << std::endl;
					ss_out << "screen2png(['data/images/world_scene_', num2str(" << (++counter) << ")]);" << std::endl;
					ss_out << "screen2eps(['data/images/world_scene_', num2str(" << counter << ")]);" << std::endl;
					ss_out << "saveas(gcf, ['data/images/world_scene_', num2str(" << counter << ")], 'fig');" << std::endl;
					ss_out << "hold off;" << std::endl;
					ss_out << "close;" << std::endl;
				}
			}
			
			// save file
			std::ofstream ofile;
			ofile.open( "PVTP/Matlab/build_graph.m" );
			ofile << ss_out.str();
			ofile.close();
			
			//Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O, G, _matlab_bounding_trajectories );
			/**/
#endif
#ifdef DEBUG
			Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O, G, matlab_bounding_trajectories );
			std::cout << "found " << matlab_bounding_trajectories.size() << " representative trajectories." << std::endl;
			std::cout << std::endl;
			for ( size_t i=0; i<matlab_bounding_trajectories.size(); ++i ) {
				Utilities::CleanTrajectory( *matlab_bounding_trajectories[i] );
			}
#endif
			
			return true;
		}
		
		bool Propagate( std::vector<PVT_S*>& S,
					   std::vector<PVT_S*>& S_goal,
					   const PVT_Point& p1,
					   const PVT_Point& p2,
					   const Interval& V_int,
					   const Interval& V_f,
					   const PVT_ObstacleSet& O,
					   const std::vector<PVT_ObstaclePoint*>& P_t,
					   const Constraints& c ) {
			
			// get nominal bounding trajectories
			std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > BoundingTrajectories;
			BoundingTrajectories.first = NULL;
			BoundingTrajectories.second = NULL;
			if ( !NextReachableSet(BoundingTrajectories, p1, p2, V_int, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: NextReachableSet failed" << std::endl;
				return false;
			}
			if ( (BoundingTrajectories.first == NULL) || (BoundingTrajectories.second == NULL) ) {
				
				// sanity check; this should never happen
				if ( !(BoundingTrajectories.first==NULL & BoundingTrajectories.second==NULL) ) {
					std::cerr << "ERROR IN Planner::Propagate: Unbalanced bounding trajectories." << std::endl;
					return false;
				}
				
#ifdef SHOW_COMMENTS
				std::cout << "Planner::Propagate: No bounding trajectories found" << std::endl;
#endif
				return true;
			}
			
#ifdef DEBUG
			if ( !Utilities::ValidTrajectory(*BoundingTrajectories.first, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: *BoundingTrajectories.first trajectory invalid:" << std::endl;
				Utilities::PrintTrajectory( *BoundingTrajectories.first );
				Utilities::CleanTrajectory( *BoundingTrajectories.first );
				delete( BoundingTrajectories.first );
				std::cerr << "---" << std::endl;
				std::cerr << p1  << " -> " << p2 << " : " << V_int << std::endl;
				return false;
			}
			if ( !Utilities::ValidTrajectory(*BoundingTrajectories.second, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: *BoundingTrajectories.second trajectory invalid:" << std::endl;
				Utilities::PrintTrajectory( *BoundingTrajectories.second );
				Utilities::CleanTrajectory( *BoundingTrajectories.second );
				delete( BoundingTrajectories.second );
				std::cerr << "---" << std::endl;
				std::cerr << p1 << " -> " << p2  << " : " << V_int << std::endl;
				return false;
			}
#endif

			// get collision bounded trajectories
			std::vector<TrajectorySegment*> T_UB;
			std::vector<TrajectorySegment*> T_LB;
			if ( !GetCollisionBoundary( T_UB, *BoundingTrajectories.first, &S_goal, true, p1, p2, V_int, V_f, O, &P_t, c) ) {
			//if ( !GetCollisionBoundary( T_UB, *BoundingTrajectories.first, NULL, true, p1, p2, V_int, V_f, O, &P_t, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: Failed finding UB." << std::endl;
				return false;
			}
			if ( !GetCollisionBoundary( T_LB, *BoundingTrajectories.second, &S_goal, false, p1, p2, V_int, V_f, O, &P_t, c) ) {
			//if ( !GetCollisionBoundary( T_LB, *BoundingTrajectories.second, NULL, false, p1, p2, V_int, V_f, O, &P_t, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: Failed finding LB." << std::endl;
				return false;
			}
			Utilities::CleanTrajectory( *BoundingTrajectories.first );
			Utilities::CleanTrajectory( *BoundingTrajectories.second );
			delete( BoundingTrajectories.first );
			delete( BoundingTrajectories.second );
			
			// unreachable via representative trajectories
			if ( T_UB.empty() && T_LB.empty() ) {
				return true;
			}
			
#ifdef DEBUG
			if ( !Utilities::ValidTrajectory(T_UB, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: T_UB trajectory invalid:" << std::endl;
				Utilities::PrintTrajectory( T_UB );
				Utilities::CleanTrajectory( T_UB );
				std::cerr << p1  << " -> " << p2 << " : " << V_int << std::endl;
				return false;
			}
			if ( !Utilities::ValidTrajectory(T_LB, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: T_LB trajectory invalid:" << std::endl;
				Utilities::PrintTrajectory( T_LB );
				Utilities::CleanTrajectory( T_LB );
				std::cerr << p1 << " -> " << p2  << " : " << V_int << std::endl;
				return false;
			}
#endif

			// get trajectory signatures
			std::vector<char> BL;
			if ( !Channel(BL, T_LB, O, P_t, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: Channel failed (1)" << std::endl;
				return false;
			}
			std::vector<char> BU;
			if ( !Channel(BU, T_UB, O, P_t, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: Channel failed (2)" << std::endl;
				return false;
			}
						
			//
			// Add velocity intervals
			//
			
			// if signatures are equal
			bool sigs_equal = !(BU.empty() || BL.empty()) && (BL.size() == BU.size());
			if ( sigs_equal ) {
				for ( size_t i=0; i<BL.size(); ++i ) {
					if ( BU[i] != BL[i] ) {
						sigs_equal = false;
						break;
					}
				}
			}
			
			// both bounds reachable
			Interval i;
			if ( sigs_equal ) {
				Utilities::ExtractFinalVelocityInterval( i, T_UB, T_LB );
				S.push_back( new PVT_S( i, BL ) );
#ifdef DEBUG
				matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(T_LB) );
				matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(T_UB) );
#endif
			
			// only singletons reachable
			} else {
				
				if ( !BL.empty() ) {
					Utilities::ExtractFinalVelocityInterval( i, T_LB, T_LB );
					S.push_back( new PVT_S( i, BL ) );
#ifdef DEBUG
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(T_LB) );
#endif
				}
				
				if ( !BU.empty() ) {
					Utilities::ExtractFinalVelocityInterval( i, T_UB, T_UB );
					S.push_back( new PVT_S( i, BU ) );
#ifdef DEBUG
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(T_UB) );
#endif
				}
			}
			
			// free memory used by trajectories
			Utilities::CleanTrajectory( T_UB );
			Utilities::CleanTrajectory( T_LB );
			
#ifdef SHOW_COMMENTS
			std::cout << "Propagated: " << V_int << ": " << p1 << " -> " << p2 << ": " << i << std::endl;
#endif
			return true;
		}
		
		bool PropagateGoal( std::vector<PVT_S*>& S_goal,
						   const PVT_ObstaclePoint& p1,
						   const Interval& V_i,
						   const Interval& V_f,
						   const PVT_ObstacleSet& O,
						   const std::vector<PVT_ObstaclePoint*>& P_t,
						   const Constraints& c ) {

			// check reachability
			std::vector<TrajectorySegment*> UB;
			std::vector<TrajectorySegment*> LB;
			if ( !GoalConnect(UB, LB, p1, V_i, V_f, O, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: GoalConnect failed" << std::endl;
				return false;
			}
			
			// Collision check UB trajectory
			std::set<PVT_Obstacle*> UB_inCollision;
			if ( !Collisions::checkTrajectory(UB_inCollision, UB, O, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: collision check failed (1)" << std::endl;
				return false;
			}
			std::set<PVT_Obstacle*> LB_inCollision;
			if ( !Collisions::checkTrajectory(LB_inCollision, LB, O, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: collision check failed (2)" << std::endl;
				return false;
			}
			
			if ( UB.empty() || LB.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::PropagateGoal: Goal unreachable from " << p1 << std::endl;
#endif
				return true;
			}
			
			if ( !UB_inCollision.empty() && !LB_inCollision.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::PropagateGoal: Empty channel classes, collision propagating " << p1 << std::endl;
#endif
				Utilities::CleanTrajectory( UB );
				Utilities::CleanTrajectory( LB );
				
				return true;
			}

			// get trajectory signatures
			std::vector<char> BL;
			if ( LB_inCollision.empty() ) {
				if ( !Channel(BL, LB, O, P_t, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (1)" << std::endl;
					std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
					return false;
				}
			}
			std::vector<char> BU;
			if ( UB_inCollision.empty() ) {
				if ( !Channel(BU, UB, O, P_t, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (2)" << std::endl;
					std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
					return false;
				}
			}

#ifdef DEBUG
			if ( !Utilities::ValidTrajectory(UB, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: UB trajectory invalid:" << std::endl;
				Utilities::PrintTrajectory( UB );
				Utilities::CleanTrajectory( UB );
				std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
				return false;
			}
			if ( !Utilities::ValidTrajectory(LB, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: LB trajectory invalid:" << std::endl;
				Utilities::PrintTrajectory( LB );
				Utilities::CleanTrajectory( LB );
				std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
				return false;
			}
#endif
			
			//
			// Add velocity intervals
			//
			
			// if signatures are equal
			bool sigs_equal = !(BU.empty() || BL.empty()) && (BL.size() == BU.size());
			if ( sigs_equal ) {
				for ( size_t i=0; i<BL.size(); ++i ) {
					if ( BU[i] != BL[i] ) {
						sigs_equal = false;
						break;
					}
				}
			}

			// both bounds reachable
			if ( sigs_equal ) {
				S_goal.push_back( new PVT_S( UB, LB, BL, p1 ) );
#ifdef DEBUG
				matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(LB) );
				matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(UB) );
#endif
				
			// only singletons reachable
			} else {
				
				if ( !BL.empty() ) {
					S_goal.push_back( new PVT_S( LB, LB, BL, p1 ) );
#ifdef DEBUG
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(LB) );
#endif
				} else {
					Utilities::CleanTrajectory( LB );
				}
				
				if ( !BU.empty() ) {
					S_goal.push_back( new PVT_S( UB, UB, BU, p1 ) );
#ifdef DEBUG
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(UB) );
#endif
				} else {
					Utilities::CleanTrajectory( UB );
				}
			}
#ifdef SHOW_COMMENTS
			std::cout << "Propagated Goal: " << V_i << ": " << p1;
			for ( size_t i=0; i<BL.size(); ++i ) {
				std::cout << ((BL[i]==0)?"0":"1") << " ";
			}
			std::cout << std::endl;
			for ( size_t i=0; i<BU.size(); ++i ) {
				std::cout << ((BU[i]==0)?"0":"1") << " ";
			}
			std::cout << std::endl;
#endif			
			return true;
		}
		
		
		
		bool Channel( std::vector<char>& hClass,
					 const std::vector<TrajectorySegment*>& T,
					 const PVT_ObstacleSet& O,
					 const std::vector<PVT_ObstaclePoint*>& P,
					 const Constraints& c ) {
			if ( T.empty() ) {
				hClass.clear();
				return true;
			}
			
			double epsilon = c.getEpsilon();
			
			// valid time corridor for this trajectory
			Interval trajectory_time_interval( T.front()->getInitialState().getTimeCoord(), T.back()->getFinalState().getTimeCoord() );
			
			for ( size_t i=0; i<P.size(); ++i ) {
				PVT_ObstaclePoint * p = P[i];
				
				double p_x = p->getPathCoord();
				double p_t = p->getTimeCoord();
				
				// if point not in corridor, disregard
				if ( !trajectory_time_interval.contains(p_t, c) ) {
					continue;
				}
				
				// origin has no edges
				if ( p->incoming_edge == NULL || p->outgoing_edge == NULL ) {
					hClass.push_back( Constants::H_CLASS_ORIGIN );
					continue;
				}
				
				// get trajectory position at obstacle time
				double traj_x = Utilities::trajectoryDisplacementAtTime( T, p_t, c );
				
				// classify before or after
				if ( Maths::approxLt(p_x, traj_x, epsilon) ) {
					hClass.push_back( Constants::H_CLASS_BEFORE );
					continue;
				}
				if ( Maths::approxGt(p_x, traj_x, epsilon) ) {
					hClass.push_back( Constants::H_CLASS_AFTER );
					continue;
				}
				
				/**
				 * At this point the vertex is on the trajectory.
				 *
				 * Classify a vertex in relation to the given trajectory according to the incoming and outgoing edges at the vertex
				 */
				
				double traj_slope = Utilities::trajectoryVelocityAtTime( T, p_t - T.front()->getInitialState().getTimeCoord(), c );
				double incoming_edge_slope = p->incoming_edge->getSlope();
				double outgoing_edge_slope = p->outgoing_edge->getSlope();
				
				if ( (p->incoming_edge->getPathDelta() >= 0.) && (p->incoming_edge->getTimeDelta() >= 0.) ) {
					if ( Maths::approxGt(incoming_edge_slope, 0., c.getEpsilon())
						&& Maths::approxEq(traj_slope, 0., c.getEpsilon()) ) {
						hClass.push_back( Constants::H_CLASS_BEFORE );
					} else {
						hClass.push_back( Constants::H_CLASS_AFTER );
					}
					continue;
				}
				
				if ( (p->incoming_edge->getPathDelta() >= 0.) && (p->incoming_edge->getTimeDelta() < 0.) ) {
					hClass.push_back( Constants::H_CLASS_BEFORE );
					continue;
				}
				
				if ( (p->incoming_edge->getPathDelta() < 0.) && (p->incoming_edge->getTimeDelta() < 0.) ) {
					if ( Maths::approxEq(traj_slope, 0., c.getEpsilon())
						&& Maths::approxLt(incoming_edge_slope, 0., c.getEpsilon()) ) {
						hClass.push_back( Constants::H_CLASS_AFTER );
					} else {
						hClass.push_back( Constants::H_CLASS_BEFORE );
					}
					continue;
				}
				
				if ( (p->incoming_edge->getPathDelta() < 0.) && (p->incoming_edge->getTimeDelta() >= 0.) ) {
					hClass.push_back( Constants::H_CLASS_AFTER );
					continue;
				}
				
				std::cerr << "ERROR IN Planner::Channel: Invalid obstacle edge." << std::endl;
				std::cerr << "Incoming: " << *p->incoming_edge << std::endl;
				std::cerr << "Outgoing: " << *p->outgoing_edge << std::endl;
				return false;

			}
			
			if ( hClass.empty() ) {
				hClass.push_back( Constants::H_CLASS_ORIGIN );
			}
			
			return true;
		}
		
		bool isSuffix( const std::vector<char>& B, const std::vector<char>& B_prime ) {
			if ( B.size() > B_prime.size() ) {
				return false;
			}
			if ( B.empty() ) {
				return true;
			}

			ssize_t j = B_prime.size() - 1;

			for ( ssize_t i=B.size()-1; i>=0; i-- ) {
				
				// compare last bit
				if ( B[i] != B_prime[j] ) {
					return false;
				}
				
				// move up
				j--;
			}
			
			// if we make it here, B is a suffix of B_prime
			return true;
		}
		
		void Merge( std::vector<Interval>& V, std::vector<PVT_S*>& S, const Constraints& c ) {

			bool modified = true;
			
			while ( modified ) {
			
				modified = false;
				
				for ( size_t i=0; i<S.size(); ++i ) {
				
					for ( size_t j=0; j<S.size(); ++j ) {
						
						// don't merge with self
						if ( i == j ) {
							continue;
						}
						
						// test for suffix
						if ( isSuffix(*S[i]->B, *S[j]->B) ) {
							
							// get interval information
							double min_i = S[i]->V.getMin();
							double max_i = S[i]->V.getMax();
							double min_j = S[j]->V.getMin();
							double max_j = S[j]->V.getMax();
							
							// modify j
							S[j]->V.setBounds( std::min(min_i, min_j), std::max(max_i, max_j) );
							
							// mark as modified
							modified = true;
							break;							
						}
					}
					
					// element i was merged, break the loop and start over
					if ( modified ) {
						delete( S[i] );
						S.erase( S.begin() + i );
						break;
					}
				}
			}
			
			// gather final velocity intervals together
			std::vector<Interval> V_tmp;
			for ( size_t i=0; i<S.size(); ++i ) {
				
				if ( !S[i]->V.isEmpty() ) {
					V_tmp.push_back( S[i]->V );
				}
			}
			
			// clear previous velocity intervals
			V.clear();
			
			// perform union of intervals
			Interval::specialUnionOfIntervals( V, V_tmp );
		}
		
		void MergeGoal( std::vector<PVT_S*>& Goal, std::vector<PVT_S*>& S, const Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			// merge information for each channel class
			bool modified = true;
			while ( modified ) {
				modified = false;
				
				for ( size_t i=0; i<S.size(); ++i ) {
					for ( size_t j=0; j<S.size(); ++j ) {
						
						// don't self merge
						if ( j == i ) {
							continue;
						}
						
						// is element i a suffix of j?
						if ( isSuffix(*S[i]->B, *S[j]->B) ) {
							
							double min_i = S[i]->UB->back()->getFinalState().getTimeCoord();
							double max_i = S[i]->LB->back()->getFinalState().getTimeCoord();
							double min_j = S[j]->UB->back()->getFinalState().getTimeCoord();
							double max_j = S[j]->LB->back()->getFinalState().getTimeCoord();
							
							// merge lower time bounding trajectory
							if ( Maths::approxLt(min_i, min_j, epsilon) ) {
								S[j]->setUB( *S[i]->UB );
							}
							
							// merge upper time bounding trajectory
							if ( Maths::approxGt(max_i, max_j, epsilon) ) {
								S[j]->setLB( *S[i]->LB );
							}
							
							// mark as modified
							modified = true;
							break;
						}
					}
					
					// element i was merged, break the loop and start over
					if ( modified ) {
						delete( S[i] );
						S.erase( S.begin() + i );
						break;
					}
				}
			}
			
			// add to Goal
			for ( size_t i=0; i<S.size(); ++i ) {
				Goal.push_back( new PVT_S( *S[i] ) );
			}
		}

		bool NextReachableSet( std::pair< std::vector<TrajectorySegment*> *, std::vector<TrajectorySegment*> * >& Reachable,
							  const PVT_Point& p1,
							  const PVT_Point& p2,
							  const Interval& V_i,
							  const Constraints& c ) {
			
			//
			// Upper bounding case
			//
			
			std::vector<TrajectorySegment*> * UB = new std::vector<TrajectorySegment*>();
			
			if ( !UpperBoundingStates(*UB, p1, p2, V_i, true, c) ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: UpperBoundingStates failed. (1)" << std::endl;
				return false;
			}
			
			if ( UB->empty() ) {
				if ( !UpperBoundingStates(*UB, p1, p2, V_i, false, c) ) {
					std::cerr << "ERROR IN Planner::NextReachableSet: UpperBoundingStates failed. (2)" << std::endl;
					return false;
				}
			}
			
			//
			// Lower bounding case
			//
			
			std::vector<TrajectorySegment*> * LB = new std::vector<TrajectorySegment*>();
			
			if ( !LowerBoundingStates(*LB, p1, p2, V_i, false, c) ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: LowerBoundingStates failed. (1)" << std::endl;
				return false;
			}
			
			if ( LB->empty() ) {
				if ( !LowerBoundingStates(*LB, p1, p2, V_i, true, c) ) {
					std::cerr << "ERROR IN Planner::NextReachableSet: LowerBoundingStates failed. (2)" << std::endl;
					return false;
				}
			}
			
#ifdef HALP //DEBUG
			if ( UB->empty() != LB->empty() ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: Mismatch in bounding trajectories." << std::endl;
				std::cerr << p1 << " -> " << p2 << " V_i: " << V_i << std::endl;
				std::cerr << "---UB---" << std::endl;
				Utilities::PrintTrajectory( *UB );
				std::cerr << "---LB---" << std::endl;
				Utilities::PrintTrajectory( *LB );
				std::cerr << "===" << std::endl;
				return false;
			}
#else
			if ( UB->empty() && !LB->empty() ) {
				delete( UB );
				UB = Utilities::CopyTrajectory( *LB );
			}
			if ( LB->empty() && !UB->empty() ) {
				delete( LB );
				LB = Utilities::CopyTrajectory( *UB );
			}
#endif
			
			// unreachable
			if ( UB->empty() || LB->empty() ) {
				Reachable.first = NULL;
				Reachable.second = NULL;
				return true;
			}
			
			Reachable.first = UB;
			Reachable.second = LB;
			
			return true;
		}
		
		bool LowerBoundingStates( std::vector<TrajectorySegment*>& T,
									 const PVT_Point& p1,
									 const PVT_Point& p2,
									 const Interval V_i,
									 bool minimize_initial_velocity,
									 const Constraints& c) {
			
			double epsilon = c.getEpsilon();
			
			// point-specific constraints
			Constraints c_tmp( c );
			double v1 = minimize_initial_velocity ? V_i.getMin() : V_i.getMax();
			double v2_min = (c_tmp.getVMin() > p2.i.getMin()) ? c_tmp.getVMin() : p2.i.getMin();
			double v2_max = (c_tmp.getVMax() < p2.i.getMax()) ? c_tmp.getVMax() : p2.i.getMax();
			double v_min = c_tmp.getVMin();
			double v_max = c_tmp.getVMax();
			double delta_x = p2.getPathCoord() - p1.getPathCoord();
			double delta_t = p2.getTimeCoord() - p1.getTimeCoord();
			c_tmp.setXLimit( delta_x );
			c_tmp.setTLimit( delta_t );
			
			// determine terminal canonical P^- coefficients
			double a_terminal = 0.5 * c.getAMin();
			double b_terminal;
			double c_terminal;
			Maths::parabolaCoefficients2( a_terminal, b_terminal, c_terminal, delta_t, delta_x, v2_min, epsilon );
			
			// determine intersection type
			double dummy_t;
			int intersection_type = Maths::lineParabolaIntersection( dummy_t, 0., v1, 0., a_terminal, b_terminal, c_terminal, epsilon );
			
			// Trivial case: LP^-
			if ( intersection_type == Maths::INTERSECTION_UNIQUE ) {
				//std::cout << "intersection: unique" << std::endl;
				
				double x_tangent = p1.getPathCoord() + Maths::yFromSlopeIntercept( v1, dummy_t, 0. );
				double t_tangent = p1.getTimeCoord() + dummy_t;
				T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1, x_tangent, t_tangent, v1) );
				T.push_back( new TrajectorySegment(x_tangent, t_tangent, v1, p2.getPathCoord(), p2.getTimeCoord(), v2_min) );
				
				return true;
			}
			
			// CASE 1: P^-LP^-
			if ( intersection_type == Maths::INTERSECTION_NONE ) {
				//std::cout << "intersection: none" << std::endl;
				
				// determine initial canonical P^+ coefficients
				double a_initial = 0.5 * c.getAMin();
				double b_initial = v1;
				double c_initial = 0.;
				
				double t_switch = 2. * (c_initial + v2_min * delta_t - delta_x) / (v2_min - b_initial);
				double a = (v2_min - b_initial) / (2. * t_switch);
				double x_switch = Maths::parabolaCanonical( a, b_initial, c_initial, t_switch, epsilon );
				
				//std::cout << "a: " << a << std::endl;
				//std::cout << "switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
				//std::cout << "delta_x: " << delta_x << ", delta_t: " << delta_t << std::endl;
				//std::cout << "v1: " << v1 << std::endl;
				
				if ( c_tmp.validT(t_switch)
					&& c_tmp.validX(x_switch)
					&& c_tmp.validA(2. * a) ) {
					
					//std::cout << "a: " << a << std::endl;
					//std::cout << "switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
					
					x_switch += p1.getPathCoord();
					t_switch += p1.getTimeCoord();
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
													   x_switch, t_switch, v2_min) );
					T.push_back( new TrajectorySegment(x_switch, t_switch, v2_min,
													   p2.getPathCoord(), p2.getTimeCoord(), v2_min) );
					
					return true;
				}
				
				// compute connecting linear segment
				double x_switch1;
				double t_switch1;
				double x_switch2;
				double t_switch2;
				Maths::tangentLineToParabolas( t_switch1,
											  x_switch1,
											  t_switch2,
											  x_switch2,
											  a_initial,
											  b_initial,
											  c_initial,
											  a_terminal,
											  b_terminal,
											  c_terminal,
											  epsilon );
				
				// if switch points are valid, velocity is necessarily valid, done
				// else try LP-
				if ( c_tmp.validX(x_switch1)
					&& c_tmp.validX(x_switch2)
					&& c_tmp.validT(t_switch1)
					&& c_tmp.validT(t_switch2)
					&& Maths::approxLe(t_switch1, t_switch2, epsilon) ) {
					
					x_switch1 += p1.getPathCoord();
					t_switch1 += p1.getTimeCoord();
					x_switch2 += p1.getPathCoord();
					t_switch2 += p1.getTimeCoord();
					double v_tangent = Maths::slope( t_switch1, x_switch1, t_switch2, x_switch2, epsilon );
					
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
													   x_switch1, t_switch1, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch1, t_switch1, v_tangent,
													   x_switch2, t_switch2, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch2, t_switch2, v_tangent,
													   p2.getPathCoord(), p2.getTimeCoord(), v2_min) );
					
					return true;
					
				} else {

					// find tangent line from origin to terminal curve
					double t_tangent;
					double x_tangent;
					Maths::tangentLineThroughPointToParabola1( t_tangent,
															  x_tangent,
															  a_terminal,
															  b_terminal,
															  c_terminal,
															  0.,
															  0.,
															  epsilon );
					
					// only proceed if (x_tangent, y_tangent) feasible
					if ( c_tmp.validX(x_tangent) && c_tmp.validT(t_tangent) ) {
						double v_linear = Maths::slope( 0., 0., t_tangent, x_tangent, epsilon );
						if ( V_i.contains(v_linear, c) ) {
							x_tangent += p1.getPathCoord();
							t_tangent += p1.getTimeCoord();
							T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v_linear, x_tangent, t_tangent, v_linear) );
							T.push_back( new TrajectorySegment(x_tangent, t_tangent, v_linear, p2.getPathCoord(), p2.getTimeCoord(), v2_min) );
							return true;
						}
					}
				}
			}
			
			//
			// CASE 2: P^+LP^-
			//
			
			if ( intersection_type == Maths::INTERSECTION_NON_UNIQUE ) {
				//std::cout << "intersection: non-unique" << std::endl;

				// determine initial canonical P^+ coefficients
				double a_initial = 0.5 * c.getAMax();
				double b_initial = v1;
				double c_initial = 0.;
				
				double t_switch = 2. * (c_initial + v2_min * delta_t - delta_x) / (v2_min - b_initial);
				double a = (v2_min - b_initial) / (2. * t_switch);
				double x_switch = Maths::parabolaCanonical( a, b_initial, c_initial, t_switch, epsilon );
				
				//std::cout << "terminal: " << a_terminal << ", " << b_terminal << ", " << c_terminal << std::endl;
				//std::cout << "initial: " << a_initial << ", " << b_initial << ", " << c_initial << std::endl;
				
				//std::cout << "a: " << a << std::endl;
				//std::cout << "switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
				//std::cout << "delta_x: " << delta_x << ", delta_t: " << delta_t << std::endl;
				//std::cout << "v1: " << v1 << std::endl;

				if ( c_tmp.validT(t_switch)
					&& c_tmp.validX(x_switch)
					&& c_tmp.validA(2. * a) ) {
					
					x_switch += p1.getPathCoord();
					t_switch += p1.getTimeCoord();
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
													   x_switch, t_switch, v2_min) );
					T.push_back( new TrajectorySegment(x_switch, t_switch, v2_min,
													   p2.getPathCoord(), p2.getTimeCoord(), v2_min) );

					return true;
				}
				
				// compute connecting linear segment
				double x_switch1;
				double t_switch1;
				double x_switch2;
				double t_switch2;
				Maths::tangentLineToParabolas( t_switch1,
											  x_switch1,
											  t_switch2,
											  x_switch2,
											  a_initial,
											  b_initial,
											  c_initial,
											  a_terminal,
											  b_terminal,
											  c_terminal,
											  epsilon );
				double v_tangent = Maths::slope( t_switch1, x_switch1, t_switch2, x_switch2, epsilon );
				
				//std::cout << std::endl;
				//std::cout << "switch1: (" << x_switch1 << ", " << t_switch1 << "), switch2: (" << x_switch2 << ", " << t_switch2 << ")" << std::endl;
				//std::cout << "v_tangent: " << v_tangent << std::endl;
				//std::cout << "v2_min: " << v2_min << std::endl;
				//std::cout << "delta_x: " << delta_x << ", delta_t: " << delta_t << std::endl;
				//std::cout << "v_tangent: " << v_tangent << std::endl;
				//std::cout << std::endl;

				// if switch points are valid, velocity is necessarily valid, done
				if ( c_tmp.validX(x_switch1)
					&& c_tmp.validX(x_switch2)
					&& c_tmp.validT(t_switch1)
					&& c_tmp.validT(t_switch2)
					&& Maths::approxLe(t_switch1, t_switch2, epsilon)
					&& c_tmp.validV(v_tangent) ) {
					
					x_switch1 += p1.getPathCoord();
					t_switch1 += p1.getTimeCoord();
					x_switch2 += p1.getPathCoord();
					t_switch2 += p1.getTimeCoord();
					
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
													   x_switch1, t_switch1, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch1, t_switch1, v_tangent,
													   x_switch2, t_switch2, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch2, t_switch2, v_tangent,
													   p2.getPathCoord(), p2.getTimeCoord(), v2_min) );
					
					return true;
					
				//
				// CASE 3: P^+LP^- with v2 > v2_min, slope L = v_max
				//
				
				} else {
					
					// find point that initial curve reaches v_max
					double t_switch1 = Maths::T_FromV1_V2_A( v1, v_max, c.getAMax() );
					double x_switch1 = Maths::parabolaCanonical( a_initial, b_initial, c_initial, t_switch1, epsilon );
					
					// find tangent line to terminal curve
					double t_switch2;
					double x_switch2;
					Maths::tangentLineThroughPointToParabola2( t_switch2,
															  x_switch2,
															  a_terminal,
															  b_terminal,
															  c_terminal,
															  t_switch1,
															  x_switch1,
															  v_max,
															  delta_t,
															  delta_x,
															  epsilon );
					double v2 = Maths::V2_FromV1_T_A( v_max, delta_t - t_switch2, c.getAMax(), epsilon );
					
					//std::cout << std::endl;
					//std::cout << "a1: " << a_initial << ", b1: " << b_initial << ", c1: " << c_initial << std::endl;
					//std::cout << "a2: " << a_terminal << ", b2: " << b_terminal << ", c2: " << c_terminal << std::endl;
					//std::cout << "v1: " << v1 << ", switch1: (" << x_switch1 << ", " << t_switch1 << "), switch2: (" << x_switch2 << ", " << t_switch2 << ")" << std::endl;
					//std::cout << "v2: " << v2 << std::endl;
					//std::cout << std::endl;
					
					// if switch points are valid, velocities necessarily valid, done
					if ( c_tmp.validX(x_switch1)
						&& c_tmp.validX(x_switch2)
						&& c_tmp.validT(t_switch1)
						&& c_tmp.validT(t_switch2)
						&& Maths::approxLe(t_switch1, t_switch2, epsilon)
						&& Maths::approxGe(v2, v2_min, epsilon)
						&& Maths::approxLe(v2, v2_max, epsilon)) {
						
						x_switch1 += p1.getPathCoord();
						t_switch1 += p1.getTimeCoord();
						x_switch2 += p1.getPathCoord();
						t_switch2 += p1.getTimeCoord();
						
						T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
														   x_switch1, t_switch1, v_max) );
						T.push_back( new TrajectorySegment(x_switch1, t_switch1, v_max,
														   x_switch2, t_switch2, v_max) );
						T.push_back( new TrajectorySegment(x_switch2, t_switch2, v_max,
														   p2.getPathCoord(), p2.getTimeCoord(), v2) );
						
						return true;
						
					//
					// CASE 3: P^+P^-
					//
					
					} else {
						
						double t_switch;
						double x_switch;
						Maths::tangentParabolaThroughPoint1( a_terminal,
															b_terminal,
															c_terminal,
															t_switch,
															x_switch,
															a_initial,
															b_initial,
															c_initial,
															v1,
															0.,
															0.,
															delta_t,
															delta_x,
															epsilon );
						
						//std::cout << "a2: " << a_terminal << ", a1: " << a_initial << ", v_i: " << v1 << ", delta_x: " << delta_x << ", delta_t: " << delta_t << std::endl;
						//std::cout << "switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
						
						double v_switch = Maths::parabolaSlope( a_initial, b_initial, t_switch, epsilon );
						double v2 = Maths::parabolaSlope( a_terminal, b_terminal, delta_t, epsilon );
						
						if ( c_tmp.validX(x_switch)
							&& c_tmp.validT(t_switch)
							&& c_tmp.validV(v_switch)
							&& Maths::approxGe(v2, v2_min, epsilon)
							&& Maths::approxLe(v2, v2_max, epsilon) ) {
							
							x_switch += p1.getPathCoord();
							t_switch += p1.getTimeCoord();
							T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
															   x_switch, t_switch, v_switch) );
							T.push_back( new TrajectorySegment(x_switch, t_switch, v_switch,
															   p2.getPathCoord(), p2.getTimeCoord(), v2) );
							
							return true;
							
						}
						
					}
				}
			}
			
			return true;
		}
		
		bool UpperBoundingStates( std::vector<TrajectorySegment*>& T,
								 const PVT_Point& p1,
								 const PVT_Point& p2,
								 const Interval V_i,
								 bool minimize_initial_velocity,
								 const Constraints& c) {
			double epsilon = c.getEpsilon();
			
			// point-specific constraints
			Constraints c_tmp( c );
			double v1 = minimize_initial_velocity ? V_i.getMin() : V_i.getMax();
			double v2_min = (c_tmp.getVMin() > p2.i.getMin()) ? c_tmp.getVMin() : p2.i.getMin();
			double v2_max = (c_tmp.getVMax() < p2.i.getMax()) ? c_tmp.getVMax() : p2.i.getMax();
			double v_min = c_tmp.getVMin();
			double v_max = c_tmp.getVMax();
			double delta_x = p2.getPathCoord() - p1.getPathCoord();
			double delta_t = p2.getTimeCoord() - p1.getTimeCoord();
			c_tmp.setXLimit( delta_x );
			c_tmp.setTLimit( delta_t );
			
			// determine terminal canonical P^+ coefficients
			double a_terminal = 0.5 * c.getAMax();
			double b_terminal;
			double c_terminal;
			Maths::parabolaCoefficients2( a_terminal, b_terminal, c_terminal, delta_t, delta_x, v2_max, epsilon );
			
			// determine intersection type
			double dummy_t;
			int intersection_type = Maths::lineParabolaIntersection( dummy_t, 0., v1, 0., a_terminal, b_terminal, c_terminal, epsilon );
			
			// Trivial case: LP^+
			if ( intersection_type == Maths::INTERSECTION_UNIQUE ) {
				//std::cout << "intersection: unique" << std::endl;

				double x_tangent = p1.getPathCoord() + Maths::yFromSlopeIntercept( v1, dummy_t, 0. );
				double t_tangent = p1.getTimeCoord() + dummy_t;
				T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1, x_tangent, t_tangent, v1) );
				T.push_back( new TrajectorySegment(x_tangent, t_tangent, v1, p2.getPathCoord(), p2.getTimeCoord(), v2_max) );
				
				return true;
			}
			
			//
			// CASE 1: P^+LP^+
			//
			
			if ( intersection_type == Maths::INTERSECTION_NONE ) {
				//std::cout << "intersection: none" << std::endl;
				
				// determine initial canonical P^+ coefficients
				double a_initial = 0.5 * c.getAMax();
				double b_initial = v1;
				double c_initial = 0.;
				
				double t_switch = 2. * (c_initial + v2_max * delta_t - delta_x) / (v2_max - b_initial);
				double a = (v2_max - b_initial) / (2. * t_switch);
				double x_switch = Maths::parabolaCanonical( a, b_initial, c_initial, t_switch, epsilon );
				if ( c_tmp.validT(t_switch)
					&& c_tmp.validX(x_switch)
					&& c_tmp.validA(2. * a) ) {
					
					//std::cout << "a: " << a << std::endl;
					//std::cout << "switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
					
					x_switch += p1.getPathCoord();
					t_switch += p1.getTimeCoord();
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1, x_switch, t_switch, v2_max) );
					T.push_back( new TrajectorySegment(x_switch, t_switch, v2_max, p2.getPathCoord(), p2.getTimeCoord(), v2_max) );
					return true;
				}
				
				// compute connecting linear segment
				double x_switch1;
				double t_switch1;
				double x_switch2;
				double t_switch2;
				Maths::tangentLineToParabolas( t_switch1,
											  x_switch1,
											  t_switch2,
											  x_switch2,
											  a_initial,
											  b_initial,
											  c_initial,
											  a_terminal,
											  b_terminal,
											  c_terminal,
											  epsilon );
				
				// if switch points are valid, velocity is necessarily valid, done
				// else try LP+
				if ( c_tmp.validX(x_switch1)
					&& c_tmp.validX(x_switch2)
					&& c_tmp.validT(t_switch1)
					&& c_tmp.validT(t_switch2)
					&& Maths::approxLe(t_switch1, t_switch2, epsilon) ) {
					
					x_switch1 += p1.getPathCoord();
					t_switch1 += p1.getTimeCoord();
					x_switch2 += p1.getPathCoord();
					t_switch2 += p1.getTimeCoord();
					double v_tangent = Maths::slope( t_switch1, x_switch1, t_switch2, x_switch2, epsilon );
					
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
														 x_switch1, t_switch1, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch1, t_switch1, v_tangent,
														 x_switch2, t_switch2, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch2, t_switch2, v_tangent,
														 p2.getPathCoord(), p2.getTimeCoord(), v2_max) );
					
					return true;
					
				} else {
					
					// find tangent line from origin to terminal curve
					double t_tangent;
					double x_tangent;
					Maths::tangentLineThroughPointToParabola1( t_tangent,
															  x_tangent,
															  a_terminal,
															  b_terminal,
															  c_terminal,
															  0.,
															  0.,
															  epsilon );
					
					// only proceed if (x_tangent, y_tangent) feasible
					if ( c_tmp.validX(x_tangent) && c_tmp.validT(t_tangent) ) {
						double v_linear = Maths::slope( 0., 0., t_tangent, x_tangent, epsilon );
						if ( V_i.contains(v_linear, c) ) {
							x_tangent += p1.getPathCoord();
							t_tangent += p1.getTimeCoord();
							T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v_linear, x_tangent, t_tangent, v_linear) );
							T.push_back( new TrajectorySegment(x_tangent, t_tangent, v_linear, p2.getPathCoord(), p2.getTimeCoord(), v2_max) );
							return true;
						}
					}
				}
			}
			
			//
			// CASE 2: P^-LP^+
			//
			
			if ( intersection_type == Maths::INTERSECTION_NON_UNIQUE ) {
				//std::cout << "intersection: non unique" << std::endl;
				
				// determine initial canonical P^- coefficients
				double a_initial = 0.5 * c.getAMin();
				double b_initial = v1;
				double c_initial = 0.;
				
				double t_switch = 2. * (c_initial + v2_max * delta_t - delta_x) / (v2_max - b_initial);
				double a = (v2_max - b_initial) / (2. * t_switch);
				double x_switch = Maths::parabolaCanonical( a, b_initial, c_initial, t_switch, epsilon );
				if ( c_tmp.validT(t_switch)
					&& c_tmp.validX(x_switch)
					&& c_tmp.validA(2. * a) ) {
					
					//std::cout << "a: " << a << std::endl;
					//std::cout << "switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
					
					x_switch += p1.getPathCoord();
					t_switch += p1.getTimeCoord();
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1, x_switch, t_switch, v2_max) );
					T.push_back( new TrajectorySegment(x_switch, t_switch, v2_max, p2.getPathCoord(), p2.getTimeCoord(), v2_max) );
					return true;
				}
				
				// compute connecting linear segment
				double x_switch1;
				double t_switch1;
				double x_switch2;
				double t_switch2;
				Maths::tangentLineToParabolas( t_switch1,
											  x_switch1,
											  t_switch2,
											  x_switch2,
											  a_initial,
											  b_initial,
											  c_initial,
											  a_terminal,
											  b_terminal,
											  c_terminal,
											  epsilon );
				double v_tangent = Maths::slope( t_switch1, x_switch1, t_switch2, x_switch2, epsilon );
				
				//std::cout << std::endl;
				//std::cout << "a1: " << a_initial << ", b1: " << b_initial << ", c1: " << c_initial << std::endl;
				//std::cout << "a2: " << a_terminal << ", b2: " << b_terminal << ", c2: " << c_terminal << std::endl;
				//std::cout << "v1: " << v1 << ", switch1: (" << x_switch1 << ", " << t_switch1 << "), switch2: (" << x_switch2 << ", " << t_switch2 << ")" << std::endl;
				//std::cout << "v_tangent: " << v_tangent << std::endl;
				//std::cout << std::endl;
				
				// if switch points are valid, velocity is necessarily valid, done
				if ( c_tmp.validX(x_switch1)
					&& c_tmp.validX(x_switch2)
					&& c_tmp.validT(t_switch1)
					&& c_tmp.validT(t_switch2)
					&& Maths::approxLe(t_switch1, t_switch2, epsilon)
					&& c_tmp.validV(v_tangent) ) {
					
					x_switch1 += p1.getPathCoord();
					t_switch1 += p1.getTimeCoord();
					x_switch2 += p1.getPathCoord();
					t_switch2 += p1.getTimeCoord();
					
					T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
													   x_switch1, t_switch1, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch1, t_switch1, v_tangent,
													   x_switch2, t_switch2, v_tangent) );
					T.push_back( new TrajectorySegment(x_switch2, t_switch2, v_tangent,
													   p2.getPathCoord(), p2.getTimeCoord(), v2_max) );
					
					return true;
				
				//
				// CASE 3: P^-LP^+ with v2 < v2_max, slope L = v2_min
				//
			
				} else {
					
					// find point that initial curve reaches v_min
					double t_switch1 = Maths::T_FromV1_V2_A( v1, v_min, c.getAMin() );
					double x_switch1 = Maths::parabolaCanonical( a_initial, b_initial, c_initial, t_switch1, epsilon );
					
					// find tangent line to terminal curve
					double t_switch2;
					double x_switch2;
					Maths::tangentLineThroughPointToParabola2( t_switch2,
															  x_switch2,
															  a_terminal,
															  b_terminal,
															  c_terminal,
															  t_switch1,
															  x_switch1,
															  v_min,
															  delta_t,
															  delta_x,
															  epsilon );
					double v2 = Maths::V2_FromV1_T_A( v_min, delta_t - t_switch2, c.getAMax(), epsilon );
					
					//std::cout << std::endl;
					//std::cout << "a1: " << a_initial << ", b1: " << b_initial << ", c1: " << c_initial << std::endl;
					//std::cout << "a2: " << a_terminal << ", b2: " << b_terminal << ", c2: " << c_terminal << std::endl;
					//std::cout << "v1: " << v1 << ", switch1: (" << x_switch1 << ", " << t_switch1 << "), switch2: (" << x_switch2 << ", " << t_switch2 << ")" << std::endl;
					//std::cout << "v2: " << v2 << std::endl;
					//std::cout << std::endl;
					
					// if switch points are valid, velocities necessarily valid, done
					if ( c_tmp.validX(x_switch1)
						&& c_tmp.validX(x_switch2)
						&& c_tmp.validT(t_switch1)
						&& c_tmp.validT(t_switch2)
						&& Maths::approxLe(t_switch1, t_switch2, epsilon)
						&& Maths::approxGe(v2, v2_min, epsilon)
						&& Maths::approxLe(v2, v2_max, epsilon) ) {
						
						x_switch1 += p1.getPathCoord();
						t_switch1 += p1.getTimeCoord();
						x_switch2 += p1.getPathCoord();
						t_switch2 += p1.getTimeCoord();
						
						T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
														   x_switch1, t_switch1, v_min) );
						T.push_back( new TrajectorySegment(x_switch1, t_switch1, v_min,
														   x_switch2, t_switch2, v_min) );
						T.push_back( new TrajectorySegment(x_switch2, t_switch2, v_min,
														   p2.getPathCoord(), p2.getTimeCoord(), v2) );
						
						return true;
					//
					// CASE 3: P^-P^+
					//
					
					} else {
						
						double x_switch;
						double t_switch;
						Maths::tangentParabolaThroughPoint1( a_terminal,
															b_terminal,
															c_terminal,
															t_switch,
															x_switch,
															a_initial,
															b_initial,
															c_initial,
															v1,
															0.,
															0.,
															delta_t,
															delta_x,
															epsilon );
						
						double v_switch = Maths::parabolaSlope( a_initial, b_initial, t_switch, epsilon );
						double v2 = Maths::parabolaSlope( a_terminal, b_terminal, delta_t, epsilon );
						
						//std::cout << "a1: " << a_initial << ", b1: " << b_initial << ", c1: " << c_initial << std::endl;
						//std::cout << "a2: " << a_terminal << ", b2: " << b_terminal << ", c2: " << c_terminal << std::endl;
						//std::cout << "v1: " << v1 << ", switch: (" << x_switch << ", " << t_switch << ")" << std::endl;
						//std::cout << "v2: " << v2 << std::endl;
						
						if ( c_tmp.validX(x_switch)
							&& c_tmp.validT(t_switch)
							&& c_tmp.validV(v_switch)
							&& Maths::approxGe(v2, v2_min, epsilon)
							&& Maths::approxLe(v2, v2_max, epsilon) ) {
							
							x_switch += p1.getPathCoord();
							t_switch += p1.getTimeCoord();
							T.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), v1,
															   x_switch, t_switch, v_switch) );
							T.push_back( new TrajectorySegment(x_switch, t_switch, v_switch,
															   p2.getPathCoord(), p2.getTimeCoord(), v2) );
							
							return true;
							
						}
						
					}
				}
			}
			
			return true;
		}
		
		bool ConnectPointToEdge( std::vector<TrajectorySegment*>& T,
								const PVT_Point& _p,
								const Interval& V_i,
								const PVT_ObstacleEdge& _edge,
								const Constraints& c ) {
			double epsilon = c.getEpsilon();

			// make sure the trajectory is initialized empty
			Utilities::CleanTrajectory( T );
			
			// translate system to origin
			PVT_Point p( _p );
			PVT_ObstacleEdge edge( _edge );
			edge.translate( -p.getPathCoord(), -p.getTimeCoord(), c );
			p.setCoords( 0., 0. );

			// first check that an edge connection is even possible
			double edge_slope = edge.getSlope();
			if ( !c.validV(edge_slope) ) {
				Utilities::CleanTrajectory( T );
				return true;
			}
			double edge_intercept = edge.getPathIntercept();

			// if edge is above p, use P^-LP^+, otherwise P^+LP^-
			double c1 = 0.;
			double a1;
			double a2;
			int relation = Maths::pointLineRelation( p.getPathCoord(), p.getTimeCoord(), edge_slope, edge_intercept, epsilon );
			
			if ( relation == Maths::POINT_ABOVE ) {
				a1 = 0.5 * c.getAMin();
				a2 = 0.5 * c.getAMax();
			} else if ( relation == Maths::POINT_BELOW ) {
				a1 = 0.5 * c.getAMax();
				a2 = 0.5 * c.getAMin();
			} else if ( relation == Maths::POINT_ON ) {
				
				// trajectory can simply trace along edge
				if ( V_i.contains(edge_slope, c) ) {
					T.push_back( new TrajectorySegment(_p.getPathCoord(), _p.getTimeCoord(), edge_slope, _p.getPathCoord(), _p.getTimeCoord(), edge_slope) );
					return true;
				}
				
				if ( Maths::approxGt(V_i.getMin(), edge_slope, epsilon) ) {
					a1 = 0.5 * c.getAMin();
					a2 = 0.5 * c.getAMax();
				} else if ( Maths::approxGt(edge_slope, V_i.getMax(), epsilon) ) {
					a1 = 0.5 * c.getAMax();
					a2 = 0.5 * c.getAMin();
				}
				
			} else {
				std::cerr << "ERROR IN Planner::ConnectPointToEdge: Unexpected point/line relation (1)." << std::endl;
				return false;
			}
			
			// compute feasible initial velocity extremum
			double discriminant = Maths::clipToZero( (c1 - edge_intercept) / a2, epsilon );
			if ( discriminant < 0. ) {
				std::cerr << "ERROR IN Planner::ConnectPointToEdge: Error computing initial feasible velocity extremum." << std::endl;
				return false;
			}
			double v_i_feasible = edge_slope - 2. * a2 * sqrt( discriminant );
			
			double b1;
			double v_min = V_i.getMin();
			double v_max = V_i.getMax();
			if ( relation == Maths::POINT_ABOVE ) {
				if ( v_i_feasible > v_max ) {
					return true;
				}
				b1 = std::max( v_i_feasible, v_min );
			} else if ( relation == Maths::POINT_BELOW ) {
				if ( v_i_feasible < v_min ) {
					return true;
				}
				b1 = std::min( v_i_feasible, v_max );
			} else if ( relation == Maths::POINT_ON ) {
				if ( v_i_feasible < v_min ) {
					b1 = v_min;
				} else if ( v_i_feasible > v_max ) {
					b1 = v_max;
				} else {
					b1 = v_i_feasible;
				}
			} else {
				std::cerr << "ERROR IN Planner::ConnectPointToEdge: Unexpected point/line relation (2)." << std::endl;
				return false;
			}
			
			// first try with no linear segment
			double a1_minus_a2 = a1 - a2;
			double edge_slope_minus_b1 = edge_slope - b1;
			double q_a = a1_minus_a2 + a1_minus_a2 * a1_minus_a2 / a2;
			double q_b = -edge_slope_minus_b1 * a1_minus_a2 / a2;
			double q_c = edge_slope_minus_b1 * edge_slope_minus_b1 / (4. * a2) + edge_intercept - c1;
			std::pair<double, double> roots;
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			// crossover time
			double t1;
			if ( a2 > 0. ) {
				t1 = roots.first;
			} else {
				t1 = roots.second;
			}
			
			if ( c.validT(t1) ) {
				std::vector<PVT_State> states;
				
				// add initial state
				states.push_back( PVT_State(p.getPathCoord(), p.getTimeCoord(), b1) );
				
				// remaining parabola coefficients
				double b2 = Maths::V2_FromV1_T_A( b1, 2. * a1_minus_a2, t1, epsilon );
				double c2 = Maths::parabolaCanonical( a1_minus_a2, b1 - b2, c1, t1, epsilon );
				
				// crossover state
				double p1 = Maths::parabolaCanonical( a1, b1, c1, t1, epsilon );
				double v1 = 2. * a1 * t1 + b1;
				
				// valid crossover?
				if ( !c.validV(v1) ) {
					
					// add linear segment
					// point on dec. curve at which velocity is v_min
					double v_min = c.getVMin();
					double t_v_min = (v_min - b1) / (2. * a1);
					double p_v_min = Maths::parabolaCanonical( a1, b1, c1, t_v_min, epsilon );
					double linear_seg_intercept = Maths::yIntercept( t_v_min, p_v_min, v_min, epsilon );
					
					// point of tangency of acc. curve on linear segment
					double t_linear = (linear_seg_intercept - edge_intercept) / (edge_slope - v_min) - (edge_slope - v_min) / (4. * a2);
					double p_linear = v_min * t_linear + linear_seg_intercept;
					
					// remaining acc. curve coefficients
					b2 = v_min - 2. * a2 * t_linear;
					c2 = p_linear - t_linear * (a2 * t_linear + b2);
					
					// point of tangency of acc. curve on edge
					double t_edge = (edge_slope - b2) / (2. * a2);
					double p_edge = edge_slope * t_edge + edge_intercept;
					
					// add states (if valid)
					if ( !Maths::isNaN(t_linear) && Maths::approxGe(t_linear, t_v_min, epsilon) ) {
						
						// initial linear state
						states.push_back( PVT_State(p_v_min, t_v_min, v_min) );
						
						// final linear state
						states.push_back( PVT_State(p_linear, t_linear, v_min) );
						
						// edge tangency state
						states.push_back( PVT_State(p_edge, t_edge, edge_slope) );
						
					}
					
				} else {
					
					// add crossover state
					states.push_back( PVT_State(p1, t1, v1) );
					
					// add tangency state
					double t2 = (edge_slope - b2) / (2. * a2);
					double p2 = Maths::parabolaCanonical( a2, b2, c2, t2, epsilon );
					double v2 = edge_slope;
					states.push_back( PVT_State(p2, t2, v2) );
					
				}
				
				for ( size_t i=1; i<states.size(); ++i ) {
					T.push_back( new TrajectorySegment(states[i-1], states[i]) );
				}
				
			}
			
			// translate trajectory to original coordinate frame
			Utilities::translateTrajectory( T, _p.getPathCoord(), _p.getTimeCoord() );
			
			return true;
		}
		
		bool ConnectEdgeToEdge( std::vector<TrajectorySegment*>& T,
							   const PVT_State& _s,
							   const PVT_ObstacleEdge& _edge_from,
							   const PVT_ObstacleEdge& _edge_to,
							   const Constraints& c ) {
			double epsilon = c.getEpsilon();
			Utilities::CleanTrajectory( T );
			
			// verify connection is possible
			if ( !c.validV(_edge_to.getSlope()) || Maths::approxEq(_edge_to.getSlope(), _edge_from.getSlope(), epsilon) ) {
				return true;
			}
			
			// translate system to origin
			PVT_State s( 0., 0., _s.getVelocityCoord() );
			PVT_ObstacleEdge edge_from( _edge_from );
			edge_from.translate( -_s.getPathCoord(), -_s.getTimeCoord(), c );
			PVT_ObstacleEdge edge_to( _edge_to );
			edge_to.translate( -_s.getPathCoord(), -_s.getTimeCoord(), c );
			
			// edge_from as canonical line
			double b1 = edge_from.getSlope();
			double c1 = edge_from.getPathIntercept();
			
			// edge_to as canonical line
			double b3 = edge_to.getSlope();
			double c3 = edge_to.getPathIntercept();
			
			// convex up or convex down?
			bool convex_up = (edge_from.getSlope() < edge_to.getSlope()) ? true : false;
			
			// find connecting parabola
			double a2 = 0.5 * (convex_up ? c.getAMax() : c.getAMin());
			double b2;
			double c2;
			double p1;
			double t1;
			double p2;
			double t2;
			if ( !Maths::tangentParabolaAtLines( a2, b2, c2, p1, t1, p2, t2, b1, c1, b3, c3, epsilon) ) {
				std::cerr << "ERROR IN Planner::ConnectEdgeToEdge: Failed to find connecting parabola." << std::endl;
				return false;
			}
			
			// verify that the edge segments contain the intersection points
			PVT_Point point1( p1, t1 );
			PVT_Point point2( p2, t2 );
			if ( !edge_from.contains(point1, c) || !edge_to.contains(point2, c) ) {
				return true;
			}
			
			// at this point the connection is valid; add it
			T.push_back( new TrajectorySegment(s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord(), p1, t1, s.getVelocityCoord()) );
			T.push_back( new TrajectorySegment(p1, t1, s.getVelocityCoord(), p2, t2, edge_to.getSlope()) );
			Utilities::translateTrajectory( T, _s.getPathCoord(), _s.getTimeCoord() );
			
			return true;
		}
		
		bool ConnectEdgeToPoint( std::vector<TrajectorySegment*>& T,
								bool maximize_terminal_velocity,
								const PVT_ObstacleEdge& _edge,
								const PVT_State& _s,
								const PVT_Point& _p,
								const Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			// make sure the trajectory is initialized empty
			Utilities::CleanTrajectory( T );
			
			// make sure point is visible from edge start state
			if ( !_s.canReach(_p, c) ) {
				return true;
			}
			
			// translate system to origin
			PVT_State s( 0., 0., _s.getVelocityCoord() );
			PVT_ObstacleEdge edge( _edge );
			edge.translate( -_s.getPathCoord(), -_s.getTimeCoord(), c );
			PVT_Point p( _p );
			p.translate( -_s.getPathCoord(), -_s.getTimeCoord() );
			
			// feasible set of terminal velocities
			Interval V_f;
			Interval V_c( c.getVMin(), c.getVMax() );
			Interval::intersect( V_f, p.i, V_c, c );
			
			// target terminal velocity
			double v_f;
			if ( maximize_terminal_velocity ) {
				v_f = V_f.getMax();
			} else {
				v_f = V_f.getMin();
			}
			
			// the edge line as a canonical parabola
			double a1 = 0.;
			double b1 = edge.getSlope();
			double c1 = edge.getPathIntercept();
			
			// the target velocity line as a canonical parabola
			double v3 = v_f;
			double c3 = Maths::yIntercept( p.getTimeCoord(), p.getTimeCoord(), v_f, epsilon );
			
			// determine appropriate curve
			double a2;
			int p_to_edge_relation = Maths::pointLineRelation( p.getPathCoord(), p.getTimeCoord(), b1, c1, epsilon );
			int s_to_vf_line_relation = Maths::pointLineRelation( s.getPathCoord(), s.getTimeCoord(), v3, c3, epsilon );
			if ( p_to_edge_relation == Maths::POINT_ABOVE ) {
				
				// must accelerate onto target velocity line
				if ( s_to_vf_line_relation == Maths::POINT_ABOVE ) {

					a2 = 0.5 * c.getAMax();
					
				// open region; attempt PLP trajectory
				} else if ( s_to_vf_line_relation == Maths::POINT_BELOW ) {

					// get reachable set of velocities
					std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
					Reachable.first = NULL;
					Reachable.second = NULL;
					PVT_Point p1( s.getPathCoord(), s.getTimeCoord() );
					Interval V_i( s.getVelocityCoord(), s.getVelocityCoord() );
					if ( !NextReachableSet(Reachable, p1, p, V_i, c) ) {
						std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Error in NextReachableSet (1)." << std::endl;
						return false;
					}
					
					// target unreachable
					if ( (Reachable.first==NULL) || (Reachable.second==NULL) ) {
						
						// sanity check; this should never fail
						if ( !(Reachable.first==NULL && Reachable.second==NULL) ) {
							std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Unbalanced bounding trajectories." << std::endl;
							return false;
						}
						
						Utilities::CleanTrajectory( T );
						return true;
					}
					
					// choose appropriate trajectory
					std::vector<TrajectorySegment*> * PLP;
					if ( maximize_terminal_velocity ) {
						PLP = Reachable.first;
						Utilities::CleanTrajectory( *Reachable.second );
					} else {
						PLP = Reachable.second;
						Utilities::CleanTrajectory( *Reachable.first );
					}
					
					// return it
					T.insert( T.begin(), PLP->begin(), PLP->end() );
					Utilities::translateTrajectory( T, _s.getPathCoord(), _s.getTimeCoord() );
					
					delete( Reachable.first );
					delete( Reachable.second );
					
					return true;
					
				} else if ( s_to_vf_line_relation == Maths::POINT_ON ) {
					
					if ( _edge.contains(_p, c) && _s.canReach(_p, c) ) {
						T.push_back( new TrajectorySegment(_s.getPathCoord(), _s.getTimeCoord(), _s.getVelocityCoord(), _p.getPathCoord(), _p.getTimeCoord(), _s.getVelocityCoord()) );
					} else {
						Utilities::CleanTrajectory( T );
					}
					return true;
					
				} else {
					
					std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Unexpected point/line relation (1)." << std::endl;
					return false;
					
				}

			} else if ( p_to_edge_relation == Maths::POINT_BELOW ) {
				
				// must decelerate onto target velocity line
				if ( s_to_vf_line_relation == Maths::POINT_BELOW ) {

					a2 = 0.5 * c.getAMin();
					
				// open region; attempt PLP trajectory
				} else if ( s_to_vf_line_relation == Maths::POINT_ABOVE ) {

					// get reachable set of velocities
					std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
					Reachable.first = NULL;
					Reachable.second = NULL;
					PVT_Point p1( s.getPathCoord(), s.getTimeCoord() );
					Interval V_i( s.getVelocityCoord(), s.getVelocityCoord() );
					if ( !NextReachableSet(Reachable, p1, p, V_i, c) ) {
						std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Error in NextReachableSet (2)." << std::endl;
						return false;
					}
					
					// target unreachable
					if ( (Reachable.first==NULL) || (Reachable.second==NULL) ) {
						
						// sanity check; this should never fail
						if ( !(Reachable.first==NULL && Reachable.second==NULL) ) {
							std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Unbalanced bounding trajectories." << std::endl;
							return false;
						}
						
						Utilities::CleanTrajectory( T );
						return true;
					}
					
					// choose appropriate trajectory
					std::vector<TrajectorySegment*> * PLP;
					if ( maximize_terminal_velocity ) {
						PLP = Reachable.first;
						Utilities::CleanTrajectory( *Reachable.second );
					} else {
						PLP = Reachable.second;
						Utilities::CleanTrajectory( *Reachable.first );
					}
					
					// return it
					T.insert( T.begin(), PLP->begin(), PLP->end() );
					Utilities::translateTrajectory( T, _s.getPathCoord(), _s.getTimeCoord() );
					
					delete( Reachable.first );
					delete( Reachable.second );
					
					return true;

				} else if ( s_to_vf_line_relation == Maths::POINT_ON ) {
					
					if ( _edge.contains(_p, c) && _s.canReach(_p, c) ) {
						T.push_back( new TrajectorySegment(_s.getPathCoord(), _s.getTimeCoord(), _s.getVelocityCoord(), _p.getPathCoord(), _p.getTimeCoord(), _s.getVelocityCoord()) );
					} else {
						Utilities::CleanTrajectory( T );
					}
					return true;
					
				} else {
					
					std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Unexpected point/line relation (2)." << std::endl;
					return false;
					
				}
				
			} else if ( p_to_edge_relation == Maths::POINT_ON ) {
				
				if ( _edge.contains(_p, c) && _s.canReach(_p, c) ) {
					T.push_back( new TrajectorySegment(_s.getPathCoord(), _s.getTimeCoord(), _s.getVelocityCoord(), _p.getPathCoord(), _p.getTimeCoord(), _s.getVelocityCoord()) );
				} else {
					Utilities::CleanTrajectory( T );
				}
				return true;
				
			} else {
				
				std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Unexpected point/line relation (1)." << std::endl;
				return false;

			}
			
			/**
			 * At this point, the connecting trajectory is of type PL. Find it.
			 */
			
			// find acc./dec. curve tangent to velocity line
			double p1;
			double t1;
			double p2;
			double t2;
			double b2;
			double c2;
			if ( !Maths::tangentParabolaThroughLine(a2, b2, c2, p1, t1, p2, t2, a1, b1, c1, v3, p.getPathCoord(), p.getTimeCoord(), epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "ERROR IN Planner::ConnectEdgeToPoint: Problem finding PL trajectory." << std::endl;
#endif
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			// verify that (p1, t1) is on the edge segment, and that (p2, t2) occurs before p
			if ( Maths::approxLt(t2, t1, epsilon)
				|| Maths::approxGt(t2, p.getTimeCoord(), epsilon)
				|| !edge.contains(p1, t1, c) ) {
				
				// if this fails, it may not be possible to get tangent to the
				// velocity line; in this case, just get a parabola through
				// the target point
				if ( !Maths::tangentParabolaThroughPoint2(a2, b2, c2, p1, t1, a1, b1, c1, p.getPathCoord(), p.getTimeCoord(), epsilon) ) {
					std::cerr << "ERROR IN Planner::ConnectEdgeToPoint: Problem finding P trajectory." << std::endl;
					return false;
				}
				
				// verify crossover point
				if ( Maths::approxGt(t1, p.getTimeCoord(), epsilon) || !edge.contains(p1, t1, c) ) {
					Utilities::CleanTrajectory( T );
					return true;
				}
				
				// verify terminal velocity
				double v_f_ = 2. * a2 * p.getTimeCoord() + b2;
				if ( !V_f.contains(v_f_, c) ) {
					Utilities::CleanTrajectory( T );
					return true;
				}
				
				// build trajectory
				T.push_back( new TrajectorySegment(s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord(), p1, t1, s.getVelocityCoord()) );
				T.push_back( new TrajectorySegment(p1, t1, s.getVelocityCoord(), p.getPathCoord(), p.getTimeCoord(), v_f_) );
				
			} else {
			
				// build trajectory
				T.push_back( new TrajectorySegment(s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord(), p1, t1, s.getVelocityCoord()) );
				T.push_back( new TrajectorySegment(p1, t1, s.getVelocityCoord(), p2, t2, v3) );
				T.push_back( new TrajectorySegment(p2, t2, v3, p.getPathCoord(), p.getTimeCoord(), v3) );
			}
			
			// translate trajectory to original coordinate frame
			Utilities::translateTrajectory( T, _s.getPathCoord(), _s.getTimeCoord() );
			
			return true;
		}
		
		bool ConnectEdgeToGoal( std::vector<TrajectorySegment*>& T,
							   const PVT_State& s,
							   const PVT_ObstacleEdge& edge,
							   const Interval& V_f,
							   bool minimize_arrival_velocity,
							   const Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			// simple LP-
			if ( minimize_arrival_velocity ) {
				
				double a = 0.5 * c.getAMin();
				
				double x0 = s.getPathCoord();
				double t0 = s.getTimeCoord();
				double v1 = s.getVelocityCoord();
				double v2 = V_f.getMin();
				double x2 = c.getXLimit();
				
				double t1 = (1. / v1) * (x2 + v1 * t0 - x0 - (v2 * v2 - v1 * v1) / (4. * a));
				double x1 = Maths::yFromSlopeIntercept( s.getVelocityCoord(), t1, edge.getPathIntercept(), epsilon );
				double t2 = t1 - (v1 - v2) / (2. * a);
				
				// if the edge does not contain the crossover point, quit
				if ( !edge.contains(x1, t1, c) ) {
					return true;
				}
				
				// build and return trajectory
				T.push_back( new TrajectorySegment(s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord(),
												   x1, t1, s.getVelocityCoord()) );
				T.push_back( new TrajectorySegment(x1, t1, s.getVelocityCoord(),
												   x2, t2, v2) );
				return true;
			}
			
			// if maximizing arrival velocity, use standard goal connect from `s'
			PVT_Point p1( s.getPathCoord(), s.getTimeCoord() );
			Interval V_i( s.getVelocityCoord(), s.getVelocityCoord() );
			if ( !BuildGoalPLP( T, p1, V_i, V_f.getMax(), false, c) ) {
				std::cerr << "ERROR IN Planner::ConnectEdgeToGoal: Failed in BuildGoalPLP." << std::endl;
				return false;
			}
			
			return true;
		}
		
		bool GetCollisionBoundary_P2E( std::vector<TrajectorySegment*>& T,
									  const PVT_Point& p1,
									  const Interval& V_i,
									  PVT_ObstacleEdge ** edge,
									  const PVT_ObstacleSet& O,
									  const Constraints& c ) {
			
			// build bounding connection to edge
			std::vector<TrajectorySegment*> T_PointToEdge;
			if ( !ConnectPointToEdge(T_PointToEdge, p1, V_i, **edge, c) ) {
				std::cerr << "ERROR IN GetCollisionBoundaryHelper: ConnectPointToEdge failed." << std::endl;
				return false;
			}
			
			// connection infeasible
			if ( T_PointToEdge.empty() ) {
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			// point is trivially already on the edge
			if ( (T_PointToEdge.size() == 1) && T_PointToEdge.front()->isNullTransition(c) ) {
				T.insert( T.end(), T_PointToEdge.begin(), T_PointToEdge.end() );
				return true;
			}
			
			// collision check trajectory
			PVT_ObstacleEdge * new_edge = NULL;
			std::set<PVT_Obstacle*> inCollision;
			if ( !Collisions::checkTrajectory(inCollision, T_PointToEdge, O, c, false, &new_edge) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundaryHelper: Problem collision checking point to edge connection." << std::endl;
				return false;
			}
			
			// target edge becomes new edge
			if ( new_edge != NULL ) {
				delete( *edge );
				*edge = new_edge;
			}
			
			// feasible connection
			if ( inCollision.empty() ) {
				T.insert( T.end(), T_PointToEdge.begin(), T_PointToEdge.end() );
				return true;
			}
			
			// containment collision, infeasible
			if ( (new_edge == NULL) || new_edge->equals(**edge, c) ) {
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			// recurse with new edge as target
			return GetCollisionBoundary_P2E( T, p1, V_i, edge, O, c );
		}
		
		bool GetCollisionBoundary_E2P( std::vector<TrajectorySegment*>& T,
									  const PVT_Point& p2,
									  PVT_ObstacleEdge ** edge,
									  bool maximize_terminal_velocity,
									  const PVT_ObstacleSet& O,
									  const Constraints& c ) {
			
			// attempt connection from terminal state of current trajectory
			PVT_State s1( T.back()->getFinalState() );
			
			// edge to goal connection
			std::vector<TrajectorySegment*> T_EdgeToPoint;
			if ( !ConnectEdgeToPoint(T_EdgeToPoint, maximize_terminal_velocity, **edge, s1, p2, c) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundaryHelper: ConnectEdgeToPoint failed." << std::endl;
				return false;
			}
			
			// connection infeasible
			if ( T_EdgeToPoint.empty() ) {
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			// collision check trajectory
			PVT_ObstacleEdge * new_edge;
			std::set<PVT_Obstacle*> inCollision;
			if ( !Collisions::checkTrajectory(inCollision, T_EdgeToPoint, O, c, false, &new_edge) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundaryHelper: Problem collision checking edge to point connection." << std::endl;
				return false;
			}
			
			// feasible connection
			if ( inCollision.empty() ) {
				T.insert( T.end(), T_EdgeToPoint.begin(), T_EdgeToPoint.end() );
				return true;
			}
			
			// containment collision
			if ( (new_edge == NULL) || new_edge->equals(**edge, c) ) {
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			// edge to edge connection
			std::vector<TrajectorySegment*> T_EdgeToEdge;
			if ( !ConnectEdgeToEdge( T_EdgeToEdge, s1, **edge, *new_edge, c) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundaryHelper: Problem finding edge to edge connection." << std::endl;
				return false;
			}
			
			// connection infeasible
			if ( T_EdgeToEdge.empty() ) {
				delete( new_edge );
				delete( *edge );
				*edge = NULL;
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			// target edge becomes new source edge
			delete( *edge );
			*edge = new_edge;
			
			// Add edge to edge connection
			T.insert( T.end(), T_EdgeToEdge.begin(), T_EdgeToEdge.end() );
			
			// recurse with new edge as source edge
			return GetCollisionBoundary_E2P( T, p2, edge, maximize_terminal_velocity, O, c );
		}
		
		bool GetCollisionBoundary( std::vector<TrajectorySegment*>& T,
								  std::vector<TrajectorySegment*>& T_B,
								  std::vector<PVT_S*> * S_goal,
								  bool maximize_terminal_velocity,
								  const PVT_Point& p1,
								  const PVT_Point& p2,
								  const Interval& V_i,
								  const Interval& V_f,
								  const PVT_ObstacleSet& O,
								  const std::vector<PVT_ObstaclePoint*> * P_t,
								  const Constraints& c ) {

			// if T_B is collision-free, append to T, return it
			PVT_ObstacleEdge * edge;
			std::set<PVT_Obstacle*> inCollision;
			if ( !Collisions::checkTrajectory(inCollision, T_B, O, c, false, &edge) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundary: Problem collision checking UB." << std::endl;
				return false;
			}
			Utilities::SanitizeTrajectory( T_B, c );
			
			// catch empty trajectories here
			if ( T_B.empty() ) {
				Utilities::CleanTrajectory( T );
				return true;
			}
			
			if ( inCollision.empty() ) {
				T.insert( T.end(), T_B.begin(), T_B.end() );
				T_B.clear();
				return true;
			}
			
			// this trajectory is in collision
			Utilities::CleanTrajectory( T_B );
			
			// if UB is in collision, but not with an edge, there is no feasible trajectory
			if ( edge == NULL ) {
				Utilities::CleanTrajectory( T );
				return true;
			}

			// connect to edge
			if ( !GetCollisionBoundary_P2E(T, p1, V_i, &edge, O, c) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundary: P2E failed." << std::endl;
				return false;
			}
			
			// infeasible
			if ( T.empty() ) {
				return true;
			}
			
			// attempt goal connection from this edge
			if ( S_goal != NULL ) {
				/**/
				std::vector<TrajectorySegment*> UB;
				std::vector<TrajectorySegment*> LB;
				if ( !ConnectEdgeToGoal( UB, T.back()->getFinalState(), *edge, V_f, false, c) ) {
					std::cerr << "ERROR IN Planner::GetCollisionBoundary: E2G failed. (UB)" << std::endl;
					return false;
				}
				if ( !ConnectEdgeToGoal( LB, T.back()->getFinalState(), *edge, V_f, true, c) ) {
					std::cerr << "ERROR IN Planner::GetCollisionBoundary: E2G failed. (LB)" << std::endl;
					return false;
				}
				
				if ( UB.empty() && !LB.empty() ) {
					for ( size_t i=0; i<LB.size(); ++i ) {
						UB.push_back( new TrajectorySegment(*LB[i]) );
					}
				}
				
				if ( LB.empty() && !UB.empty() ) {
					for ( size_t i=0; i<UB.size(); ++i ) {
						LB.push_back( new TrajectorySegment(*UB[i]) );
					}
				}

				// Collision check UB trajectory
				std::set<PVT_Obstacle*> UB_inCollision;
				if ( !Collisions::checkTrajectory(UB_inCollision, UB, O, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: collision check failed (1)" << std::endl;
					return false;
				}
				std::set<PVT_Obstacle*> LB_inCollision;
				if ( !Collisions::checkTrajectory(LB_inCollision, LB, O, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: collision check failed (2)" << std::endl;
					return false;
				}

				// get trajectory signatures
				std::vector<char> BL;
				if ( LB_inCollision.empty() ) {
					if ( !Channel(BL, LB, O, *P_t, c) ) {
						std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (1)" << std::endl;
						std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
						return false;
					}
				}
				std::vector<char> BU;
				if ( UB_inCollision.empty() ) {
					if ( !Channel(BU, UB, O, *P_t, c) ) {
						std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (2)" << std::endl;
						std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
						return false;
					}
				}

#ifdef DEBUG
				if ( !Utilities::ValidTrajectory(UB, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: UB trajectory invalid:" << std::endl;
					Utilities::PrintTrajectory( UB );
					Utilities::CleanTrajectory( UB );
					std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
					return false;
				}
				if ( !Utilities::ValidTrajectory(LB, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: LB trajectory invalid:" << std::endl;
					Utilities::PrintTrajectory( LB );
					Utilities::CleanTrajectory( LB );
					std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
					return false;
				}
#endif
				
				//
				// Add velocity intervals
				//

				// if signatures are equal
				bool sigs_equal = !(BU.empty() || BL.empty()) && (BL.size() == BU.size());
				if ( sigs_equal ) {
					for ( size_t i=0; i<BL.size(); ++i ) {
						if ( BU[i] != BL[i] ) {
							sigs_equal = false;
							break;
						}
					}
				}

				// both bounds reachable
				if ( sigs_equal ) {
					S_goal->push_back( new PVT_S( UB, LB, BL, p1 ) );
#ifdef DEBUG
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(LB) );
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(UB) );
#endif
					
				// only singletons reachable
				} else {
					
					if ( !BL.empty() ) {
						S_goal->push_back( new PVT_S( LB, LB, BL, p1 ) );
#ifdef DEBUG
						matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(LB) );
#endif
					} else {
						Utilities::CleanTrajectory( LB );
					}
					
					if ( !BU.empty() ) {
						S_goal->push_back( new PVT_S( UB, UB, BU, p1 ) );
#ifdef DEBUG
						matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(UB) );
#endif
					} else {
						Utilities::CleanTrajectory( UB );
					}
				}
				/**/
			}
			
			// connect to point
			if ( !GetCollisionBoundary_E2P(T, p2, &edge, maximize_terminal_velocity, O, c) ) {
				std::cerr << "ERROR IN Planner::GetCollisionBoundary: E2P failed." << std::endl;
				return false;
			}
			
			// attempt goal connection from this edge
			if ( !T.empty() && (S_goal != NULL) ) {
				/**/
				std::vector<TrajectorySegment*> UB;
				std::vector<TrajectorySegment*> LB;
				if ( !ConnectEdgeToGoal( UB, T.back()->getFinalState(), *edge, V_f, false, c) ) {
					std::cerr << "ERROR IN Planner::GetCollisionBoundary: E2G failed. (UB)" << std::endl;
					return false;
				}
				if ( !ConnectEdgeToGoal( LB, T.back()->getFinalState(), *edge, V_f, true, c) ) {
					std::cerr << "ERROR IN Planner::GetCollisionBoundary: E2G failed. (LB)" << std::endl;
					return false;
				}

				if ( UB.empty() && !LB.empty() ) {
					for ( size_t i=0; i<LB.size(); ++i ) {
						UB.push_back( new TrajectorySegment(*LB[i]) );
					}
				}
				
				if ( LB.empty() && !UB.empty() ) {
					for ( size_t i=0; i<UB.size(); ++i ) {
						LB.push_back( new TrajectorySegment(*UB[i]) );
					}
				}

				// Collision check UB trajectory
				std::set<PVT_Obstacle*> UB_inCollision;
				if ( !Collisions::checkTrajectory(UB_inCollision, UB, O, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: collision check failed (1)" << std::endl;
					return false;
				}
				std::set<PVT_Obstacle*> LB_inCollision;
				if ( !Collisions::checkTrajectory(LB_inCollision, LB, O, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: collision check failed (2)" << std::endl;
					return false;
				}
				
				// get trajectory signatures
				std::vector<char> BL;
				if ( LB_inCollision.empty() ) {
					if ( !Channel(BL, LB, O, *P_t, c) ) {
						std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (1)" << std::endl;
						std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
						return false;
					}
				}
				std::vector<char> BU;
				if ( UB_inCollision.empty() ) {
					if ( !Channel(BU, UB, O, *P_t, c) ) {
						std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (2)" << std::endl;
						std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
						return false;
					}
				}
				
#ifdef DEBUG
				if ( !Utilities::ValidTrajectory(UB, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: UB trajectory invalid:" << std::endl;
					Utilities::PrintTrajectory( UB );
					Utilities::CleanTrajectory( UB );
					std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
					return false;
				}
				if ( !Utilities::ValidTrajectory(LB, c) ) {
					std::cerr << "ERROR IN Planner::PropagateGoal: LB trajectory invalid:" << std::endl;
					Utilities::PrintTrajectory( LB );
					Utilities::CleanTrajectory( LB );
					std::cerr << "p1: " << p1 << ", V_i: " << V_i << ", V_f: " << V_f << std::endl;
					return false;
				}
#endif
				
				//
				// Add velocity intervals
				//
				
				// if signatures are equal
				bool sigs_equal = !(BU.empty() || BL.empty()) && (BL.size() == BU.size());
				if ( sigs_equal ) {
					for ( size_t i=0; i<BL.size(); ++i ) {
						if ( BU[i] != BL[i] ) {
							sigs_equal = false;
							break;
						}
					}
				}
				
				// both bounds reachable
				if ( sigs_equal ) {
					S_goal->push_back( new PVT_S( UB, LB, BL, p1 ) );
#ifdef DEBUG
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(LB) );
					matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(UB) );
#endif
					
				// only singletons reachable
				} else {
					
					if ( !BL.empty() ) {
						S_goal->push_back( new PVT_S( LB, LB, BL, p1 ) );
#ifdef DEBUG
						matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(LB) );
#endif
					} else {
						Utilities::CleanTrajectory( LB );
					}
					
					if ( !BU.empty() ) {
						S_goal->push_back( new PVT_S( UB, UB, BU, p1 ) );
#ifdef DEBUG
						matlab_bounding_trajectories.push_back( Utilities::CopyTrajectory(UB) );
#endif
					} else {
						Utilities::CleanTrajectory( UB );
					}
				}
				/**/
			}
			
			return true;
		}
		
		bool GoalConnect( std::vector<TrajectorySegment*>& UB,
						 std::vector<TrajectorySegment*>& LB,
						 const PVT_Point& p1,
						 const Interval& V_i,
						 const Interval& V_f,
						 const PVT_ObstacleSet& O,
						 const Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			double x_limit = c.getXLimit();
			double t_limit = c.getTLimit();
			
			// trivial case: point lies in goal region
			if ( Maths::approxEq(p1.getPathCoord(), x_limit, epsilon)
				&& Maths::approxLe(p1.getTimeCoord(), t_limit, epsilon) ) {
				
				// at this point, we're at the goal, verify that velocities are valid
				Interval f_V( V_f );
				f_V.intersect( V_i, c );
				
				// we're at the goal point, but cannot achieve desired velocities
				if ( f_V.isEmpty() ) {
					Utilities::CleanTrajectory( UB );
					Utilities::CleanTrajectory( LB );
					return true;
				}
				
				// add goal transitions
				UB.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), f_V.getMax(),
													p1.getPathCoord(), p1.getTimeCoord(), f_V.getMax()) );
				LB.push_back( new TrajectorySegment(p1.getPathCoord(), p1.getTimeCoord(), f_V.getMin(),
													p1.getPathCoord(), p1.getTimeCoord(), f_V.getMin()) );
			}
			
			double v_g_min = V_f.getMin();
			double v_g_max = V_f.getMax();
			double v_max = c.getVMax();
			double a_min = c.getAMin();
			double a_max = c.getAMax();
			
			double v_f_min = v_g_min;
			double v_f_max = v_g_max;
			
			//
			// From the bound of V_i, calculate the fastest and slowest we can
			// arrive at x_limit while respecting dynamic constraints
			//
			
			double v1_min = V_i.getMin();
			double v1_max = V_i.getMax();
			
			double p_x = p1.getPathCoord();
			double p_t = p1.getTimeCoord();
			
			double delta_x = x_limit - p_x;
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( t_limit - p_t );
			
			//
			// PART 1: First find absolute velocity limits at goal, then
			// intersect with V_f, then compute bounding trajectories from there
			//
			
			// max arrival velocity
			double delta_t = Maths::motionT_FromV1_X1_X2_A( v1_max, p_x, x_limit, a_max, epsilon );
			if ( c_point.validT(delta_t) ) {
				
				// max arrival velocity
				v_f_max = Maths::V2_FromV1_T_A( v1_max, delta_t, a_max, epsilon );
				if ( !c_point.validV(v_f_max) ) {

					// point at which velocity maxes
					double t_i = Maths::T_FromV1_V2_A( v1_max, v_max, a_max, epsilon );
					double x_i = Maths::motionX_FromV1_T1_T2_A( v1_max, 0., t_i, a_max, epsilon );
					
					// linear segment to goal
					double t_f = Maths::T_FromV1_V2_X1_X2( v_max, v_max, 0., delta_x - x_i, epsilon );
					
					// total time
					double delta_t = t_i + t_f;
					
					// if this exceeds bounds, the goal is unreachable
					if ( !c_point.validT(delta_t) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::GoalConnect: Goal unreachable, delta_t " << delta_t << " not valid in [0, " << c_point.getTLimit() << "]" << std::endl;
#endif
						return true;
					}
					
					// Goal is reachabe at max velocity
					v_f_max = v_max;
				}
			}
			
			// min arrival velocity
			std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
			Reachable.first = NULL;
			Reachable.second = NULL;
			PVT_Point p2( x_limit, t_limit );
			if ( !NextReachableSet(Reachable, p1, p2, V_i, c) ) {
				std::cerr << "ERROR IN API:GoalConnect: NextReachableSet failed" << std::endl;
				return false;
			}
			
			// If the time limit cannot be reached with v_min, then the trajectory
			// must hit the x_limit below the t_limit with some v > v_min; find
			// that time
			if ( (Reachable.first == NULL) || (Reachable.second == NULL) ) {
				
				// try decelerating from v1_min first
				double v1 = v1_min;
				double t_test = Maths::motionT_FromV1_X1_X2_A( v1, 0., delta_x, a_min, epsilon );
				if ( !c_point.validT(t_test) ) {
					v1 = v1_max;
					t_test = Maths::motionT_FromV1_X1_X2_A( v1, 0., delta_x, a_min, epsilon );
					
					if ( !c_point.validT(t_test) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::GoalConnect: Goal unreachable, case 2." << std::endl;
#endif
						return true;
					}
				}
				
				// calculate arrival velocity
				v_f_min = Maths::V2_FromV1_T_A( v1, t_test, a_min, epsilon );
				
			} else {
				
				// make sure minium velocity is feasible
				v1_min = std::min( Reachable.first->front()->getInitialState().getVelocityCoord(), Reachable.second->front()->getInitialState().getVelocityCoord() );
				
				// min arrival velocity
				v_f_min = Reachable.second->back()->getFinalState().getVelocityCoord();
			}
			
			// velocity range at goal that is dynamically feasible
			Interval V_g( v_f_min, v_f_max );
			
			V_g.intersect( V_f, c );
			if ( V_g.isEmpty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::GoalConnect: Goal unreachable, case 3." << std::endl;
#endif
				return true;
			}
			
			// now with valid bounds on arrival velocity, compute corresponding trajectories
			v_f_min = V_g.getMin();
			v_f_max = V_g.getMax();
			
			//
			// PART 2: Build representative trajectories that arrive at the goal
			// velocity bounds
			//
			
			// find crossover point with v1 = v1_min, a1 = a_max, a2 = a_min
			if ( !BuildGoalPLP(UB, p1, V_i, v_f_max, false, c) ) {
				std::cerr << "ERROR IN Planner::GoalConnect: BuildGoalPLP failed (1)" << std::endl;
				//std::cerr << p1 << std::endl;
				//std::cerr << c << std::endl;
				return false;
			}
			
			// build trajectory with min arrival velocity
			if ( !BuildGoalPLP(LB, p1, V_i, v_f_min, true, c) ) {
				std::cerr << "ERROR IN Planner::GoalConnect: BuildGoalPLP failed (2)" << std::endl;
				return false;
			}
			
			// free memory used by trajectories
			if ( Reachable.first != NULL ) {
				Utilities::CleanTrajectory( *Reachable.first );
				delete( Reachable.first );
			}
			if ( Reachable.second != NULL ) {
				Utilities::CleanTrajectory( *Reachable.second );
				delete( Reachable.second );
			}
			
#ifdef SHOW_COMMENTS
			std::cout << "Planner::GoalConnect: succeeded." << std::endl;
#endif
			return true;
		}
		
		bool BuildGoalPLP( std::vector<TrajectorySegment*>& T,
						  const PVT_Point& p1,
						  const Interval& V_i,
						  double v2,
						  bool minimize_initial_velocity,
						  const Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			double a_max = c.getAMax();
			double a_min = c.getAMin();
			double v_max = c.getVMax();
			
			double x_limit = c.getXLimit();
			double p_x = p1.getPathCoord();
			double p_t = p1.getTimeCoord();
			
			if ( !c.validX(p_x) || !c.validT(p_t) ) {
				return true;
			}
			
			double delta_x = x_limit - p_x;
			
			double v0 = minimize_initial_velocity ? V_i.getMin() : V_i.getMax();
			
			//
			// Compute bounding trajectories:
			// Assume initial acceleration is a_max, final is a_min. First compute
			// switching time by ignoring velocity constraints along trajectory. If
			// there exists a valid switching time, then check velocity constraint at
			// that time. If it fails, insert a linear segment.
			//
			
			double a1 = a_max;
			double a2 = a_min;
			double qa = a1 * (a2 - a1);
			double qb = 2. * v0 * (a2 - a1);
			double qc = v2 * v2 - v0 * v0 - 2. * a2 * delta_x;
			std::pair<double, double> roots;
			Maths::quadraticOrdered( roots, qa, qb, qc, epsilon );
			double t1 = roots.second;
			if ( Maths::isNaN(t1) ) {
				//std::cerr << "ERROR IN Planner::BuildGoalPLP: Failed computing switching time." << std::endl;
				return false;
			}
			
			// ensure the switch occurs at valid time; this only occurs when
			// the initial velocity is too fast
			t1 = Maths::clipToZero( t1, epsilon );
			if ( t1 < 0. ) {
				t1 = 0.;
				double discriminant = Maths::clipToZero( v2 * v2 - 2. * a2 * delta_x, epsilon );
				if ( discriminant < 0. ) {
					//std::cerr << "ERROR IN Planner::BuildGoalPLP: Unable to compute valid initial velocity." << std::endl;
					return false;
				}
				v0 = sqrt( discriminant );
			}
			
			// make sure this initial velocity is valid
			if ( !V_i.contains(v0, c) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildGoalPLP: Calculated initial velocity " << v0 << " invalid, goal unreachable." << std::endl;
#endif
				return true;
			}
			
			// calculate velocity at switching time
			double v1 = v0 + a1 * t1;

			// if switching velocity is outside bounds, insert linear segment
			if ( !c.validV(v1) ) {

				// build linear segment
				t1 = Maths::T_FromV1_V2_A( v0, v_max, a1, epsilon );
				double t2 = Maths::T_FromV1_V2_A( v_max, v2, a2, epsilon );
				double x2 = Maths::motionX_FromV1_T1_T2_A( v_max, 0., t2, a2, epsilon );
				double x1 = Maths::motionX_FromV1_T1_T2_A( v0, 0., t1, a1, epsilon );
				double delta_x_linear = delta_x - x1 - x2;
				double delta_t_linear = Maths::T_FromV1_V2_X1_X2( v_max, v_max, 0., delta_x_linear, epsilon );
				
				// make sure time is valid
				double total_time = p_t + t1 + delta_t_linear + t2;
				if ( c.validT(total_time) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildGoalPLP: Linear segment case." << std::endl;
#endif
					
					// if we make it here, we're done
					PVT_State s1( p_x, p_t, v0 );
					PVT_State s2( p_x + x1, p_t + t1, v_max );
					PVT_State s3( p_x + x1 + delta_x_linear, p_t + t1 + delta_t_linear, v_max );
					PVT_State s4( p_x + x1 + delta_x_linear + x2, total_time, v2 );
					T.push_back( new TrajectorySegment(s1, s2) );
					T.push_back( new TrajectorySegment(s2, s3) );
					T.push_back( new TrajectorySegment(s3, s4) );
					return true;

				}
			} else {
				double t2 = Maths::T_FromV1_V2_A( v1, v2, a2, epsilon );
				double total_time = p_t + t1 + t2;
				if ( c.validT(total_time) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildGoalPLP: No linear segment case." << std::endl;
#endif
					
					// if we make it here, we're done
					double x1 = Maths::motionX_FromV1_T1_T2_A( v0, 0., t1, a1, epsilon );
					double x2 = Maths::motionX_FromV1_T1_T2_A( v1, 0., t2, a2, epsilon );
					
					PVT_State s1( p_x, p_t, v0 );
					PVT_State s2( p_x + x1, p_t + t1, v1 );
					PVT_State s3( p_x + x1 + x2, total_time, v2 );
					T.push_back( new TrajectorySegment(s1, s2) );
					T.push_back( new TrajectorySegment(s2, s3) );
					return true;
					
				}
			}
			
			//
			// At this point, the time constraint has been violated. The case
			// where an initial velocity is too fast has already been handled,
			// thus, the initial velocity is now too slow. Use NextReachableSet
			// to compute bounding trajectories to the limit point, and return
			// the appropriate one.
			//
			
			// set up problem
			double delta_x_mirrored = x_limit - p_x;
			double delta_t_mirrored = c.getTLimit() - p_t;
			PVT_Point p1_mirrored( 0., 0. );
			PVT_Point p2_mirrored( delta_x_mirrored, delta_t_mirrored );
			Constraints c_mirrored( c, true );
			Interval V_i_mirrored( v2, v2 );
			std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable_mirrored;
			if ( !NextReachableSet(Reachable_mirrored, p1_mirrored, p2_mirrored, V_i_mirrored, c_mirrored) ) {
				std::cerr << "ERROR IN Planner::BuildGoalPLP: Unable to compute adjusted intial velocity. (1)" << std::endl;
				return false;
			}
			if ( (Reachable_mirrored.first==NULL) || (Reachable_mirrored.second==NULL) ) {
				//std::cerr << "ERROR IN Planner::BuildGoalPLP: Unable to compute adjusted intial velocity. (2)" << std::endl;
				//std::cerr << "first: " << Reachable_mirrored.first << ", second: " << Reachable_mirrored.second << std::endl;
				//std::cerr << "delat_x_mirrored: " << delta_x_mirrored << ", delta_t_mirrored: " << delta_t_mirrored << std::endl;
				return true;
			}
			
			double max_i_mirrored = Reachable_mirrored.first->back()->getFinalState().getVelocityCoord();
			double min_i_mirrored = Reachable_mirrored.second->back()->getFinalState().getVelocityCoord();
			Interval V_i_raw( min_i_mirrored, max_i_mirrored );
			
			// release memory
			Utilities::CleanTrajectory( *Reachable_mirrored.first );
			delete( Reachable_mirrored.first );
			Utilities::CleanTrajectory( *Reachable_mirrored.second );
			delete( Reachable_mirrored.second );
			
			// V_i_raw now contains the valid initial velocities for reaching the time limit; intersect with given initial velocities
			Interval V_i_intersected;
			Interval::intersect( V_i_intersected, V_i, V_i_raw, c );
			
			// V_i now contains time limit reachable velocities; build appropriate trajectory
			double v1_final = minimize_initial_velocity ? V_i_intersected.getMin() : V_i_intersected.getMax();
			PVT_State s1( p1.getPathCoord(), p1.getTimeCoord(), v1_final );
			PVT_State s2( c.getXLimit(), c.getTLimit(), v2 );
			if ( !BuildPLP(T, s1, s2, c) ) {
				std::cerr << "ERROR IN Planner::BuildGoalPLP: Unable to compute adjusted initial velocity. (3)" << std::endl;
				return false;
			}
			
			return true;
		}
		
		bool BuildCollisionBoundedPLP( std::vector<TrajectorySegment*>& T,
									  const PVT_State& s1,
									  const PVT_State& s2,
									  const PVT_ObstacleSet& O,
									  const Constraints& c ) {
			
			// first try normal trajectory
			if ( !BuildPLP(T, s1, s2, c) ) {
				std::cerr << "ERROR IN Planner::BuildCollisionBoundedPLP: Error building PLP." << std::endl;
				return false;
			}
			
			PVT_ObstacleEdge * edge;
			std::set<PVT_Obstacle*> inCollision;
			if ( !Collisions::checkTrajectory(inCollision, T, O, c, false, &edge) ) {
				std::cerr << "ERROR IN Planner::BuildCollisionBoundedPLP: Error collision checking PLP." << std::endl;
				return false;
			}
			if ( inCollision.empty() ) {
				return true;
			}
			
			// T is in collision, clear it
			Utilities::CleanTrajectory( T );
			
			// If T is in collision, but not with an edge, no sol
			if ( edge == NULL ) {
				return true;
			}
			
			// There is collision with an edge, attempt collision boundary connection
			
			
			return true;
		}
		
		bool BuildPLP( std::vector<TrajectorySegment*>& T,
					  const PVT_State& s1,
					  const PVT_State& s2,
					  const Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			double a_min = c.getAMin();
			double a_max = c.getAMax();
			double v_min = c.getVMin();
			double v_max = c.getVMax();
			Interval v_feasible( v_min, v_max );
			
			double v1 = s1.getVelocityCoord();
			double v2 = s2.getVelocityCoord();
			
			// average velocity
			double x1 = s1.getPathCoord();
			double t1 = s1.getTimeCoord();
			double x2 = s2.getPathCoord();
			double t2 = s2.getTimeCoord();
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			double v_avg = Maths::avgVelFromX1_X2_T1_T2( x1, x2, t1, t2, epsilon );
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( delta_t );
			
			// quick check of feasibility
			if ( !c_point.validV(v_avg) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildPLP: average velocity makes trajectory infeasible" << std::endl;
#endif
				return true;
			}
			
			// if initial/final/average velocity happen to be the same, we're done
			if ( Maths::approxEq(v1, v2, epsilon) && Maths::approxEq(v2, v_avg, epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildPLP: Case 1." << std::endl;
#endif
				PVT_State s1( x1, t1, v1 );
				PVT_State s2( x2, t2, v2 );
				T.push_back( new TrajectorySegment(s1, s2) );
				return true;
			}
			
			// if final velocity < average velocity, try final P curve - (minus)
			double a2, x_star, t_star;
			if ( Maths::approxLt(v2, v_avg, epsilon) ) {
			
				a2 = a_min;
			
			// otherwise, try + (plus)
			} else {
				
				a2 = a_max;
			}
			
			// origin of final P curve
			std::pair<double, double> origin;
			if ( !Maths::parabolaOriginOffset(origin, v2, a2, epsilon) ) {
				std::cerr << "ERROR IN Planner::BuildPLP: parabolaOriginOffset failed (1)" << std::endl;
				return false;
			}
			
			x_star = delta_x + origin.first;
			t_star = delta_t + origin.second;

			// Decide whether initial acceleration should be + or -
			// If the initial velocity line intersects the acceleration curve,
			// a1 should decelerate; otherwise accelerate
			double a1;
			std::pair<double, double> roots;
			double q_a = a2;
			double q_b = -2. * (a2 * t_star + v1);
			double q_c = 2. * x_star + a2 * t_star * t_star;
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_min;
				} else {
					a1 = a_max;
				}
			} else {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_max;
				} else {
					a1 = a_min;
				}
			}
			
			// try to find the tangent line between curves
			std::pair< std::pair<double, double>, std::pair<double, double> > coords;
			double v_tangent = Maths::parabolasTangentLine1( coords, v1, a1, a2, x_star, t_star, epsilon );
			if ( !Maths::isNaN(coords.first.first) && c.validV(v_tangent) ) {
				double x1_tangent = coords.first.first;
				double t1_tangent = coords.first.second;
				double x2_tangent = coords.second.first;
				double t2_tangent = coords.second.second;
				if ( c_point.validX(x1_tangent) && c_point.validT(t1_tangent)
					&& c_point.validX(x2_tangent) && c_point.validT(t2_tangent) ) {

					if ( v_feasible.contains(v_tangent, c_point) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::BuildPLP: Case 2a." << std::endl;
#endif
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x1_tangent, t1 + t1_tangent, v_tangent );
						PVT_State s3( x1 + x2_tangent, t1 + t2_tangent, v_tangent );
						PVT_State s4( x2, t2, v2 );
						T.push_back( new TrajectorySegment(s1, s2) );
						T.push_back( new TrajectorySegment(s2, s3) );
						T.push_back( new TrajectorySegment(s3, s4) );
						return true;
					}
				}
			}
				
			// if that fails, try without linear segment
			q_a = a1 - a2;
			q_b = 2. * delta_t * (a2 - a1);
			q_c = 2. * delta_x - delta_t * (a2 * delta_t + 2. * v1);
			Maths::quadratic( roots, q_a, q_b, q_c, epsilon );
			if ( !Maths::isNaN(roots.first) ) {
				double t_tangent = roots.first;
				if ( c_point.validT(t_tangent) ) {
					
					double x_tangent = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_tangent, a1, epsilon );
					double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
					double v2_final = Maths::V2_FromV1_T_A( v1 + a1 * t_tangent, delta_t - t_tangent, a2, epsilon );
					if ( Maths::approxEq(v2, v2_final, epsilon) ) {
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x_tangent, t1 + t_tangent, v_tangent );
						PVT_State s3( x2, t2, v2 );
						if ( !s1.equals(s2, c) ) {
							T.push_back( new TrajectorySegment(s1, s2) );
						}
						if ( !s2.equals(s3, c) ) {
							T.push_back( new TrajectorySegment(s2, s3) );
						}
#ifdef SHOW_COMMENTS
						std::cout << "BuildPLP: Case 2b." << std::endl;
#endif
						return true;
					}
				}
			}
			
			// if nothing was found, try swapping the final P curve sign
			if ( Maths::approxLt(a2, 0., epsilon) ) {
				a2 = a_max;
			} else {
				a2 = a_min;
			}
			
			// origin of final P curve
			if ( !Maths::parabolaOriginOffset(origin, v2, a2, epsilon) ) {
				std::cerr << "ERROR IN Planner::BuildPLP: parabolaOriginOffset failed (2)" << std::endl;
				return false;
			}
			
			x_star = delta_x + origin.first;
			t_star = delta_t + origin.second;

			// Decide whether initial acceleration should be + or -
			// If the initial velocity line intersects the acceleration curve,
			// a1 should decelerate; otherwise accelerate
			q_a = a2;
			q_b = -2. * (a2 * t_star + v1);
			q_c = 2. * x_star + a2 * t_star * t_star;
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_min;
				} else {
					a1 = a_max;
				}
			} else {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_max;
				} else {
					a1 = a_min;
				}
			}
			
			// try to find the tangent line between curves
			v_tangent = Maths::parabolasTangentLine1( coords, v1, a1, a2, x_star, t_star, epsilon );
			if ( !Maths::isNaN(coords.first.first) && c.validV(v_tangent) ) {
				double x1_tangent = coords.first.first;
				double t1_tangent = coords.first.second;
				double x2_tangent = coords.second.first;
				double t2_tangent = coords.second.second;
				if ( c_point.validX(x1_tangent) && c_point.validT(t1_tangent)
					&& c_point.validX(x2_tangent) && c_point.validT(t2_tangent) ) {
					if ( v_feasible.contains(v_tangent, c_point) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::BuildPLP: Case 3a." << std::endl;
#endif
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x1_tangent, t1 + t1_tangent, v_tangent );
						PVT_State s3( x1 + x2_tangent, t1 + t2_tangent, v_tangent );
						PVT_State s4( x2, t2, v2 );
						T.push_back( new TrajectorySegment(s1, s2) );
						T.push_back( new TrajectorySegment(s2, s3) );
						T.push_back( new TrajectorySegment(s3, s4) );
						return true;
					}
				}
			}
			// if that fails, try without linear segment
			q_a = a1 - a2;
			q_b = 2. * delta_t * (a2 - a1);
			q_c = 2. * delta_x - delta_t * (a2 * delta_t + 2. * v1);
			Maths::quadratic( roots, q_a, q_b, q_c, epsilon );
			if ( !Maths::isNaN(roots.first) ) {
				double t_tangent = roots.first;
				if ( c_point.validT(t_tangent) ) {
					
					double x_tangent = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_tangent, a1, epsilon );
					double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
					double v2_final = Maths::V2_FromV1_T_A( v1 + a1 * t_tangent, delta_t - t_tangent, a2, epsilon );
					if ( Maths::approxEq(v2, v2_final, epsilon) ) {
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x_tangent, t1 + t_tangent, v_tangent );
						PVT_State s3( x2, t2, v2 );
						if ( !s1.equals(s2, c) ) {
							T.push_back( new TrajectorySegment(s1, s2) );
						}
						if ( !s2.equals(s3, c) ) {
							T.push_back( new TrajectorySegment(s2, s3) );
						}
#ifdef SHOW_COMMENTS
						std::cout << "BuildPLP: Case 3b." << std::endl;
#endif
						return true;
					}
				}
			}

			
#ifdef SHOW_COMMENTS
			std::cout << "Planner::BuildPLP: Unreachable" << std::endl;
#endif
			T.clear();
			return true;
		}
		
		/**
		 * Purely an internal recursive helper for BuildOptimalTrajectory
		 */
		bool BuildOptimalTrajectoryHelper( std::vector<TrajectorySegment*>& T,
										  size_t G_index,
										  const std::vector<PVT_G*>& G,
										  const PVT_ObstacleSet& O,
										  const Constraints& c ) {

			// current state of interest: this is the state we want to connect
			// to from some earlier point in the field
			PVT_State s2( T.front()->getInitialState() );
			PVT_Point p2( s2.getPathCoord(), s2.getTimeCoord(), Interval(s2.getVelocityCoord(), s2.getVelocityCoord()) );
			
			// starting at beginning, look for point nearest origin that connects to p
			for ( size_t i=0; i<G_index; ++i ) {
				
				// point of origin to connect to state s2
				PVT_Point p1( *G[i]->p );
								
				if ( G[i]->V.empty() || p1.equals(p2, c) ) {
					continue;
				}

				// attempt connection from each disjoint interval
				bool found = false;
				std::vector<TrajectorySegment*> T_test;
				for ( size_t j=0; j<G[i]->V.size(); ++j ) {
					
					// velocity interval at p1
					Interval V_i( G[i]->V[j] );
					
					// get nominal bounding trajectories
					std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > BoundingTrajectories;
					BoundingTrajectories.first = NULL;
					BoundingTrajectories.second = NULL;
					if ( !NextReachableSet(BoundingTrajectories, p1, p2, V_i, c) ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectoryHelper: NextReachableSet failed" << std::endl;
						return false;
					}

					if ( (BoundingTrajectories.first == NULL) || (BoundingTrajectories.second == NULL) ) {
						
						// sanity check; this should never happen
						if ( !(BoundingTrajectories.first==NULL & BoundingTrajectories.second==NULL) ) {
							std::cerr << "ERROR IN Planner::BuildOptimalTrajectoryHelper: Unbalanced bounding trajectories." << std::endl;
							return false;
						}
						
						continue;
					}
					
					// get collision bounded trajectories
					std::vector<TrajectorySegment*> T_UB;
					std::vector<TrajectorySegment*> T_LB;
					if ( !GetCollisionBoundary( T_UB, *BoundingTrajectories.first, NULL, true, p1, p2, V_i, V_i, O, NULL, c) ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectoryHelper: Failed finding UB." << std::endl;
						return false;
					}
					if ( !GetCollisionBoundary( T_LB, *BoundingTrajectories.second, NULL, false, p1, p2, V_i, V_i, O, NULL, c) ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectoryHelper: Failed finding LB." << std::endl;
						return false;
					}
					
					Utilities::CleanTrajectory( *BoundingTrajectories.first );
					Utilities::CleanTrajectory( *BoundingTrajectories.second );
					delete( BoundingTrajectories.first );
					delete( BoundingTrajectories.second );
					
#ifdef HALP //DEBUG
					if ( T_UB.empty() != T_LB.empty() ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectoryHelper: Mismatch in bounding trajectories." << std::endl;
						Utilities::CleanTrajectory( T );
						return false;
					}
#else
					if ( T_UB.empty() && !T_LB.empty() ) {
						T_test = T_LB;
						found = true;
						break;
					}
					if ( T_LB.empty() && !T_UB.empty() ) {
						T_test = T_UB;
						found = true;
						break;
					}
#endif

					// break out if you find something
					if ( !T_UB.empty() ) {
						Utilities::CleanTrajectory( T_LB );
						T_test = T_UB;
						found = true;
						break;
					}
					if ( !T_LB.empty() ) {
						Utilities::CleanTrajectory( T_UB );
						T_test = T_LB;
						found = true;
						break;
					}

				}

				// if a connection was found, move on
				if ( found ) {
					T.insert( T.begin(), T_test.begin(), T_test.end() );

#ifdef DEBUG
					if ( !Utilities::ValidTrajectory(T, c) ) {
						Utilities::PrintTrajectory( T_test );
						std::cerr << std::endl << std::endl;
						Utilities::PrintTrajectory( T );
						Utilities::CleanTrajectory( T );
						return false;
					}
#endif

					if ( !BuildOptimalTrajectoryHelper(T, i, G, O, c) ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectoryHelper: Recursion failed." << std::endl;
						return false;
					}
					
					return true;
				}

			}
		
			return true;
			
		}
		
		bool BuildOptimalTrajectory( std::vector<TrajectorySegment*>& T,
									std::vector<PVT_G*>& G,
									std::vector<PVT_S*>& Goal,
									const PVT_ObstacleSet& O,
									const Constraints& c ) {
			T.clear();
			
			if ( Goal.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildOptimalTrajectory: No goal-reachable intervals." << std::endl;
#endif
				return true;
			}
			
			// fastest goal connection, add to T
			size_t fastest_index = 0;
			double fastest_time = std::numeric_limits<double>::max();
			for ( size_t i=0; i<Goal.size(); ++i ) {
				double cur_time =  Goal[i]->UB->back()->getFinalState().getTimeCoord();
				if ( cur_time < fastest_time ) {
					fastest_time = cur_time;
					fastest_index = i;
				}
			}
			for ( size_t i=0; i<Goal[fastest_index]->UB->size(); ++i ) {
				T.push_back( new TrajectorySegment(*Goal[fastest_index]->UB->at(i)) );
			}
			
			// ensure that intervals are sorted
			struct PVT_G_Comparator comp;
			comp.epsilon = c.getEpsilon();
			std::sort( G.begin(), G.end(), comp );
			
			// recurse backwards through field
			if ( !BuildOptimalTrajectoryHelper(T, G.size()-1, G, O, c) ) {
				std::cerr << "ERROR IN Planner::BuildOptimalTrajectory: BuildOptimalTrajectoryHelper failed." << std::endl;
				return false;
			}
			
			// remove redundant states
			Utilities::SanitizeTrajectory( T, c );
			
			if ( !T.empty() && !T.front()->getInitialState().atPoint(0., 0., c) ) {
				std::cerr << "ERROR IN PLANNER::BuildOptimalTrajectory: Failed to reach origin trajectory." << std::endl;
				Utilities::PrintTrajectory( T );
				return false;
			}

			return true;
		}
		
	} // end Planner namespace

} // end PVTP namespace
