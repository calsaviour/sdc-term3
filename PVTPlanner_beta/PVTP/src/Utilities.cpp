#include <fstream>
#include <PVTP/Utilities.hpp>
#include <PVTP/ScenarioEvaluator.hpp>

namespace PVTP {
	
	namespace Utilities {
		
		bool IntersectionOfReachableSets( std::vector<PVT_G*>& G_intersect,
										 const std::vector<PVT_G*>& G1,
										 const std::vector<PVT_G*>& G2,
										 const Constraints& c ) {
			
			// these should match exactly
			if ( G_intersect.size() != G1.size() ) {
				std::cerr << "ERROR IN: Utilities::IntersectionOfReachableSets: G_intersect and G1 differ in size: " << G_intersect.size() << " " << G1.size() << std::endl;
				return false;
			}

			// iterate over sets, storing union in G1
			size_t G2_index = 0;
			for ( size_t i=0; i<G1.size(); i++ ) {
				
				// these should match exactly
				if ( !G_intersect[i]->p->equals(*G1[i]->p, c) ) {
					std::cerr << "ERROR IN: Utilities::IntersectionOfReachableSets: G_union and G1, G2 differ at index " << i << "." << std::endl;
					return false;
				}
				
				// verify that we're at the same point
				if ( !G1[i]->p->equals(*G2.at(G2_index)->p, c) ) {
					continue;
				}
				
				// store the intersection in a G_intersect
				Interval::intersect( G_intersect[i]->V, G1[i]->V, G2.at(G2_index)->V, c );
				
				if ( ++G2_index >= G2.size() ) {
					break;
				}
			}
			
			// successful completion
			return true;
		}
		
		void SpecialUnionOfReachableSets( std::vector<PVT_G*>& G1,
										 const std::vector<PVT_G*>& G2,
										 const Constraints& c ) {
			
			// iterate over sets, storing union in G1
			size_t G2_index = 0;
			for ( size_t i=0; i<G1.size(); i++ ) {
				
				// verify that we're at the same point
				if ( !G1[i]->p->equals(*G2.at(G2_index)->p, c) ) {
					continue;
				}

				// create a new vector of intervals that contains those from G1 and G2
				size_t G1_v_size = G1[i]->V.size();
				size_t G2_v_size = G2.at(G2_index)->V.size();
				std::vector<Interval> interval_union;
				for ( size_t j=0; j<G1_v_size; j++ ) {
					interval_union.push_back( G1[i]->V[j] );
				}
				for ( size_t j=0; j<G2_v_size; j++ ) {
					interval_union.push_back( G2.at(G2_index)->V[j] );
				}
				
				// put an empty interval vector into G1
				G1[i]->V.clear();
				
				// perform special union with combined G1 and G2 intervals
				Interval::specialUnionOfIntervals( G1[i]->V, interval_union );
				
				if ( ++G2_index >= G2.size() ) {
					break;
				}
			}
			
			// successful completion
			return;
		}
		
		void TranslateAndMirrorObstacleSet( PVT_ObstacleSet& O_backward,
										   PVT_ObstacleSet& O_forward,
										   const PVT_Point& p,
										   const Constraints& c ) {
			
			// number of obstacles
			size_t obstacle_count = O_forward.obstacles.size();
			
			for ( size_t i=0; i<obstacle_count; i++ ) {
				
				PVT_Obstacle * obs_backwards = new PVT_Obstacle( *O_forward.obstacles[i], c );
				
				for ( size_t j=0; j<obs_backwards->vertices.size(); j++ ) {
					
					PVT_ObstaclePoint * obs_p = obs_backwards->vertices[j];
					
					// mirror about x and t axes
					obs_p->setCoords( -obs_p->getPathCoord(), -obs_p->getTimeCoord() );
					
					// translate s.t. p is origin
					obs_p->translate( p.getPathCoord(), p.getTimeCoord() );
				}

				// add to set
				O_backward.obstacles.push_back( obs_backwards );
			}
			
			// add pseudo obstacle with only one reachable point that will
			// correspond to the origin of the untransformed system
			double obs_coords[4];
			obs_coords[0] = p.getPathCoord();
			obs_coords[1] = c.getXLimit() + 1.;
			obs_coords[2] = -1.;
			obs_coords[3] = p.getTimeCoord();
			
			PVT_Obstacle * obs_origin = new PVT_Obstacle( obs_coords, c );
			
			O_backward.obstacles.push_back( obs_origin );
		}
		
		void MirrorAndTranslateReachableSets( std::vector<PVT_G*>& G, const PVT_Point& p ) {
			
			// the reachable intervals exist in G, but the points have to
			// be transformed back to the original space
			for ( size_t j=0; j<G.size(); j++ ) {
				
				// point with reachable interval
				PVT_ObstaclePoint * p_reachable = G[j]->p;
				
				// first undo mirror about axes
				p_reachable->setCoords( -p_reachable->getPathCoord(), -p_reachable->getTimeCoord() );
				
				// now undo translation
				p_reachable->translate( p.getPathCoord(), p.getTimeCoord() );
			}
			
		}
		
		void CleanResults( std::vector<PVT_G*>& G, std::vector<PVT_S*>& Goal ) {
			CleanG( G );
			CleanGoal( Goal );
		}
		
		void CleanG( std::vector<PVT_G*>& G ) {
			for ( size_t i=0; i<G.size(); i++ ) {
				if ( G[i] != NULL ) {
					delete( G[i] );
				}
			}
			G.clear();
		}
		
		void CleanGoal( std::vector<PVT_S*>& Goal ) {
			for ( size_t i=0; i<Goal.size(); i++ ) {
				delete( Goal[i] );
			}
			Goal.clear();
		}
		
		void CleanTrajectory( std::vector<TrajectorySegment*>& T ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				delete( T[i] );
			}
			T.clear();
		}
		
		std::vector<TrajectorySegment*> * CopyTrajectory( std::vector<TrajectorySegment*>& T ) {
			std::vector<TrajectorySegment*> * new_traj = new std::vector<TrajectorySegment*>();
			for ( size_t i=0; i<T.size(); i++ ) {
				new_traj->push_back( new TrajectorySegment(*T[i]) );
			}
			return new_traj;
		}
		
		void SanitizeTrajectory( std::vector<TrajectorySegment*>& T, const Constraints& c ) {

			std::vector<TrajectorySegment*> _T;
			_T = T;
			T.clear();
			
			for ( size_t i=0; i<_T.size(); ++i ) {
				if ( !_T[i]->isNullTransition(c) ) {
					T.push_back( _T[i] );
				}
			}

		}
		
		void DescribeResults( std::vector<PVT_G*>& G, std::vector<PVT_S*>& Goal, bool verbose ) {
			DescribeG( G, verbose );
			DescribeGoal( Goal, verbose );
		}
		
		void DescribeG( const std::vector<PVT_G*>& G, bool verbose ) {
			int cnt = 0;
			if ( !G.empty() ) {
				for ( size_t i=0; i<G.size(); i++ ) {
					if ( true || !G[i]->V.empty() ) {
						cnt++;
						if ( verbose ) {
							std::cout << "Intervals: ";
							for ( size_t j=0; j<G[i]->V.size(); j++ ) {
								std::cout << G[i]->V[j] << " ";
							}
							std::cout << std::endl << "Point: " << *G[i]->p;
							std::cout << std::endl << std::endl;
						}
					}
				}
			}
			std::cout << "Points with reachable intervals: " << cnt << std::endl;
		}
		
		void DescribeGoal( std::vector<PVT_S*>& Goal, bool verbose ) {
			if ( verbose && !Goal.empty() ) {
				
				// Construct a comparator for sorting
				struct PVT_S_Comparator comp2;
				
				// Sort by value, then by type (min or max) descendingly
				std::sort( Goal.begin(), Goal.end(), comp2 );
				for ( size_t i=0; i<Goal.size(); i++ ) {
					std::cout << "UB: " << std::endl;
					for ( size_t j=0; j<Goal[i]->UB->size(); j++ ) {
						std::cout << *Goal[i]->UB->at(j) << std::endl;
					}
					std::cout << std::endl;
					std::cout << "LB: " << std::endl;
					for ( size_t j=0; j<Goal[i]->LB->size(); j++ ) {
						std::cout << *Goal[i]->LB->at(j) << std::endl;
					}
					std::cout << std::endl;
				}
			}
			std::cout << "Goal-reachable intervals: " << Goal.size() << std::endl;
		}
		
		void PrintTrajectory( const std::vector<TrajectorySegment*>& T ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T[i] << std::endl;
			}
		}
		
		void PrintCollisionSet( const std::set<PVT_Obstacle*>& inCollision ) {
			std::set<PVT_Obstacle*>::iterator it_j;
			if ( inCollision.empty() ) {
				std::cout << "Collision set empty." << std::endl;
			}
			for ( it_j=inCollision.begin(); it_j!=inCollision.end(); it_j++ ) {
				std::cout << **it_j << std::endl;
			}
		}
		
		void PrintSignature( const std::vector<char>& hClass ) {
			for ( size_t i=0; i<hClass.size(); i++ ) {
				std::cout << (int)hClass[i] << " ";
			}
			std::cout << std::endl;
		}
		
		void ExtractInitialVelocityInterval( Interval& I,
											const std::vector<TrajectorySegment*>& UB,
											const std::vector<TrajectorySegment*>& LB ) {
			if ( UB.empty() || LB.empty() ) {
				I.setEmpty( true );
				return;
			}
			double min = LB.front()->getInitialState().getVelocityCoord();
			double max = UB.front()->getInitialState().getVelocityCoord();
			
			// set bounds lazily so that the ordering of the arguments doesn't matter
			I.setBoundsLazy( min, max );
		}
		
		void ExtractFinalVelocityInterval( Interval& I,
										  const std::vector<TrajectorySegment*>& UB,
										  const std::vector<TrajectorySegment*>& LB ) {
			if ( UB.empty() || LB.empty() ) {
				I.setEmpty( true );
				return;
			}
			double min = LB.back()->getFinalState().getVelocityCoord();
			double max = UB.back()->getFinalState().getVelocityCoord();
			
			// set bounds lazily so that the ordering of the arguments doesn't matter
			I.setBoundsLazy( min, max );
		}
		
		double truncateValue( double value, double min, double max ) {
			if ( value < min ) {
				return min;
			}
			if ( value > max ) {
				return max;
			}
			return value;
		}
		
		double trajectoryVelocityAtTime( const std::vector<TrajectorySegment*>& T, double time_delta, const Constraints& c, bool truncate ) {
			if ( T.empty() || (time_delta < 0.) ) {
				return std::numeric_limits<double>::quiet_NaN();
			}

			double total_time = T.back()->getFinalState().getTimeCoord() - T.front()->getInitialState().getTimeCoord();
			
			if ( time_delta > total_time ) {
				if ( truncate ) {
					time_delta = total_time;
				} else {
					return std::numeric_limits<double>::quiet_NaN();
				}
			}
			
			double initial_time = T.front()->getInitialState().getTimeCoord();
			double current_velocity = T.front()->getInitialState().getVelocityCoord();
			for ( size_t i=0; i<T.size(); ++i ) {
				if ( T[i]->isNullTransition(c) ) {
					continue;
				}
				
				double t1 = T[i]->getInitialState().getTimeCoord() - initial_time;
				double t2 = T[i]->getFinalState().getTimeCoord() - initial_time;
				double acc = T[i]->getAcceleration();
				
				// if time is within this segment, we're done
				if ( time_delta <= t2 ) {
					return Maths::V2_FromV1_T_A( current_velocity, time_delta - t1, acc, 0. );
				}
				
				// otherwise move to the end of the segment, update current velocity, repeat
				current_velocity = T[i]->getFinalState().getVelocityCoord();
			}
			
			return current_velocity;
		}
		
		double trajectoryDisplacementAtTime( const std::vector<TrajectorySegment*>& T, double time, const Constraints& c, bool truncate ) {

			if ( T.empty() || (time < 0.) ) {
				return std::numeric_limits<double>::quiet_NaN();
			}
			
			double final_time = T.back()->getFinalState().getTimeCoord();
			if ( time > final_time ) {
				if ( truncate ) {
					time = final_time;
				} else {
					return std::numeric_limits<double>::quiet_NaN();
				}
			}
			
			double current_displacement = T.front()->getInitialState().getPathCoord();
			for ( size_t i=0; i<T.size(); ++i ) {
				if ( T[i]->isNullTransition(c) ) {
					continue;
				}
				
				double t1 = T[i]->getInitialState().getTimeCoord();
				double t2 = T[i]->getFinalState().getTimeCoord();
				double acc = T[i]->getAcceleration();
				double vel = T[i]->getInitialState().getVelocityCoord();
				
				//std::cout << "time: " << time << ", t1: " << t1 << ", t2: " << t2 << std::endl;
				
				// if time is within this segment, we're done
				if ( (time >= t1) && (time <= t2) ) {
					current_displacement += Maths::motionX_FromV1_T1_T2_A( vel, t1, time, acc, 0. );
					break;
				}
				
				// otherwise move to the end of the segment, update displacement, repeat
				current_displacement = T[i]->getFinalState().getPathCoord();
			}
			
			return current_displacement;
		}
		
		void translateTrajectory( std::vector<TrajectorySegment*>& T, double p_offset, double t_offset ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				T[i]->translate( p_offset, t_offset );
			}
		}
		
		bool trajectoryIsExecutable( const std::vector<TrajectorySegment*>& T, double time_step, double range ) {
			
			// for testing
			return true;
			
			if ( T.empty() || (time_step < 0.) || (range < 0.) ) {
				return false;
			}
			if ( range > time_step ) {
				return true;
			}
			
			// get the duration of each segment
			// compute the remainder when divided by the time step
			// compare (time_step - remainder) with range
			for ( size_t i=0; i<T.size(); i++ ) {
				double duration = T[i]->getDuration();
				double from_last_time_step = fmod( duration, time_step );
				double to_next_time_step = time_step - from_last_time_step;
				if ( (from_last_time_step > range) && (to_next_time_step > range) ) {
					return false;
				}
			}
			
			return true;
		}
		
		bool ValidTrajectory( std::vector<TrajectorySegment*>& T, const Constraints& c ) {
			Utilities::SanitizeTrajectory( T, c );
			
			// trivially valid
			if ( T.empty() ) {
				return true;
			}
			
			double last_path_position = 0.;
			double last_time_position = 0.;
			for ( size_t i=0; i<T.size(); i++ ) {
				TrajectorySegment * cur_seg = T[i];
				
				// validate continuity
				if ( i > 0 ) {
					if ( !cur_seg->getInitialState().equals(T[i-1]->getFinalState(), c) ) {
						std::cerr << "ERROR DETECTED IN Utilities::ValidTrajectory: Disjoint segments:" << std::endl;
						std::cerr << *T[i-1] << std::endl;
						std::cerr << *cur_seg << std::endl;
						return false;
					}
				}

				// validate monotonicity
				if ( Maths::approxLt(cur_seg->getInitialState().getPathCoord(), last_path_position, c.getEpsilon())
					|| Maths::approxLt(cur_seg->getInitialState().getTimeCoord(), last_time_position, c.getEpsilon()) ) {
					std::cerr << "ERROR DETECTED IN Utilities::ValidTrajectory: Trajectory violates monotonicity constraints." << std::endl;
					return false;
				}
				last_path_position = cur_seg->getFinalState().getPathCoord();
				last_time_position = cur_seg->getFinalState().getTimeCoord();
				
				// validate first derivative dynamics
				if ( !c.validV(cur_seg->getInitialState().getVelocityCoord()) ) {
					std::cerr << "ERROR DETECTED IN Utilities::ValidTrajectory: Trajectory violates velocity constraints (1), " << cur_seg->getInitialState().getVelocityCoord() << std::endl;
					return false;
				}
				if ( !c.validV(cur_seg->getInitialState().getVelocityCoord()) ) {
					std::cerr << "ERROR DETECTED IN Utilities::ValidTrajectory: Trajectory violates velocity constraints (2), " << cur_seg->getFinalState().getVelocityCoord() << std::endl;
					return false;
				}
				
				// validate second derivative dynamics
				double acc = cur_seg->getAcceleration();
				if ( !c.validA(acc) ) {
					std::cerr << "ERROR DETECTED IN Utilities::ValidTrajectory: Trajectory segment:" << std::endl;
					std::cerr << *cur_seg << std::endl;
					std::cerr << "violates acceleration constraints with acceleration: " << acc << std::endl;
					return false;
				}
				
				// validate motion
				double x = Maths::motionX_FromV1_T1_T2_A( cur_seg->getInitialState().getVelocityCoord(),
														 cur_seg->getInitialState().getTimeCoord(),
														 cur_seg->getFinalState().getTimeCoord(),
														 acc );
				if ( !Maths::approxEq(x, cur_seg->getDisplacement(), c.getEpsilon()) ) {
					std::cerr << "ERROR DETECTED IN Utilities::ValidTrajectory: Trajectory segment:" << std::endl;
					std::cerr << *cur_seg << std::endl;
					std::cerr << "violates displacement constraints, computed: " << x << ", actual: " << cur_seg->getDisplacement() << std::endl;
					return false;
				}
			}
			
			// no problems found
			return true;
		}
		
		bool ControllerFromTrajectory( Scenario::Controller& controller,
									  std::vector<TrajectorySegment*>& T,
									  double time_step,
									  const Constraints& c,
									  double cur_velocity ) {
			
			// verify trajectory
			//if ( !ValidTrajectory(T, c) ) {
			//	return false;
			//}
			Utilities::SanitizeTrajectory( T, c );
			
			// build controller
			controller.reset();
			double cur_time = 0.;
			double T_duration = T.back()->getFinalState().getTimeCoord() - T.front()->getInitialState().getTimeCoord();
			while ( cur_time <= T_duration ) {
				
				// get optimal control
				double control = getOptimalExecutableControl( T, cur_velocity, cur_time, time_step, c );
				
				// truncate it
				control = truncateValue( control, c.getAMin(), c.getAMax() );
				
				// add to controller
				controller.addControl( cur_time, control );

				// maintain velocity
				cur_velocity = Maths::V2_FromV1_T_A( cur_velocity, time_step, control );
				
				// step in time
				cur_time += time_step;
			}
			
			return true;
		}
		
		double getOptimalExecutableControl( const std::vector<TrajectorySegment*>& T,
										   double cur_velocity,
										   double cur_time,
										   double time_step,
										   const Constraints& c ) {
			
			if ( T.empty() || (time_step==0.) ) {
				return 0.;
			}
			
			// get what displacement and velocity would be under perfect control
			double optimal_velocity = trajectoryVelocityAtTime( T, cur_time + time_step, c );
			double optimal_displacement = trajectoryDisplacementAtTime( T, cur_time + time_step, c ) - trajectoryDisplacementAtTime( T, cur_time, c );
			
			return getOptimalExecutableControl( cur_velocity, optimal_displacement, optimal_velocity, time_step );
			
		}
		
		double getOptimalExecutableControl( double initial_velocity, double optimal_displacement, double optimal_velocity, double time_step, double C1, double C2 ) {
			
			// compute executable control that minimizes deviation from vel + displacement
			// Err = optimal_velocity - (v + a * t) + optimal_displacement - (v * t + 1/2 * a * t^2)

			// solve assuming Err = 0
			return (C1 * (optimal_velocity - initial_velocity) + C2 * (optimal_displacement - initial_velocity * time_step)) / (C1 * time_step + 0.5 * C2 * time_step * time_step);
		}
		
		std::string GetMatlabObstacles( const PVT_ObstacleSet& O, double ttc ) {
			std::stringstream ss;
			ss.precision( Constants::DEBUGGING_OUTPUT_PRECISION );
			ss << "O = {};" << std::endl;
			for ( size_t i=0; i<O.obstacles.size(); i++ ) {
				PVT_Obstacle * o = O.obstacles[i];
				ss << "O{" << (i + 1) << "} = [";
				for ( size_t j=0; j<o->vertices.size(); j++ ) {
					ss << o->vertices[j]->getPathCoord() << " " << o->vertices[j]->getTimeCoord() << ";" << std::endl;
				}
				ss << "];" << std::endl;
			}

			ss << "O_ttc = {};" << std::endl;
			ss << "O_ttc2 = {};" << std::endl;
			ss << "O_ttc_barriers = {};" << std::endl;
			if ( ttc > 0. ) {
				for ( size_t i=0; i<O.obstacles.size(); i++ ) {
					PVT_Obstacle * o = O.obstacles[i];
					ss << "O_ttc{" << (i + 1) << "} = [";
					for ( size_t j=0; j<o->vertices.size(); j++ ) {
						ss << o->vertices[j]->getPathCoord() << " " << o->vertices[j]->getTimeCoord() - ttc << ";" << std::endl;
					}
					ss << "];" << std::endl;
					ss << "O_ttc2{" << (i + 1) << "} = [";
					for ( size_t j=0; j<o->vertices.size(); j++ ) {
						ss << o->vertices[j]->getPathCoord() - 3. << " " << o->vertices[j]->getTimeCoord() << ";" << std::endl;
					}
					ss << "];" << std::endl;
					ss << "O_ttc_barriers{" << (i + 1) << "} = [" << (o->getMinPathPoint().getPathCoord() - 3.) << " " << (o->getMinPathPoint().getTimeCoord() - ttc) << ";" << std::endl;
					ss << (o->getMinPathPoint().getPathCoord() - 3.) << " " << o->getMinPathPoint().getTimeCoord() << ";" << std::endl;
					ss << o->getMinPathPoint().getPathCoord() << " " << o->getMinPathPoint().getTimeCoord() << ";" << std::endl;
					ss << o->getMinPathPoint().getPathCoord() << " " << (o->getMinPathPoint().getTimeCoord() - ttc) << "];" << std::endl;
				}
			}
			
			return ss.str();
		}
		
		std::string GetMatlabTrajectory( const std::vector<TrajectorySegment*>& T, std::string T_color, std::string suffix ) {
			std::stringstream ss;
			ss.precision( Constants::DEBUGGING_OUTPUT_PRECISION );
			ss << "T" << suffix << " = [" << std::endl;
			for ( size_t i=0; i<T.size(); i++ ) {
				ss << T[i]->getInitialState().getPathCoord() << " ";
				ss << T[i]->getInitialState().getTimeCoord() << " ";
				ss << T[i]->getInitialState().getVelocityCoord() << ";" << std::endl;
			}
			if ( !T.empty() ) {
				ss << T.back()->getFinalState().getPathCoord() << " ";
				ss << T.back()->getFinalState().getTimeCoord() << " ";
				ss << T.back()->getFinalState().getVelocityCoord();
			}
			ss << "];" << std::endl;
			if ( T_color.empty() ) {
				ss << "drawTrajectory( T" << suffix << " );" << std::endl;
			} else {
				ss << "drawTrajectory( T" << suffix << ", '" << T_color << "' );" << std::endl;
			}
			return ss.str();
		}
		
		void StringExplode( std::string str, std::string separator, std::vector<std::string>& results ) {
			int found = str.find_first_of(separator);
			while ( found != (int)std::string::npos ) {
				if ( found > 0 ) {
					results.push_back(str.substr(0,found));
				}
				str = str.substr( found + 1 );
				found = str.find_first_of( separator );
			}
			if( str.length() > 0 ) {
				results.push_back(str);
			}
		}
		
		std::string GetMatlabReachableSets( const std::vector<PVT_G*> G ) {
			std::stringstream ss;
			for ( size_t i=0; i<G.size(); i++ ) {
				for ( size_t j=0; j<G[i]->V.size(); j++ ) {
					ss << "drawVelocityRange( [" << G[i]->p->getPathCoord() << " " << G[i]->p->getTimeCoord() << "], [" << G[i]->V[j].getMin() << " " << G[i]->V[j].getMax() << "], TEST_MARGINS );" << std::endl;
				}
			}
			return ss.str();
		}
		
		std::string GetMatlabPath( const Scenario::PathModel& path, bool driver_path, bool bike_path ) {
			std::stringstream ss;
			for ( size_t i=0; i<path.segments.size(); i++ ) {
				
				if ( path.segments[i].isVertical() ) {
					ss << "X = [" << path.segments[i].getOrigin().getX() << "; " << path.segments[i].getOrigin().getX() << "];" << std::endl;
					ss << "Y = [" << path.segments[i].getOrigin().getY() << "; " << path.segments[i].getEndPoint().getY() << "];" << std::endl;
				} else {
					ss << "a = " << path.segments[i].coefficients[0] << ";" << std::endl;
					ss << "b = " << path.segments[i].coefficients[1] << ";" << std::endl;
					ss << "c = " << path.segments[i].coefficients[2] << ";" << std::endl;
					ss << "parabola = @(x) a*x.^2 + b*x + c;" << std::endl;
					if ( path.segments[i].getOrigin().getX() > path.segments[i].getEndPoint().getX() ) {
						ss << "X = " << path.segments[i].getEndPoint().getX() << ":0.01:" << path.segments[i].getOrigin().getX() << ";" << std::endl;
					} else {
						ss << "X = " << path.segments[i].getOrigin().getX() << ":0.01:" << path.segments[i].getEndPoint().getX() << ";" << std::endl;
					}
					ss << "Y = parabola(X);" << std::endl;
				}
				//ss << "plot(X, Y, '" << (driver_path?"r":"g") << "', 'LineWidth', 2);" << std::endl;
				//ss << "plot(X, Y, '" << (driver_path?"r":"g") << "');" << std::endl;
				std::string LineWidth = "2";
				if ( driver_path ) {
					ss << "plot(X, Y, 'r', 'LineWidth', " << LineWidth  << ");" << std::endl;
				} else if ( bike_path ) {
					ss << "plot(X, Y, 'b', 'Color', [0 0 0.498], 'LineWidth', " << LineWidth << ");" << std::endl;
				} else {
					ss << "plot(X, Y, 'g', 'Color', [0 0.498 0], 'LineWidth', " << LineWidth << ");" << std::endl;
				}

			}
			return ss.str();
		}
		
		std::string GetMatlabVehicleModel( const Scenario::VehicleModel& vm, bool driver_vehicle ) {
			std::stringstream ss;
			ss << "X = zeros(" << vm.getVertices().size() << ", 1);" << std::endl;
			ss << "Y = zeros(" << vm.getVertices().size() << ", 1);" << std::endl;
			for ( size_t i=0; i<vm.getVertices().size(); i++ ) {
				ss << "X(" << (i + 1) << ", 1) = " << vm.getVertices()[i].getX() << ";" << std::endl;
				ss << "Y(" << (i + 1) << ", 1) = " << vm.getVertices()[i].getY() << ";" << std::endl;
			}
			std::string LineWidth = "2";
			//ss << "h1 = patch( X, Y, " << (driver_vehicle?"[0.8 0.4 0.4]":"[0.8 0.8 0.8]") << ", 'EdgeColor', 'k', 'LineWidth', " << LineWidth << " );" << std::endl;
			//ss << "h1 = patch( X, Y, " << (driver_vehicle?"[0.8 0.4 0.4]":"[0.8 0.8 0.8]") << ", 'EdgeColor', 'k' );" << std::endl;
			ss << "h1 = patch( X, Y, " << (driver_vehicle?"[0.8 0.4 0.4]":"[0.8 0.8 0.8]") << ", 'EdgeColor', 'k', 'LineWidth', " << LineWidth << " );" << std::endl;
			ss << "hasbehavior( h1, 'legend', false );" << std::endl;
			return ss.str();
		}
		
		std::string GetMatlabStandardFigure( bool unit_aspect_ratio ) {
			std::stringstream ss;
			ss << "figure;" << std::endl;
			//ss << "figure('Position', [-505 -30 2048 768]);" << std::endl;
			ss << "hold on;" << std::endl;
			if ( unit_aspect_ratio ) {
				ss << "daspect([1 1 1]);" << std::endl;
			}
			return ss.str();
		}
		
		std::string GetMatlabWorldSceneString( const Scenario::VehicleModel& dm, Scenario::VehicleModel& wo ) {
			std::vector<Scenario::VehicleModel> WO;
			WO.push_back( Scenario::VehicleModel(wo) );
			return GetMatlabWorldSceneString( dm, WO );
		}
		
		std::string GetMatlabWorldSceneString( const Scenario::VehicleModel& dm, std::vector<Scenario::VehicleModel>& WO ) {
			std::stringstream ss;
			
			// get driver path
			std::string driver_path = GetMatlabPath( dm.getPath(), true );
			ss << driver_path.c_str() << std::endl;
			
			for ( size_t i=0; i<WO.size(); i++ ) {
				
				// get obstacle path
				std::string obstacle_path = GetMatlabPath( WO[i].getPath() );
				ss << obstacle_path.c_str() << std::endl;
				
			}

			for ( size_t i=0; i<WO.size(); i++ ) {
				
				// get obstacle model
				std::string obstacle_model = GetMatlabVehicleModel( WO[i] );
				ss << obstacle_model.c_str() << std::endl;
				
			}
			
			// get driver model
			std::string driver_model = GetMatlabVehicleModel( dm, true );
			ss << driver_model.c_str() << std::endl;
			
			return ss.str();
		}
		
		void PlayScenario( std::string output_path,
						  Scenario::VehicleModel& dm,
						  std::vector<Scenario::VehicleModel>& WO,
						  std::vector<TrajectorySegment*>& T,
						  double simulation_time_step,
						  double recording_time_step,
						  const Constraints& c ) {
			if ( T.empty() ) {
				return;
			}
			
			// get figure
			std::string figure = GetMatlabStandardFigure( true );
			
			// set bounds
			std::string bounds = "\n"
			//"x_bounds = [-505 0];\n"
			//"y_bounds = [-20 20];\n"
			"x_bounds = [0 100];\n"
			"y_bounds = [-200 100];\n"
			"xlim(x_bounds);\n"
			"ylim(y_bounds);\n"
			"\n";
			
			// background image
			std::string bg_image = "\n"
			"% Flip the image upside down before showing it\n"
			//"img = imread('bg_image.bmp');\n"
			"img = imread('sr37_bg.jpg');\n"
			"imagesc(x_bounds, y_bounds, flipdim(img,1));\n"
			"\n"
			"% set the y-axis back to normal.\n"
			"set(gca,'ydir','normal');\n"
			"\n";

			// get paths
			std::stringstream ss_paths;
			std::string driver_path = GetMatlabPath( dm.getPath(), true );
			for ( size_t i=0; i<WO.size(); i++ ) {
				std::string obstacle_path = GetMatlabPath( WO[i].getPath() );
				/**
				std::string obstacle_path;
				if ( WO[i].getPath().getLength() > 70. ) {
					obstacle_path = GetMatlabPath( WO[i].getPath(), false, true );
				} else {
					obstacle_path = GetMatlabPath( WO[i].getPath() );
				}
				/**/
				ss_paths << obstacle_path.c_str() << std::endl;
			}
			ss_paths << driver_path.c_str() << std::endl;
			
			// generate controller for driver vehicle
			Scenario::Controller controller;
			ControllerFromTrajectory( controller, T, simulation_time_step, c );
			dm.setController( controller );
			
			// draw one frame for each time step
			double T_duration = T.back()->getFinalState().getTimeCoord() - T.front()->getInitialState().getTimeCoord();
			
			// make sure globals are set
			std::stringstream ss_scene;
			ss_scene << "setup;" << std::endl;
			
			// run through scenario
			size_t counter = 0;
			double cur_time = 0.;
			double record_next_frame = cur_time + recording_time_step;
			while ( cur_time <= T_duration ) {
				
				// forward simulation driver vehicle
				if ( !dm.translateInTime(simulation_time_step) ) {
					break;
				}
				std::string driver_model = GetMatlabVehicleModel( dm, true );
				
				// forward simulate world obstacle vehicles
				std::stringstream WO_models;
				for ( size_t i=0; i<WO.size(); i++ ) {
					WO[i].translateInTime( simulation_time_step );
					WO_models << GetMatlabVehicleModel( WO[i] );
				}
				
				if ( Maths::approxGe(cur_time, record_next_frame, c.getEpsilon()) ) {
					// draw scene
					ss_scene << figure;
					
					ss_scene << "set(gcf,'Visible','off');" << std::endl;
					ss_scene << bounds << std::endl;
					ss_scene << bg_image << std::endl;
					
					ss_scene << ss_paths.str().c_str();
					ss_scene << WO_models.str() << std::endl;
					ss_scene << driver_model << std::endl;
					//ss_scene << "print('-dpng',['data/images/world_scene_', num2str(" << (++counter) << ")]);" << std::endl;
					ss_scene << "display(['Printing ', num2str(" << (counter+1) << ")]);" << std::endl;
					//ss_scene << "screen2png(['data/images/world_scene_', num2str(" << (++counter) << ")]);" << std::endl;
					ss_scene << "screen2eps(['data/images/world_scene_', num2str(" << (++counter) << ")]);" << std::endl;
					ss_scene << "%saveas(gcf, ['data/images/world_scene_', num2str(" << counter << ")], 'fig');" << std::endl;
					ss_scene << "hold off;" << std::endl;
					ss_scene << "close;" << std::endl;
					
					record_next_frame += recording_time_step;
				}
				
				cur_time += simulation_time_step;
			}
			
			// save file
			std::ofstream ofile;
			ofile.open( output_path.c_str() );
			ofile << ss_scene.str();
			ofile.close();
		}
		
		void GetMatlabWorldScene( std::string output_path, const Scenario::VehicleModel& dm, Scenario::VehicleModel& wo ) {
			std::vector<Scenario::VehicleModel> WO;
			WO.push_back( Scenario::VehicleModel(wo) );
			GetMatlabWorldScene( output_path, dm, WO );
		}
		
		void GetMatlabWorldScene( std::string output_path, const Scenario::VehicleModel& dm, std::vector<Scenario::VehicleModel>& WO ) {
			std::stringstream ss;
			ss << GetMatlabStandardFigure( true );
			
			ss << GetMatlabWorldSceneString( dm, WO );
			
			// save file
			std::ofstream ofile;
			ofile.open( output_path.c_str() );
			ofile << ss.str();
			ofile.close();
		}
		
		void GenerateMatlabReachableSets( const std::string output_path, const PVT_ObstacleSet& O, const std::vector<PVT_G*>& G ) {
			
			// get obstacles
			std::string Obs = GetMatlabObstacles( O );
			
			// get reachable sets
			std::string reachable_sets = GetMatlabReachableSets( G );
			
			// generate script
			std::stringstream ss;
			ss << GetMatlabStandardFigure();
			
			// make sure globals are set
			ss << "setup;" << std::endl;
			
			// draw obstacles
			ss << Obs << std::endl;
			ss << "s_cnt = size(O);" << std::endl;
			ss << "cnt = s_cnt(1,2);" << std::endl;
			ss << "for i=1:cnt" << std::endl;
			ss << "clr = i / cnt;" << std::endl;
			ss << "h1 = patch( O{i}(:,1), O{i}(:,2), [0.8 0.8 0.8], 'EdgeColor', 'k' );" << std::endl;
			//ss << "h1 = patch( O{i}(:,1), O{i}(:,2), [clr clr clr], 'EdgeColor', 'k', 'FaceAlpha', 1.0 );" << std::endl;
			ss << "hasbehavior( h1, 'legend', false );" << std::endl;
			ss << "end" << std::endl;
			
			// draw reachable sets
			ss << reachable_sets << std::endl;
			
			// save file
			std::ofstream ofile;
			ofile.open( output_path.c_str() );
			ofile << ss.str();
			ofile.close();
		}
		
		std::string GenerateMatlabSceneString( const PVT_ObstacleSet& O, std::vector< std::vector<TrajectorySegment*>* >& T ) {
			
			std::stringstream ss;
			
			// get obstacles
			std::string Obs = GetMatlabObstacles( O );
			std::string Obs_ttc = GetMatlabObstacles( O, 3. );
			
			// get trajectory
			std::vector<std::string> Trajes;
			for ( size_t i=0; i<T.size(); i++ ) {
				std::ostringstream conv;
				conv << i;
				Trajes.push_back( GetMatlabTrajectory(*T[i], "b", conv.str()) );
			}
			
			// draw trajectories
			for ( size_t i=0; i<Trajes.size(); i++ ) {
				ss << Trajes[i] << std::endl;
			}
			
			// draw obstacles
			ss << Obs << std::endl;
			ss << "s_cnt = size(O);" << std::endl;
			ss << "cnt = s_cnt(1,2);" << std::endl;
			ss << "for i=1:cnt" << std::endl;
			ss << "clr = i / cnt;" << std::endl;
			ss << "h1 = patch( O{i}(:,1), O{i}(:,2), [0.8 0.8 0.8], 'EdgeColor', 'k' );" << std::endl;
			//ss << "h1 = patch( O{i}(:,1), O{i}(:,2), [clr clr clr], 'EdgeColor', 'k', 'FaceAlpha', 1.0 );" << std::endl;
			ss << "hasbehavior( h1, 'legend', false );" << std::endl;
			ss << "end" << std::endl;
			
			return ss.str();
			
		}
		
		void GenerateMatlabScene( std::string output_path, const PVT_ObstacleSet& O, const std::vector<PVT_G*>& G, std::vector< std::vector<TrajectorySegment*>* >& T ) {
			
			// generate script
			std::stringstream ss;
			
			// make sure globals are set
			ss << "setup;" << std::endl;
			ss << GetMatlabStandardFigure() << std::endl;
			ss << GenerateMatlabSceneString( O, T );
			
			// draw reachable sets
			ss << GetMatlabReachableSets( G ) << std::endl;
			
			// save file
			std::ofstream ofile;
			ofile.open( output_path.c_str() );
			ofile << ss.str();
			ofile.close();
			
		}
		
		void GenerateMatlabScene( std::string output_path, const PVT_ObstacleSet& O, std::vector< std::vector<TrajectorySegment*>* >& T ) {
			
			// generate script
			std::stringstream ss;
			
			// make sure globals are set
			ss << "setup;" << std::endl;
			ss << GetMatlabStandardFigure() << std::endl;
			ss << GenerateMatlabSceneString( O, T );
			
			// save file
			std::ofstream ofile;
			ofile.open( output_path.c_str() );
			ofile << ss.str();
			ofile.close();
			
		}
		
		void GenerateMatlabScene( std::string output_path, const PVT_ObstacleSet& O, std::vector<TrajectorySegment*>& T ) {
			std::vector< std::vector<TrajectorySegment*>* > T_list;
			T_list.push_back( &T );
			GenerateMatlabScene( output_path, O, T_list );
		}
		
	} // end Utilities namespace

} // end PVTP namespace