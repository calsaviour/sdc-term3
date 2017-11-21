#ifndef PVTP_TRAJECTORY_SEGMENT_H
#define PVTP_TRAJECTORY_SEGMENT_H

#include <iostream>
#include <PVTP/Maths.hpp>
#include <PVTP/PVT_State.hpp>

namespace PVTP {
	
	/**
	 * Trajectory Segment class. Store an initial PVT state, and a final PVT
	 * state, and provide utility methods for extracting information about the
	 * segment. A "trajectory segment" is defind as a segment of a trajectory
	 * across which constant acceleration is experienced.
	 */
	class TrajectorySegment {
		
		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const TrajectorySegment& traj);
		
	public:
		
		/**
		 * TrajectorySegment constructor: Default.
		 */
		TrajectorySegment ( );

		/**
		 * TrajectorySegment constructor: Constructed states.
		 */
		TrajectorySegment ( const PVT_State * initial, const PVT_State * final );
		
		/**
		 * TrajectorySegment constructor: Specified states.
		 */
		TrajectorySegment ( const PVT_State& initial, const PVT_State& final );
		
		/**
		 * TrajectorySegment constructor: Copy
		 */
		TrajectorySegment ( const TrajectorySegment& Ts );

		/**
		 * TrajectorySegment constructor: Specified coordinates.
		 */
		TrajectorySegment ( double p_i, double t_i, double v_i, double p_f, double t_f, double v_f );
		
		/**
		 * Translate this segment in the path-time plane
		 */
		void translate( double p_offset, double t_offset ) {
			this->initial.translate( p_offset, t_offset );
			this->final.translate( p_offset, t_offset );
		}
		
		/**
		 * Compute acceleration across this segment
		 */
		double getAcceleration( void ) const {
			return this->acceleration;
		}
		
		/**
		 * Get the time duration of this segment
		 */
		double getDuration( void ) const {
			return this->duration;
		}
		
		/**
		 * Get the displacement for this segment
		 */
		double getDisplacement( void ) const {
			return this->displacement;
		}
		
		/**
		 * Compute whether this is a null transition, that is, whether the 
		 * initial and final states are essentially the same.
		 *
		 * A trajectory segment is a null transition iff intial and final times
		 * are approximately equal.
		 */
		bool isNullTransition( const Constraints& c ) const {
			return Maths::approxEq( this->getInitialState().getTimeCoord(), this->getFinalState().getTimeCoord(), c.getEpsilon() );
		}
		
		/**
		 * Accessor for initial state.
		 */
		const PVT_State& getInitialState( void ) const {
			return this->initial;
		}
		
		/**
		 * Accessor for final state.
		 */
		const PVT_State& getFinalState( void ) const {
			return this->final;
		}
		
	private:
		
		/**
		 * Initial state of trajectory segment
		 */
		PVT_State initial;
		
		/**
		 * Final state of trajectory segment
		 */
		PVT_State final;
		
		/**
		 * Storage for computed acceleration
		 */
		double acceleration;
		
		/**
		 * Storage for computed duration
		 */
		double duration;
		
		/**
		 * Storage for computed displacement
		 */
		double displacement;
		
		/**
		 * Convenience method used by constructors
		 */
		void init( void ) {
			this->acceleration = Maths::A_FromV1_V2_T1_T2( this->getInitialState().getVelocityCoord(),
														  this->getFinalState().getVelocityCoord(),
														  this->getInitialState().getTimeCoord(),
														  this->getFinalState().getTimeCoord() );
			this->duration = this->getFinalState().getTimeCoord() - this->getInitialState().getTimeCoord();
			this->displacement = this->getFinalState().getPathCoord() - this->getInitialState().getPathCoord();
		}
		
	};
	
	/**
	 * A function object used as a comparator to sort trajectory sets by a coordinate.
	 *
	 * Sort order: Ascending.
	 */
	struct Trajectory_Comparator {
		double epsilon;
		bool operator() (std::vector<TrajectorySegment*> * A, std::vector<TrajectorySegment*> * B) {
			double valA = A->back()->getInitialState().getPathCoord();
			double valB = B->back()->getInitialState().getPathCoord();
			return Maths::approxLt( valA, valB, epsilon );
		}
	};

} // end PVTP namespace

#endif
