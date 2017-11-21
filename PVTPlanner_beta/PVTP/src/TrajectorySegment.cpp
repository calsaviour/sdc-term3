#include <PVTP/Maths.hpp>
#include <PVTP/TrajectorySegment.hpp>

namespace PVTP {
	TrajectorySegment::TrajectorySegment ( ) {
		this->init();
	}

	TrajectorySegment::TrajectorySegment ( const PVT_State * initial, const PVT_State * final ) {
		this->initial.setCoords( *initial );
		this->final.setCoords( *final );
		this->init();
	}
	
	TrajectorySegment::TrajectorySegment ( const PVT_State& initial, const PVT_State& final ) {
		this->initial.setCoords( initial );
		this->final.setCoords( final );
		this->init();
	}
	
	TrajectorySegment::TrajectorySegment ( double p_i, double t_i, double v_i, double p_f, double t_f, double v_f ) {
		this->initial.setCoords( p_i, t_i, v_i );
		this->final.setCoords( p_f, t_f, v_f );
		this->init();
	}
	
	TrajectorySegment::TrajectorySegment ( const TrajectorySegment& Ts ) {
		this->initial.setCoords( Ts.initial );
		this->final.setCoords( Ts.final );
		this->init();
	}
	
	std::ostream& operator<<(std::ostream& out, const TrajectorySegment& traj) {
		return out << traj.getInitialState() << " -> " << traj.getFinalState();
	}
	
} // end PVTP namespace
