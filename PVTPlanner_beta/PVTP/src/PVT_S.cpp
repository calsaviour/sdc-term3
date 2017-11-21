#include <PVTP/Utilities.hpp>
#include <PVTP/PVT_S.hpp>

namespace PVTP {
	
	PVT_S::PVT_S ( const Interval& i ) {
		this->V.setBounds( i.getMin(), i.getMax() );
		this->B = new std::vector<char>();
		this->UB = NULL;
		this->LB = NULL;
		this->p = NULL;
	}

	PVT_S::PVT_S ( const Interval& i, const std::vector<char>& B ) {
		this->V.setBounds( i.getMin(), i.getMax() );
		this->B = new std::vector<char>( B );
		this->UB = NULL;
		this->LB = NULL;
		this->p = NULL;
	}
	
	PVT_S::PVT_S ( const std::vector<TrajectorySegment*>& UB,
				  const std::vector<TrajectorySegment*>& LB,
				  const std::vector<char>& B,
				  const PVT_Point& p ) {
		
		this->B = new std::vector<char>( B );
		
		this->UB = new std::vector<TrajectorySegment*>( UB );
		if ( &UB != &LB ) {
			this->LB = new std::vector<TrajectorySegment*>( LB );
		} else {
			this->LB = new std::vector<TrajectorySegment*>( LB.size() );
			for ( size_t i=0; i<LB.size(); i++ ) {
				this->LB->at(i) = new TrajectorySegment( *LB[i] );
			}
		}
		
		double v_min = LB.back()->getFinalState().getVelocityCoord();
		double v_max = UB.back()->getFinalState().getVelocityCoord();
		this->V.setBounds( v_min, v_max );
		
		this->p = new PVT_Point( p );
	}
	
	PVT_S::PVT_S ( const PVT_S& S ) {
		this->B = new std::vector<char>( *S.B );
		this->V.setBounds( S.V.getMin(), S.V.getMax() );
		if ( S.UB != NULL ) {
			this->UB = new std::vector<TrajectorySegment*>( S.UB->size(), NULL );
			for ( size_t i=0; i<S.UB->size(); i++ ) {
				this->UB->at(i) = new TrajectorySegment( *S.UB->at(i) );
			}
		} else {
			this->UB = NULL;
		}
		if ( S.LB != NULL ) {
			this->LB = new std::vector<TrajectorySegment*>( S.LB->size(), NULL );
			for ( size_t i=0; i<S.LB->size(); i++ ) {
				this->LB->at(i) = new TrajectorySegment( *S.LB->at(i) );
			}
		} else {
			this->LB = NULL;
		}
		this->p = new PVT_Point( *S.p );
	}
	
	PVT_S::~PVT_S () {
		if ( this->UB != NULL ) {
			Utilities::CleanTrajectory( *this->UB );
			delete( this->UB );
		}
		if ( this->LB != NULL ) {
			Utilities::CleanTrajectory( *this->LB );
			delete( this->LB );
		}
		if ( this->p != NULL ) {
			delete( this->p );
		}
		this->B->clear();
		delete( this->B );
	}
	
	void PVT_S::getReachableInterval( Interval& V ) const {
		if ( (this->UB == NULL) && (this->LB == NULL) ) {
			V.setEmpty( true );
			return;
		}
		if ( this->UB == NULL ) {
			Utilities::ExtractFinalVelocityInterval( V, *this->LB, *this->LB );
			return;
		}
		if ( this->LB == NULL ) {
			Utilities::ExtractFinalVelocityInterval( V, *this->UB, *this->UB );
			return;
		}
		Utilities::ExtractFinalVelocityInterval( V, *this->UB, *this->LB );
	}
	
	void PVT_S::setUB( const std::vector<TrajectorySegment*>& UB ) {
		if ( this->UB != NULL ) {
			Utilities::CleanTrajectory( *this->UB );
			this->UB->resize( UB.size() );
		} else {
			this->UB = new std::vector<TrajectorySegment*>( UB.size(), NULL );
		}
		for ( size_t i=0; i<UB.size(); i++ ) {
			this->UB->at(i) = new TrajectorySegment( *UB[i] );
		}
	}
	
	void PVT_S::setLB( const std::vector<TrajectorySegment*>& LB ) {
		if ( this->LB != NULL ) {
			Utilities::CleanTrajectory( *this->LB );
			this->LB->resize( LB.size() );
		} else {
			this->LB = new std::vector<TrajectorySegment*>( LB.size(), NULL );
		}
		for ( size_t i=0; i<LB.size(); i++ ) {
			this->LB->at(i) = new TrajectorySegment( *LB[i] );
		}
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_S& s) {
		out << s.V << " ";
		for ( size_t i=0; i<s.B->size(); i++ ) {
			out << ((s.B->at(i)==0)?"0":"1") << " ";
		}
		return out;
	}
	
} // end PVTP namespace