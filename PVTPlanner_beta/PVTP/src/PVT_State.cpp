#include <PVTP/Constants.hpp>
#include <PVTP/PVT_State.hpp>
#include <PVTP/Maths.hpp>

namespace PVTP {

	PVT_State::PVT_State ( ) {
		this->setCoords( std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN() );
	}
	
	PVT_State::PVT_State ( const PVT_State& s ) {
		this->setCoords( s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord() );
	}
	
	PVT_State::PVT_State ( double p, double t, double v ) {
		this->setCoords( p, t, v );
	}
	
	PVT_State::PVT_State ( const PVT_Point& p, double v ) {
		this->setCoords( p.getPathCoord(), p.getTimeCoord(), v );
	}

	bool PVT_State::canReach( const PVT_Point& p2, const Constraints& c ) const {
		
		double epsilon = c.getEpsilon();
		
		// monotonicity in x
		double x1 = this->getPathCoord();
		double x2 = p2.getPathCoord();
		double delta_x = x2 - x1;
		if ( !c.validX(delta_x) ) {
			return false;
		}
		
		// montonicity in t
		double t1 = this->getTimeCoord();
		double t2 = p2.getTimeCoord();
		double delta_t = t2 - t1;
		if ( !c.validT(delta_t) ) {
			return false;
		}
		
		// no change in position, trivially reachable
		bool delta_x_zero = Maths::approxEq(delta_x, 0., epsilon);
		bool delta_t_zero = Maths::approxEq(delta_t, 0., epsilon);
		if ( delta_x_zero && delta_t_zero ) {
			return true;
		}
		
		// instantaneous change in position, unreachable
		if ( delta_t_zero && !delta_x_zero ) {
			return false;
		}
		
		// average velocity
		double v_avg = delta_x / delta_t;
		if ( !c.validV(v_avg) ) {
			return false;
		}

		// deceleration
		double v1 = this->getVelocityCoord();
		if ( Maths::approxGt(v1, v_avg, epsilon) ) {
			double t_inf = Maths::T_FromV1_V2_A( v1, 0., c.getAMin(), epsilon );
			double x_inf = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_inf, c.getAMin(), epsilon );
			if ( Maths::approxGt(x_inf, c.getXLimit(), epsilon) && Maths::approxLt(t_inf, c.getTLimit(), epsilon) ) {
				return false;
			}
		}
		
		// acceleration
		double a_req = Maths::motionA_FromV1_X1_X2_T1_T2( this->getVelocityCoord(), x1, x2, t1, t2, epsilon );
		if ( !c.validA( a_req ) ) {
#ifdef SHOW_COMMENTS
			std::cout << "a_req: " << a_req << " [" << c.getAMin() << ", " << c.getAMax() << "]" << std::endl;
#endif
			return false;
		}
		
		// p2 is reachable from s1
		return true;
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_State& pvt_s) {
		return out << "(" << pvt_s.getPathCoord() << ", " << pvt_s.getTimeCoord() << ", " << pvt_s.getVelocityCoord() << ")";
	}
} // end PVTP namespace