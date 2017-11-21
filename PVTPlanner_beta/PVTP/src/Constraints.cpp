#include <cmath>
#include <limits>
#include <PVTP/Maths.hpp>
#include <PVTP/Constraints.hpp>

namespace PVTP {
	
	Constraints::Constraints ( const Constraints& c, bool mirrored ) {
		if ( mirrored ) {
			this->init( c.getXLimit(), c.getTLimit(), c.getVMin(), c.getVMax(), -c.getAMax(), -c.getAMin(), c.getEpsilon() );
		} else {
			this->init( c.getXLimit(), c.getTLimit(), c.getVMin(), c.getVMax(), c.getAMin(), c.getAMax(), c.getEpsilon() );
		}
		
		// init increments epsilon by machine epsilon; undo that, since this is copy constructor
		this->setEpsilon( c.getEpsilon() );
	}
	
	Constraints::Constraints ( double x_limit, double t_limit, double v_min, double v_max, double a_min, double a_max, double epsilon ) {
		if ( epsilon < 0. ) {
			throw Constants::EPSILON_VIOLATION;
		}
		if ( !((a_min < -epsilon) && (a_max > epsilon)) ) {
			throw Constants::ACC_VIOLATION;
		}
		if ( v_min > v_max ) {
			throw Constants::VEL_VIOLATION;
		}
		if ( x_limit < epsilon ) {
			throw Constants::PATH_VIOLATION;
		}
		if ( t_limit <= epsilon ) {
			throw Constants::TIME_VIOLATION;
		}
		this->init( x_limit, t_limit, v_min, v_max, a_min, a_max, epsilon );
	}
	
	void Constraints::init( double x_limit, double t_limit, double v_min, double v_max, double a_min, double a_max, double epsilon ) {
		this->x_limit = x_limit;
		this->t_limit = t_limit;
		this->v_max = v_max;
		this->v_min = v_min;
		this->a_max = a_max;
		this->a_min = a_min;
		
		// Pad epsilon with the machine-dependent rounding error
		epsilon += std::numeric_limits<double>::epsilon();
		
		this->epsilon = epsilon;
	}
	
	std::string Constraints::exceptionMessage( int e ) {
		std::stringstream ss;
		ss << "Constraint construction failed: ";
		switch ( e ) {
			case Constants::ACC_VIOLATION:
				ss << "a_min must be < 0, a_max must be > 0. ";
				break;
			case Constants::VEL_VIOLATION:
				ss << "v_min must be < v_max.";
				break;
			case Constants::PATH_VIOLATION:
				ss << "x_limit must be > 0.";
				break;
			case Constants::TIME_VIOLATION:
				ss << "t_limit must be > 0.";
				break;
			case Constants::EPSILON_VIOLATION:
				ss << "epsilon must be >= 0.";
				break;
			default:
				return "";
		}
		return ss.str();
	}

	std::ostream& operator<<(std::ostream& out, const Constraints& c) {
		return out << "x limit: " << c.getXLimit() << ", t limit: " << c.getTLimit() << ", [v_min, v_max]: [" << c.getVMin() << ", " << c.getVMax() << "], [a_min, a_max]: [" << c.getAMin() << ", " << c.getAMax() << "]" << ", epsilon: " << c.getEpsilon();
	}
	
} // end PVTP namespace