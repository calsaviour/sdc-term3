#include <PVTP/Controller.hpp>

namespace Scenario {
	
	Controller::Controller () {
		this->control_sequence.insert( std::pair<double, double>(0., 0.) );
		this->time_accumulator = 0.;
	}
	
	Controller::Controller ( const Controller& controller ) {
		this->control_sequence = controller.control_sequence;
		this->time_accumulator = controller.time_accumulator;
	}
	
	double Controller::getControl( double time ) {
		
		// we assume non-negative time and at least one control
		if ( (time < 0) || this->control_sequence.empty() ) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		
		// dummy control pair for searching
		std::pair<double, double> search_for( time, 0. );
		
		// attempt insertion of dummy control
		std::pair< std::set< std::pair<double, double>, ControlCompare >::iterator, bool> ret;
		ret = this->control_sequence.insert( search_for );
		
		// control was found, return it
		if ( !ret.second ) {
			return ret.first->second;
		}
		
		// otherwise, get previous control pair
		std::set< std::pair<double, double>, ControlCompare >::iterator it;
		it = ret.first;
		double control = (--it)->second;
		
		// remove dummy insertion
		this->control_sequence.erase( ret.first );
		
		return control;
	}
	
	void Controller::addControl( double time, double control ) {
		
		// we assume non-negative time
		if ( time < 0. ) {
			return;
		}
		
		// build new control
		std::pair<double, double> control_point( time, control );
		
		// get index for control
		std::pair< std::set< std::pair<double, double>, ControlCompare >::iterator, bool> ret;
		ret = this->control_sequence.insert( control_point );

		// replace existing control if times are equal
		if ( !ret.second ) {
			std::pair<double, double> copy = *ret.first;
			copy.second = control_point.second;
			this->control_sequence.erase( ret.first );
			this->control_sequence.insert( copy );
		}

	}

	std::ostream& operator<<( std::ostream& out, const Controller& controller ) {
		std::set< std::pair<double, double>, Controller::ControlCompare >::iterator it;
		if ( controller.control_sequence.empty() ) {
			out << "(empty)" << std::endl;
			return out;
		}
		for ( it = controller.control_sequence.begin(); it != controller.control_sequence.end(); it++ ) {
			out << "[" << it->first << ", " << it->second << "]" << std::endl;
		}
		return out;
	}

} // end Scenario namespace
