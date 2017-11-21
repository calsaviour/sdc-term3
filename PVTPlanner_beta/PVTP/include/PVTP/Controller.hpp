#ifndef PVTP_CONTROLLER_H
#define PVTP_CONTROLLER_H

#include <set>
#include <limits>
#include <iostream>
#include <vector>

namespace Scenario {
	
	/**
	 * This class acts as a controller for a vehicle object. It provides a
	 * mapping between time and control ("control" being longitudinal control,
	 * acceleration or deceleration), beginning at some fixed point in time.
	 * The unit of time is seconds, and time is tracked on a per-scenario basis;
	 * that is, time 0 occurs for all agents when the scenario begins, and is
	 * measured uniformly across all agents.
	 *
	 * Control is assumed to be continuous and control change discrete and
	 * instantaneous. Thus, the controller stores a time-deliniated sequence of
	 * control changes. If the controller is queried with an arbitrary time t_q,
	 * the control returned belongs to the control pair (t, u) where t <= t_q.
	 *
	 * Times must be >= 0.
	 */
	class Controller {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const Controller& controller );
		
	public:
		
		/**
		 * Assignment operator
		 */
		Controller& operator=( const Controller& rhs ) {
			if ( this != &rhs ) {
				this->control_sequence = rhs.control_sequence;
				this->time_accumulator = rhs.time_accumulator;
			}
			return *this;
		}
		
		/**
		 * Comparator for control set
		 */
		struct ControlCompare {
			bool operator() (const std::pair<double, double>& lhs, const std::pair<double, double>& rhs) const {
				return lhs.first < rhs.first;
			}
		};
		
		/**
		 * Constructor
		 */
		Controller ();
		
		/**
		 * Copy constructor
		 */
		Controller ( const Controller& controller );
		
		/**
		 * Add a time/control pair to the control sequence. If the given time
		 * already exists in the control sequence, it is replaced.
		 */
		void addControl( double time, double control );
		
		/**
		 * Retrieve a control given a time. If a control for the given time is
		 * not explicitly stored in the sequence, return the control from the
		 * next nearest time that does not exceed the given time.
		 */
		double getControl( double time );
		
		/**
		 * Same as above, but takes a time delta and manages an accumulated time
		 * stored internally.
		 */
		double getControlFromTimeDelta( double delta_t ) {
			this->time_accumulator += delta_t;
			return this->getControl( this->time_accumulator );
		}
		
		/**
		 * Resets the time accumulator
		 */
		void reset( void ) {
			this->time_accumulator = 0.;
			this->control_sequence.clear();
		}
		
		/**
		 * Accessor for control sequence
		 */
		const std::set< std::pair<double, double>, ControlCompare >& getControlSequence( void ) const {
			return this->control_sequence;
		}
		
	private:
		
		/**
		 * This vector of pairs is the control sequence, with the first 
		 * member being control time, and the second being control.
		 *
		 * This is stored as a set, so attempting to add a duplicate time will
		 * discard the given time and preserve what is already there.
		 */
		std::set< std::pair<double, double>, ControlCompare > control_sequence;
		
		/**
		 * Time accumulator for getting controls from a time delta
		 */
		double time_accumulator;
		
	};
	
} // end Scenario namespace

#endif
