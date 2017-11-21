#include <utility>
#include <stack>
#include <PVTP/Interval.hpp>
#include <PVTP/Constants.hpp>

namespace PVTP {
	
	Interval::Interval ( void ) {
		this->setEmpty( true );
	}
	
	Interval::Interval ( const Interval& i ) {
		this->empty = i.empty;
		this->min = i.min;
		this->max = i.max;
	}
	
	Interval::Interval ( double min, double max ) {
		this->init( min, max );
	}
	
	void Interval::init( double min, double max ) {
		if ( this->setBounds( min, max ) );
		else
			this->setEmpty( true );
	}
	
	bool Interval::setBounds( double min, double max ) {
		if ( this->boundsValid(min, max) ) {
			this->min = min;
			this->max = max;
			this->setEmpty( false );
			return true;
		}
		return false;
	}
	
	void Interval::setBoundsLazy( double b1, double b2 ) {
		if ( this->boundsValid(b1, b2) ) {
			this->setBounds( b1, b2 );
		} else {
			this->setBounds( b2, b1 );
		}
	}
	
	bool Interval::contains( double num, const Constraints& c ) const {
		if ( this->isEmpty() ) {
			return false;
		}
		double epsilon = c.getEpsilon();
		return Maths::approxGe( num, this->getMin(), epsilon ) && Maths::approxLe( num, this->getMax(), epsilon );
	}
	
	bool Interval::containsOpen( double x, const Constraints& c ) const {
		return this->contains( x, c )
		&& !Maths::approxEq( x, this->getMin(), c.getEpsilon() )
		&& !Maths::approxEq( x, this->getMax(), c.getEpsilon() );
	}
	
	void Interval::intersect( const Interval& B, const Constraints& c ) {
		if ( this->isEmpty() || B.isEmpty() ) {
			this->setEmpty( true );
			return;
		}
		if ( Maths::approxGt(B.getMin(), this->getMax(), c.getEpsilon())
			|| Maths::approxGt(this->getMin(), B.getMax(), c.getEpsilon()) ) {
			this->setEmpty( true );
			return;
		}
		double range_min = std::max( this->getMin(), B.getMin() );
		double range_max = std::min( this->getMax(), B.getMax() );
		this->setBounds( std::min(range_min, range_max), std::max(range_min, range_max) ); // deal with negative ranges
	}
	
	void Interval::intersect( Interval& result, const Interval& A, const Interval& B, const Constraints& c ) {
		if ( A.isEmpty() || B.isEmpty() ) {
			result.setEmpty( true );
			return;
		}
		if ( Maths::approxGt(B.getMin(), A.getMax(), c.getEpsilon())
			|| Maths::approxGt(A.getMin(), B.getMax(), c.getEpsilon()) ) {
			result.setEmpty( true );
			return;
		}
		double range_min = std::max( A.getMin(), B.getMin() );
		double range_max = std::min( A.getMax(), B.getMax() );
		result.init( std::min(range_min, range_max), std::max(range_min, range_max) ); // deal with negative ranges
	}
	
	void Interval::intersect( std::vector<Interval>& result,
							 const Interval& A,
							 const std::vector<Interval>& B,
							 const Constraints& c ) {
		if ( A.isEmpty() ) {
			return;
		}
		for ( size_t i=0; i<B.size(); i++ ) {
			if ( B[i].isEmpty() ) {
				continue;
			}
			Interval B_int;
			intersect( B_int, A, B[i], c );
			if ( B_int.isEmpty() ) {
				continue;
			}
			result.push_back( B_int );
		}
	}
	
	void Interval::intersect( std::vector<Interval>& result,
						  const std::vector<Interval>& A,
						  const std::vector<Interval>& B,
						  const Constraints& c ) {
		if ( A.empty() || B.empty() ) {
			return;
		}
		
		for ( size_t i=0; i<A.size(); i++ ) {
			intersect( result, A[i], B, c );
		}
	}
	
	void Interval::specialUnion( std::pair<Interval, Interval>& result,
								const Interval& A,
								const Interval& B,
								const Constraints& c ) {
		double epsilon = c.getEpsilon();
		if ( A.isSubset(B, c) ) {
			result.first.init( B.getMin(), B.getMax() );
			result.second.setEmpty( true );
			return;
		}
		if ( B.isSubset(A, c) ) {
			result.first.init( A.getMin(), A.getMax() );
			result.second.setEmpty( true );
			return;
		}
		if ( Maths::approxLe(A.getMin(), B.getMin(), epsilon)
			&& Maths::approxGe(A.getMax(), B.getMin(), epsilon) ) {
			result.first.init( A.getMin(), B.getMax() );
			result.second.setEmpty( true );
			return;
		}
		if ( Maths::approxLe(B.getMin(), A.getMin(), epsilon)
			&& Maths::approxGe(B.getMax(), A.getMin(), epsilon) ) {
			result.first.init( B.getMin(), A.getMax() );
			result.second.setEmpty( true );
			return;
		}
		result.first.init( A.getMin(), A.getMax() );
		result.second.init( B.getMin(), B.getMax() );
	}
	
	void Interval::specialUnionOfIntervals( std::vector<Interval>& result,
										   const std::vector<Interval>& intervals ) {
		if ( intervals.empty() ) {
			return;
		}

		size_t i = 0;
		size_t j = 0;
		std::vector< std::pair<double, int> > values( 2 * intervals.size() );
		for ( i=0; i<intervals.size(); i++ ) {
			if ( intervals[i].isEmpty() ) {
				continue;
			}
			values[j++] = std::make_pair( intervals[i].getMin(), Constants::INTERVAL_MIN_MARKER );
			values[j++] = std::make_pair( intervals[i].getMax(), Constants::INTERVAL_MAX_MARKER );
		}
		
		// Construct a comparator for sorting
		struct IntervalComparator comp;
		
		// Sort by value, then by type (min or max) descendingly
		std::sort( values.begin(), values.end(), comp );
		
		// The stack will be used to construct the merged intervals
		std::stack< std::pair<double, int> > Stk;
		for ( size_t i=0; i<values.size(); i++ ) {
			
			// If the stack is empty, add the next value; should always be max
			if ( Stk.empty() ) {
				Stk.push( values.at(i) );
				continue;
			}
			
			// If the stack is not empty, do a peek:
			// If the top is a max value...
			if ( Stk.top().second == Constants::INTERVAL_MAX_MARKER ) {
				
				// ...if the incoming value is a min, pop...
				if ( values[i].second == Constants::INTERVAL_MIN_MARKER ) {
					
					std::pair<double, int> tmp = Stk.top();
					Stk.pop();
					
					// If that pop resulted in an empty stack, save interval
					if ( Stk.empty() ) {
						result.push_back( Interval(values[i].first, tmp.first) );
					}
				
				// ...otherwise, add value to stack
				} else {
					
					Stk.push( values[i] );
					
				}
				
			}
			
			// It should never be the case that a min element is at the top of the stack
		}
		
		// result now contains the merged intervals
	}
	
	double Interval::getSetMin( const std::vector<Interval>& V ) {
		if ( V.empty() ) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		double min = std::numeric_limits<double>::max();
		for ( size_t i=0; i<V.size(); i++ ) {
			if ( V[i].getMin() < min ) {
				min = V[i].getMin();
			}
		}
		return min;
	}
	
	double Interval::getSetMax( const std::vector<Interval>& V ) {
		if ( V.empty() ) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		double max = -std::numeric_limits<double>::max();
		for ( size_t i=0; i<V.size(); i++ ) {
			if ( V[i].getMax() > max ) {
				max = V[i].getMax();
			}
		}
		return max;
	}

	bool Interval::setContains( const std::vector<Interval> V, double val, const Constraints& c ) {
		for ( size_t i=0; i<V.size(); i++ ) {
			if ( V[i].contains(val, c) ) {
				return true;
			}
		}
		return false;
	}
	
	bool Interval::isSubset( const Interval& B, const Constraints& c ) const {
		double epsilon = c.getEpsilon();
		
		if ( this->isEmpty() ) {
			return true;
		}
		if ( B.isEmpty() ) {
			return false;
		}
		if ( Maths::approxGe(this->getMin(), B.getMin(), epsilon)
				&& Maths::approxLe(this->getMax(), B.getMax(), epsilon) ) {
			return true;
		}
		return false;			
	}
	
	bool Interval::boundsValid( double min, double max ) {
		if ( min > max ) {
			return false;
		}
		return true;
	}
	
	void Interval::setEmpty( bool empty ) {
		if ( empty ) {
			this->min = std::numeric_limits<double>::quiet_NaN();
			this->max = std::numeric_limits<double>::quiet_NaN();
		}
		this->empty = empty;
	}
	
	bool Interval::isEqual( const Interval& B, const Constraints& c ) const {
		double epsilon = c.getEpsilon();
		return Maths::approxEq( this->getMin(), B.getMin(), epsilon )
		&& Maths::approxEq( this->getMax(), B.getMax(), epsilon );
	}
	
	std::ostream& operator<<(std::ostream& out, const Interval& i) {
		if ( i.isEmpty() ) {
			return out << "(empty)";
		}
		out << "[";
		if ( i.getMin() == -std::numeric_limits<double>::max() ) {
			out << "-Inf";
		} else {
			out << i.getMin();
		}
		out << ", ";
		if ( i.getMax() == std::numeric_limits<double>::max() ) {
			out << "+Inf";
		} else {
			out << i.getMax();
		}
		out << "]";
		return out;
	}
} // end PVTP namespace