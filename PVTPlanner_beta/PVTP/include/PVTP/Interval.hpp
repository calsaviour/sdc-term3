#ifndef PVTP_INTERVAL_H
#define PVTP_INTERVAL_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <PVTP/Maths.hpp>
#include <PVTP/Constraints.hpp>

namespace PVTP {
	
	/**
	 * Interval class. This class is for contructing and handling intervals and
	 * interval operations. Operations typically take a constraints parameter
	 * in order to perform fuzzy comparisons.
	 */
	class Interval {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const Interval& i);
		
	public:
		
		/**
		 * Assignment operator
		 */
		Interval& operator=( const Interval& rhs ) {
			if ( this != &rhs ) {
				this->min = rhs.min;
				this->max = rhs.max;
				this->empty = rhs.empty;
			}
			return *this;
		}
		
		/**
		 * Interval constructor: Empty interval
		 */
		Interval ( void );
		
		/**
		 * Copy constructor
		 */
		Interval ( const Interval& i );

		/**
		 * Interval contructor: Specified interval
		 */
		Interval ( double min, double max );
		
		/**
		 * Set the bounds of this interval. This assignment fails if the
		 * bounds are invalid.
		 */
		bool setBounds( double min, double max );
		
		/**
		 * Set the bounds of this intervals; auto-determine order.
		 */
		void setBoundsLazy( double b1, double b2 );
		
		/**
		 * Set the minimum bound of this interval
		 */
		void setMin( double min ) {
			this->min = min;
		}
		
		/**
		 * Set the maximum bound of this interval
		 */
		void setMax( double max ) {
			this->max = max;
		}
		
		/**
		 * Whether this interval contains a given real.
		 */
		bool contains( double x, const Constraints& c ) const;
		
		/**
		 * Treats the interval as open when checking for containment.
		 */
		bool containsOpen( double x, const Constraints& c ) const;
		
		/**
		 * Intersect this interval with another interval.
		 */
		void intersect( const Interval& B, const Constraints& c );
		
		/**
		 * Intersect an interval A with another interval B, store in result.
		 */
		static void intersect( Interval& result, const Interval& A, const Interval& B, const Constraints& c );
		
		/**
		 * Intersect an interval with a set of intervals
		 */
		static void intersect( std::vector<Interval>& result,
							  const Interval& A,
							  const std::vector<Interval>& B,
							  const Constraints& c );
		
		/**
		 * Intersect a set of disjoint intervals with a set of disjoing intervals
		 */
		static void intersect( std::vector<Interval>& result,
							  const std::vector<Interval>& A,
							  const std::vector<Interval>& B,
							  const Constraints& c );
		
		/**
		 * Union an interval A with another interval B, store in result.
		 * If A and B are disjoint, the result shall be a disjoint range. For
		 * this reason, the result is a vector of two intervals. If the result
		 * is not disjoing, the second interval is set to empty.
		 */
		static void specialUnion( std::pair<Interval, Interval>& result,
								 const Interval& A,
								 const Interval& B,
								 const Constraints& c );
		
		/**
		 * Merge a set of unions such that the result is a disjoing union of
		 * intervals where the resulting disjoing union contains exactly those
		 * elements that appeared in at least one of the original intervals.
		 *
		 * The resulting intervals get intervals.front()'s constraint set.
		 */
		static void specialUnionOfIntervals( std::vector<Interval>& result,
											const std::vector<Interval>& intervals );
		
		/**
		 * Given a set of velocity intervals, find the minimum.
		 */
		static double getSetMin( const std::vector<Interval>& V );
		
		/**
		 * Given a set of velocity intervals, find the minimum.
		 */
		static double getSetMax( const std::vector<Interval>& V );
		
		/**
		 * Given a set of velocity intervals, check for containment
		 */
		static bool setContains( const std::vector<Interval> V, double val, const Constraints& c );
		
		/**
		 * Determine whether the interval A is a subset of a given interval B
		 */
		bool isSubset( const Interval& B, const Constraints& c ) const;
		
		/**
		 * Test given bounds for validity.
		 */
		bool boundsValid( double min, double max );
		
		/**
		 * Set whether this interval is empty or not
		 */
		void setEmpty( bool empty );
		
		/**
		 * Convenience method to set interval to number line
		 */
		void setFull( void ) {
			this->setBounds( -std::numeric_limits<double>::max(), std::numeric_limits<double>::max() );
		}
		
		/**
		 * Determine whether the interval A is epsilon equal to a given interval B
		 */
		bool isEqual( const Interval& B, const Constraints& c ) const;
		
		/**
		 * Whether this interval is empty
		 */
		bool isEmpty( void ) const {
			return this->empty;
		}
		
		/**
		 * Accessor for min bound
		 */
		double getMin( void ) const {
			return this->min;
		}
		
		/**
		 * Accessor for max bound
		 */
		double getMax( void ) const {
			return this->max;
		}
		
		/**
		 * Get midpoint of interval
		 */
		double getMidPoint( const Constraints& c ) const {
			if ( Maths::approxEq(this->getMax(), this->getMin(), c.getEpsilon()) ) {
				return this->getMax();
			}
			double midpoint = (this->getMax() + this->getMin()) / 2.;
			if ( this->contains(midpoint, c) ) {
				return midpoint;
			}
			return this->getMax();
		}
		
		/**
		 * Initialize an interval object.
		 */
		void init( double min, double max );
	
	private:
		
		/**
		 * The interval minimum
		 */
		double min;
		
		/**
		 * The interval maximum
		 */
		double max;
		
		/**
		 * Whether this interval is empty.
		 */
		bool empty;
	};
	
	/**
	 * A function object used as a comparator in specialUnionOfIntervals.
	 *
	 * Sort order: Descending.
	 */
	struct IntervalComparator {
		bool operator() (std::pair<double, int> A, std::pair<double, int> B) {
			if ( A.first > B.first ) {
				return true;
			} else if ( A.first == B.first ) {
				if ( A.second > B.second ) {
					return true;
				}
			}
			return false;
		}
	};

} // end PVTP namespace

#endif