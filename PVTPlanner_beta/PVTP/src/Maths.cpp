#include <iostream>
#include <stdio.h>
#include <PVTP/Maths.hpp>
#include <PVTP/Constants.hpp>

namespace PVTP {
	
	namespace Maths {
	
		// this method should be redefined as here: http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
		bool approxEq( double A, double B, double epsilon ) {
			double diff = ( A > B ) ? A - B : B - A;
			return fabs(diff) <= epsilon;
		}
		
		double clipToZero( double num, double epsilon ) {
			if ( fabs(num) < epsilon ) {
				return 0.;
			}
			return num;
		}
		
		double circleGeodesic( double target_rotation, double epsilon ) {
			if ( target_rotation > M_PI ) {
				return target_rotation - M_2PI;
			}
			return target_rotation;
		}
		
		double Horners( const std::vector<double>& coefficients, double x, double epsilon ) {
			if ( coefficients.empty() ) {
				return std::numeric_limits<double>::quiet_NaN();
			}
			double result = coefficients[0];
			for ( size_t i=1; i<coefficients.size(); i++ ) {
				result = result * x + coefficients[i];
			}
			return result;
		}
		
		double secondOrderArcLength( double a, double b, double c, double x1, double x2, double epsilon ) {
			
			double L;
			
			if ( Maths::approxEq(a, 0., epsilon) ) {
				
				if ( Maths::approxEq(b, 0., epsilon) ) {

					// infinite solutions
					if ( Maths::approxEq(c, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
						std::cout << "WARNING IN Maths::parabolaArcLength: All coefficients are zero." << std::endl;
#endif
						return std::numeric_limits<double>::quiet_NaN();
					}
					
					// horizontal line
					L = x2 - x1;

				} else {
				
					if ( !Maths::isNaN(b) && !Maths::isNaN(c) ) {

						// sloped line
						double y1 = yFromSlopeIntercept( b, x1, c, epsilon );
						double y2 = yFromSlopeIntercept( b, x2, c, epsilon );
						return L2_Distance( x1, y1, x2, y2, epsilon );
						
					} else {

						// vertical or invalid line
#ifdef SHOW_COMMENTS
						std::cout << "WARNING IN Maths::parabolaArcLength: Vertical or invalid line." << std::endl;
#endif
						return std::numeric_limits<double>::quiet_NaN();

					}

				}

			} else {
				
				// parabolic solution
				double D1 = 2. * a * x1 + b;
				double D2 = 2. * a * x2 + b;
				double S1 = sqrt( 1. + D1 * D1 );
				double S2 = sqrt( 1. + D2 * D2 );

				L = 1. / (4. * a) * ( log( (D2 + S2) / (D1 + S1) ) + D2 * S2 - D1 * S1 );
				
			}

			return fabs( L );
		}
		
		double slope( double x1, double y1, double x2, double y2, double epsilon ) {
			double delta_x = x2 - x1;
			double delta_y = y2 - y1;

			if ( Maths::approxEq(delta_x, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::slope: denominator 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			if ( Maths::approxEq(delta_y, 0., epsilon) ) {
				return 0.;
			}

			return delta_y / delta_x;
		}
		
		double xFromSlopeIntercept( double m, double y, double b, double epsilon ) {
			if ( approxEq(m, 0., epsilon) ) {
				return std::numeric_limits<double>::quiet_NaN();
			}
			if ( approxEq(y, b, epsilon) ) {
				return 0.;
			}
			return (y - b) / m;
		}
		
		void lineIntersection( double& x, double& y, double m1, double b1, double m2, double b2, double epsilon ) {
			if ( approxEq(m1, m2, epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::lineIntersection: parallel lines." << std::endl;
#endif
				x = std::numeric_limits<double>::quiet_NaN();
				y = std::numeric_limits<double>::quiet_NaN();
				return;
			}
			
			x = (b2 - b1) / (m1 - m2);
			y = Maths::yFromSlopeIntercept( m1, x, b1, epsilon );
		}

		int lineParabolaIntersection( double& _x, double x, double m_line, double b_line, double a, double b, double c, double epsilon ) {
			
			// determine appropriate coefficients
			double q_a = a;
			double q_b = b - m_line;
			double q_c = c - x;
			
			std::pair<double, double> roots;
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			// no intersection
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
				return INTERSECTION_NONE;
			}
			
			// tangent line
			if ( Maths::approxEq(roots.first, roots.second, epsilon) ) {
				_x = roots.first;
				return INTERSECTION_UNIQUE;
			}
			
			// cuts across parabola
			return INTERSECTION_NON_UNIQUE;
		}
		
		double avgVelFromX1_X2_T1_T2( double x1, double x2, double t1, double t2, double epsilon) {
			double delta_t = t2 - t1;

			if ( Maths::approxEq(delta_t, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::avgVelFromX1_X2_T1_T2: delta_t = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (x2 - x1) / delta_t;
		}
		
		double parabolaDelX_FromV1_V2_A( double v1, double v2, double acc, double epsilon ) {

			if ( Maths::approxEq(acc, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolaInflectionPoint: a = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (v1 * v1 -  v2 * v2) / (2. * acc);
		}
		
		bool parabolaCoefficients1( double& a,
								  double& b,
								  double& c,
								  double x1,
								  double t1,
								  double v1,
								  double t2,
								  double v2,
								  double epsilon ) {
			
			double numerator = v2 - v1;
			double denominator = t2 - t1;
			
			if ( Maths::approxEq(denominator, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolaCoefficients: division by zero" << std::endl;
#endif
				return false;
			}
			
			double frac = numerator / denominator;
			
			a = 0.5 * frac;
			b = v1 - frac * t1;
			c = x1 - t1 * (a * t1 + b);
			
			return true;
		}
		
		void quadratic( std::pair<double, double>& roots, double a, double b, double c, double epsilon ) {
			quadraticOrdered( roots, a, b, c, epsilon );
			double x1 = roots.first;
			double x2 = roots.second;

			// by convention, if both are positive, first < second
			if ( !signbit(x1) && !signbit(x2) ) {
				if ( x1 > x2 ) {
					roots.first = x2;
					roots.second = x1;
				} else {
					roots.first = x1;
					roots.second = x2;
				}
				return;
			}
			
			// by convention, positive or real root first
			if ( !signbit(x1) || isNaN(x2) ) {
				roots.first = x1;
				roots.second = x2;
				return;
			}
			
			// otherwise, if some mix of negatives or NaN, order unimportant
			roots.first = x2;
			roots.second = x1;
		}
		
		void quadraticOrdered( std::pair<double, double>& roots, double a, double b, double c, double epsilon ) {
			
			// trivial case
			if ( Maths::approxEq(a, 0., epsilon) && Maths::approxEq(b, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::quadratic: coeffients are zero" << std::endl;
#endif
				roots.first = std::numeric_limits<double>::quiet_NaN();
				roots.second = std::numeric_limits<double>::quiet_NaN();
				return;
			}
			
			double q, x1, x2;
			
			// linear case
			if ( Maths::approxEq(a, 0., epsilon) ) {
				x1 = -c / b;
				x2 = x1;
				roots.first = x2;
				roots.second = x1;
				return;
			}
			
			// sort of linear case
			if ( Maths::approxEq(c, 0., epsilon) ) {
				if ( b < 0. ) {
					roots.second = 0.;
					roots.first = -b / a;
				} else {
					roots.first = 0.;
					roots.second = -b / a;
				}
				return;
			}
			
			// simple squared case
			double discriminant;
			if ( Maths::approxEq(b, 0., epsilon) ) {
				discriminant = -a * c;
				if ( discriminant < 0. ) {
					discriminant = clipToZero( -c / a, epsilon );
					if ( discriminant < 0. ) {
#ifdef SHOW_COMMENTS
						std::cout << "WARNING IN Maths::quadratic: discriminant < 0 (1)" << std::endl;
#endif
						roots.first = std::numeric_limits<double>::quiet_NaN();
						roots.second = std::numeric_limits<double>::quiet_NaN();
						return;
					}
				}
				x1 = sqrt(discriminant) / a;
				x2 = -x1;
				roots.first = x1;
				roots.second = x2;
				return;
			}
			
			// quadratic case
			discriminant = b * b - 4 * a * c;
			if ( discriminant < 0. ) {
				discriminant = clipToZero( discriminant, epsilon );
				if ( discriminant < 0. ) {
#ifdef SHOW_COMMENTS
					std::cout << "WARNING IN Maths::quadratic: discriminant = " << discriminant << " < 0 (2)" << std::endl;
#endif
					roots.first = std::numeric_limits<double>::quiet_NaN();
					roots.second = std::numeric_limits<double>::quiet_NaN();
					return;
				}
			}
			
			// UNIQUE SOLUTION
			if ( Maths::approxEq(discriminant, 0., epsilon) ) {
				x1 = -b / (2. * a);
				x2 = x1;
				roots.first = x2;
				roots.second = x1;
				return;
			}
			
			// NON-UNIQUE SOLUTION
			q = -0.5 * (b + ((b<0.)?-1.:1.) * sqrt(discriminant));
			
			if ( b < 0. ) {
				roots.first = q / a;
				roots.second = c / q;
			} else {
				roots.first = c / q;
				roots.second = q / a;
			}
			return;
		}
		
		double motionV1_FromX1_X2_T1_T2_A( double x1, double x2, double t1, double t2, double acc, double epsilon ) {
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			if ( Maths::approxEq(delta_t, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::motionV_FromX1_X2_T1_T2_A: delta_t == 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (delta_x / delta_t) - 0.5 * acc * delta_t;
		}
		
		double motionA_FromV1_X1_X2_T1_T2( double v1, double x1, double x2, double t1, double t2, double epsilon ) {
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			if ( approxEq(delta_t, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::motionA_FromV1_X1_X2_T1_T2: delta_t == 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return 2. * (delta_x - v1 * delta_t) / (delta_t * delta_t);
		}
		
		double motionT_FromV1_X1_X2_A( double v1, double x1, double x2, double acc, double epsilon ) {
			std::pair<double, double> roots;
			quadratic( roots, acc, 2. * v1, -2. * (x2 - x1), epsilon );
			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::motionT_FromV1_X1_X2_A: No solution to quadratic.";
#endif
			}
			return roots.first;
		}
		
		double A_FromV1_V2_T1_T2( double v1, double v2, double t1, double t2, double epsilon ) {
			double delta_v = clipToZero( v2 - v1, epsilon );
			double delta_t = clipToZero( t2 - t1, epsilon );
			
			if ( Maths::approxEq(delta_t, 0., epsilon) ) {

				if ( Maths::approxEq(delta_v, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
					std::cout << "WARNING IN Maths::A_FromV1_V2_T1_T2: delta_t = 0, delta_v != 0" << std::endl;
#endif
					return std::numeric_limits<double>::quiet_NaN();
				}
				return 0.;
			}
			
			return delta_v / delta_t;
		}
		
		double T_FromV1_V2_A( double v1, double v2, double acc, double epsilon ) {
			if ( Maths::approxEq(acc, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::T_FromV1_V2_A: acc = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (v2 - v1) / acc;
		}
		
		int pointParabolaRelation( double p, double t, double a, double b, double c, double epsilon ) {
			double p_test = parabolaCanonical( a, b, c, t, epsilon );
			if ( approxLt(p_test, p, epsilon) ) {
				return POINT_ABOVE;
			}
			if ( approxGt( p_test, p, epsilon) ) {
				return POINT_BELOW;
			}
			return POINT_ON;
		}
		
		int pointLineRelation( double p, double t, double b, double c, double epsilon ) {
			double p_test = yFromSlopeIntercept( b, t, c, epsilon );
			if ( approxLt(p_test, p, epsilon) ) {
				return POINT_ABOVE;
			}
			if ( approxGt( p_test, p, epsilon) ) {
				return POINT_BELOW;
			}
			return POINT_ON;
		}
		
		void tangentLineToParabolas( double& x1,
							   double& y1,
							   double& x2,
							   double& y2,
							   double a1,
							   double b1,
							   double c1,
							   double a2,
							   double b2,
							   double c2,
							   double epsilon ) {
			x1 = std::numeric_limits<double>::quiet_NaN();
			y1 = std::numeric_limits<double>::quiet_NaN();
			x2 = std::numeric_limits<double>::quiet_NaN();
			y2 = std::numeric_limits<double>::quiet_NaN();
			
			if ( approxEq(a1, 0., epsilon) || approxEq(a2, 0., epsilon) ) {
				return;
			}
			
			double b2_minus_b1 = b2 - b1;
			double a2_over_a1 = a2 / a1;
			
			double q_a = a2 - a2 * a2_over_a1;
			double q_b = -a2_over_a1 * b2_minus_b1;
			double q_c = -b2_minus_b1 * b2_minus_b1 / (4. * a1) + c1 - c2;
			std::pair<double, double> roots;
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			x2 = (a2 < 0.) ? roots.second : roots.first;
			y2 = parabolaCanonical( a2, b2, c2, x2, epsilon );
			x1 = (2. * a2 * x2 + b2_minus_b1) / (2. * a1);
			y1 = parabolaCanonical( a1, b1, c1, x1, epsilon );
		}
		
		void tangentLineThroughPointToParabola1( double& x_tangent,
												double& y_tangent,
												double a,
												double b,
												double c,
												double x,
												double y,
												double epsilon ) {
			x_tangent = std::numeric_limits<double>::quiet_NaN();
			y_tangent = std::numeric_limits<double>::quiet_NaN();
			
			double q_a = a;
			double q_b = -2. * a * y;
			double q_c = -c + x - b * y;
			std::pair<double, double> roots;
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			x_tangent = (a < 0.) ? roots.second : roots.first;
			y_tangent = parabolaCanonical( a, b, c, x_tangent, epsilon );
		}
		
		void tangentLineThroughPointToParabola2( double& x_tangent,
												double& y_tangent,
												double a,
												double& b,
												double& c,
												double x1,
												double y1,
												double v1,
												double x2,
												double y2,
												double epsilon ) {
			x_tangent = std::numeric_limits<double>::quiet_NaN();
			y_tangent = std::numeric_limits<double>::quiet_NaN();
			b = std::numeric_limits<double>::quiet_NaN();
			c = std::numeric_limits<double>::quiet_NaN();
			
			double q_a = a;
			double q_b = -2. * a * x2;
			double q_c = a * x2 * x2 + v1 * (x2 - x1) - y2 + y1;
			std::pair<double, double> roots;
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			x_tangent = (a < 0.) ? roots.first : roots.second;
			y_tangent = v1 * x_tangent + y1 - v1 * x1;
			b = v1 - 2. * a * x_tangent;
			c = y2 - x2 * (a * x2 + b);
		}
		
		bool tangentParabolaThroughLine( double a2,
										double& b2,
										double& c2,
										double& p1,
										double& t1,
										double& p2,
										double& t2,
										double a1,
										double b1,
										double c1,
										double b3,
										double p3,
										double t3,
										double epsilon ) {
			// initialize remaining coefficients
			b2 = std::numeric_limits<double>::quiet_NaN();
			c2 = std::numeric_limits<double>::quiet_NaN();
			p1 = std::numeric_limits<double>::quiet_NaN();
			t1 = std::numeric_limits<double>::quiet_NaN();
			p2 = std::numeric_limits<double>::quiet_NaN();
			t2 = std::numeric_limits<double>::quiet_NaN();
			double c3 = p3 - b3 * t3;
			
			if ( Maths::approxEq(a1, a2, epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::tangentParabolaThroughLine: a1 == a2" << std::endl;
#endif
				return false;
			}
			
			// find the crossover time for the intersection of the reaction parabola with the linear segment
			std::pair<double, double> roots;
			double b3_minus_b1 = b3 - b1;
			double q_a = a1 * a2;
			double q_b = -a2 * b3_minus_b1;
			double q_c = 0.25 * b3_minus_b1 * b3_minus_b1 + (a1 - a2) * (c3 - c1);
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::tangentParabolaThroughLine: Invalid intersection." << std::endl;
				std::cout << roots.first << " " << roots.second << std::endl;
#endif
				return false;
			}
			
			// determine appropriate intersection time
			if ( a2 < 0. ) {
				t2 = roots.second;
			} else {
				t2 = roots.first;
			}
			
			// compute remaining coefficients
			b2 = yFromSlopeIntercept( -2. * a2, t2, b3, epsilon );
			c2 = parabolaCanonical( -a2, -(b2 - b3), c3, t2, epsilon );
			t1 = (b3_minus_b1 - 2. * a2 * t2) / (2. * (a1 - a2));
			p1 = parabolaCanonical( a1, b1, c1, t1, epsilon );
			p2 = yFromSlopeIntercept( b3, t2, c3, epsilon );
			
			return true;
		}
		
		void tangentParabolaThroughPoint1( double a2,
										  double& b2,
										  double& c2,
										  double& x_switch,
										  double& y_switch,
										  double a1,
										  double b1,
										  double c1,
										  double v_i,
										  double x1,
										  double y1,
										  double x2,
										  double y2,
										  double epsilon ) {
			// initialize remaining coefficients
			b2 = std::numeric_limits<double>::quiet_NaN();
			c2 = std::numeric_limits<double>::quiet_NaN();
			
			double a1_minus_a2 = a1 - a2;
			
			double q_a = -a1_minus_a2;
			double q_b = 2. * a1_minus_a2 * x2;
			double q_c = y1 - x1 * (a1 * x1 + b1) - y2 + x2 * (a2 * x2 + b1);
			
			//q_a = -(a1 - a2);
			//q_b = 2 * (a1 - a2) * t_f;
			//q_c = x0 - a1 * t0 * t0 - b1 * t0 - x_f + a2 * t_f * t_f + b1 * t_f;
			
			std::pair<double, double> roots;
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			x_switch = (a1 > 0.) ? roots.first : roots.second;
			y_switch = parabolaCanonical( a1, b1, c1, x_switch, epsilon );
			b2 = 2. * a1_minus_a2 * x_switch + b1;
			c2 = y2 - x2 * (a2 * x2 + b2);
		}

		bool tangentParabolaThroughPoint2( double a2,
										 double& b2,
										 double& c2,
										 double& p1,
										 double& t1,
										 double a1,
										 double b1,
										 double c1,
										 double p2,
										 double t2,
										 double epsilon ) {
			// initialize remaining coefficients
			b2 = std::numeric_limits<double>::quiet_NaN();
			c2 = std::numeric_limits<double>::quiet_NaN();
			p1 = std::numeric_limits<double>::quiet_NaN();
			t1 = std::numeric_limits<double>::quiet_NaN();

			// find the crossover time
			std::pair<double, double> roots;
			double q_a = a1 - a2;
			double q_b = 2. * t2 * (a2 - a1);
			double q_c = p2 - t2 * (a2 * t2 + b1) - c1;
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "ERROR IN Maths::tangentParabolaThroughPoint: Invalid intersection." << std::endl;
#endif
				return false;
			}
			
			// determine appropriate intersection time
			if ( a2 < 0. ) {
				t1 = roots.second;
			} else {
				t1 = roots.first;
			}
			
			// compute remaining coefficients
			p1 = parabolaCanonical( a1, b1, c1, t1, epsilon );
			b2 = yFromSlopeIntercept( 2. * (a1 - a2), t1, b1, epsilon );
			c2 = parabolaCanonical( -a2, 2. * (a2 - a1) * t1 - b1, p2, t2, epsilon );
			
			return true;
		}
		
		bool tangentParabolaAtLines( double a,
									double& b,
									double& c,
									double& p1,
									double& t1,
									double& p2,
									double& t2,
									double m1,
									double y1,
									double m2,
									double y2,
									double epsilon ) {
			
			if ( approxEq(m1, m2, epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::tangentParabolaAtLines: Parallel lines." << std::endl;
#endif
				return false;
			}
			
			if ( approxEq(a, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::tangentParabolaAtLines: a == 0." << std::endl;
#endif
				return false;
			}
			
			double m2_minus_m1 = m2 - m1;
			t1 = (y1 - y2) / m2_minus_m1 - m2_minus_m1 / (4. * a);
			p1 = yFromSlopeIntercept( m1, t1, y1, epsilon );
			t2 = m2_minus_m1 / (2. * a) + t1;
			p2 = yFromSlopeIntercept( m2, t2, y2, epsilon );
			
			return true;
		}
		
		double T_FromV1_V2_X1_X2( double v1, double v2, double x1, double x2, double epsilon ) {
			double v_sum = v1 + v2;
			if ( Maths::approxEq(v_sum, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::T_FromV1_V2_X1_X2: v1 + v2 = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return 2. * (x2 - x1) / v_sum;
		}
		
		double parabolasTangentLine1( std::pair< std::pair<double, double>, std::pair<double, double> >& points, double v1, double a1, double a2, double x_star, double t_star, double epsilon ) {
			std::pair<double, double> roots;
			//double q_a = 1. - (a2 / a1);
			double q_a = ( a1 - a2 ) / a1;
			//double q_b = 2. * a2 * t_star + 2. * a2 / a1 * v1;
			double q_b = a2 * ( 2. * t_star + 2. / a1 * v1 );
			//double q_c = -(a2 / a1 * v1 * v1 + 2. * a2 * x_star);
			double q_c = -a2 * ( v1 * v1 / a1 + 2. * x_star);
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );

			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolasTangentLine: no solution to quadratic" << std::endl;
#endif
				points.first.first = std::numeric_limits<double>::quiet_NaN();
				points.first.second = std::numeric_limits<double>::quiet_NaN();
				points.second.first = std::numeric_limits<double>::quiet_NaN();
				points.second.second = std::numeric_limits<double>::quiet_NaN();
				return std::numeric_limits<double>::quiet_NaN();
			}

			// slope of tangent line, for out purposes, a negative slope is unusable
			double v;
			if ( a2 < a1 ) {
				v = roots.second;
			} else {
				v = roots.first;
			}
			
			// x-intercept
			double v_diff = v1 - v;
			
			// find the first point
			//double t1 = (-2. * v_diff) / (2. * a1);
			double t1 = -v_diff / a1;
			double x1 = motionX_FromV1_T1_T2_A( v1, 0., t1, a1, epsilon );
			points.first.first = x1;
			points.first.second = t1;
			
			// find second point
			//double t2 = 2. * (a2 * t_star + v) / (2. * a2);
			//double t2 = (a2 * t_star + v) / a2;
			double t2 = t_star + v / a2;
			double x2 = x_star + motionX_FromV1_T1_T2_A( 0., t2, t_star, a2, epsilon );
			points.second.first = x2;
			points.second.second = t2;
			
			return v;
		}
		
		bool parabolaOriginOffset( std::pair<double, double>& origin, double v, double acc, double epsilon ) {
			if ( Maths::approxEq(acc, 0., epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::OriginFinalP_Plus: acceleration = 0, no inflection point." << std::endl;
#endif
				origin.first = std::numeric_limits<double>::quiet_NaN();
				origin.second = origin.first;
				return false;
			}
			origin.first = -parabolaDelX_FromV1_V2_A( v, 0., acc, epsilon );
			if ( isNaN(origin.first) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::OriginFinalP_Plus: parabolaDelX_FromV1_V2_A failed." << std::endl;
#endif
				origin.first = std::numeric_limits<double>::quiet_NaN();
				origin.second = origin.first;
				return false;
			}
			origin.second = -v / acc;
			return true;
		}
		
		double L2_Distance( double x1, double y1, double x2, double y2, double epsilon ) {
			
			bool x_equal = Maths::approxEq( x1, x2, epsilon );
			bool y_equal = Maths::approxEq( y1, y2, epsilon );
			
			if ( x_equal && y_equal ) {
				return 0.;
			}
			
			double y_diff = y2 - y1;
			
			if ( x_equal ) {
				return fabs( y_diff );
			}
			
			double x_diff = x2 - x1;
			
			if ( y_equal ) {
				return fabs( x_diff );
			}
			
			return sqrt( x_diff * x_diff + y_diff * y_diff );
		}
		
		bool computeLineToPointReferencePoint( double& reference_x,
											  double& reference_y,
											  double a,
											  double b,
											  double c,
											  double reference_slope,
											  double delta_x1,
											  double delta_y1,
											  double delta_x2,
											  double delta_y2,
											  double vertex_x,
											  double vertex_y,
											  double epsilon ) {
			double m_r = reference_slope;
			
			// common term in computations
			double term = sqrt( 1. + m_r * m_r );
			
			double delta_x = delta_x2 - delta_x1;
			double delta_y = delta_y2 - delta_y1;
			
			double m_d;
			if ( Maths::approxEq(m_r, 0., epsilon) ) {
				m_d = slope( delta_x1, delta_y1, delta_x2, delta_y2, epsilon );
			} else {
				double num = delta_x * m_r + delta_y;
				double den = delta_x + (delta_y1 - delta_y2) * m_r;
				if ( Maths::approxEq(den, 0., epsilon) ) {
					m_d = std::numeric_limits<double>::quiet_NaN();
				} else {
					m_d = num / den;
				}
			}
			
			if ( Maths::isNaN(m_d) ) {
				reference_x = vertex_x - delta_x1 / term + delta_y1 * m_r / term;
			} else {
				double q_a = a;
				double q_b = b - m_d;
				double q_c = c + ( 1. / term ) * ( delta_x1 * m_r + delta_y1 - delta_x1 * m_d + delta_y1 * m_r * m_d ) + m_d * vertex_x - vertex_y;
				std::pair<double, double> roots;
				quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
				if ( a <= 0. ) {
					reference_x = roots.first;
				} else {
					reference_x = roots.second;
				}
			}
			
			reference_y = parabolaCanonical( a, b, c, reference_x, epsilon );
			
			return !Maths::isNaN( reference_x );
		}
		
		bool computePointToLineReferencePoint( double& reference_x,
											  double& reference_y,
											  double a,
											  double b,
											  double c,
											  double reference_slope,
											  double delta_x,
											  double delta_y,
											  double edge_x1,
											  double edge_y1,
											  double edge_x2,
											  double edge_y2,
											  double epsilon ) {
			
			// compute line information
			double m_e = slope( edge_x1, edge_y1, edge_x2, edge_y2, epsilon );
			double b_e = yIntercept( edge_x1, edge_y1, m_e, epsilon );
			
			// slope of curve at point
			double m_r = reference_slope;
			
			// common term in computations
			double term = sqrt( 1. + m_r * m_r );
			
			// vertical edge
			if ( Maths::isNaN(m_e) ) {
				reference_x = edge_x1 - delta_x / term + delta_y * m_r / term;
			} else {
				double q_a = a;
				double q_b = b - m_e;
				double q_c = c - ( 1 / term ) * ( delta_x * m_e - delta_x * m_r - delta_y * m_r * m_e - delta_y + b_e * term );
				std::pair<double, double> roots;
				quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
				if ( a <= 0. ) {
					reference_x = roots.first;
				} else {
					reference_x = roots.second;
				}
			}
			
			reference_y = parabolaCanonical( a, b, c, reference_x, epsilon );
			
			return !Maths::isNaN( reference_x );
		}
		
		double approxEllipsePerimeter( double a, double b, double epsilon ) {
			
			double h_root = (a - b) / (a + b);
			double h = h_root * h_root;
			double h_times_3 = 3. * h;

			return M_PI * (a + b) * (1. + (h_times_3 / (10. + sqrt(4. - h_times_3))) + ((4. / M_PI) - (14. / 11.)) * pow(h, 12.));
			
		}
		
#ifdef BUILDGSL
#if BUILDGSL
		double ellipseArcLength( double x1, double x2, double a, double b, double epsilon ) {
			
			// get coordinates on ellipse of arc end points
			double ratio = x1 / a;
			double y1 = b * sqrt( 1. - ratio * ratio );
			ratio = x2 / a;
			double y2 = b * sqrt( 1. - ratio * ratio );
			
			// calculate angles for endoints
			double phi_1 = atan( y1 / x1 );
			double phi_2 = atan( y2 / x2 );
			
			// ellipse modulus (eccentricity)
			double k = sqrt( 1 - ((b * b) / (a * a)) );
			
			// calculate lengths
			double arc_1 = gsl_sf_ellint_E( phi_1, k, Constants::GSL_MODE );
			double arc_2 = gsl_sf_ellint_E( phi_2, k, Constants::GSL_MODE );
			
			return a * ( arc_2 - arc_1 );
		}
#endif
#endif
		
	} // end Maths namespace
	
} // end PVTP namespace
