#ifndef PVTP_MATHS_H
#define PVTP_MATHS_H

#define _USE_MATH_DEFINES

#include <math.h>

#define M_2PI (2 * M_PI)

#include <limits>
#include <algorithm>
#include <vector>

#define DEG2RAD(x) ((x) * (M_PI / 180.))
#define RAD2DEG(x) ((x) * (180. / M_PI))

namespace PVTP {
	
	/**
	 * This namespace contains math functions specific to the library.
	 * All operations are performed in PT space.
	 * All functions in this namespace take the epsilon parameter as their last
	 * parameter. The epsilon parameter is used when fuzzy comparisons must be
	 * made.
	 */
	namespace Maths {
		
		/**
		 * Indicator for when a point is above a curve
		 */
		static const int POINT_ABOVE = 1;
		
		/**
		 * Indicator for when a point is below a curve
		 */
		static const int POINT_BELOW = -1;
		
		/**
		 * Indicator for when a point is on a curve
		 */
		static const int POINT_ON = 0;
		
		/**
		 * Indicator for when there is no intersection
		 */
		static const int INTERSECTION_NONE = -1;
		
		/**
		 * Indicator for when an intersection is tangent
		 */
		static const int INTERSECTION_UNIQUE = 0;
		
		/**
		 * Indicator for non-unique intersection
		 */
		static const int INTERSECTION_NON_UNIQUE = 1;
		
		/**
		 * Error indicator for indicator functions
		 */
		static const int ERR = -2;
		
		/**
		 * Test for NaN. According to IEEE standards, comparisons involving
		 * NaN will always evaluate to false. Therefore, NaN will be the only
		 * value that fails a test for whether it is equal to itself. This
		 * routine exploits that by using the != operator.
		 */
		inline bool isNaN( double num ) {
			return num != num;
		}
		
		/**
		 * Test whether two floating point numbers are equal within
		 * some pre-defined epsilon: A ~= B.
		 *
		 * This comparison forms the basis for the other approximate
		 * comparisons. It is assumed that the fp implementation uses round
		 * to nearest for number representation. Epsilon is padded with the
		 * machine-dependent rounding error before being compared with the
		 * difference between A and B.
		 *
		 * This ensures that [0, epsilon_padded] contains the intended epsilon,
		 * which ensures that approxEq will always return true when expected to.
		 * 
		 * However, under this scheme it is possible that true will be returned
		 * when false is expected. For that error to occur on a 64-bit machine,
		 * it must be that:
		 *
		 *			epsilon < abs(A-B) < epsilon + 1.5 * ( 2.22045e-16 )
		 *
		 * NOTE: At some point, should convert this to check relative rather than
		 * absolute error.
		 */
		bool approxEq( double A, double B, double epsilon = 0. );
		
		/**
		 * Test whether two floating point numbers are less-than within
		 * some pre-defined epsilon: A ~< B
		 */
		inline bool approxLt( double A, double B, double epsilon = 0. ) {
			return !approxEq(A, B, epsilon) && (A < B);
		}
		
		/**
		 * Test whether two floating point numbers are less-than-or-equal
		 * within some pre-defined epsilon: A ~<= B
		 */
		inline bool approxLe( double A, double B, double epsilon = 0. ) {
			return approxEq(A, B, epsilon) || (A < B);
		}
		
		/**
		 * Test whether two floating point numbers are greater-than within
		 * some pre-defined epsilon: A ~> B
		 */
		inline bool approxGt( double A, double B, double epsilon = 0. ) {
			return !approxEq(A, B, epsilon) && (A > B);
		}
		
		/**
		 * Test whether two floating point numbers are greater-than-or-equal
		 * within some pre-defined epsilon: A ~>= B
		 */
		inline bool approxGe( double A, double B, double epsilon = 0. ) {
			return approxEq(A, B, epsilon) || (A > B);
		}
		
		/**
		 * Take numbers very close to zero and round them to zero.
		 */
		double clipToZero( double num, double epsilon );
		
		/**
		 * Geodesic in radians for rotations on the plane
		 */
		double circleGeodesic( double target_rotation, double epsilon = 0. );
		
		/**
		 * Given coordinates for two 2D points, determine slope going from p1 -> p2
		 *
		 * NOTE: For cases where slope is infinite, std::numeric_limits<double>::quiet_NaN() is returned.
		 */
		double slope( double x1, double y1, double x2, double y2, double epsilon = 0. );
		
		/**
		 * Given canonical parabola coefficients, compute slope at a given x
		 */
		inline double parabolaSlope( double a, double b, double x, double epsilon = 0. ) {
			return 2. * a * x + b;
		}
		
		/**
		 * Given a point and a slope, determine y intercept for a line
		 */
		inline double yIntercept( double x, double y, double m, double epsilon = 0. ) {
			return y - m * x;
		}
		
		/**
		 * Horner's method for evaluating a polynomial.
		 *
		 * NOTE: This method assumes coefficients are in order of descending power
		 */
		double Horners( const std::vector<double>& coefficients, double x, double epsilon = 0. );
		
		/**
		 * The arc length between two points on a canonical parabola
		 */
		double secondOrderArcLength( double a, double b, double c, double x1, double x2, double epsilon = 0. );
		
		/**
		 * Given the slope-intercept parameterization of a line and a x value, determine y
		 */
		inline double yFromSlopeIntercept( double m, double x, double b, double epsilon = 0. ) {
			return m * x + b;
		}
		
		/**
		 * Given the slope-intercept parameterization of a line and a y value, determine x
		 */
		double xFromSlopeIntercept( double m, double y, double b, double epsilon = 0. );
		
		/**
		 * Given the slope-intercept parameterization of two lines, compute their intersection.
		 *
		 * NOTE: This method assumes that neither line is vertical
		 */
		void lineIntersection( double& x, double& y, double m1, double b1, double m2, double b2, double epsilon = 0. );
		
		/**
		 * Given the slope-intercept parameterization of a line, and a canonical parabola, compute type of intersection:
		 *
		 * INTERSECTION_NONE: no intersection, INTERSECTION_UNIQUE: tangent, INTERSECTION_NON_UNIQUE: non-tangent intersection
		 *
		 * NOTE: This does *not* handle lines parallel to the parabola's axis of symmetry! Return ERR when function fails.
		 */
		int lineParabolaIntersection( double& _x, double x, double m_line, double b_line, double a, double b, double c, double epsilon = 0. );
		
		/**
		 * Calculate average velocity given initial and final position, and
		 * initial and final time.
		 */
		double avgVelFromX1_X2_T1_T2( double p1, double p2, double t1, double t2, double epsilon = 0. );
		
		/**
		 * Calculate the distance coordinate at which a parabola inflects given
		 * the first derivative, v, of your current point, and the second
		 * derivative, a, of the parabola.
		 */
		double parabolaDelX_FromV1_V2_A( double v1, double v2, double acc, double epsilon = 0. );

		/**
		 * Compute the value of a parabola given canonical coefficients and point.
		 */
		inline double parabolaCanonical( double a,
								 double b,
								 double c,
								 double t,
								 double epsilon = 0. ) {
			return t * (a * t + b) + c;
		}
		
		/**
		 * Determine the canonical coefficients for a parabola given two first derivatives
		 */
		bool parabolaCoefficients1( double& a,
								  double& b,
								  double& c,
								  double x1,
								  double t1,
								  double v1,
								  double t2,
								  double v2,
								  double epsilon = 0. );
		
		/**
		 * Determine remaining canonical coefficients for a parabola given a first and second derivative.
		 */
		inline void parabolaCoefficients2( double a,
										  double& b,
										  double& c,
										  double x,
										  double y,
										  double v,
										  double epsilon = 0. ) {
			b = v - 2. * a * x;
			c = y - x * (a * x + b);
		}
		
		/**
		 * Solve a quadratic. By convention, the first root is preferred, in
		 * descending order of priority, to be minimum if both are positive,
		 * positive, or real.
		 */
		void quadratic( std::pair<double, double>& roots,
					   double a,
					   double b,
					   double c,
					   double epsilon = 0. );

		/**
		 * Identical to above, but roots returned with strict ordering:
		 * first = -b + sqrt(disc), second = -b - sqrt(disc)
		 */
		void quadraticOrdered( std::pair<double, double>& roots,
							  double a,
							  double b,
							  double c,
							  double epsilon = 0. );
		
		/**
		 * Equation of motion. Get initial velocity from change in position,
		 * change in time, and accelerations.
		 */
		double motionV1_FromX1_X2_T1_T2_A( double x1,
										  double x2,
										  double t1,
										  double t2,
										  double acc,
										  double epsilon = 0. );
		
		/**
		 * Equation of motion. Get change in position from initial velocity,
		 * change in time, and acceleration.
		 */
		inline double motionX_FromV1_T1_T2_A( double v1, double t1, double t2, double acc, double epsilon = 0. ) {
			return parabolaCanonical( 0.5 * acc, v1, 0., (t2 - t1), epsilon );
		}
		
		/**
		 * Equation of motion. Get change in position from initial velocity
		 * and change in time.
		 */
		inline double motionX_FromV1_T1_T2( double v1, double t1, double t2, double epsilon = 0. ) {
			return v1 * (t2 - t1);
		}

		/**
		 * Equation of motion. Get acceleration from initial velocity,
		 * change in time, and change in position.
		 */
		double motionA_FromV1_X1_X2_T1_T2( double v1,
										  double x1,
										  double x2,
										  double t1,
										  double t2,
										  double epsilon = 0. );
		
		/**
		 * Equation of motion. Get time from initial velocity, change in position,
		 * and acceleration.
		 */
		double motionT_FromV1_X1_X2_A( double v1, double x1, double x2, double acc, double epsilon = 0. );
		
		/**
		 * Final velocity given initial velocity, time, and acceleration
		 */
		inline double V2_FromV1_T_A( double v1, double t, double acc, double epsilon = 0. ) {
			return v1 + acc * t;
		}

		/**
		 * Acceleration given rinal velocity given initial velocity, and time
		 */
		double A_FromV1_V2_T1_T2( double v1, double v2, double t1, double t2, double epsilon = 0. );
		
		/**
		 * Change in time given initial velocity, final velocity, and acceleration
		 */
		double T_FromV1_V2_A( double v1, double v2, double acc, double epsilon = 0. );
		
		/**
		 * Change in time given initial velocity, final velocity, and change in
		 * position.
		 */
		double T_FromV1_V2_X1_X2( double v1, double v2, double x1, double x2, double epsilon = 0. );
		
		/**
		 * Given a point (p, t) and canonical parabola coefficients {a, b, c},
		 * determine with the point is above, below, or on the line,
		 * indicated by returning 1, -1, 0, respectively.
		 */
		int pointParabolaRelation( double p, double t, double a, double b, double c, double epsilon = 0. );
		
		/**
		 * Given a point (p, t) and canonical line coefficients {b, c},
		 * determine with the point is above, below, or on the line,
		 * indicated by returning 1, -1, 0, respectively.
		 */
		int pointLineRelation( double p, double t, double b, double c, double epsilon = 0. );
		
		/**
		 * Given two canonical parabolas, determine the points on each that define
		 * a line tangent to both.
		 */
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
									double epsilon = 0. );
		
		/**
		 * Given a point and a parabola, find a point on the parabola that, along
		 * with the given point, defines a line tangent to the parabola (if one exists).
		 */
		void tangentLineThroughPointToParabola1( double& x_tangent,
												double& y_tangent,
												double a,
												double b,
												double c,
												double x,
												double y,
												double epsilon = 0. );
		
		/**
		 * Given a point and a parabola, find a point on the parabola that, along
		 * with the given point, defines a line tangent to the parabola (if one exists).
		 */
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
												double epsilon = 0. );

		/**
		 * Given two points and a set of canonical parabola coefficients, determine
		 * a second parabola tangent to that given that intersects the given point.
		 *
		 * The canonical coefficients of the determined parabola are stored in
		 * a2, b2, and c2, where a2 is given, and b2 and c2 are determined.
		 * The coefficients of the given parabola are in a1, b1, c1. The point
		 * is given in (p2, t2). The variables p1 and t1 store the coordinates
		 * of the parabolas' intersection.
		 */
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
										  double epsilon );
		
		/**
		 * Given a point and a set of canonical parabola coefficients, determine
		 * a second parabola tangent to that given that intersects the given point.
		 *
		 * The canonical coefficients of the determined parabola are stored in
		 * a2, b2, and c2, where a2 is given, and b2 and c2 are determined.
		 * The coefficients of the given parabola are in a1, b1, c1. The point
		 * is given in (p2, t2). The variables p1 and t1 store the coordinates
		 * of the parabolas' intersection.
		 */
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
										 double epsilon = 0. );
		
		/**
		 * Given a point, a line, and a set of canonical parabola coefficients, determine
		 * a second parabola that connects the first parabola to the line at 
		 * tangent points, where the line intersects the given point.
		 *
		 * The canonical coefficients of the determined parabola are stored in
		 * a2, b2, and c2, where a2 is given, and b2 and c2 are determined.
		 * The coefficients of the given parabola are in a1, b1, c1. The point
		 * is given in (p3, t3). The variables p1 and t1 store the coordinates
		 * of the parabolas' intersection, and p2 and t2 store the coordinates
		 * of the intersection of the second parabola with the line.
		 */
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
										double epsilon = 0. );
		
		/**
		 * Given the slope-intercept parameterization of two lines, find the
		 * parabola of specified curvature that connects the two lines at
		 * tangent points (p1, t1), (p2, t2)
		 */
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
									double epsilon = 0. );
		
		/**
		 * Find tangent points of line tangent to two parabolas, where one
		 * is centered at the origin, and the other at (x_star, t_star).
		 */
		double parabolasTangentLine1( std::pair< std::pair<double, double>, std::pair<double, double> >& points,
									 double v1,
									 double a1,
									 double a2,
									 double x_star,
									 double t_star,
									 double epsilon = 0. );
		
		/**
		 * Find the offset from the origin of the point on the parabola specified
		 * by second derivative acc that has first derivative v. In 'origin', the
		 * first member is offset in path, second is offset in time.
		 */
		bool parabolaOriginOffset( std::pair<double, double>& origin,
								  double v,
								  double acc,
								  double epsilon = 0. );
		
		/**
		 * Calculate Euclidean distance between two coordinates
		 */
		double L2_Distance( double x1, double y1, double x2, double y2, double epsilon = 0. );
		
		/**
		 * Given a static translation for any point on given curve, compute a
		 * transform that uses that static translation and computes a rotation
		 * based on the slope of the curve at any point. This defines a line at
		 * a known transform T from any point on the curve. Given in addition a
		 * point, compute the point on the curve x_r such that the line contains
		 * the point.
		 */
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
											  double epsilon = 0. );
		
		/**
		 * Given a static translation for any point on given curve, compute a 
		 * transform that uses that static translation and computes a rotation
		 * based on the slope of the curve at any point. This defines a point at
		 * a known transform T from any point on the curve. Given in addition the
		 * parameterization for a line, compute the point on the curve x_r such
		 * that the point x_t = T * x_r lies on the line.
		 */
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
											  double epsilon = 0. );
		
		/**
		 * Calculate the approximate quarter arc length of an ellipse using
		 * the Cantrell-Ramanujan approximation:
		 *
		 * http://www.ebyte.it/library/docs/math05a/EllipsePerimeterApprox05.html#C
		 *
		 * This method assumes a and b are positive.
		 */
		double approxEllipsePerimeter( double a, double b, double epsilon = 0. );
		
#ifdef BUILDGSL
#if BUILDGSL
		/**
		 * Calculate the arc length of an ellipse between two point on the 
		 * semi-major axis. Assume that these points are contained within 
		 * the same quadrant.
		 *
		 * http://mathworld.wolfram.com/Ellipse.html
		 *
		 * This method assumes the ellipse is centered at the origin, that the
		 * arc is contained entirely within a single quadrant, and that x2 >= x1.
		 *
		 * This method uses GSL to compute the incomplete elliptical integral,
		 * and uses a fast approximation that is accurate to 5 x 10^-4
		 */
		double ellipseArcLength( double x1, double x2, double a, double b, double epsilon = 0. );
#endif
#endif
		
	} // end Maths namespace
	
} // end PVTP namespace

#endif