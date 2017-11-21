/***
 * WarningMapPolygon
 *
 * This is just a file for random tests I've been doing. Much of the code is for
 * previous iterations of the library, and so may not work.
 */

#include <iostream>
#include <sys/time.h>
#include <PVTP/Constraints.hpp>
#include <PVTP/Maths.hpp>
#include <PVTP/Interval.hpp>
#include <PVTP/PVT_Point.hpp>
#include <PVTP/PVT_Obstacle.hpp>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/PVT_State.hpp>
#include <PVTP/TrajectorySegment.hpp>
#include <PVTP/Planner.hpp>
#include <PVTP/Collisions.hpp>
#include <PVTP/Utilities.hpp>
#include <PVTP/Controller.hpp>
#include <PVTP/ScenarioGenerator.hpp>
#include <PVTP/ScenarioEvaluator.hpp>
#include <PVTP/image_utils/SImage.h>
#include <PVTP/image_utils/SImageIO.h>
#include <PVTP/image_utils/utils.h>
#include <PVTP/VehicleModel.hpp>
#include <PVTP/ExceptionHandler.hpp>

#define NAIVE 1

using namespace PVTP;
using namespace Scenario;

int main () {
	double tmp, new_eps;
	
	try {
		double vehicle_path_length = 75.;
		double max_time = 50.;
		double v_min = 0.;
		double v_max = 27.;
		double a_min = -10.;
		double a_max = 8.;
		//Constraints c( vehicle_path_length, max_time, v_min, v_max, a_min, a_max );
		
		//Constraints c( 67.3403568041163, 50., 0., 10., -4., 4. );
		//Constraints c( 67.3403568041163, 50., 0., 50., -10., 8. );
		//Constraints c( 67.3403568041163, 50., 0., 10., -5., 7. );
		//Constraints c( 64., 20., 0., 50., -10., 8. );
		//Constraints c( 250., 250., 0., 50., -10., 8. );
		Constraints c( 10., 15., 0., 50., -1., 1. );
		
		//Constraints c( 63.3699999999996350652509136125, 100., 0., 50., -1., 1. );
		//Constraints c( 63.3699999999996350652509136125, 100., 0., 15., -10., 8. );
		
		//Constraints c( 35., 30., 0., 20., -10., 8. );// 1.00000222044604928674527816276e-10 );
		std::vector<TrajectorySegment*> T;
		//Constraints c( 62.3404, 48.65, 0., 50., -10., 8. );
		//Constraints c( 67.2279, 48.65, 0., 50., -10., 8. );
		//Constraints c( 65.54035680411629982700106, 49.35000000000000142108547, 0., 50., -10., 8. );
		//Constraints c( 67.14035680411629414265917, 48.64999999999999857891453, 0., 50., -10., 8. );
		//Constraints c( 66.91535680411629982700106, 48.60000000000000142108547, 0., 50., -10., 8. );
		//Constraints c( 63.89035680411629414265917, 48.04999999999999715782906, 0., 50., -10., 8. );
		//Constraints c( 63.89035680411629414265917, 48.39999999999999857891453, 0., 50., -10., 8. );
		//Constraints c( 64., 60., 0., 50., -10., 8. );
		//Constraints c( 64, 51.85000000000002273736754, 0., 24., -10., 8. );
		//Constraints c( 6.315000000000001278976924, 5.250000000000049737991503, 0., 50., -10., 8. );
		//Constraints c( 6.314999999999996838084826, 5.250000000000048849813084, 0., 50., -10., 8. );
		
		std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
		//std::cout.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
		
		std::cout << std::endl << "SYSTEM CONSTRAINTS: " << c << std::endl;
		std::cout << std::endl << "Machine epsilon: " << std::numeric_limits<double>::epsilon() << std::endl;
		std::cout << std::endl;
		
		/**/
		double x1;
		double y1;
		double x2;
		double y2;
		double a1 = 2.5;
		double b1 = 5.;
		double c1 = 1.;
		double a2 = 2.5;
		double b2 = 35.;
		double c2 = 75.;
		Maths::tangentLineToParabolas( x1, y1, x2, y2, a1, b1, c1, a2, b2, c2, c.getEpsilon() );
		std::cout << "x1: " << x1 << ", y1: " << y1 << ", x2: " << x2 << ", y2: " << y2 << std::endl;
		//return 0;
		/**/
		
		/**
		// parabola coefficients
		double p_a = -0.03;
		double p_b = 1.;
		double p_c = 1.;
		
		// test point-to-line computation
		double reference_x;
		double reference_y;
		double reference_slope = 1.;
		double delta_x = 1.;
		double delta_y = 0.;
		double line_slope = -4.;
		double line_intercept = 34.08;
		double edge_x1 = 7.;
		double edge_y1 = 6.08;
		double edge_x2 = 6.5;
		double edge_y2 = 8.08;
		if ( !Maths::computePointToLineReferencePoint(reference_x,
													  reference_y,
													  p_a,
													  p_b,
													  p_c,
													  reference_slope,
													  delta_x,
													  delta_y,
													  edge_x1,
													  edge_y1,
													  edge_x2,
													  edge_y2,
													  c.getEpsilon()) ) {
			std::cout << "point-to-line failed" << std::endl;
			return 1;
		}
		std::cout << "point-to-line (x_r, y_r): " << reference_x << ", " << reference_y << std::endl;
		
		// test line-to-point computation
		double delta_x1 = 1.;
		double delta_y1 = 1.;
		double delta_x2 = 1.;
		double delta_y2 = -1.;
		if ( !Maths::computeLineToPointReferencePoint(reference_x,
													  reference_y,
													  p_a,
													  p_b,
													  p_c,
													  reference_slope,
													  delta_x1,
													  delta_y1,
													  delta_x2,
													  delta_y2,
													  edge_x1,
													  edge_y1,
													  c.getEpsilon()) ) {
			std::cout << "line-to-point failed" << std::endl;
			return 1;
		}
		std::cout << "line-to-point (x_r, y_r): " << reference_x << ", " << reference_y << std::endl;
		//return 0;
		
		// test second order arc length
		double x1 = 0.;
		double x2 = 5.8;
		double arc_length = Maths::secondOrderArcLength( p_a, p_b, p_c, x1, x2, c.getEpsilon() );
		std::cout << "Arc length is: " << arc_length << std::endl;
		//return 0;
		/**/
		
		
		/**/
		std::vector< std::pair<double, double> > obs1;
		obs1.push_back( std::pair<double, double>(-1., 0.) );
		obs1.push_back( std::pair<double, double>(-1., 2.) );
		obs1.push_back( std::pair<double, double>(7., 10.) );
		obs1.push_back( std::pair<double, double>(7., 8.) );
		std::vector< std::pair<double, double> > obs2;
		obs2.push_back( std::pair<double, double>(0., 3.) );
		obs2.push_back( std::pair<double, double>(0., 5.) );
		obs2.push_back( std::pair<double, double>(8., 9.) );
		obs2.push_back( std::pair<double, double>(8., 7.) );
		std::vector< std::pair<double, double> > obs3;
		obs3.push_back( std::pair<double, double>(8.5, 0.) );
		obs3.push_back( std::pair<double, double>(8.5, 7.5) );
		obs3.push_back( std::pair<double, double>(9.5, 7.5) );
		obs3.push_back( std::pair<double, double>(9.5, 0.) );
		std::vector< std::pair<double, double> > obs4;
		obs4.push_back( std::pair<double, double>(2., 0.) );
		obs4.push_back( std::pair<double, double>(6., 4.) );
		obs4.push_back( std::pair<double, double>(6., 2.) );
		obs4.push_back( std::pair<double, double>(4., 0.) );
		std::vector< std::vector< std::pair<double, double> > > obstacles_test;
		obstacles_test.push_back( obs4 );
		obstacles_test.push_back( obs3 );
		obstacles_test.push_back( obs2 );
		obstacles_test.push_back( obs1 );
		PVT_ObstacleSet O_test( obstacles_test, c );
		
		/*
		ssize_t p_step = 10;
		ssize_t t_step = 7;
		ssize_t square_width = 6;
		ssize_t square_height = 2;
		for ( ssize_t i=0; i<square_width; i++ ) {
			for ( ssize_t j=0; j<square_height; j++ ) {
				
				PVT_Obstacle * tmp = new PVT_Obstacle( obs2, c );
				tmp->translateObstacle( i * p_step - 1, j * t_step - 1 );
				O_test.obstacles.push_back( tmp );

			}

		}
		*/
		O_test.translateObstacles( -.5, 0. );
		/**/
		
		
		/**
		 * BEGIN: Test world obstacles
		 */
		
		// construct path segments
		std::vector<PathSegment> segments;
		
		/**
		 * FORWARD OBSTACLE PATH
		 */
		std::vector<double> seg1_coeffs;
		seg1_coeffs.push_back( 0. );
		seg1_coeffs.push_back( 0. );
		seg1_coeffs.push_back( 10. );
		XY_Point seg1_origin( 0., 10. );
		double seg1_length = 4.;
		PathSegment seg1( seg1_coeffs, seg1_origin, seg1_length, c );
		segments.push_back( seg1 );
		
		std::vector<double> seg2_coeffs;
		seg2_coeffs.push_back( 0. );
		seg2_coeffs.push_back( -1. );
		seg2_coeffs.push_back( 14. );
		XY_Point seg2_origin( seg1_length, 10. );
		double seg2_length = sqrt( 50. );
		PathSegment seg2( seg2_coeffs, seg2_origin, seg2_length, c );
		segments.push_back( seg2 );
		
		std::vector<double> seg3_coeffs;
		seg3_coeffs.push_back( 0. );
		seg3_coeffs.push_back( 0. );
		seg3_coeffs.push_back( 5. );
		XY_Point seg3_origin( 9., 5. );
		double seg3_length = 4.;
		PathSegment seg3( seg3_coeffs, seg3_origin, seg3_length, c );
		//segments.push_back( seg3 );
		/**
		 */
		
		/**
		 * BACKWARD OBSTACLE PATH
		 *
		std::vector<double> seg1_coeffs;
		seg1_coeffs.push_back( 0. );
		seg1_coeffs.push_back( 0. );
		seg1_coeffs.push_back( 5. );
		XY_Point seg1_origin( 13., 5. );
		double seg1_length = -4.;
		PathSegment seg1( seg1_coeffs, seg1_origin, seg1_length, c );
		segments.push_back( seg1 );
		
		std::vector<double> seg2_coeffs;
		seg2_coeffs.push_back( 0. );
		seg2_coeffs.push_back( -1. );
		seg2_coeffs.push_back( 14. );
		XY_Point seg2_origin( seg1.getEndPoint() );
		double seg2_length = -sqrt( 50. );
		PathSegment seg2( seg2_coeffs, seg2_origin, seg2_length, c );
		segments.push_back( seg2 );
		
		std::vector<double> seg3_coeffs;
		seg3_coeffs.push_back( 0. );
		seg3_coeffs.push_back( 0. );
		seg3_coeffs.push_back( 10. );
		XY_Point seg3_origin( seg2.getEndPoint() );
		double seg3_length = -4.;
		PathSegment seg3( seg3_coeffs, seg3_origin, seg3_length, c );
		segments.push_back( seg3 );
		/**
		 */
		 
		/**
		std::vector<double> seg_lin_coeffs;
		seg_lin_coeffs.push_back( 0. );
		seg_lin_coeffs.push_back( std::numeric_limits<double>::max() );
		seg_lin_coeffs.push_back( std::numeric_limits<double>::quiet_NaN() );
		XY_Point seg_lin_origin( 15., 15. );
		double seg_lin_length = 30.;
		//PathSegment seg_lin( seg_lin_coeffs, seg_lin_origin, seg_lin_length, c );
		//segments.push_back( seg_lin );
		/**/
		
		/**
		std::vector<double> seg_lin_coeffs;
		seg_lin_coeffs.push_back( 0. );
		seg_lin_coeffs.push_back( 0. );
		seg_lin_coeffs.push_back( 15. );
		XY_Point seg_lin_origin( 15., 15. );
		double seg_lin_length = 30.;
		PathSegment seg_lin( seg_lin_coeffs, seg_lin_origin, seg_lin_length, c );
		segments.push_back( seg_lin );
		/**/
	
		// construct path
		PathModel path( segments, c );
		
		//std::cout << "Obstacle path:" << std::endl;
		//std::cout << path << std::endl;

		// test
		double x;
		double y;
		double position = 4.;
		/**
		if ( !path.getPositionVectorAtArcLength(x, y, position, c) ) {
			std::cerr << "get position failed 1" << std::endl;
			return 1;
		}
		std::cout << "Coordinates at arc length " << position << ": (" << x << ", " << y << ")" << std::endl;
		
		position = 10.;
		if ( !path.getPositionVectorAtArcLength(x, y, position, c) ) {
			std::cerr << "get position failed 1" << std::endl;
			return 1;
		}
		std::cout << "Coordinates at arc length " << position << ": (" << x << ", " << y << ")" << std::endl;
		
		position = 15.;
		if ( !path.getPositionVectorAtArcLength(x, y, position, c) ) {
			std::cerr << "get position failed 1" << std::endl;
			return 1;
		}
		std::cout << "Coordinates at arc length " << position << ": (" << x << ", " << y << ")" << std::endl;
		
		std::cout << std::endl;
		/**/
		// move forward
		/**
		double t = 1.;
		if ( !wo.translateInTime( t, c ) ) {
			std::cerr << "obstacle translation failed" << std::endl;
			return 1;
		}
		std::cout << wo << std::endl;
		std::cout << "====" << std::endl;
		/**/

		// construct path segments
		std::vector<PathSegment> user_segments;
		
		/**
		std::vector<double> user_seg1_coeffs;
		user_seg1_coeffs.push_back( 0. );
		user_seg1_coeffs.push_back( 10. );
		XY_Point user_seg1_origin( 0., 10. );
		double user_seg1_length = 8.;
		PathSegment user_seg1( user_seg1_coeffs, user_seg1_origin, user_seg1_length, c );
		user_segments.push_back( user_seg1 );
		
		std::vector<double> user_seg2_coeffs;
		user_seg2_coeffs.push_back( -1. );
		user_seg2_coeffs.push_back( 18. );
		XY_Point user_seg2_origin( 8., 10. );
		double user_seg2_length = sqrt( 50. );
		PathSegment user_seg2( user_seg2_coeffs, user_seg2_origin, user_seg2_length, c );
		user_segments.push_back( user_seg2 );
		
		std::vector<double> user_seg3_coeffs;
		user_seg3_coeffs.push_back( 0. );
		user_seg3_coeffs.push_back( 5. );
		XY_Point user_seg3_origin( 13., 5. );
		double user_seg3_length = 8.;
		PathSegment user_seg3( user_seg3_coeffs, user_seg3_origin, user_seg3_length, c );
		user_segments.push_back( user_seg3 );
		/**/
		
		/** horizontal **/
		std::vector<double> user_seg_coeffs;
		user_seg_coeffs.push_back( 0. );
		user_seg_coeffs.push_back( 0. );
		user_seg_coeffs.push_back( 8. );
		XY_Point user_seg_origin( 0., 8. );
		double user_seg_length = 13.;
		PathSegment user_seg( user_seg_coeffs, user_seg_origin, user_seg_length, c );
		user_segments.push_back( user_seg );
		/**/
		
		/** vertical **/
		std::vector<PathSegment> user_segments_v;
		std::vector<double> user_seg_coeffs_v;
		user_seg_coeffs_v.push_back( 0. );
		user_seg_coeffs_v.push_back( std::numeric_limits<double>::quiet_NaN() );
		user_seg_coeffs_v.push_back( std::numeric_limits<double>::quiet_NaN() );
		XY_Point user_seg_origin_v( 4., 3. );
		double user_seg_length_v = 13.;
		PathSegment user_seg_v( user_seg_coeffs_v, user_seg_origin_v, user_seg_length_v, c );
		user_segments_v.push_back( user_seg_v );
		/**/
		
		// construct path
		PathModel user_path( user_segments, c );
		PathModel user_path_v( user_segments_v, c );
		
		//std::cout << "User path:" << std::endl;
		//std::cout << user_path << std::endl;
		
		//return 0;
		
		// construct world obstacle
		XY_Point wo_reference_point( path.getInitialPoint() );
		std::vector<XY_Point> vertex_offsets;
		vertex_offsets.push_back( XY_Point(-.5, .25) );
		vertex_offsets.push_back( XY_Point(.5, .25) );
		vertex_offsets.push_back( XY_Point(.5, -.25) );
		vertex_offsets.push_back( XY_Point(-.5, -.25) );
		double velocity = 5.;
		double acc = 0.;
		position = 0.;
		
		// construct driver model
		XY_Point driver_reference( user_path.getInitialPoint() );
		std::vector<XY_Point> driver_vertex_offsets;

		/** backward triangle **
		driver_vertex_offsets.push_back( XY_Point(0., .5) );
		driver_vertex_offsets.push_back( XY_Point(1., .5) );
		driver_vertex_offsets.push_back( XY_Point(1., -.5) );
		/**/
		
		/** forward triangle **
		driver_vertex_offsets.push_back( XY_Point(0., .5) );
		driver_vertex_offsets.push_back( XY_Point(1., .5) );
		driver_vertex_offsets.push_back( XY_Point(0., -.5) );
		/**/
		
		/** square **/
		driver_vertex_offsets.push_back( XY_Point(0., .25) );
		driver_vertex_offsets.push_back( XY_Point(1., .25) );
		driver_vertex_offsets.push_back( XY_Point(1., -.25) );
		driver_vertex_offsets.push_back( XY_Point(0., -.25) );
		/**/

		/** diamond **
		driver_vertex_offsets.push_back( XY_Point(0., .0) );
		driver_vertex_offsets.push_back( XY_Point(.5, .5) );
		driver_vertex_offsets.push_back( XY_Point(1., 0.) );
		driver_vertex_offsets.push_back( XY_Point(.5, -.5) );
		/**/

		/** CO-LINEAR PATHS **
		VehicleModel driver_model( driver_reference,
								  driver_vertex_offsets,
								  user_path,
								  c,
								  0. );
		
		VehicleModel wo( driver_reference,
						vertex_offsets,
						user_path,
						c,
						position,
						std::numeric_limits<double>::quiet_NaN(),
						velocity,
						acc );
		/**/
		
		/** ORIGINAL PATHS **/
		VehicleModel driver_model( driver_vertex_offsets,
								  user_path,
								  c );
		
		Controller controller;
		controller.addControl( 0., c.getAMin() / 2. );
		VehicleModel wo( vertex_offsets,
						path,
						c,
						controller,
						position,
						c.getVMax() / 2.7 );
						//c.getVMax() / 10. );
						//velocity );
		/**/

		/** SWAPPED PATHS **
		VehicleModel driver_model( wo_reference_point, //driver_reference,
								  driver_vertex_offsets,
								  path, //user_path,
								  c,
								  0. );
		
		VehicleModel wo( driver_reference, //wo_reference_point,
						vertex_offsets,
						user_path, //path,
						//user_path_v,
						c,
						position,
						std::numeric_limits<double>::quiet_NaN(),
						velocity,
						acc );
		/**/

		/**
		// test arc length calculation
		XY_Point start_point1( 0., 10. );
		XY_Point end_point1( 8., 10. );
		double arc_length = user_path.getArcLengthBetweenPoints( start_point1, end_point1, c );
		if ( Maths::isNaN(arc_length) ) {
			std::cerr << "error calculating arc length" << std::endl;
			return 1;
		}
		std::cout << "Arc length: " << arc_length << std::endl;
		
		// test arc length calculation
		XY_Point start_point2( 4., 10. );
		XY_Point end_point2( 6., 10. );
		arc_length = user_path.getArcLengthBetweenPoints( start_point2, end_point2, c );
		if ( Maths::isNaN(arc_length) ) {
			std::cerr << "error calculating arc length" << std::endl;
			return 1;
		}
		std::cout << "Arc length: " << arc_length << std::endl;
		
		// test arc length calculation
		XY_Point start_point3( 4., 10. );
		XY_Point end_point3( 10., 8. );
		arc_length = user_path.getArcLengthBetweenPoints( start_point3, end_point3, c );
		if ( Maths::isNaN(arc_length) ) {
			std::cerr << "error calculating arc length" << std::endl;
			return 1;
		}
		std::cout << "Arc length: " << arc_length << std::endl;
		
		// test arc length calculation
		XY_Point start_point4( 4., 10. );
		XY_Point end_point4( 20., 5. );
		arc_length = user_path.getArcLengthBetweenPoints( start_point4, end_point4, c );
		if ( Maths::isNaN(arc_length) ) {
			std::cerr << "error calculating arc length" << std::endl;
			return 1;
		}
		std::cout << "Arc length: " << arc_length << std::endl;
		/**/
		
		/**
		std::cout << "Driver vehicle:" << std::endl;
		std::cout << driver_model << std::endl;
		std::cout << std::endl;
		
		std::cout << "Obstacle vehicle:" << std::endl;
		std::cout << wo << std::endl;
		std::cout << std::endl;
		
		std::cout << "==================================" << std::endl;
		std::cout << "==================================" << std::endl;
		std::cout << std::endl;
		/**/
		
		Utilities::GetMatlabWorldScene( "PVTP/Matlab/world_scene.m", driver_model, wo );
		//return 0;
		
		PVT_ObstacleSet O;
		std::vector<VehicleModel> WO;
		WO.push_back( wo );
		
		double first_path_position = std::numeric_limits<double>::quiet_NaN();
		double last_path_position = std::numeric_limits<double>::quiet_NaN();
		VehicleModel dm( driver_model );
		VehicleModel woo( wo );
		
		//woo.moveToPathPosition( 5.5, c );
		std::cout << "Driver vehicle:" << std::endl;
		std::cout << dm << std::endl;
		std::cout << std::endl;
		
		std::cout << "Obstacle vehicle:" << std::endl;
		std::cout << woo << std::endl;
		std::cout << std::endl;
		
		std::cout << "==================================" << std::endl;
		std::cout << "==================================" << std::endl;
		std::cout << std::endl;

		VehicleModel _dm( dm );
		VehicleModel _wo( woo );
		
		/**
		 * BEGIN: S_W diagram
		 */
		
		// move to junction
		VehicleModel wo_initial( woo );
		VehicleModel wo_prev( woo );
		VehicleModel wo_next( woo );
		VehicleModel wo_final( woo );
		wo_initial.moveToPathPosition( 2.8, c );
		wo_prev.moveToPathPosition( 3.999, c );
		wo_next.moveToPathPosition( 4.001, c );
		//VehicleModel * swept = VehicleModelUncertain::getSweptRegion( wo_prev, wo_next );
		VehicleModel * swept = VehicleModelUncertain::getSweptRegion( wo_initial, wo_prev );
		wo_final.moveToPathPosition( 5., c );
		std::vector<VehicleModel> junction_obs;
		junction_obs.push_back( wo_initial );
		junction_obs.push_back( wo_prev );
		//junction_obs.push_back( wo_next );
		junction_obs.push_back( *swept );
		//junction_obs.push_back( wo_final );
		
		// draw them
		Utilities::GetMatlabWorldScene( "PVTP/Matlab/junction_scene.m", dm, junction_obs );
		
		//return 0;
		
		/**
		 * END: S_W diagram
		 */
		
		/**
		XY_Point intersection_point( 5., 8.5 );
		XY_Point reference_test( 4.54289321881345209419578168308, 9.45710678118654790580421831692 );
		double edge_x1 = 5.60355339059327306472368945833;
		double edge_y1 = 9.10355339059327484108052885858;
		double edge_x2 = 4.89644660940672604709789084154;
		double edge_y2 = 8.39644660940672693527631054167;
		size_t i = 1;
		size_t j = 0;
		if ( Obstacles::checkPoint(first_path_position,
						last_path_position,
						intersection_point,
						reference_test,
						edge_x1,
						edge_y1,
						edge_x2,
						edge_y2,
						i,
						*dm.path,
						c) ) {
			std::cout << "first_path_position: " << first_path_position << ", last_path_position: " << last_path_position << std::endl;
		} else {
			std::cout << "fail" << std::endl;
		}
		//return 0;
		/**/
		/**
		PVT_ObstacleSet O_overlay;
		double start_resolution = 3.;
		double end_resolution = start_resolution;
		double zoom_rate = .1;
		//double zoom_rate = 0.125;
		double current_resolution = start_resolution;
		while ( current_resolution >= end_resolution ) {
			
			//std::cout << "resolution: " << current_resolution << std::endl;
			
			PVT_ObstacleSet O_tmp;
			std::vector<VehicleModel> WO_tmp;
			WO_tmp.push_back( WO.front() );
			std::vector<VehicleModelUncertain> WO_uncertain;
			WO_uncertain.push_back( VehicleModelUncertain(WO.front(), current_resolution, current_resolution) );
			//WO_uncertain.push_back( VehicleModelUncertain(WO.front(), 0., 0.) );
			//if ( !Obstacles::ForwardObstacleSimulator<VehicleModel>(O_tmp, WO_tmp, driver_model, c, current_resolution, 1.5, true) ) {
			//if ( !Obstacles::ForwardObstacleSimulator<VehicleModelUncertain>(O_tmp, WO_uncertain, driver_model, c, current_resolution, c.getTLimit(), true, true) ) {
			if ( !Obstacles::ForwardObstacleSimulator<VehicleModelUncertain>(O_tmp, WO_uncertain, driver_model, c, 0.005, 2., true, false) ) {
				std::cerr << "error in forward simulation" << std::endl;
				return 1;
			}
			
			//std::cout << O_tmp.obstacles.size() << " obstacles." << std::endl;
			
			O_overlay.addObstacles( O_tmp, c );
			
			//current_resolution *= zoom_rate;
			current_resolution -= zoom_rate;
		}
		
		Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O_overlay, T );
		//return 0;
		/**
		
		// run planner
		std::vector<PVT_G*> G1;
		std::vector<PVT_S*> Goal1;
		Interval V_f1( c.getVMin(), c.getVMax() );
		Interval V_i11( 0., 0. );
		
		c.setXLimit( driver_model.getPath().getLength() );
		c.setTLimit( 5. );
		
		// benchmarking
		struct timeval startTime11;
		struct timeval endTime11;
		
		// get the current time
		// - NULL because we don't care about time zone
		std::cout << "Planning... ";
		gettimeofday(&startTime11, NULL);
		if ( !Planner::Forward( G1, Goal1, V_i11, V_f1, O, c) ) {
			std::cerr << "Planner failed!" << std::endl;
			return 1;
		}
		gettimeofday(&endTime11, NULL);
		std::cout << "complete." << std::endl;
		
		// calculate time in microseconds
		double tS11 = startTime11.tv_sec * 1000000 + (startTime11.tv_usec);
		double tE11 = endTime11.tv_sec * 1000000  + (endTime11.tv_usec);
		std::cout << "Time elapsed: " << tE11 - tS11 << " microseconds" << std::endl;
		std::cout << std::endl;
		
		if ( !Goal1.empty() ) {
			if ( !Planner::BuildOptimalTrajectory(T, G1, Goal1, O, c) ) {
				std::cerr << "Failed to build optimal trajectory!" << std::endl;
				return 1;
			}
			std::cout << "Found optimal trajectory:" << std::endl;
			Utilities::PrintTrajectory( T );
			std::cout << std::endl;
		} else {
			std::cout << "No feasible trajectory." << std::endl;
			Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O, T );
			return 1;
		}
		std::cout << std::endl;
		
		double final_path_position = T.back()->getFinalState().getPathCoord();
		double final_time = T.back()->getFinalState().getTimeCoord();
		
		// output for visually debugging
		//Utilities::DescribeG( G, true );
		Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O, T );
		
		// free memory
		Utilities::CleanTrajectory( T );
		Utilities::CleanResults( G1, Goal1 );
		
		//std::cout << O_overlay.obstacles.size() << " total obstacles." << std::endl;
		
		//std::cout << Utilities::GetMatlabObstacles( O_overlay ) << std::endl;
		
		//Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O_overlay, T );
		
		//return 0;
		/**
		 * END: Test world obstacles
		 */
		
		/**
		std::vector<double> coefficients;
		coefficients.push_back( 2. );
		coefficients.push_back( -3. );
		coefficients.push_back( 0. );
		coefficients.push_back( 1. );
		coefficients.push_back( 7. );
		std::cout << "Horners: " << Maths::Horners( coefficients, 2., c.getEpsilon() ) << std::endl;
		return 0;
		/**/

		/**
		 * BEGIN: Trajectory displacement computations
		 */
		/**
		// linear portion
		T.push_back( new TrajectorySegment(0., 0., 1., 2., 2., 1) );
		
		// parabolic portion
		T.push_back( new TrajectorySegment(2., 2., 1., 5.5, 3., 6.) );
		
		// visualize
		std::cout << Utilities::GetMatlabTrajectory( T, "r" );
		
		std::cout << "Time at 1.5m: " << Utilities::trajectoryTimeAtDisplacement(T, 1.5) << "s" << std::endl;
		std::cout << "Time at 3m: " << Utilities::trajectoryTimeAtDisplacement(T, 3.) << "s" << std::endl;
		std::cout << "Path at 1.5s: " << Utilities::trajectoryDisplacementAtTime(T, 1.5) << "m" << std::endl;
		std::cout << "Path at 2.5s: " << Utilities::trajectoryDisplacementAtTime(T, 2.5) << "m" << std::endl;
		
		return 0;
		/**/
		/**
		 * END: Trajectory displacement computations
		 */
		
		/**
		double p2 = 9.;
		double t2 = 2.;
		double a2 = 5.;
		double b2;
		double c2;
		double a1 = 1.;
		double b1 = 1.;
		double c1 = 0.;
		double pp1;
		double t1;
		Maths::tangentParabolaThroughPoint( a2, b2, c2, pp1, t1, a1, b1, c1, p2, t2, c.getEpsilon() );
		std::cout << "b2: " << b2 << ", c2: " << c2 << ", p1: " << pp1 << ", t1: " << t1 << std::endl;
		return 0;
		/**/
		
		/**
		double p3 = 8.;
		double t3 = 4.;
		double a2 = -5.;
		double b2;
		double c2;
		double a1 = 1.;
		double b1 = 1.;
		double c1 = 0.;
		double b3 = 0.5;
		double c3 = p3 - b3 * t3;
		double pp1;
		double t1;
		double p2;
		double t2;
		Maths::tangentParabolaThroughLine( a2, b2, c2, pp1, t1, p2, t2, a1, b1, c1, b3, p3, t3, c.getEpsilon() );
		std::cout << "b2: " << b2 << ", c2: " << c2 << ", p1: " << pp1 << ", t1: " << t1 << ", p2: " << p2 << ", t2: " << t2 << std::endl;
		return 0;
		/**/
		
		/**
		double p3 = 6.;
		double t3 = 5.;
		double a1 = 0.5 * 0.1;
		double b1 = 0.5;
		double c1 = 0.;
		double a2 = 0.5 * 8;
		double a3 = 0.5 * -10.;
		double v3 = 0.;
		
		double pp1;
		double t1;
		double p2;
		double t2;
		double b2;
		double c2;
		Maths::tangentParabolaThroughParabola( a2, b2, c2, pp1, t1, p2, t2, a1, b1, c1, a3, v3, p3, t3, c.getEpsilon() );
		std::cout << "b2: " << b2 << ", c2: " << c2 << ", p1: " << pp1 << ", t1: " << t1 << ", p2: " << p2 << ", t2: " << t2 << std::endl;
		return 0;
		/**/
		
		/**
		Interval i( 0., std::numeric_limits<double>::max() );
		PVT_Point _p1( 0., 0. );
		PVT_Point target( 7.25, 6.75, i );
		//Interval V_i( c.getVMin(), c.getVMax() );
		Interval V_i( 0., 0. );
		
		// get nominal bounding trajectories
		std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > BoundingTrajectories;
		BoundingTrajectories.first = NULL;
		BoundingTrajectories.second = NULL;
		if ( !Planner::NextReachableSet(BoundingTrajectories, _p1, target, V_i, c) ) {
			std::cerr << "ERROR IN Planner::Propagate: NextReachableSet failed" << std::endl;
			return false;
		}
		if ( (BoundingTrajectories.first == NULL) || (BoundingTrajectories.second == NULL) ) {
			std::cout << "Planner::Propagate: No bounding trajectories found" << std::endl;
			return true;
		}
		
		for ( size_t i=0; i<1; i++ ) {
			std::vector<TrajectorySegment*> T_UB;
			if ( !Planner::GetCollisionBoundary(T_UB, *BoundingTrajectories.first, NULL, true, _p1, target, V_i, V_i, O_test, NULL, c) ) {
				std::cerr << "Error." << std::endl;
				return 1;
			}
			Utilities::SanitizeTrajectory( T_UB, c );
			
			std::vector<TrajectorySegment*> T_LB;
			if ( !Planner::GetCollisionBoundary(T_LB, *BoundingTrajectories.second, NULL, false, _p1, target, V_i, V_i, O_test, NULL, c) ) {
				std::cerr << "Error." << std::endl;
				return 1;
			}
			Utilities::SanitizeTrajectory( T_LB, c );
			
			std::vector< std::vector<TrajectorySegment*>* > T_list;
			T_list.push_back( &T_UB );
			T_list.push_back( &T_LB );
			
			if ( T_UB.empty() && T_LB.empty() ) {
				std::cout << "No trajectory found." << std::endl;
			} else {
				std::cout << "Upper bounding trajectory: " << std::endl;
				Utilities::PrintTrajectory( T_UB );
				std::cout << "Lower bounding trajectory: " << std::endl;
				Utilities::PrintTrajectory( T_LB );
			}
			
			Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O_test, T_list );
			
			Utilities::CleanTrajectory( T_UB );
			Utilities::CleanTrajectory( T_LB );
		}
		
		return 0;
		/**/

		/**
		PVT_Point pp1( 0., 0. );
		Interval V_i( 3., c.getVMax() );
		PVT_ObstaclePoint p_o_1( -0.1, 0., Constants::H_CLASS_ORIGIN, c );
		PVT_ObstaclePoint p_o_2( 3.4, 1., Constants::H_CLASS_ORIGIN, c );
		PVT_ObstacleEdge _edge( p_o_1, p_o_2, c );
		std::cout << "edge: " << _edge << std::endl;
		if ( !Planner::ConnectPointToEdge( T, pp1, V_i, _edge, c ) ) {
			std::cerr << "Error connecting to edge." << std::endl;
			return 1;
		}
		if ( !T.empty() ) {
			std::cout << "Connecting trajectory: " << std::endl;
			Utilities::PrintTrajectory( T );
		} else {
			std::cout << "No feasible connection." << std::endl;
		}
		
		return 0;
		/**/
		
		/**
		PVT_State s( 0., 0., 1.5 );
		PVT_Point p( 1.4, 0.7 );
		PVT_ObstaclePoint p_o_1( 0., 0., Constants::H_CLASS_ORIGIN, c );
		PVT_ObstaclePoint p_o_2( 1.5, 1., Constants::H_CLASS_ORIGIN, c );
		PVT_ObstacleEdge _edge( p_o_1, p_o_2, c );
		std::cout << "edge: " << _edge << std::endl;
		if ( !Planner::ConnectEdgeToPoint(T, false, _edge, s, p, c) ) {
			std::cerr << "Error connecting to point." << std::endl;
			return 1;
		}
		if ( !T.empty() ) {
			std::cout << "Connecting trajectory: " << std::endl;
			Utilities::PrintTrajectory( T );
		} else {
			std::cout << "No feasible connection." << std::endl;
		}
		
		return 0;
		/**/

		/**
		//
		// BEGIN: Reaction time calculation
		//
		
		std::vector<PVT_G*> G;
		std::vector<PVT_S*> Goal;
		
		/**
		double boxes[][4] = {
			//{ 6., 7., 4., 6. },
			//{ 4., 6., 6., 7. },
			{ 5., 6., 3., 5. }
		};
		PVT_ObstacleSet O( boxes, 1, c );
		/**/
		/**
		
		std::cout << O << std::endl;
		
		double initial_velocity = 0.5;
		//double initial_velocity = 3.;
		double control = 0.;
		//double control = 1.;
		
		Interval V_i( initial_velocity, initial_velocity );
		Interval V_f( c.getVMin(), c.getVMax() );
		
		// do forward propagation
		if ( !Planner::Forward(G, Goal, V_i, V_f, O, c) ) {
			std::cerr << "Error running Forward. Quitting." << std::endl;
			return 1;
		}
		
		// do backward propagation
		std::vector<PVT_G*> G_intersect;
		if ( !Planner::Backward(G_intersect, G, Goal, V_i, O, c) ) {
			std::cerr << "Error running Backward. Quitting." << std::endl;
			return 1;
		}
		Utilities::CleanResults( G, Goal );
		
		// find last feasible reaction trajectory
		std::vector<TrajectorySegment*> last_feasible_reaction;
		if ( !Evaluator::computeLastFeasibleReaction(last_feasible_reaction, initial_velocity, control, G_intersect, O, V_f, c) ) {
			std::cerr << "Error getting last feasible reaction. Quitting." << std::endl;
			return 1;
		}
		Utilities::CleanG( G_intersect );
		
		// print out result
		std::cout << "Last feasible reaction trajectory:" << std::endl;
		if ( last_feasible_reaction.empty() ) {
			std::cout << "Reaction not necessary for current horizon." << std::endl;
		} else {
			Utilities::PrintTrajectory( last_feasible_reaction );
			std::cout << std::endl;
		}
		std::cout << std::endl;
		return 0;
		
		//
		// END: Reaction time calculation
		//
		/**/
		
		/**
		//
		// BEGIN: Channel tests
		//
		
		std::cout << std::endl;
		std::cout << "Channel tests:" << std::endl;
		std::cout << std::endl;
		
		// get points
		std::vector<PVT_ObstaclePoint*> P_t;
		O_test.getPotentiallyReachableVertices( P_t, c );
		
		// add origin, if necessary
		PVT_ObstaclePoint p1( 0., 0., Constants::H_CLASS_ORIGIN, c );
		if ( true ) {
			P_t.push_back( &p1 );
		}
		
		// sort points by t-coord
		struct PVT_ObstaclePointComparator comp;
		comp.epsilon = c.getEpsilon();
		std::sort( P_t.begin(), P_t.end(), comp );
		
		std::vector<char> hClass;
		std::vector<TrajectorySegment*> T_test;
		PVT_State s1( 8.,   14.,   0. );
		PVT_State s2( 8.,   14.,   0. );
		PVT_State s3( 7.99999999999999911182158029987,   14.6771243444677050860036615632,   0. );
		PVT_State s4( 15.,   16.,  10.5830052442583628646843862953 );
		T_test.push_back( new TrajectorySegment(s1, s2) );
		T_test.push_back( new TrajectorySegment(s2, s3) );
		T_test.push_back( new TrajectorySegment(s3, s4) );
		std::cout << Utilities::GetMatlabTrajectory( T_test ) << std::endl;
		std::cout << "Classification for: long traj" << std::endl;
		if ( !Planner::Channel( hClass, T_test, O_test, P_t, c) ) {
			std::cout << "Channel classification failed." << std::endl;
		} else {
			if ( hClass.empty() ) {
				std::cout << "Empty channel classification due to collision." << std::endl;
			} else {
				std::cout << "Channel classification (length " << hClass.size() << "):" << std::endl;
				Utilities::PrintSignature( hClass );
				//std::cout << "Should be:" << std::endl;
				//std::cout << "0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 0 0 1 0 0 1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0" << std::endl;
			}
		}
		return 0;
		/**/

		/**
		//
		// BEGIN: Collisions tests
		//
		
		// obstacle: inverted pentagon
		std::vector< std::pair<double, double> > obs1;
		obs1.push_back( std::pair<double, double>(-1., 0.) );
		obs1.push_back( std::pair<double, double>(-1., 2.) );
		obs1.push_back( std::pair<double, double>(7., 10.) );
		obs1.push_back( std::pair<double, double>(7., 8.) );
		std::vector< std::pair<double, double> > obs2;
		obs2.push_back( std::pair<double, double>(0., 3.) );
		obs2.push_back( std::pair<double, double>(0., 5.) );
		obs2.push_back( std::pair<double, double>(8., 9.) );
		obs2.push_back( std::pair<double, double>(8., 7.) );
		std::vector< std::vector< std::pair<double, double> > > obstacles__;
		obstacles__.push_back( obs1 );
		//obstacles__.push_back( obs2 );
		PVT_ObstacleSet O_test( obstacles__, c );
		
		std::cout << "Collisions tests:" << std::endl;
		std::cout << std::endl;
		
		std::cout << "Obstacles: " << std::endl;
		std::cout << O_test << std::endl;
		std::cout << std::endl;
		
		std::vector<TrajectorySegment*> T_collision;
		T_collision.push_back( new TrajectorySegment(0.0625, 1.0625, 1., 4.29289, 5.29289, 1.) );
		
		std::cout << "Trajectory:" << std::endl;
		Utilities::PrintTrajectory( T_collision );
		std::cout << std::endl;
		
		PVT_ObstacleEdge * edge;
		std::set<PVT_Obstacle*> inCollision;
		if ( !Collisions::checkTrajectory(inCollision, T_collision, O_test, c, false, &edge) ) {
			std::cout << "Collision check failed." << std::endl;
		} else {
			if ( !inCollision.empty() ) {
				std::cout << "In collision with:" << std::endl;
				std::set<PVT_Obstacle*>::iterator it_j;
				for ( it_j=inCollision.begin(); it_j!=inCollision.end(); it_j++ ) {
					std::cout << **it_j << std::endl;
				}
				if ( edge != NULL ) {
					std::cout << "Collision with edge: " << *edge << std::endl;
				} else {
					std::cout << "Non-edge collision." << std::endl;
				}
			} else {
				std::cout << "Collision-free." << std::endl;
			}
		}
		std::cout << std::endl;
		return 0;
		//
		// END: Collisions tests
		//
		/**/
		
		/**
		//
		// BEGIN: Labelling tests
		//
		std::cout << std::endl << "Labelling tests:" << std::endl;
		
		double boxes1[][4] = {
			//{ 0., 8., 2., 10. } // force acceleration
			{1., 3., 0., 6. } // force braking
		};
		PVT_ObstacleSet O( boxes1, 1, c );
		
		double velocity = 5.;
		double control = 0.;
		double t_step = 0.1;
		double t_horizon = 1.;
		double reaction_horizon = 1.;
		
		std::vector<int> braking_labels;
		std::vector<int> acc_labels;
		std::vector<int> coasting_labels;
		if ( !Evaluator::getLabelling(braking_labels, acc_labels, coasting_labels, velocity, control, t_step, t_horizon, reaction_horizon, O, c) ) {
			std::cerr << "Failure in getLabelling." << std::endl;
			return 1;
		}
		
		std::cout << "Obstacles:" << std::endl;
		std::cout << O << std::endl;
		
		double new_velocity;
		double path_offset;
		Vehicle::calculateControlEffects( new_velocity,
										 path_offset,
										 velocity,
										 control,
										 t_horizon,
										 c.getAMin(),
										 c.getAMax(),
										 c.getVMin(),
										 c.getVMax() );
		
		std::cout << "Predicted trajectory:" << std::endl;
		TrajectorySegment t_seg( 0., 0., velocity, path_offset, t_horizon, new_velocity );
		std::vector<TrajectorySegment*> traj;
		traj.push_back( &t_seg );
		Utilities::PrintTrajectory( traj );
		std::cout << std::endl;
		
		std::cout << "Acceleration labels: ";
		for ( size_t i=0; i<acc_labels.size(); i++ ) {
			switch ( acc_labels[i] ) {
				case Evaluator::CONTROL_AVAILABLE_LABEL:
					std::cout << "x ";
					break;
				case Evaluator::P_CIS:
					std::cout << "o ";
					
				default:
					break;
			}
		}
		std::cout << std::endl;
		
		std::cout << "Braking labels: ";
		for ( size_t i=0; i<braking_labels.size(); i++ ) {
			switch ( braking_labels[i] ) {
				case Evaluator::CONTROL_AVAILABLE_LABEL:
					std::cout << "x ";
					break;
				case Evaluator::P_CIS:
					std::cout << "o ";
					
				default:
					break;
			}
		}
		std::cout << std::endl;
		
		std::cout << "Coasting labels: ";
		for ( size_t i=0; i<coasting_labels.size(); i++ ) {
			switch ( coasting_labels[i] ) {
				case Evaluator::CONTROL_AVAILABLE_LABEL:
					std::cout << "x ";
					break;
				case Evaluator::P_CIS:
					std::cout << "o ";
					
				default:
					break;
			}
		}
		std::cout << std::endl << std::endl;
		
		return 0;
		//
		// END: Labelling tests
		//
		/**/
		
		/**
		//
		// BEGIN: Warning map tests
		//
		std::cout << std::endl << "Warning map tests:" << std::endl;
		
		// obstacles: rectangles
		/**
		double boxes1[][4] = {
			{ 29.5/5., 37.5/5., 20.5/5., 31.5/5. },
			{ 22.5/5., 30.5/5., 32.5/5., 37.5/5. }
		};
		PVT_ObstacleSet O( boxes1, 2, c );
		/**/
		
		// obstacle: inverted pentagon
		/**
		std::vector< std::pair<double, double> > obs;
		obs.push_back( std::pair<double, double>(4., 7.) );
		obs.push_back( std::pair<double, double>(6., 7.) );
		obs.push_back( std::pair<double, double>(7., 6.) );
		obs.push_back( std::pair<double, double>(5., 5.) );
		obs.push_back( std::pair<double, double>(3., 6.) );
		std::vector< std::vector< std::pair<double, double> > > obstacles;
		obstacles.push_back( obs );
		PVT_ObstacleSet O( obstacles, c );
		/**/
		/**
		PVT_Point p( 27.5/5., 27.5/5. );
		for ( size_t i=0; i<O.obstacles->size(); i++ ) {
			std::cout << *O.obstacles->at(i) << "contains " << p << ": " << O.obstacles->at(i)->containsPoint(p, c) << std::endl << std::endl;
		}
		
		double u_step = 0.05;
		std::vector<double> control_grid;
		Evaluator::getControlGrid( control_grid, u_step, c );
		std::cout << "control_grid: " << control_grid.size() << std::endl;

		double p_step = 0.025;
		double t_step = 0.025;
		std::vector<PVT_Point*> space_grid;
		Evaluator::getPT_SpaceGrid( space_grid, p_step, t_step, c );
		std::cout << "space_grid: " << space_grid.size() << std::endl;
		
		double velocity = 0.;
		double control = 0.;
		double t_horizon = 1.;
		double time_step = 0.1;

#ifndef NAIVE
		// PERCEPTION METHOD
		std::string method = "perception";
		std::cout << method << " method..." << std::endl;
		double ttc_fore = 2.;
		double ttc_left = 2.;
		std::vector< std::pair<PVT_Point*, double>* > warning_map;
		if ( !Evaluator::evaluatePVT_S_SafeSafety( warning_map, velocity, control, ttc_fore, ttc_left, t_horizon, space_grid, O, c ) ) {
			std::cerr << "Error running evaluatePVT_S_SafeSafety" << std::endl;
			return 1;
		}
#endif

#ifdef NAIVE
		// NAIVE METHOD
		std::string method = "naive";
		std::cout << method << " method..." << std::endl;
		std::vector< std::pair<PVT_Point*, double>* > warning_map;
		if ( !Evaluator::evaluatePT_SpaceSafety( warning_map, velocity, space_grid, control_grid, t_step, t_horizon, control, O, c ) ) {
			std::cerr << "Error running evaluatePT_SpaceSafety" << std::endl;
			return 1;
		}
#endif
		std::cout << "warning_map: " << warning_map.size() << std::endl;
		
		size_t rows = (size_t)( c.getTLimit() / t_step ) + 1;
		size_t cols = (size_t)( c.getXLimit() / p_step ) + 1;
		SDoublePlane _map_image( rows, cols );
		for ( size_t i=0; i<warning_map.size(); i++ ) {
			size_t _r = (size_t)( 0.5 + (double)(warning_map[i]->first->getTimeCoord()) / t_step );
			size_t _c = (size_t)( 0.5 + (double)(warning_map[i]->first->getPathCoord()) / p_step );
			_map_image[_r][_c] = warning_map[i]->second;
		}
		
		size_t scale = 8;
		size_t row_index = 0;
		size_t col_index = 0;
		SDoublePlane map_image( rows * scale, cols * scale );
		for ( size_t i=0; i<(rows*scale); i++ ) {
			row_index = (i%scale)==0 ? i / scale : row_index;
			for ( size_t j=0; j<(cols*scale); j++ ) {
				col_index = (j%scale)==0 ? j / scale : col_index;
				map_image[i][j] =_map_image[row_index][col_index];
			}
		}
		
#ifndef NAIVE
		// PERCEPTION METHOD
		SDoublePlane r_map_image( map_image.rows(), map_image.cols() );
		SDoublePlane g_map_image( map_image.rows(), map_image.cols() );
		SDoublePlane b_map_image( map_image.rows(), map_image.cols() );
		for ( size_t i=0; i<map_image.rows(); i++ ) {
			for ( size_t j=0; j<map_image.cols(); j++ ) {
				size_t row_index = map_image.rows() - 1 - i;
				switch ( (const int)map_image[i][j] ) {
						
					case Evaluator::P_CIS:
						r_map_image[row_index][j] = 126;
						g_map_image[row_index][j] = 0;
						b_map_image[row_index][j] = 0;
						break;

					case Evaluator::P_RISKY_STRONG:
						r_map_image[row_index][j] = 255;
						g_map_image[row_index][j] = 0;
						b_map_image[row_index][j] = 0;
						break;
						
					case Evaluator::P_ICIS:
						r_map_image[row_index][j] = 255;
						g_map_image[row_index][j] = 96;
						b_map_image[row_index][j] = 96;
						break;
						
					case Evaluator::P_RISKY_WEAK:
						r_map_image[row_index][j] = 255;
						g_map_image[row_index][j] = 200;
						b_map_image[row_index][j] = 200;
						break;
						
					case Evaluator::P_SAFE:
						r_map_image[row_index][j] = 255;
						g_map_image[row_index][j] = 255;
						b_map_image[row_index][j] = 255;
						break;
					
					case Evaluator::P_IN_OBS:
						r_map_image[row_index][j] = 0;
						g_map_image[row_index][j] = 0;
						b_map_image[row_index][j] = 0;
						break;
						
					default:
						break;
				}
			}
		}
		std::stringstream ss;
		ss << method << "_output.png";
		SImageIO::write_png_file( ss.str().c_str(), r_map_image, g_map_image, b_map_image );
#endif

#ifdef NAIVE
		// NAIVE METHOD
		SDoublePlane scaled_map_image( map_image.rows(), map_image.cols() );
		SDoublePlane _scaled_map_image = ScaleImageValues( map_image );
		for ( size_t i=0; i<_scaled_map_image.rows(); i++ ) {
			for ( size_t j=0; j<_scaled_map_image.cols(); j++ ) {
				scaled_map_image[scaled_map_image.rows()-1-i][j] = _scaled_map_image[i][j];
			}
		}
		std::stringstream ss;
		ss << method << "_output.png";
		SImageIO::write_png_file( ss.str().c_str(), scaled_map_image, scaled_map_image, scaled_map_image );
#endif
		
		return 0;
		//
		// END: Warning map tests
		//
		/**/
		
		//
		// BEGIN: PVT Obstacle tests
		//
		/**
		std::cout << std::endl << "PVT Obstacle tests:" << std::endl;
		
		double box1[4] = { 1., 2., 1., 2. };
		double box2[4] = { 1., 2., 1., 2. };
		PVT_Obstacle obs1( box1, c );
		PVT_Obstacle obs2( box2, c );
		std::cout << "Box 1: " << obs1 << std::endl;
		std::cout << "Box 2: " << obs2 << std::endl;
		std::cout << "Box 1 == Box 2: " << (obs1.equals(obs2, c) ? "yes" : "no") << std::endl;
		
		std::cout << std::endl;
		/**/
		//
		// END: PVT Obstacle tests
		//
		
		//IO::GenerateScenarioSet( c );
		//IO::GenerateScenarios( c );
		//return 0;
		
		/**
		 * REQUIRES GSL
		 *
		double A = 4.;
		double B = 2.;
		double perim = Maths::approxEllipsePerimeter( A, B, c.getEpsilon() );
		std::cout << std::endl << "Ellipse perimeter with A = " << A << ", B = " << B << ": " << perim << std::endl;
		 
		double from = A - 1;
		double to = 1;
		double arc_length = Maths::ellipseArcLength( from, to, A, B, 0. );
		std::cout << "Ellipse arc length from x = " << from << " to x = " << to << ": " << arc_length << std::endl;
		std::cout << std::endl;
		 */
		
		/**
		//
		// BEGIN: API tests
		//
		std::cout << std::endl << "API tests:" << std::endl;
		
		PVT_Point p8(14.6999999999999992894572642399, 2.75010000000000021103119252075);
		PVT_Point p9(46.7999999999999971578290569596, 10.8000000000000007105427357601, Interval(8., 8.) );
		Interval V_i(0., 10.);
		std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
		Reachable.first = NULL;
		Reachable.second = NULL;
		if ( !Planner::NextReachableSet(Reachable, p8, p9, V_i, c) ) {
			std::cout << "Planner::NextReachableSet failed." << std::endl;
		} else {
			if ( Reachable.first == NULL ) {
				std::cout << "Next reachable set, upper-bounding states: empty." << std::endl;
			} else {
				std::cout << "Next reachable set, upper-bounding states:" << std::endl;
				Utilities::PrintTrajectory( *Reachable.first );
				Utilities::ValidTrajectory( *Reachable.first, c );
			}
			if ( Reachable.second == NULL ) {
				std::cout << "Next reachable set, lower-bounding states: empty." << std::endl;
			} else {
				std::cout << "Next reachable set, lower-bounding states:" << std::endl;
				Utilities::PrintTrajectory( *Reachable.second );
				Utilities::ValidTrajectory( *Reachable.second, c );
			}
		}
		std::cout << std::endl;
		return 0;
		/**/

		/**
		//Start state: (0, 0, 1), goal point: (2.91160755660011405510090298776, 0.911607556600115165323927612917, [0, 1])
		T.clear();
		PVT_State s_i(0., 0., 1.);
		Interval p_constraints(0., 1.);
		PVT_Point p6(2.91160755660011405510090298776, 0.911607556600115165323927612917);
		/**/

		/**
		T.clear();
		std::cout << "Upper-bounding states for: " << s_i << " -> " << p6 << ":" << std::endl;
		if ( Planner::UpperBoundingStates( T, s_i, p6, c ) ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T.at(i) << std::endl;
			}
		} else {
			std::cout << "Upper-bounding states routine failed." << std::endl;
		}
		return 0;
		/**/

		/**
		T.clear();
		std::cout << "Lower-bounding states for: " << s_i << " -> " << p6 << ":" << std::endl;
		if ( Planner::LowerBoundingStates( T, s_i, p6, c ) ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T.at(i) << std::endl;
			}
		} else {
			std::cout << "Lower-bounding states routine failed." << std::endl;
		}
		return 0;
		/**/

		/**
		T.clear();
		std::cout << "Lower-bounding states for: " << s_i << " -> " << p7 << ":" << std::endl;
		if ( Planner::LowerBoundingStates( T, s_i, p7, c ) ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T.at(i) << std::endl;
			}
		} else {
			std::cout << "Lower-bounding states routine failed." << std::endl;
		}
		std::cout << std::endl;
		/**/

		/**
		Interval* I = new Interval();
		PVT_Point pp1( 0., 0. );
		PVT_Point p2( 42.8, 32.75 );
		PVT_Point p3( 4., 1. );
		PVT_Point p4( 4., 2. );
		PVT_Point p5( 1., 4. );
		if ( !Planner::FindInitialVelocityRange( *I, pp1, p2, c ) ) {
			std::cout << "Planner::FindInitialVelocityRange Test failed." << std::endl;
		} else {
			std::cout << "Initial velocity range for " << pp1 << " -> " << p2 << ": " << *I << std::endl;
		}
		delete( I );
		return 0;
		/**/
		/**
		 I->setEmpty( true );
		 if ( !Planner::FindInitialVelocityRange( *I, p1, p3, c ) ) {
		 std::cout << "Planner::FindInitialVelocityRange Test failed." << std::endl;
		 } else {
		 std::cout << "Initial velocity range for " << p1 << " -> " << p3 << ": " << *I << std::endl;
		 }
		 std::cout << "Should be approx.: [2, 6]" << std::endl;
		 I->setEmpty( true );
		 if ( !Planner::FindInitialVelocityRange( *I, p1, p4, c ) ) {
		 std::cout << "Planner::FindInitialVelocityRange Test failed." << std::endl;
		 } else {
		 std::cout << "Initial velocity range for " << p1 << " -> " << p4 << ": " << *I << std::endl;
		 }
		 std::cout << "Should be approx.: [0, 5.65685]" << std::endl;
		 I->setEmpty( true );
		 if ( !Planner::FindInitialVelocityRange( *I, p1, p5, c ) ) {
		 std::cout << "Planner::FindInitialVelocityRange Test failed." << std::endl;
		 } else {
		 std::cout << "Initial velocity range for " << p1 << " -> " << p5 << ": " << *I << std::endl;
		 }
		 std::cout << "Should be approx.: [0, 2.82843]" << std::endl;
		 I->setEmpty( true );
		 
		 std::cout << std::endl;
		 //
		 // END: API tests
		 //
		/**/
		
		
		/**
		 * BEGIN: Test control sequence
		 *
		Controller controller;
		controller.addControl( 0.5, 4. );
		controller.addControl( 0.75, 5. );
		controller.addControl( 0.1, 2. );
		controller.addControl( 0.3, 3. );
		controller.addControl( 0., 1. );
		controller.addControl( -0.5, 6. );
		controller.addControl( 4., 7. );
		std::cout << "Control sequence [time, control]:" << std::endl;
		std::cout << controller << std::endl << std::endl;
		
		std::cout << "Control sequence, random access (time: control):" << std::endl;
		double time = -1.;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 0.;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 0.5;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 0.51;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 4.;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 4.3;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 3.1;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		time = 0.01;
		std::cout << time << ": " << controller.getControl( time ) << std::endl;
		std::cout << std::endl;
		/**
		 * END: Test control sequence
		 */
		
		/**
		 * BEGIN: Test scenario construction
		 *
		
		// begin and end points for obstacle path
		XY_Point obs_begin( 20., 15. );
		XY_Point obs_end( 20., -15. );
		
		// construct obstacle path
		Path obstacle_path( obs_begin, obs_end );

		// construct obstacle controller
		Controller obstacle_controller;
		obstacle_controller.addControl( 0., 0. );

		// construct obstacle vehicle
		// ( double width, double length, double min_vel, double max_vel, double min_acc, double max_acc )
		Vehicle obstacle_vehicle( 3., 5.5, 0., 10., -4., 4. );
		obstacle_vehicle.setVelocity( 5. );

		// construct scenario obstacle
		ScenarioObstacle obstacle( obstacle_vehicle, obstacle_path, obstacle_controller );
		
		// begin and end point for user path
		XY_Point user_begin( 0., 0. );
		XY_Point user_end( 35., 0. );
		
		// construct user path
		Path user_path( user_begin, user_end );
		
		// construct user vehicle
		Vehicle user_vehicle( 3., 5.5, 0., 10., -4., 4. );
		
		// construct scenario user
		// ScenarioUser ( Vehicle& vehicle, Path& path, double final_vel_min, double fina_vel_max, double alpha, Constraints& c )
		ScenarioUser user( user_vehicle, user_path, 0., 5., 0.95, c );
		
		// add obstacle to user's scenario
		user.getScenario().addObstacle( obstacle );
		
		// construct set for storing path-time obstacle
		PVT_ObstacleSet pt_obstacles;
		
		// compute path-time obstacles
		if ( user.getScenario().computePathTimeObstacles( pt_obstacles, c ) ) {
			std::cout << "Obstacle field size: " << pt_obstacles.obstacles->size() << std::endl;
			std::cout << pt_obstacles << std::endl;
			std::cout << std::endl;
			
			double x_offset = 1.;
			double t_offset = 2.;
			std::cout << "Translate obstacles by (" << x_offset << ", " << t_offset << "):" << std::endl;
			pt_obstacles.translateObstacles( x_offset, t_offset );
			std::cout << pt_obstacles << std::endl;

		} else {
			std::cout << "PT Obstacle construction failed." << std::endl;
		}
		
		std::cout << std::endl;
		/**
		 * END: Test scenario construction
		 */
		
		//return 0;
		
		/**
		std::cout << "BuildGoalPLP tests:" << std::endl;
		// p1: (5, 3, [-Inf, +Inf]), V_i: [0, 8.93308457365091790336464327993], vf: 0
		PVT_Point p13( 5., 3. );
		Interval V_i_goal( 0., 50. );
		double vf = 0.;
		std::vector<TrajectorySegment*> T_goal;
		if ( !Planner::BuildGoalPLP(T_goal, p13, V_i_goal, vf, true, c) ) {
			std::cout << "Planner::BuildGoalPLP failed." << std::endl;
		} else {
			if ( T_goal.empty() ) {
				std::cout << "No goal trajectory found." << std::endl;
			} else {
				Utilities::PrintTrajectory( T_goal );
			}
		}
		std::cout << std::endl;
		return 0;
		/**/
		
		/**
		//(36, 6.051282051282051099860837, [-Inf, +Inf]) [10.67912324841437943234723 11.13563054225920723183663 ] -> (44, 8.066666666666666429819088, 13.09644028880568633610437)
		PVT_State s1( 36, 6.051282051282051099860837, 10.67912324841437943234723 );
		PVT_State s2( 44, 8.066666666666666429819088, 13.09644028880568633610437 );
		std::cout << s1 << " -> " << s2 << std::endl;
		if ( !Planner::BuildPLP(T, s1, s2, c) ) {
			std::cout << "Planner::BuildPLP failed." << std::endl;
		} else {
			if ( T.size() == 0 ) {
				std::cout << "Planner::BuildPLP: No trajectory found." << std::endl;
			} else {
				for ( size_t i=0; i<T.size(); i++ ) {
					std::cout << *T.at(i) << std::endl;
				}	
			}
		}
		std::cout << std::endl;
		return 0;
		/**/
		
		/**
		PVT_State s_i( 49.5, 3.248148148148148450786721, 15.23154621172781730820134 );
		PVT_Point p( 64., 60. );
		Planner::UpperBoundingStates( T, s_i, p, c );
		Utilities::PrintTrajectory( T );
		T.clear();
		std::cout << std::endl;
		return 0;
		/**
		PVT_State s_i( 5.754841661031391,   7.745294492671754,   9.362852819271437 );
		PVT_Point p( 23.087968956077532,  10.012966068335706 );
		Planner::LowerBoundingStates( T, s_i, p, c );
		for ( size_t i=0; i<T.size(); i++ ) {
			std::cout << *T.at(i) << std::endl;
		}
		T.clear();
		std::cout << std::endl;
		return 0;
		/**/
		
		/**
		Interval V_i;
		PVT_Point p1( 14.7, 2.7501 );
		PVT_Point p2( 29.4, 5.5002 );
		Planner::FindInitialVelocityRange( V_i, p1, p2, c );
		std::cout << V_i << std::endl << std::endl;

		PVT_State s_i( p1.getPathCoord(), p1.getTimeCoord(), 0. );
		Planner::UpperBoundingStates( T, s_i, p2, c );
		for ( size_t k=0; k<T.size(); k++ ) {
			std::cout << *T.at(k) << std::endl;
		}
		T.clear();
		std::cout << std::endl;
		
		Planner::LowerBoundingStates( T, s_i, p2, c );
		for ( size_t k=0; k<T.size(); k++ ) {
			std::cout << *T.at(k) << std::endl;
		}
		T.clear();
		std::cout << std::endl;
		return 0;
		/**
		Interval V_i2;
		PVT_Point p22( 12.375, 2.25 );
		PVT_State s_i2( 0, 0, 10. );
		Planner::UpperBoundingStates( T, s_i2, p22, c );
		Utilities::PrintTrajectory( T );
		T.clear();
		std::cout << std::endl;
		
		Planner::LowerBoundingStates( T, s_i2, p22, c );
		Utilities::PrintTrajectory( T );
		T.clear();
		std::cout << std::endl;
		return 0;
		/**/

		
		
		/**
		double crazy_collision_boxes[][4] = {
			//{-10.68500000000001293187779, -2.185000000000007602807273, -3.149999999999986144416653, -1.449999999999994404475956}
			{-10.68500000000001648459147, -2.185000000000012043699371, -3.149999999999986588505863, -1.449999999999994848565166}
		};
		size_t crazy_collision_obstacle_field_size = 1;
		/**/
		/**
		double crazy_collision_boxes[][4] = {
			{8.714999999999998081534613, 16.59999999999999786837179, 0., 34.57076380574687135549539},
			{21.99499999999999744204615, 29.87999999999999545252649, 0., 37.49126180030111044061414},
			{35.27499999999999147348717, 43.15999999999998948396751, 0., 49.7999999999999900524017}
		};
		size_t crazy_collision_obstacle_field_size = 3;
		/**/
		
		/**
		double crazy_collision_boxes[][4] = {
			/**
			// O_staircase: 150
			{    0.6500,    1.1500,    0.6500,    1.1500},
			{    1.5500,    2.0500,    1.5500,    2.0500},
			{    2.4500,    2.9500,    2.4500,    2.9500},
			{    3.3500,    3.8500,    3.3500,    3.8500},
			{    4.2500,    4.7500,    4.2500,    4.7500},
			// 5
			{    5.1500,    5.6500,    5.1500,    5.6500},
			{    6.0500,    6.5500,    6.0500,    6.5500},
			{    6.9500,    7.4500,    6.9500,    7.4500},
			{    7.8500,    8.3500,    7.8500,    8.3500},
			{    8.7500,    9.2500,    8.7500,    9.2500},
			// 10
			{    9.6500,   10.1500,    9.6500,   10.1500},
			{   10.5500,   11.0500,   10.5500,   11.0500},
			{   11.4500,   11.9500,   11.4500,   11.9500},
			{   12.3500,   12.8500,   12.3500,   12.8500},
			{   13.2500,   13.7500,   13.2500,   13.7500},
			// 15
			{   14.1500,   14.6500,   14.1500,   14.6500},
			{   15.0500,   15.5500,   15.0500,   15.5500},
			{   15.9500,   16.4500,   15.9500,   16.4500},
			{   16.8500,   17.3500,   16.8500,   17.3500},
			{   17.7500,   18.2500,   17.7500,   18.2500},
			{   18.6500,   19.1500,   18.6500,   19.1500},
			{   19.5500,   20.0500,   19.5500,   20.0500},
			{   20.4500,   20.9500,   20.4500,   20.9500},
			{   21.3500,   21.8500,   21.3500,   21.8500},
			{   22.2500,   22.7500,   22.2500,   22.7500},
			// 25
			{   23.1500,   23.6500,   23.1500,   23.6500},
			{   24.0500,   24.5500,   24.0500,   24.5500},
			{   24.9500,   25.4500,   24.9500,   25.4500},
			{   25.8500,   26.3500,   25.8500,   26.3500},
			{   26.7500,   27.2500,   26.7500,   27.2500},
			{   27.6500,   28.1500,   27.6500,   28.1500},
			{   28.5500,   29.0500,   28.5500,   29.0500},
			{   29.4500,   29.9500,   29.4500,   29.9500},
			{   30.3500,   30.8500,   30.3500,   30.8500},
			{   31.2500,   31.7500,   31.2500,   31.7500},
			{   32.1500,   32.6500,   32.1500,   32.6500},
			{   33.0500,   33.5500,   33.0500,   33.5500},
			{   33.9500,   34.4500,   33.9500,   34.4500},
			{   34.8500,   35.3500,   34.8500,   35.3500},
			{   35.7500,   36.2500,   35.7500,   36.2500},
			{   36.6500,   37.1500,   36.6500,   37.1500},
			{   37.5500,   38.0500,   37.5500,   38.0500},
			{   38.4500,   38.9500,   38.4500,   38.9500},
			{   39.3500,   39.8500,   39.3500,   39.8500},
			{   40.2500,   40.7500,   40.2500,   40.7500},
			{   41.1500,   41.6500,   41.1500,   41.6500},
			{   42.0500,   42.5500,   42.0500,   42.5500},
			{   42.9500,   43.4500,   42.9500,   43.4500},
			{   43.8500,   44.3500,   43.8500,   44.3500},
			{   44.7500,   45.2500,   44.7500,   45.2500},
			// 50
			{   45.6500,   46.1500,   45.6500,   46.1500},
			{   46.5500,   47.0500,   46.5500,   47.0500},
			{   47.4500,   47.9500,   47.4500,   47.9500},
			{   48.3500,   48.8500,   48.3500,   48.8500},
			{   49.2500,   49.7500,   49.2500,   49.7500},
			{   50.1500,   50.6500,   50.1500,   50.6500},
			{   51.0500,   51.5500,   51.0500,   51.5500},
			{   51.9500,   52.4500,   51.9500,   52.4500},
			{   52.8500,   53.3500,   52.8500,   53.3500},
			{   53.7500,   54.2500,   53.7500,   54.2500},
			{   54.6500,   55.1500,   54.6500,   55.1500},
			{   55.5500,   56.0500,   55.5500,   56.0500},
			{   56.4500,   56.9500,   56.4500,   56.9500},
			{   57.3500,   57.8500,   57.3500,   57.8500},
			{   58.2500,   58.7500,   58.2500,   58.7500},
			{   59.1500,   59.6500,   59.1500,   59.6500},
			{   60.0500,   60.5500,   60.0500,   60.5500},
			{   60.9500,   61.4500,   60.9500,   61.4500},
			{   61.8500,   62.3500,   61.8500,   62.3500},
			{   62.7500,   63.2500,   62.7500,   63.2500},
			{   63.6500,   64.1500,   63.6500,   64.1500},
			{   64.5500,   65.0500,   64.5500,   65.0500},
			{   65.4500,   65.9500,   65.4500,   65.9500},
			{   66.3500,   66.8500,   66.3500,   66.8500},
			{   67.2500,   67.7500,   67.2500,   67.7500},
			// 75
			{   68.1500,   68.6500,   68.1500,   68.6500},
			{   69.0500,   69.5500,   69.0500,   69.5500},
			{   69.9500,   70.4500,   69.9500,   70.4500},
			{   70.8500,   71.3500,   70.8500,   71.3500},
			{   71.7500,   72.2500,   71.7500,   72.2500},
			{   72.6500,   73.1500,   72.6500,   73.1500},
			{   73.5500,   74.0500,   73.5500,   74.0500},
			{   74.4500,   74.9500,   74.4500,   74.9500},
			{   75.3500,   75.8500,   75.3500,   75.8500},
			{   76.2500,   76.7500,   76.2500,   76.7500},
			{   77.1500,   77.6500,   77.1500,   77.6500},
			{   78.0500,   78.5500,   78.0500,   78.5500},
			{   78.9500,   79.4500,   78.9500,   79.4500},
			{   79.8500,   80.3500,   79.8500,   80.3500},
			{   80.7500,   81.2500,   80.7500,   81.2500},
			{   81.6500,   82.1500,   81.6500,   82.1500},
			{   82.5500,   83.0500,   82.5500,   83.0500},
			{   83.4500,   83.9500,   83.4500,   83.9500},
			{   84.3500,   84.8500,   84.3500,   84.8500},
			{   85.2500,   85.7500,   85.2500,   85.7500},
			{   86.1500,   86.6500,   86.1500,   86.6500},
			{   87.0500,   87.5500,   87.0500,   87.5500},
			{   87.9500,   88.4500,   87.9500,   88.4500},
			{   88.8500,   89.3500,   88.8500,   89.3500},
			{   89.7500,   90.2500,   89.7500,   90.2500},
			// 100
			{   90.6500,   91.1500,   90.6500,   91.1500},
			{   91.5500,   92.0500,   91.5500,   92.0500},
			{   92.4500,   92.9500,   92.4500,   92.9500},
			{   93.3500,   93.8500,   93.3500,   93.8500},
			{   94.2500,   94.7500,   94.2500,   94.7500},
			{   95.1500,   95.6500,   95.1500,   95.6500},
			{   96.0500,   96.5500,   96.0500,   96.5500},
			{   96.9500,   97.4500,   96.9500,   97.4500},
			{   97.8500,   98.3500,   97.8500,   98.3500},
			{   98.7500,   99.2500,   98.7500,   99.2500},
			{   99.6500,  100.1500,   99.6500,  100.1500},
			{  100.5500,  101.0500,  100.5500,  101.0500},
			{  101.4500,  101.9500,  101.4500,  101.9500},
			{  102.3500,  102.8500,  102.3500,  102.8500},
			{  103.2500,  103.7500,  103.2500,  103.7500},
			{  104.1500,  104.6500,  104.1500,  104.6500},
			{  105.0500,  105.5500,  105.0500,  105.5500},
			{  105.9500,  106.4500,  105.9500,  106.4500},
			{  106.8500,  107.3500,  106.8500,  107.3500},
			{  107.7500,  108.2500,  107.7500,  108.2500},
			{  108.6500,  109.1500,  108.6500,  109.1500},
			{  109.5500,  110.0500,  109.5500,  110.0500},
			{  110.4500,  110.9500,  110.4500,  110.9500},
			{  111.3500,  111.8500,  111.3500,  111.8500},
			{  112.2500,  112.7500,  112.2500,  112.7500},
			// 125
			{  113.1500,  113.6500,  113.1500,  113.6500},
			{  114.0500,  114.5500,  114.0500,  114.5500},
			{  114.9500,  115.4500,  114.9500,  115.4500},
			{  115.8500,  116.3500,  115.8500,  116.3500},
			{  116.7500,  117.2500,  116.7500,  117.2500},
			{  117.6500,  118.1500,  117.6500,  118.1500},
			{  118.5500,  119.0500,  118.5500,  119.0500},
			{  119.4500,  119.9500,  119.4500,  119.9500},
			{  120.3500,  120.8500,  120.3500,  120.8500},
			{  121.2500,  121.7500,  121.2500,  121.7500},
			{  122.1500,  122.6500,  122.1500,  122.6500},
			{  123.0500,  123.5500,  123.0500,  123.5500},
			{  123.9500,  124.4500,  123.9500,  124.4500},
			{  124.8500,  125.3500,  124.8500,  125.3500},
			{  125.7500,  126.2500,  125.7500,  126.2500},
			{  126.6500,  127.1500,  126.6500,  127.1500},
			{  127.5500,  128.0500,  127.5500,  128.0500},
			{  128.4500,  128.9500,  128.4500,  128.9500},
			{  129.3500,  129.8500,  129.3500,  129.8500},
			{  130.2500,  130.7500,  130.2500,  130.7500},
			{  131.1500,  131.6500,  131.1500,  131.6500},
			{  132.0500,  132.5500,  132.0500,  132.5500},
			{  132.9500,  133.4500,  132.9500,  133.4500},
			{  133.8500,  134.3500,  133.8500,  134.3500},
			{  134.7500,  135.2500,  134.7500,  135.2500}
			// 150

			/**
			// O_3
			//{1.1241,    2.6241,    4.1877,    4.6877},
			//{0.4210,    1.9210,    2.0534,    2.5534},
			//{2.2813,    3.7813,    1.0874,    1.5874},
			//{2.4726,    3.9726,    7.2504,    7.7504},
			//{4.8258,    6.3258,    1.7376,    2.2376},
			//{6.9486,    8.4486,    1.6403,    2.1403},
			//{6.8001,    8.3001,    5.9109,    6.4109},
			//{3.4552,    4.9552,    7.8523,    8.3523},
			//{1.9333,    3.4333,    4.6407,    5.1407},
			//{5.5973,    7.0973,    6.2230,    6.7230},
			//{5.5653,    7.0653,    1.5555,    2.0555},
			//{5.4345,    6.9345,    8.3544,    8.8544},
			//{5.4558,    6.9558,    4.8475,    5.3475},
			//{0.9914,    2.4914,    6.0277,    6.5277},
			{5.0209,    6.5209,    0.5608,    1.0608}
			/**/
			/**
			// O_2
			//{7.1655,    8.6655,    3.2316,    3.7316},
			//{1.4430,    2.9430,    6.0771,    6.5771},
			//{2.0545,    3.5545,    2.7503,    3.2503},
			//{3.0276,    4.5276,    4.7603,    5.2603},
			//{0.7680,    2.2680,    7.3256,    7.8256},
			//{5.0387,    6.5387,    5.3287,    5.8287},
			//{3.0667,    4.5667,    3.1001,    3.6001},
			//{7.1298,    8.6298,    2.7934,    3.2934},
			//{3.0653,    4.5653,    4.0970,    4.5970},
			//{4.5947,    6.0947,    3.8425,    4.3425},
			//{1.3306,    2.8306,    3.3067,    3.8067},
			//{2.9194,    4.4194,    4.9957,    5.4957},
			//{1.3779,    2.8779,    6.5616,    7.0616},
			//{5.5568,    7.0568,    3.8568,    4.3568},
			{6.3478,    7.8478,    3.8995,    4.3995}
			/**/
			/**
			// O_1
			//{1.0360,    2.5360,    4.3946,    4.8946},
			//{5.7410,    7.2410,    6.7861,    7.2861},
			//{2.2910,    3.7910,    3.6161,    4.1161},
			//{4.4747,    5.9747,    2.5700,    3.0700},
			//{7.0010,    8.5010,    0.5665,    1.0665},
			//{3.2774,    4.7774,    5.9730,    6.4730},
			//{5.1133,    6.6133,    3.9013,    4.4013},
			//{5.5567,    7.0567,    4.0898,    4.5898},
			//{3.2785,    4.7785,    5.4338,    5.9338},
			//{4.8385,    6.3385,    0.7549,    1.2549},
			//{1.0183,    2.5183,    2.9344,    3.4344},
			//{6.7863,    8.2863,    6.8181,    7.3181},
			//{1.5622,    3.0622,    6.1697,    6.6697},
			//{2.1133,    3.6133,    1.3153,    1.8153},
			{5.8348,    7.3348,    1.3563,    1.8563}
			/**/
			/**
			{2.5, 12, 15.45, 16.3},
			{2.5, 12, 11.05, 11.9},
			{2.5, 12, 7.65, 8.5},
			{2.5, 12, 5.25, 6.1},
			{2.5, 12, 2.65, 3.5},
			{2.5, 12, 0.95, 1.8},
			{10.5, 20, 0, 0.115385},
			{10.5, 20, 2.07692, 2.73077},
			{10.5, 20, 4.69231, 5.34615},
			{10.5, 20, 6.53846, 7.19231},
			{10.5, 20, 9.15385, 9.80769},
			{10.5, 20, 11, 11.6538},
			{18.5, 28, 1.43478, 2.17391},
			{18.5, 28, 5.78261, 6.52174},
			{18.5, 28, 7.86957, 8.6087},
			{18.5, 28, 9.95652, 10.6957},
			{18.5, 28, 11, 11.7391},
			{18.5, 28, 12.7391, 13.4783},
			{26.5, 36, 1.12821, 2},
			{26.5, 36, 3.58974, 4.46154},
			{26.5, 36, 6.05128, 6.92308},
			{26.5, 36, 8.51282, 9.38462},
			{26.5, 36, 10.9744, 11.8462},
			{26.5, 36, 13.4359, 14.3077},
			{34.5, 44, 1.57143, 2.38095},
			{34.5, 44, 4.80952, 5.61905},
			{34.5, 44, 8.04762, 8.85714},
			{34.5, 44, 10.9048, 11.7143},
			{34.5, 44, 12.619, 13.4286},
			{34.5, 44, 14.9048, 15.7143},
			{42.5, 52, 0.916667, 1.625},
			{42.5, 52, 2.91667, 3.625},
			{42.5, 52, 4.91667, 5.625},
			{42.5, 52, 6.91667, 7.625},
			{42.5, 52, 8.91667, 9.625},
			{50.5, 60, 1.65, 2.5},
			{50.5, 60, 5.05, 5.9},
			{50.5, 60, 8.45, 9.3},
			{50.5, 60, 9.65, 10.5},
			{50.5, 60, 12.15, 13},
			{50.5, 60, 13.65, 14.5}
			/**/
			/**
			{2.5, 12.0, 15.45, 16.3},
			{2.5, 12.0, 13.05, 13.9},
			{2.5, 12.0, 9.65, 10.5},
			{2.5, 12.0, 7.25, 8.1},
			{2.5, 12.0, 2.65, 3.5},
			{2.5, 12.0, 0.95, 1.8},
			{10.5, 20.0, 0.2222222222222222, 0.8518518518518519},
			{10.5, 20.0, 2.0, 2.6296296296296298},
			{10.5, 20.0, 3.7777777777777777, 4.407407407407407},
			{10.5, 20.0, 5.555555555555555, 6.185185185185185},
			{10.5, 20.0, 7.333333333333333, 7.962962962962963},
			{10.5, 20.0, 9.11111111111111, 9.74074074074074},
			{18.5, 28.0, 1.3529411764705883, 2.3529411764705883},
			{18.5, 28.0, 7.823529411764706, 8.823529411764707},
			{18.5, 28.0, 9.470588235294118, 10.470588235294118},
			{18.5, 28.0, 12.294117647058824, 13.294117647058824},
			{18.5, 28.0, 14.882352941176471, 15.882352941176471},
			{18.5, 28.0, 17.235294117647058, 18.235294117647058},
			{26.5, 36.0, 1.1282051282051282, 2.0},
			{26.5, 36.0, 3.58974358974359, 4.461538461538462},
			{26.5, 36.0, 6.051282051282051, 6.923076923076923},
			{26.5, 36.0, 8.512820512820513, 9.384615384615385},
			{26.5, 36.0, 10.974358974358974, 11.846153846153847},
			{26.5, 36.0, 13.435897435897436, 14.307692307692308},
			{34.5, 44.0, 2.2, 3.3333333333333335},
			{34.5, 44.0, 8.066666666666666, 9.2},
			{34.5, 44.0, 11.266666666666667, 12.4},
			{34.5, 44.0, 15.266666666666667, 16.4},
			{34.5, 44.0, 17.666666666666668, 18.8},
			{34.5, 44.0, 20.866666666666667, 22.0},
			{42.5, 52.0, 0.0, 0.0},
			{42.5, 52.0, 1.2222222222222223, 2.1666666666666665},
			{42.5, 52.0, 3.888888888888889, 4.833333333333333},
			{42.5, 52.0, 6.555555555555555, 7.5},
			{42.5, 52.0, 9.222222222222221, 10.166666666666666},
			{42.5, 52.0, 11.88888888888889, 12.833333333333334},
			{50.5, 60.0, 1.1, 1.6666666666666667},
			{50.5, 60.0, 4.033333333333333, 4.6},
			{50.5, 60.0, 5.633333333333334, 6.2},
			{50.5, 60.0, 6.433333333333334, 7.0},
			{50.5, 60.0, 8.1, 8.666666666666666},
			{50.5, 60.0, 9.433333333333334, 10.0}
			/**/
			/**
			 {38.5, 48, -7.63333333333333463599501556018, -6.78333333333333321490954403998},
			 {38.5, 48, -5.23333333333333428072364768013, -4.38333333333333463599501556018},
			 {38.5, 48, -1.83333333333333392545227980008, -0.983333333333334280723647680134},
			 {38.5, 48, 0.566666666666666429819088079967, 1.41666666666666607454772019992},
			 {38.5, 48, 5.16666666666666607454772019992, 6.01666666666666571927635231987},
			 {38.5, 48, 6.86666666666666625218340413994, 7.71666666666666589691203625989},
			 {30.5, 40, 7.81481481481481399242738916655, 8.44444444444444464181742660003},
			 {30.5, 40, 6.03703703703703631333610246656, 6.66666666666666607454772019992},
			 {30.5, 40, 4.25925925925925863424481576658, 4.88888888888888839545643349993},
			 {30.5, 40, 2.48148148148148095515352906659, 3.11111111111111071636514679994},
			 {30.5, 40, 0.703703703703703276062242366606, 1.33333333333333303727386009996},
			 {30.5, 40, -1.07407407407407440302904433338, -0.444444444444444641817426600028},
			 {22.5, 32, 6.31372549019607731679570861161, 7.31372549019607731679570861161},
			 {22.5, 32, -0.156862745098040434754693706054, 0.843137254901960453423725994071},
			 {22.5, 32, -1.80392156862745167700268211775, -0.803921568627451677002682117745},
			 {22.5, 32, -4.62745098039215818630509602372, -3.62745098039215818630509602372},
			 {22.5, 32, -7.21568627450980493165388907073, -6.21568627450980493165388907073},
			 {22.5, 32, -9.56862745098039191304906125879, -8.56862745098039191304906125879},
			 {14.5, 24, 6.66666666666666607454772019992, 7.53846153846153832489562773844},
			 {14.5, 24, 4.20512820512820439944334793836, 5.07692307692307664979125547688},
			 {14.5, 24, 1.7435897435897427243389756768, 2.61538461538461497468688321533},
			 {14.5, 24, -0.717948717948718950765396584757, 0.153846153846153299582510953769},
			 {14.5, 24, -3.17948717948718062586976884631, -2.30769230769230837552186130779},
			 {14.5, 24, -5.64102564102564230097414110787, -4.76923076923077005062623356935},
			 {6.5, 16, 5.33333333333333214909544039983, 6.46666666666666589691203625989},
			 {6.5, 16, -0.533333333333333214909544039983, 0.59999999999999964472863211995},
			 {6.5, 16, -3.73333333333333428072364768013, -2.6000000000000014210854715202},
			 {6.5, 16, -7.73333333333333250436680827988, -6.6000000000000014210854715202},
			 {6.5, 16, -10.1333333333333346359950155602, -9.00000000000000177635683940025},
			 {6.5, 16, -13.3333333333333339254522798001, -12.2000000000000010658141036402},
			 {-1.5, 8, 6.5, 7.4444444444444437536390068999},
			 {-1.5, 8, 3.83333333333333303727386009996, 4.77777777777777679091286699986},
			 {-1.5, 8, 1.16666666666666607454772019992, 2.11111111111111071636514679994},
			 {-1.5, 8, -1.5, -0.555555555555555358182573399972},
			 {-1.5, 8, -4.16666666666666785090455960017, -3.22222222222222320908713300014},
			 {-9.5, -0, 6.99999999999999911182158029987, 7.56666666666666642981908807997},
			 {-9.5, -0, 4.06666666666666642981908807997, 4.63333333333333285963817615993},
			 {-9.5, -0, 2.46666666666666589691203625989, 3.03333333333333232673112433986},
			 {-9.5, -0, 1.66666666666666607454772019992, 2.23333333333333250436680827988},
			 {-9.5, -0, -0, 0.566666666666666429819088079967},
			 {-9.5, -0, -1.33333333333333392545227980008, -0.766666666666667495633191720117},
			 {50.5, 65, -1, 8.66666666666666607454772019992},
			/**/
		//};
		//size_t crazy_collision_obstacle_field_size = 150;
		//size_t crazy_collision_obstacle_field_size = 41;
		//size_t crazy_collision_obstacle_field_size = 42;
		//size_t crazy_collision_obstacle_field_size = 1;
		/**/
		/**/
		double crazy_collision_boxes[][4] = {
			// 78
			{1.5, 11.5, 10.7382, 16.4445},
			{1.5, 11.5, 22.7382, 28.4445},
			{1.5, 11.5, 34.7382, 40.4445},
			{1.5, 11.5, 46.7382, 52.4445},
			{1.5, 11.5, 58.7382, 64.4445},
			{1.5, 11.5, 70.7382, 76.4445},
			{6.3, 15.5, 4.1058, 7.0041},
			{6.3, 15.5, 12.1058, 15.0041},
			{6.3, 15.5, 20.1058, 23.0041},
			{6.3, 15.5, 28.1058, 31.0041},
			{6.3, 15.5, 36.1058, 39.0041},
			{6.3, 15.5, 44.1058, 47.0041},
			{10.6, 19.3, 7.991, 10.9832},
			{10.6, 19.3, 17.591, 20.5832},
			{10.6, 19.3, 27.191, 30.1832},
			{10.6, 19.3, 36.791, 39.7832},
			{10.6, 19.3, 46.391, 49.3832},
			{10.6, 19.3, 55.991, 58.9832},
			{14.7, 23.3, 1.0998, 2.7501},
			{14.7, 23.3, 7.0998, 8.7501},
			{14.7, 23.3, 13.0998, 14.7501},
			{14.7, 23.3, 19.0998, 20.7501},
			{14.7, 23.3, 25.0998, 26.7501},
			{14.7, 23.3, 31.0998, 32.7501},
			{18.8, 27.3, 3.5714, 5.4286},
			{18.8, 27.3, 10.4286, 12.2857},
			{18.8, 27.3, 17.2857, 19.1429},
			{18.8, 27.3, 24.1429, 26.0},
			{18.8, 27.3, 31.0, 32.8571},
			{18.8, 27.3, 37.8571, 39.7143},
			{22.8, 31.3, 7.4545, 9.8182},
			{22.8, 31.3, 16.1818, 18.5455},
			{22.8, 31.3, 24.9091, 27.2727},
			{22.8, 31.3, 33.6364, 36.0},
			{22.8, 31.3, 42.3636, 44.7273},
			{22.8, 31.3, 51.0909, 53.4545},
			{26.8, 35.3, 16.2857, 20.0},
			{26.8, 35.3, 30.0, 33.7143},
			{26.8, 35.3, 43.7143, 47.4286},
			{26.8, 35.3, 57.4286, 61.1429},
			{26.8, 35.3, 71.1429, 74.8571},
			{26.8, 35.3, 84.8571, 88.5714},
			{30.8, 39.3, 3.3333, 5.0667},
			{30.8, 39.3, 9.7333, 11.4667},
			{30.8, 39.3, 16.1333, 17.8667},
			{30.8, 39.3, 22.5333, 24.2667},
			{30.8, 39.3, 28.9333, 30.6667},
			{30.8, 39.3, 35.3333, 37.0667},
			{34.8, 43.3, 3.5714, 5.4286},
			{34.8, 43.3, 10.4286, 12.2857},
			{34.8, 43.3, 17.2857, 19.1429},
			{34.8, 43.3, 24.1429, 26.0},
			{34.8, 43.3, 31.0, 32.8571},
			{34.8, 43.3, 37.8571, 39.7143},
			{38.8, 47.3, 4.1667, 6.3333},
			{38.8, 47.3, 12.1667, 14.3333},
			{38.8, 47.3, 20.1667, 22.3333},
			{38.8, 47.3, 28.1667, 30.3333},
			{38.8, 47.3, 36.1667, 38.3333},
			{38.8, 47.3, 44.1667, 46.3333},
			{42.8, 51.3, 1.125, 2.75},
			{42.8, 51.3, 7.125, 8.75},
			{42.8, 51.3, 13.125, 14.75},
			{42.8, 51.3, 19.125, 20.75},
			{42.8, 51.3, 25.125, 26.75},
			{42.8, 51.3, 31.125, 32.75},
			{46.8, 55.3, 8.2, 10.8},
			{46.8, 55.3, 17.8, 20.4},
			{46.8, 55.3, 27.4, 30.0},
			{46.8, 55.3, 37.0, 39.6},
			{46.8, 55.3, 46.6, 49.2},
			{46.8, 55.3, 56.2, 58.8},
			{50.8, 59.3, 4.1667, 6.3333},
			{50.8, 59.3, 12.1667, 14.3333},
			{50.8, 59.3, 20.1667, 22.3333},
			{50.8, 59.3, 28.1667, 30.3333},
			{50.8, 59.3, 36.1667, 38.3333},
			{50.8, 59.3, 44.1667, 46.3333}
		};
		size_t crazy_collision_obstacle_field_size = 78;
		/**/
		/**
		double crazy_collision_boxes[][4] = {
			{10.600000000000001, 19.3, 1.395511131117074, 2.8915868908702445}
		};
		size_t crazy_collision_obstacle_field_size = 1;
		/**/
		/**
		double crazy_collision_boxes[][4] = {
		{10.600000000000001, 19.3, 1.29551, 2.79159}
		};
		size_t crazy_collision_obstacle_field_size = 1;
		/**/
		/**
		double crazy_collision_boxes[][4] = {
			{5.6, 14.3, 0.0455111, 1.54159}
		};
		size_t crazy_collision_obstacle_field_size = 1;
		/**/
		/**
		 double crazy_collision_boxes[][4] = {
		 {5.6, 14.3, 0.0455111, 1.54159}
		 };
		 size_t crazy_collision_obstacle_field_size = 1;
		 /**/
		/**
		 double crazy_collision_boxes[][4] = {
		 {8.800000000000000710542736, 17.5, 0.7455111311170740107101551, 2.241586890870244541673628}
		 };
		 size_t crazy_collision_obstacle_field_size = 1;
		 /**/
		/**
		 double crazy_collision_boxes[][4] = {
		 {10.40000000000000213162821, 19.10000000000000142108547, 0.04551113111707394409677363, 1.541586890870244364037944}
		 };
		 size_t crazy_collision_obstacle_field_size = 1;
		 /**/
		/**
		 double crazy_collision_boxes[][4] = {
		 {10.17500000000000071054274, 18.875, -0.004488868882926100312147355, 1.491586890870244319629023}
		 };
		 size_t crazy_collision_obstacle_field_size = 1;
		 /**/
		/**
		 double crazy_collision_boxes[][4] = {
		 {7.150000000000001243449788, 15.85000000000000142108547, -0.5544888688829261447210683, 0.9415868908702442752201023}
		 };
		 size_t crazy_collision_obstacle_field_size = 1;
		 /**/
		/**
		double crazy_collision_boxes[][4] = {
			{7.150000000000001243449788, 15.85000000000000142108547, -0.2044888688829260559032264, 1.291586890870244364037944}
		};
		size_t crazy_collision_obstacle_field_size = 1;
		/**/
		/**
		 double crazy_collision_boxes[][4] = {
		 {7.150000000000001243449788, 15.85000000000000142108547, -0.1044888688829259670853844, 1.391586890870244452855786}
		 };
		 size_t crazy_collision_obstacle_field_size = 1;
		 /**/
		PVT_ObstacleSet crazy_collision_obstacle_field( crazy_collision_boxes, crazy_collision_obstacle_field_size, c );

		/**
		std::cout << "Original obstacle set:" << std::endl;
		std::cout << crazy_collision_obstacle_field << std::endl;
		
		std::cout << "Translated and mirrored obstacle set:" << std::endl;
		PVT_Point p( 16., 2. ); 
		PVT_ObstacleSet translated_mirrored_obstacle_field = Utilities::TranslateAndMirrorObstacleSet( crazy_collision_obstacle_field, p, c );
		std::cout << translated_mirrored_obstacle_field << std::endl;
		/**/
		
		/**
		PVT_State s_to_goal(50.8, 46.3333, 5.2915);
		PVT_Point p1(50.8, 46.3333);
		Interval V_i(5.2915, 5.2915);
		Interval V_f(0., 50.);
		std::vector<TrajectorySegment*> UB;
		std::vector<TrajectorySegment*> LB;
		if ( !Planner::GoalConnect( UB, LB, p1, V_i, V_f, crazy_collision_obstacle_field, c ) ) {
			std::cout << "Goal connect failed." << std::endl;
		} else {
			std::cout << "UB:" << std::endl;
			for ( size_t i=0; i<UB.size(); i++ ) {
				std::cout << *UB.at(i) << std::endl;
			}
			std::cout << std::endl;
			std::cout << "LB:" << std::endl;
			for ( size_t i=0; i<LB.size(); i++ ) {
				std::cout << *LB.at(i) << std::endl;
			}
			std::cout << std::endl;
		}
		/**/
		
		/**/
		//
		// BEGIN: Forward tests
		//
		//std::cout << "Forward tests, " << crazy_collision_obstacle_field_size << " obstacles:" << std::endl;
		std::cout << "Forward tests, " << O_test.obstacles.size() << " obstacles:" << std::endl;
		//std::cout << crazy_collision_obstacle_field << std::endl;
		//std::cout << O_test << std::endl;
		
		std::vector<PVT_G*> G;
		std::vector<PVT_S*> Goal;
		
		//(3.1000000000000014210854715202, 5.4999999999999964472863211995, 0)
		PVT_Point pp( 3.1000000000000014210854715202, 5.4999999999999964472863211995 );
		Interval V_f( c.getVMin(), c.getVMax() );
		Interval V_i1( 0., 0. );
		
		/**
		Interval V_ii2( 11.086037983034679, 16.217974851766613 );
		//PVT_Point p_start( 9.500000000000000, 0.766666666666667 );
		PVT_Point p_start( 0., 0. );
		PVT_Point p_goal( 16.000000000000000, 1.366666666666667 );
		std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
		Planner::NextReachableSet( Reachable, p_start, p_goal, V_i1, c );
		std::vector<TrajectorySegment*> * UB = Reachable.first;
		std::vector<TrajectorySegment*> * LB = Reachable.second;
		
		std::vector<PVT_Obstacle*> inCollision;
		if ( Reachable.second != NULL ) {
			std::cout << "LB:" << std::endl;
			Utilities::PrintTrajectory( *Reachable.second );
			if ( !Collisions::checkTrajectory(inCollision, *Reachable.second, crazy_collision_obstacle_field, c) ) {
				std::cout << "Collision check failed." << std::endl;
			} else {
				if ( !inCollision.empty() ) {
					std::cout << "In collision with:" << std::endl;
					for ( size_t i=0; i<inCollision.size(); i++ ) {
						std::cout << *inCollision.at(i) << std::endl;
					}
				} else {
					std::cout << "Collision-free." << std::endl;
				}
			}
		}

		return 0;
		/**/
		
		/**/
		// ProcessTime example
		struct timeval startTime1;
		struct timeval endTime1;
		// get the current time
		// - NULL because we don't care about time zone
		gettimeofday(&startTime1, NULL);
		// algorithm goes here
		
		//c.setXLimit( 60. ); //vehicle_path_length );
		//c.setTLimit( 16. ); //max_time );
		V_i1.setBounds( 0., 0. );

		size_t iterations = 1;
		std::cout << std::endl << "Benchmarking: " << iterations << " iteration" << (iterations>1?"s":"") << "..." << std::endl << std::endl;
		for ( size_t test=0; test<iterations; test++ ) {
			//Planner::Forward( G, Goal, V_i1, V_f, crazy_collision_obstacle_field, c );
			Planner::Forward( G, Goal, V_i1, V_f, O_test, c );
			//std::vector<PVT_G*> G_intersect;
			//Planner::Backward( G_intersect, G, Goal, V_i1, crazy_collision_obstacle_field, c );
			//Planner::BuildOptimalTrajectory(T, G_intersect, Goal, crazy_collision_obstacle_field, c);
			//Planner::BuildOptimalTrajectory(T, G, Goal, crazy_collision_obstacle_field, c);
			Planner::BuildOptimalTrajectory(T, G, Goal, O_test, c);
			Utilities::PrintTrajectory( T );
			if ( Utilities::ValidTrajectory( T, c) ) {
				std::cout << "Trajectory: VALID" << std::endl;
			} else {
				std::cout << "Trajectory: INVALID" << std::endl;
			}
			//Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", crazy_collision_obstacle_field, T );
			Utilities::GenerateMatlabScene( "PVTP/Matlab/scene.m", O_test, T );
			//Utilities::GenerateMatlabReachableSets( "PVTP/Matlab/sets.m", O_test, G );
			Utilities::CleanTrajectory( T );
			//Utilities::DescribeG( G, true );
			//Utilities::DescribeGoal( Goal, true );
			//Utilities::CleanG( G_intersect );
			Utilities::CleanResults( G, Goal );
		}
		
		// collision test
		/**
		std::vector<TrajectorySegment*> T_collision_test;
		T_collision_test.push_back( new TrajectorySegment(0., 0., 0., 35.205555555555363866915286053, 2.9667134827766634685985991382, 23.7337078622133077487887931056) );
		T_collision_test.push_back( new TrajectorySegment(35.205555555555363866915286053, 2.9667134827766634685985991382, 23.7337078622133077487887931056, 63.3699999999996492761056288145, 5.34008426899799459874884632882, 0) );
		std::set<PVT_Obstacle*> inCollision;
		Collisions::checkTrajectory( inCollision, T_collision_test, crazy_collision_obstacle_field, c );
		if ( !inCollision.empty() ) {
			std::cout << "In collision with:" << std::endl;
			Utilities::PrintCollisionSet( inCollision );
		} else {
			std::cout << "Collision free." << std::endl;
		}
		/**/

		/**/
		// get the end time
		gettimeofday(&endTime1, NULL);
		// calculate time in microseconds
		double tS1 = startTime1.tv_sec * 1000000 + (startTime1.tv_usec);
		double tE1 = endTime1.tv_sec * 1000000  + (endTime1.tv_usec);
		std::cout << "Total Time Taken for " << iterations << " iterations: " << tE1 - tS1 << " microseconds" << std::endl;
		double per_iteration_time = ((tE1 - tS1) / iterations);
		std::cout << "Average per iteration time: " << per_iteration_time << " microseconds" << std::endl;
		//double matlab_per_iteration_time = 202755682.948;
		//std::cout << "Comparison to MATLAB per iteration time of " << matlab_per_iteration_time << " microseconds: " << (matlab_per_iteration_time / per_iteration_time) << "x" << std::endl;
		/**/
		return 0;
		/**/
		if ( Evaluator::getOptimalTrajectory( T, V_i1, V_f, crazy_collision_obstacle_field, c ) ) {
			std::cout << "Time-optimal trajectory:" << std::endl;
			Utilities::SanitizeTrajectory( T, c );
			Utilities::PrintTrajectory( T );
			/**
			std::vector<TrajectorySegment*> T_collision;
			T_collision.push_back( new TrajectorySegment(pp.getPathCoord(), 5.4999999999999964472863211995, -1.08244916406134899781191412013e-16,
														 pp.getPathCoord(), 6.52565835097474344905776888481, -1.08244916406134899781191412013e-16) );
			Utilities::translateTrajectory( T_collision, -pp.getPathCoord(), -pp.getTimeCoord() );
			
			std::set<PVT_Obstacle*> inCollision;
			Collisions::checkTrajectory( inCollision, T_collision, O_polygons, c );
			if ( !inCollision.empty() ) {
				std::cout << "In collision." << std::endl;
			} else {
				std::cout << "Collision free." << std::endl;
			}
			Utilities::PrintTrajectory( T_collision );
			inCollision.clear();
			Collisions::checkTrajectory( inCollision, T, O_polygons, c );
			if ( !inCollision.empty() ) {
				std::cout << "In collision." << std::endl;
			} else {
				std::cout << "Collision free." << std::endl;
			}
			Utilities::PrintTrajectory( T );
			/**/
			
			//Utilities::translateTrajectory( T, pp.getPathCoord(), pp.getTimeCoord() );
		}

		/**
		if ( !Planner::Forward(G, Goal, V_i1, V_f, crazy_collision_obstacle_field, c) ) {
			std::cerr << "Forward failed." << std::endl;
		} else {
			//std::cout << "Forward results:" << std::endl;
			//Utilities::DescribeG( G, true );
			//Utilities::DescribeGoal( Goal, true );
			/**
			for ( size_t i=0; i<Goal.size(); i++ ) {
				std::vector<PVT_Obstacle*> inCollision;
				if ( !Collisions::checkTrajectory(inCollision, *Goal.at(i)->UB, crazy_collision_obstacle_field, c) ) {
					std::cout << "Collision check failed." << std::endl;
				} else {
					if ( !inCollision.empty() ) {
						std::cout << "In collision with:" << std::endl;
						for ( size_t i=0; i<inCollision.size(); i++ ) {
							std::cout << *inCollision.at(i) << std::endl;
						}
					} else {
						std::cout << "Collision-free." << std::endl;
					}
				}				
			}

			std::vector<PVT_G*> G_intersect;
			//Goal.erase(Goal.begin());
			if ( !Planner::Backward(G_intersect, G, Goal, V_i1, crazy_collision_obstacle_field, c) ) {
				std::cerr << "Backward failed." << std::endl;
			} else {
				//std::cout << "Backward results:" << std::endl;
				//Utilities::DescribeG( G_intersect, true );
			}
	
			if ( !Planner::BuildOptimalTrajectory(T, G_intersect, Goal, crazy_collision_obstacle_field, c) ) {

			//if ( !Planner::BuildOptimalTrajectory(T, G, Goal, crazy_collision_obstacle_field, c) ) {
				std::cout << "Failed to build time-optimal trajectory." << std::endl;
			} else {
				std::cout << "Time-optimal trajectory:" << std::endl;
				Utilities::SanitizeTrajectory( T, c );
				Utilities::PrintTrajectory( T );
			}
			//Utilities::CleanTrajectory( T );
			std::cout << std::endl;

		}
		/**/
		
		std::cout << std::endl;
		return 0;
		//
		// END: Forward tests
		//
		/**
		
		//
		// BEGIN: Propagate Goal tests
		//
		
		std::cout << "Goal propagation tests:" << std::endl;
		
		std::vector<PVT_S*> Ss;
		PVT_ObstaclePoint p101( 15.5000, 4.1058, Constants::H_CLASS_ORIGIN );
		PVT_ObstaclePoint p102( 31.3000, 7.4545, Constants::H_CLASS_ORIGIN );
		Interval V_ii( 0., 10. );
		Interval V_ff( 0., 5. );
		
		if ( !Planner::PropagateGoal( Ss, p101, V_ii, V_ff, crazy_collision_obstacle_field, P_t, c )) {
			std::cout << "Goal propagation failed." << std::endl;
		} else {
			
			std::cout << "Goal propagated intervals: " << Ss.size() << std::endl;
			for ( size_t i=0; i<Ss.size(); i++ ) {
				std::cout << *Ss.at(i)->V << " ";
				for ( size_t j=0; j<Ss.at(i)->B->size(); j++ ) {
					std::cout << ((Ss.at(i)->B->at(j)==0)?"0":"1") << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		}
		
		//
		// END: Propagate Goal tests
		//
		
		//
		// BEGIN: BuildGoalPLP tests
		//
		T.clear();
		v1 = 1.;
		vf = 0.;
		p13.setCoords( 1., 5. );
		if ( !Planner::BuildGoalPLP(T, p13, v1, vf, c) ) {
			std::cout << "Planner::BuildGoalPLP failed." << std::endl;
		} else {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T.at(i) << std::endl;
			}
		}
		std::cout << std::endl;

		T.clear();
		v1 = 10.;
		vf = 10.;
		p13.setCoords( 50., 10. );
		if ( !Planner::BuildGoalPLP(T, p13, v1, vf, c) ) {
			std::cout << "Planner::BuildGoalPLP failed." << std::endl;
		} else {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T.at(i) << std::endl;
			}
		}
		std::cout << std::endl;
		
		std::cout << std::endl;
		//
		// END: BuildGoalPLP tests
		//

		//
		// BEGIN: Merge tests
		//
		
		std::cout << "Propagation tests:" << std::endl;
		
		std::vector<PVT_S*> S;
		PVT_Point p11( 15.5000, 4.1058 );
		PVT_Point p12( 31.3000, 7.4545 );
		Interval V_int( 0., 10. );
		if ( !Planner::Propagate( S, p11, p12, V_int, crazy_collision_obstacle_field, P_t, c )) {
			std::cout << "Propagation failed." << std::endl;
		} else {

			std::cout << "Propagated intervals: " << S.size() << std::endl;
			for ( size_t i=0; i<S.size(); i++ ) {
				std::cout << *S.at(i)->V << " ";
				for ( size_t j=0; j<S.at(i)->B->size(); j++ ) {
					std::cout << ((S.at(i)->B->at(j)==0)?"0":"1") << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			
			std::vector<char> tmp;
			tmp.push_back(1);
			PVT_S * S_el = new PVT_S( *(new Interval(2., 6.)), tmp );
			S.push_back( S_el );
			
			for ( size_t i=0; i<S.size(); i++ ) {
				std::cout << *S.at(i)->V << " ";
				for ( size_t j=0; j<S.at(i)->B->size(); j++ ) {
					std::cout << ((S.at(i)->B->at(j)==0)?"0":"1") << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			
			std::cout << "Merge tests:" << std::endl;
			std::vector<Interval*> V;
			Planner::Merge( V, S, c );
			for ( size_t i=0; i<V.size(); i++ ) {
				std::cout << *V.at(i) << std::endl;
			}
		}
		
		std::cout << std::endl;
		//
		// END: Merge tests
		//

		//
		// BEGIN: isSuffix tests
		//
		std::cout << "isSuffix tests:" << std::endl;
		
		std::vector<char> B1;
		B1.push_back(0);
		B1.push_back(1);
		B1.push_back(1);
		
		std::vector<char> B_prime;
		B_prime.push_back(1);
		B_prime.push_back(0);
		B_prime.push_back(1);
		B_prime.push_back(1);
		
		std::cout << "0 1 1 is suffix of 1 0 1 1: " << Planner::isSuffix( B1, B_prime ) << std::endl;

		B_prime.clear();
		B_prime.push_back(1);
		B_prime.push_back(0);
		B_prime.push_back(1);
		B_prime.push_back(0);
		
		std::cout << "0 1 1 is suffix of 1 0 1 0: " << Planner::isSuffix( B1, B_prime ) << std::endl;

		B_prime.clear();
		B_prime.push_back(0);
		B_prime.push_back(1);
		B_prime.push_back(1);
		
		std::cout << "0 1 1 is suffix of 0 1 1: " << Planner::isSuffix( B1, B_prime ) << std::endl;
		B_prime.clear();

		B_prime.clear();
		B_prime.push_back(1);
		B_prime.push_back(1);
		
		std::cout << "0 1 1 is suffix of 1 1: " << Planner::isSuffix( B1, B_prime ) << std::endl;
		
		std::cout << std::endl;
		//
		// END: isSuffix tests
		//		
		/**/
		
		/**
		// ProcessTime example
		struct timeval startTime;
		struct timeval endTime;
		// get the current time
		// - NULL because we don't care about time zone
		gettimeofday(&startTime, NULL);
		// algorithm goes here

		size_t trials = 1;
		for ( size_t test=0; test<trials; test++ ) {
			Planner::Channel( hClass, T_collision, crazy_collision_obstacle_field, P_t, c);
		}
		
		// get the end time
		gettimeofday(&endTime, NULL);
		// calculate time in microseconds
		double tS = startTime.tv_sec*1000000 + (startTime.tv_usec);
		double tE = endTime.tv_sec*1000000  + (endTime.tv_usec);
		std::cout << "Total Time Taken for " << trials << " iterations: " << tE - tS << " microseconds" << std::endl;
		std::cout << "Average per iteration time: " << ((tE - tS) / trials) << " microseconds" << std::endl;
		
		/**
		
		double collision_boxes[][4] = {
			{1., 2., 1., 2. },
			{3., 4., 3., 4. },
			{5., 6., 5., 6. },
			{7., 8., 7., 8. }
		};
		PVT_ObstacleSet collision_obstacle_field( collision_boxes, 4 );
		
		P_t.clear();
		collision_obstacle_field.getAllVertices( P_t, c );
		
		// add origin
		P_t.push_back( &origin );
		
		comp.epsilon = c.getEpsilon();
		std::sort( P_t.begin(), P_t.end(), comp );
		
		TrajectorySegment * t1 = new TrajectorySegment( 1.5, 0., 0., 1.5, 3., 0. );
		TrajectorySegment * t2 = new TrajectorySegment( 0., 0., 0., 1., 2., 1. );
		TrajectorySegment * t3 = new TrajectorySegment( 0., 0., 1., 4., 4., 1. );
		
		hClass.clear();
		T_collision.clear();
		T_collision.push_back( t1 );
		
		std::cout << "Classification for: " << *t1 << std::endl;
		if ( !Planner::Channel( hClass, T_collision, collision_obstacle_field, P_t, c) ) {
			std::cout << "Channel classification failed." << std::endl;
		} else {
			if ( hClass.empty() ) {
				std::cout << "Empty channel classification due to collision." << std::endl;
			} else {
				std::cout << "Channel classification (length " << hClass.size() << "):" << std::endl;
				for ( size_t i=0; i<hClass.size(); i++ ) {
					std::cout << (hClass.at(i)?"1":"0") << " ";
				}
				std::cout << std::endl;
			}
		}
		
		T_collision.clear();
		T_collision.push_back( t2 );
		std::cout << "Classification for: " << *t2 << std::endl;
		if ( !Planner::Channel( hClass, T_collision, collision_obstacle_field, P_t, c) ) {
			std::cout << "Channel classification failed." << std::endl;
		} else {
			if ( hClass.empty() ) {
				std::cout << "Empty channel classification due to collision." << std::endl;
			} else {
				std::cout << "Channel classification (length " << hClass.size() << "):" << std::endl;
				for ( size_t i=0; i<hClass.size(); i++ ) {
					std::cout << (hClass.at(i)?"1":"0") << " ";
				}
				std::cout << std::endl;
			}
		}
		
		T_collision.clear();
		T_collision.push_back( t3 );
		std::cout << "Classification for: " << *t3 << std::endl;
		if ( !Planner::Channel( hClass, T_collision, collision_obstacle_field, P_t, c) ) {
			std::cout << "Channel classification failed." << std::endl;
		} else {
			if ( hClass.empty() ) {
				std::cout << "Empty channel classification due to collision." << std::endl;
			} else {
				std::cout << "Channel classification (length " << hClass.size() << "):" << std::endl;
				for ( size_t i=0; i<hClass.size(); i++ ) {
					std::cout << (hClass.at(i)?"1":"0") << " ";
				}
				std::cout << std::endl;
			}
		}
		
		std::cout << std::endl;
		//
		// END: Channel tests
		//

		//
		// BEGIN: Trajectory Segment tests
		//
		std::cout << std::endl << "Trajectory Segment tests:" << std::endl;
		
		PVT_State s1(0., 0., 0.);
		PVT_State s2(2., 1., 1.);
		PVT_State s3(1., 1., 1.);
		PVT_State s4(4., 4., 4.);
		TrajectorySegment traj1( s1, s2 );
		TrajectorySegment traj2( s3, s4 );
		TrajectorySegment traj3( s2, s3 );
		std::cout << traj1 << std::endl;
		std::cout << traj2 << std::endl;
		std::cout << traj3 << std::endl;
		
		std::cout << std::endl;
		//
		// END: Trajectory Segment tests
		//
		
		//
		// BEGIN: PVT State tests
		//
		std::cout << std::endl << "PVT State tests:" << std::endl;
		
		PVT_State pvt_s( 1., 1., 1. );
		std::cout << "State: " << pvt_s << std::endl;
		p1.setCoords( 0., 0. );
		p2.setCoords( 2., 2. );
		std::cout << "Can reach " << p1 << ": " << pvt_s.canReach( p1, c ) << std::endl;
		std::cout << "Can reach " << p2 << ": " << pvt_s.canReach( p2, c ) << std::endl;
		
		std::cout << std::endl;	
		//
		// END: PVT State tests
		//
		
		//
		// BEGIN: PVT Obstacle Field tests
		//
		std::cout << std::endl << "PVT Obstacle Set tests:" << std::endl;
		
		double boxes[][4] = {
			{1., 2., 1., 2. },
			{3., 4., 3., 4. },
			{5., 6., 5., 6. },
			{7., 8., 7., 8. }
		};
		PVT_ObstacleSet obstacle_field( boxes, 4 );
		std::cout << obstacle_field << std::endl;
		std::vector<PVT_ObstaclePoint*> vertices;
		obstacle_field.getAllVertices( vertices, c );
		std::cout << "Vertices: " << std::endl;
		for ( size_t i=0; i<vertices.size(); i++ ) {
			std::cout << *(vertices.at(i)) << std::endl;
		}
		
		std::cout << std::endl;
		//
		// END: PVT Obstacle Field tests
		//
		
		//
		// BEGIN: PVT Point tests
		//
		std::cout << std::endl << "PVT Point tests:" << std::endl;
		
		PVT_Point pvt_p1( 1., 2. );
		PVT_Point pvt_p2( 3., 4., *(new Interval(-1., 1.)) );
		PVT_Point pvt_p3( 1., 2. );
		std::cout << "Points are: " << pvt_p1 << ", and " << pvt_p2 << std::endl;
		std::cout << pvt_p1 << " == " << pvt_p2 << ": " << pvt_p1.equals( pvt_p2, c ) << std::endl;
		std::cout << pvt_p1 << " == " << pvt_p3 << ": " << pvt_p1.equals( pvt_p3, c ) << std::endl;
		
		std::cout << std::endl;
		//
		// END: PVT Point tests
		//
		
		//
		// BEGIN: Interval tests
		//
		std::cout << std::endl << "Interval tests:" << std::endl;
		
		Interval A( -1.01, 1.01 );
		Interval B( -1., 1. );
		
		std::cout << A << " subseteq " << B << " with epsilon " << c.getEpsilon() << ": " << A.isSubset( B, c ) << std::endl;
		tmp = c.getEpsilon();
		new_eps = .001;
		c.setEpsilon( new_eps );
		std::cout << A << " subseteq " << B << " with epsilon " << c.getEpsilon() << ": " << A.isSubset( B, c ) << std::endl;
		c.setEpsilon( tmp );
		
		std::cout << A << " == " << B << " with epsilon " << c.getEpsilon() << ": " << A.isEqual( B, c ) << std::endl;
		tmp = c.getEpsilon();
		new_eps = .001;
		c.setEpsilon( new_eps );
		std::cout << A << " == " << B << " with epsilon " << c.getEpsilon() << ": " << A.isEqual( B, c ) << std::endl;
		c.setEpsilon( tmp );
		
		double num = 0.;
		std::cout << num << " in " << A << ": " << A.contains( num, c ) << std::endl;
		num = -1.2;
		std::cout << num << " in " << A << ": " << A.contains( num, c ) << std::endl;
		
		A.setBounds(-2., 0.);
		B.setBounds(-1., 1.);
		Interval result = *(new Interval());
		Interval::intersect( result, A, B );
		std::cout << A << " intersect " << B << ": " << result << std::endl;
		A.setBounds(-2., 0.);
		B.setBounds(1., 2.);
		Interval::intersect( result, A, B );
		std::cout << A << " intersect " << B << ": " << result << std::endl;
		
		A.setBounds(-2., 1.);
		B.setBounds(-1., 2.);
		std::pair<Interval*, Interval*> union_result(new Interval(), new Interval());
		Interval::specialUnion( union_result, A, B, c );
		std::cout << A << " union " << B << ": " << *union_result.first << " " << *union_result.second << std::endl;
		
		A.setBounds(-2., -1.);
		B.setBounds(1., 2.);
		delete(union_result.first);
		delete(union_result.second);
		union_result.first = new Interval();
		union_result.second = new Interval();
		Interval::specialUnion( union_result, A, B, c );
		std::cout << A << " union " << B << ": " << *union_result.first << " " << *union_result.second << std::endl;
		delete(union_result.first);
		delete(union_result.second);
		
		A.setBounds(-2., -1.);
		B.setBounds(1., 2.);
		Interval C(3., 4.);
		Interval D(5., 6.);
		Interval E(7., 8.);
		std::vector<Interval*> intervals;
		intervals.push_back( &A );
		intervals.push_back( &B );
		intervals.push_back( &C );
		intervals.push_back( &D );
		intervals.push_back( &E );
		std::vector<Interval*> results;
		std::cout << A << " union " << B << " union " << C << " union " << D << " union " << E << ": ";
		Interval::specialUnionOfIntervals( results, intervals );
		for ( size_t i=0; i<results.size(); i++ ) {
			std::cout << *results.at(i) << " ";
		}
		std::cout << std::endl;
		
		A.setBounds(-2., -1.);
		B.setBounds(-1., 2.);
		C.setBounds(3., 4.);
		D.setBounds(5., 6.);
		E.setBounds(5.5, 8.);
		results.clear();
		std::cout << A << " union " << B << " union " << C << " union " << D << " union " << E << ": ";
		Interval::specialUnionOfIntervals( results, intervals );
		for ( size_t i=0; i<results.size(); i++ ) {
			std::cout << *results.at(i) << " ";
		}
		std::cout << std::endl;
		
		std::cout << std::endl;
		//
		// END: Interval tests
		//
		/**
		//
		// BEGIN: Maths tests
		//
		std::cout << std::endl << "Maths tests:" << std::endl;
		double epsilon = c.getEpsilon();
		
		std::pair< std::pair<double, double>, std::pair<double, double> > coords;
		Maths::parabolasTangentLine( coords, 1., -4., 4., 4., 4., epsilon );
		std::cout << "parabolasTangentLine(1, -4, 4, 4, 4, epsilon): (" << coords.first.first << ", " << coords.first.second << ") -> (" << coords.second.first << ", " << coords.second.second << ")" << std::endl;
		std::cout << "Should be approx.: (0.0072572, 0.00736571) -> (4.11774, 4.24263)" << std::endl;
		Maths::parabolasTangentLine( coords, 0., -4., 4., 4., 8., epsilon );
		std::cout << "parabolasTangentLine(0, -4, 4, 4, 8, epsilon): (" << coords.first.first << ", " << coords.first.second << ") -> (" << coords.second.first << ", " << coords.second.second << ")" << std::endl;
		std::cout << "Should be approx.: (-0.03031, -0.123106) -> (4.03031, 8.12311)" << std::endl;
		Maths::parabolasTangentLine( coords, 0., -10., 8., 8., 4., epsilon );
		std::cout << "parabolasTangentLine(0, -10, 8, 8, 4, epsilon): (" << coords.first.first << ", " << coords.first.second << ") -> (" << coords.second.first << ", " << coords.second.second << ")" << std::endl;
		std::cout << "Should be approx.: (-0.176014, -0.187624) -> (8.22002, 4.23453)" << std::endl;
		Maths::parabolasTangentLine( coords, 0., -4., -4., 8., 4., epsilon );
		std::cout << "parabolasTangentLine(0, -4, -4, 8, 4, epsilon): (" << coords.first.first << ", " << coords.first.second << ") -> (" << coords.second.first << ", " << coords.second.second << ")" << std::endl;
		std::cout << "Should be approx.: (-0.5, -0.5) -> (7.5, 3.5)" << std::endl;
		Maths::parabolasTangentLine( coords, 0., 4., 4., 8., 4., epsilon );
		std::cout << "parabolasTangentLine(0, 4, 4, 8, 4, epsilon): (" << coords.first.first << ", " << coords.first.second << ") -> (" << coords.second.first << ", " << coords.second.second << ")" << std::endl;
		std::cout << "Should be approx.: (0.5, 0.5) -> (8.5, 4.5)" << std::endl;
		
		
		double NaN = std::numeric_limits<double>::quiet_NaN();
		double inf = std::numeric_limits<double>::infinity();
		num = 0.;
		std::cout << "isNaN(" << NaN << "): " << Maths::isNaN( NaN ) << std::endl;
		std::cout << "isNaN(" << inf << "): " << Maths::isNaN( inf ) << std::endl;
		std::cout << "isNaN(" << num << "): " << Maths::isNaN( num ) << std::endl;
		std::cout << std::endl;
		
		double dA = 1.;
		double dB = 1.1;
		std::cout << "Epsilon: " << epsilon << std::endl;
		std::cout << dA << "==" << dB << ": " << Maths::approxEq( dA, dB, epsilon ) << std::endl;
		std::cout << dA << ">=" << dB << ": " << Maths::approxGe( dA, dB, epsilon ) << std::endl;
		std::cout << dA << "<=" << dB << ": " << Maths::approxLe( dA, dB, epsilon ) << std::endl;
		std::cout << dA << ">" << dB << ": " << Maths::approxGt( dA, dB, epsilon ) << std::endl;
		std::cout << dA << "<" << dB << ": " << Maths::approxLt( dA, dB, epsilon ) << std::endl;
		std::cout << dA << "!=" << dB << ": " << Maths::approxNe( dA, dB, epsilon ) << std::endl;
		
		epsilon *= 10.;
		std::cout << "Epsilon: " << epsilon << std::endl;
		std::cout << dA << "==" << dB << ": " << Maths::approxEq( dA, dB, epsilon ) << std::endl;
		std::cout << dA << ">=" << dB << ": " << Maths::approxGe( dA, dB, epsilon ) << std::endl;
		std::cout << dA << "<=" << dB << ": " << Maths::approxLe( dA, dB, epsilon ) << std::endl;
		std::cout << dA << ">" << dB << ": " << Maths::approxGt( dA, dB, epsilon ) << std::endl;
		std::cout << dA << "<" << dB << ": " << Maths::approxLt( dA, dB, epsilon ) << std::endl;
		std::cout << dA << "!=" << dB << ": " << Maths::approxNe( dA, dB, epsilon ) << std::endl;
		
		std::cout << std::endl;
		//
		// END: Maths tests
		//
		/**/
		
	} catch ( int e ) {
		ExceptionHandler::exceptionMessage( e );
		return -1;
	}
}