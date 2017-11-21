/***
 * PVT Planner
 */

#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <PVTP/Planner.hpp>
#include <PVTP/ScenarioGenerator.hpp>
#include <PVTP/ExceptionHandler.hpp>
#include <PVTP/Utilities.hpp>

using namespace PVTP;
using namespace Scenario;

int main () {
	double tmp, new_eps;
	
	try {
		
		// all units in meters and seconds
		double vehicle_path_length = 250.;
		double max_time = 41.;
		double v_min = 0.;
		double v_max = 13.4;
		double a_min = -10.;
		double a_max = 8.;
		Constraints c( vehicle_path_length, max_time, v_min, v_max, a_min, a_max );

		std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
		//std::cout.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
		
		std::cout << std::endl << "SYSTEM CONSTRAINTS: " << c;
		std::cout << std::endl << "Machine epsilon: " << std::numeric_limits<double>::epsilon() << std::endl;
		std::cout << std::endl;
		
		//
		// Scenario Paths
		//
		
		// driver path
		std::vector<PathSegment> bus_path_segments;
		XY_Point seg_start( 0., 0. );
		XY_Point seg_end( 0., 0. );
		int student_count = 0;
		
		seg_start.setCoords( -1.1561778423698428, 0. );
		seg_end.setCoords( -144.2909773772417, 0. );
		bus_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		PathModel bus_path( bus_path_segments, c );
		
		std::cout << "Bus path:" << std::endl;
		std::cout << bus_path << std::endl;
		std::cout << std::endl;
		
		// student paths
		std::vector<PathModel> student_paths;
		std::vector<PathSegment> student_path_segments;
		
		// next student path
		seg_start.setCoords( -69.37055190853786, 13.73498826118272 );
		seg_end.setCoords( -74.68896689277733, 4.905555305765622 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -74.68896689277733, 4.905555305765622 );
		seg_end.setCoords( -74.45773096556498, -11.772293111186924 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -74.45773096556498, -11.772293111186924 );
		seg_end.setCoords( -78.15750843531383, -17.331582947350725 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -79.31360797124422, 19.294351403906273 );
		seg_end.setCoords( -77.69496000016501, 5.232664458220427 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -77.69496000016501, 5.232664458220427 );
		seg_end.setCoords( -77.23248808775497, -12.75325124050229 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -77.23248808775497, -12.75325124050229 );
		seg_end.setCoords( -78.61990440507132, -16.023422056736862 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -80.7014971827419, -27.14199958139532 );
		seg_end.setCoords( -77.92666582423267, -12.426219716519077 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -77.92666582423267, -12.426219716519077 );
		seg_end.setCoords( -77.92666582423267, 15.370176883889231 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -88.79472598606829, -23.217701342647725 );
		seg_end.setCoords( -91.33832001558935, -8.174912098611376 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -91.33832001558935, -8.174912098611376 );
		seg_end.setCoords( -92.26326163595427, 5.886799174055147 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -92.26326163595427, 5.886799174055147 );
		seg_end.setCoords( -89.02597273276798, 22.2375950515927 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -88.10107244568295, -28.122972667508737 );
		seg_end.setCoords( -88.56354430008355, -11.772106346347346 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -88.56354430008355, -11.772106346347346 );
		seg_end.setCoords( -71.22090124545248, 11.446084304990801 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );

		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -90.64406275411686, 24.526723731224944 );
		seg_end.setCoords( -90.18159202127134, 7.194884812550576 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -90.18159202127134, 7.194884812550576 );
		seg_end.setCoords( -77.69485351883648, -10.791016422149788 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -77.69485351883648, -10.791016422149788 );
		seg_end.setCoords( -67.05799864835998, -13.40714517033458 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -47.17214295103341, -14.061579543331256 );
		seg_end.setCoords( -39.07888313010947, -14.06157535597857 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -39.07888313010947, -14.06157535597857 );
		seg_end.setCoords( -34.45417386789737, 3.924344733988951 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -34.45417386789737, 3.924344733988951 );
		seg_end.setCoords( -26.12969824026976, 4.905398087319547 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -101.51258052358766, -12.425977777159508 );
		seg_end.setCoords( -114.23055439266955, -10.4638656145496 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -114.23055439266955, -10.4638656145496 );
		seg_end.setCoords( -123.94243734580567, 9.157131585150715 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path
		seg_start.setCoords( -151.22819520380258, -6.5388704766572125 );
		seg_end.setCoords( -140.82258544109604, -6.5388635544398035 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -140.82258544109604, -6.5388635544398035 );
		seg_end.setCoords( -128.10464188283098, 8.176887391207117 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );

		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path (bicycle, turn off)
		seg_start.setCoords( -20.642902016117002, -1.6350707349451907 );
		seg_end.setCoords( -69.37068124665369, -0.6538225713805081 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -69.37068124665369, -0.6538225713805081 );
		seg_end.setCoords( -80.93245489027407, 3.5973995707471342 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -80.93245489027407, 3.5973995707471342 );
		seg_end.setCoords( -81.8573944995878, 22.891339989630893 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		// next student path (bicycle, turn on)
		seg_start.setCoords( -80.47026753023648, -27.796037244713784 );
		seg_end.setCoords( -80.70150349610228, -13.080256640865006 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -80.70150349610228, -13.080256640865006 );
		seg_end.setCoords( -84.86374496458178, -1.9616841080692993 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -84.86374496458178, -1.9616841080692993 );
		seg_end.setCoords( -92.03204761819859, 0.3274348616477232 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		seg_start.setCoords( -92.03204761819859, 0.3274348616477232 );
		seg_end.setCoords( -207.47170283052736, 1.6357744387003228 );
		student_path_segments.push_back( PathSegment(seg_start, seg_end, c) );
		
		// construct path
		student_paths.push_back( PathModel(student_path_segments, c) );
		student_path_segments.clear();
		
		std::cout << "Student path " << (++student_count) << ":" << std::endl;
		std::cout << student_paths.back() << std::endl;
		std::cout << std::endl;
		
		//
		// Kirkwood Scenario: Vehicle Models
		//
		
		// bike model: 1.5m x 0.5m rectangle
		double bike_length = 2.;
		double bike_width = 1.;
		std::vector<XY_Point> bike_vertex_offsets;
		bike_vertex_offsets.push_back( XY_Point(0., bike_width) );
		bike_vertex_offsets.push_back( XY_Point(bike_length, bike_width) );
		bike_vertex_offsets.push_back( XY_Point(bike_length, -bike_width) );
		bike_vertex_offsets.push_back( XY_Point(0., -bike_width) );
		
		// car model: 4m x 1.5m rectangle
		double car_length = 4.;
		double car_width = 1.5;
		std::vector<XY_Point> bus_vertex_offsets;
		bus_vertex_offsets.push_back( XY_Point(0., car_width) );
		bus_vertex_offsets.push_back( XY_Point(car_length, car_width) );
		bus_vertex_offsets.push_back( XY_Point(car_length, -car_width) );
		bus_vertex_offsets.push_back( XY_Point(0., -car_width) );
		
		// construct car model
		VehicleModel bus_model( bus_vertex_offsets,
							   bus_path,
							   c );
		
		std::cout << "Car:" << std::endl;
		std::cout << bus_model << std::endl;
		std::cout << std::endl;
		
		// student model: octagon
		double radius = 0.25;
		double side_length = 2. * radius * cos( M_PI / 4. );
		std::vector<XY_Point> student_vertex_offsets;
		student_vertex_offsets.push_back( XY_Point(-(radius+side_length), radius)  );
		student_vertex_offsets.push_back( XY_Point(-radius, (radius+side_length))  );
		student_vertex_offsets.push_back( XY_Point(radius, (radius+side_length))  );
		student_vertex_offsets.push_back( XY_Point((radius+side_length), radius)  );
		student_vertex_offsets.push_back( XY_Point((radius+side_length), -radius)  );
		student_vertex_offsets.push_back( XY_Point(radius, -(radius+side_length))  );
		student_vertex_offsets.push_back( XY_Point(-radius, -(radius+side_length))  );
		student_vertex_offsets.push_back( XY_Point(-(radius+side_length), -radius)  );

		// variables for student parameters
		double student_position = 0.;
		double student_v_min = 0.;
		double student_v_max = 1.5; // http://www.usroads.com/journals/p/rej/9710/re971001.htm
		double student_a_min = -2.;
		double student_a_max = 2.; // hard to find any decent reference on this
		double bike_v_min = 0.;
		double bike_v_max = 7.;
		std::vector<VehicleModel> students;
		
		//
		// FOR DEMO
		//
		
		// build a student model for each path
		for ( size_t i=0; i<student_paths.size(); ++i ) {
			
			// skip the guy blocking the middle
			if ( (i == 5) || (i == 6) || (i == 3) ) {
				continue;
			}
			
			Constraints student_c( student_paths[i].getLength(),
								  c.getTLimit(),
								  student_v_min,
								  student_v_max,
								  student_a_min,
								  student_a_max );
			
			Constraints bike_c( student_paths[i].getLength(),
							   c.getTLimit(),
							   bike_v_min,
							   bike_v_max,
							   student_a_min,
							   student_a_max );
			
			if ( i == 9 ) {
				// bicycle, turn off
				Controller bike_controller;
				bike_controller.addControl( 0., bike_c.getAMax() );
				students.push_back( VehicleModel(bike_vertex_offsets,
												 student_paths[i],
												 bike_c,
												 bike_controller,
												 0.,
												 0.) );
			} else if ( i == 10 ) {
				// bicycle, turn on
				Controller bike_controller;
				bike_controller.addControl( 10.5, bike_c.getAMax() );
				students.push_back( VehicleModel(bike_vertex_offsets,
												 student_paths[i],
												 bike_c,
												 bike_controller,
												 0.,
												 bike_c.getVMin()) );
			} else if ( i == 4 ) {
				//continue;
				students.push_back( VehicleModel(student_vertex_offsets,
												 student_paths[i],
												 student_c,
												 0., //10.,
												 student_c.getVMax()) );
			} else if ( i == 5 ) {
				students.push_back( VehicleModel(student_vertex_offsets,
												 student_paths[i],
												 student_c,
												 0., //10.,
												 student_c.getVMax()) );
			} else if ( i == 1 ) {
				students.push_back( VehicleModel(student_vertex_offsets,
												 student_paths[i],
												 student_c,
												 0., //3.,
												 student_c.getVMax()) );
			} else if ( i == 7 ) {
				Controller student_controller;
				student_controller.addControl( 0., student_c.getAMax() );
				students.push_back( VehicleModel(student_vertex_offsets,
												 student_paths[i],
												 student_c,
												 student_controller,
												 0., //10.,
												 student_c.getVMin()) );
			} else {
				students.push_back( VehicleModel(student_vertex_offsets,
												 student_paths[i],
												 student_c,
												 student_position,
												 student_c.getVMax()) );
			}
			
			std::cout << "Student " << (i+1) << ":" << std::endl;
			std::cout << students.back() << std::endl;
			std::cout << std::endl;
		}

		// draw world scene
		std::stringstream ss1;
		ss1 << Utilities::GetMatlabStandardFigure( true ) << std::endl;
		for ( size_t i=0; i<students.size(); i++ ) {
			ss1 << Utilities::GetMatlabPath( students[i].getPath() ) << std::endl;
		}
		for ( size_t i=0; i<students.size(); i++ ) {
			ss1 << Utilities::GetMatlabVehicleModel( students[i] ) << std::endl;
		}
		ss1 << Utilities::GetMatlabPath( bus_model.getPath(), true );
		ss1 << Utilities::GetMatlabVehicleModel( bus_model, true ) << std::endl;
		std::ofstream ofile1;

		//
		// PVT Obstacle Construction
		//
		
		VehicleModel _bus_model( bus_model );
		std::vector<VehicleModel> _students;
		std::vector<VehicleModelUncertain> students_uncertain;
		for ( size_t i=0; i<students.size(); ++i ) {
			_students.push_back( VehicleModel(students[i]) );
			students_uncertain.push_back( VehicleModelUncertain(students[i], 0., 0.) );
		}
		
		// benchmarking
		struct timeval startTime1;
		struct timeval endTime1;

		std::cout << "Contructing PT obstacles... ";
		PVT_ObstacleSet O;
		double time_step = .1;
		double simulation_time_limit = 25.;
		gettimeofday(&startTime1, NULL);
		if ( !Obstacles::ForwardObstacleSimulator<VehicleModelUncertain>(O, students_uncertain, bus_model, c, time_step, simulation_time_limit) ) {
			std::cerr << "error in forward simulation" << std::endl;
			return 1;
		}

		gettimeofday(&endTime1, NULL);
		std::cout << "constructed " << O.obstacles.size() << " PT obstacles from " << students.size() << " world obstacles." << std::endl;
		std::vector<PVT_ObstaclePoint*> P_t;
		O.getPotentiallyReachableVertices( P_t, c );
		std::cout << P_t.size() << " vertices." << std::endl;
		std::cout << "Time resolution: " << time_step << std::endl;
		
		// calculate time in microseconds
		double tS1 = startTime1.tv_sec * 1000000 + (startTime1.tv_usec);
		double tE1 = endTime1.tv_sec * 1000000  + (endTime1.tv_usec);
		std::cout << "Time elapsed: " << tE1 - tS1 << " microseconds" << std::endl;
		std::cout << std::endl;

		// run planner
		std::vector<PVT_G*> G;
		std::vector<PVT_S*> Goal;
		Interval V_f( 0., 0. );
		Interval V_i( 0., 0. );
		
		double plan1_time = std::numeric_limits<double>::quiet_NaN();
		double plan2_time = std::numeric_limits<double>::quiet_NaN();
		
		//
		// Part 1
		//
		
		// stop at stop sign
		c.setXLimit( 70. );
		
		// get the current time
		// - NULL because we don't care about time zone
		std::cout << "Planning... ";
		gettimeofday(&startTime1, NULL);
		if ( !Planner::Forward( G, Goal, V_i, V_f, O, c) ) {
			std::cerr << "Planner failed!" << std::endl;
			return 1;
		}
		gettimeofday(&endTime1, NULL);
		std::cout << "complete." << std::endl;

		// calculate time in microseconds
		tS1 = startTime1.tv_sec * 1000000 + (startTime1.tv_usec);
		tE1 = endTime1.tv_sec * 1000000  + (endTime1.tv_usec);
		plan1_time = tE1 - tS1;
		std::cout << "Time elapsed: " << plan1_time << " microseconds" << std::endl;
		std::cout << std::endl;
		
		// trajectory for part 1
		std::vector<TrajectorySegment*> T1;

		if ( !Goal.empty() ) {
			if ( !Planner::BuildOptimalTrajectory(T1, G, Goal, O, c) ) {
				std::cerr << "Failed to build optimal trajectory!" << std::endl;
			} else {
				std::cout << "Found optimal trajectory:" << std::endl;
				Utilities::PrintTrajectory( T1 );
				std::cout << std::endl;
			}
		} else {
			std::cout << "No feasible trajectory." << std::endl;
		}
		std::cout << std::endl;
		
		double final_path_position = T1.empty() ? 0 : T1.back()->getFinalState().getPathCoord();
		double final_time = T1.empty() ? 0 : T1.back()->getFinalState().getTimeCoord();
		
		// output for visually debugging
		//Utilities::DescribeG( G, true );
		//Utilities::GenerateMatlabScene( "PVTP/Matlab/scene1.m", O, T1 );

		// free memory
		Utilities::CleanTrajectory( T1 );
		Utilities::CleanResults( G, Goal );
		
		//
		// Part 2
		//
		
		// proceed from stop sign
		O.translateObstacles( -final_path_position, -final_time );
		c.setXLimit( bus_model.getPath().getLength() - final_path_position );
		
		// get the current time
		// - NULL because we don't care about time zone
		std::cout << "Planning... ";
		gettimeofday(&startTime1, NULL);
		if ( !Planner::Forward( G, Goal, V_i, V_f, O, c) ) {
			std::cerr << "Planner failed!" << std::endl;
			return 1;
		}
		gettimeofday(&endTime1, NULL);
		std::cout << "complete." << std::endl;
		
		// calculate time in microseconds
		tS1 = startTime1.tv_sec * 1000000 + (startTime1.tv_usec);
		tE1 = endTime1.tv_sec * 1000000  + (endTime1.tv_usec);
		plan2_time = tE1 - tS1;
		std::cout << "Time elapsed: " << plan2_time << " microseconds" << std::endl;
		std::cout << std::endl;
		std::cout << "Total planning time: " << (plan1_time + plan2_time) << std::endl;
		std::cout << std::endl;
		
		// trajectory for part 2
		std::vector<TrajectorySegment*> T2;
		
		if ( !Goal.empty() ) {
			if ( !Planner::BuildOptimalTrajectory(T2, G, Goal, O, c) ) {
				std::cerr << "Failed to build optimal trajectory!" << std::endl;
				return 1;
			}
			std::cout << "Found optimal trajectory:" << std::endl;
			Utilities::PrintTrajectory( T2 );
			std::cout << std::endl;
		} else {
			std::cout << "No feasible trajectory." << std::endl;
		}
		std::cout << std::endl;
		
		// output for visually debugging
		//Utilities::DescribeG( G, true );
		//Utilities::GenerateMatlabScene( "PVTP/Matlab/scene2.m", O, T2 );
		
		// free memory
		Utilities::CleanTrajectory( T1 );
		Utilities::CleanTrajectory( T2 );
		Utilities::CleanResults( G, Goal );
		
		return 0;
		
	} catch ( int e ) {

		// print out appropriate error
		ExceptionHandler::exceptionMessage( e );
		return -1;

	}
	
	return 0;

}