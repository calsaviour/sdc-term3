#include <PVTP/Utilities.hpp>
#include <PVTP/VehicleModelUncertain.hpp>

#ifdef BUILDBOOST1
#if BUILDBOOST1

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>

using boost::geometry::model::d2::point_xy;
using boost::geometry::append;
using boost::geometry::make;

using namespace PVTP;

namespace Scenario {
	
	VehicleModelUncertain::VehicleModelUncertain ( const VehicleModel& vm, double velocity_spread, double acc_spread ) {
		this->min_estimate_model = new VehicleModel( vm );
		this->max_estimate_model = new VehicleModel( vm );
		
		// vehicle's initial velocity range
		double v_i_min = vm.getVelocity() - velocity_spread;
		double v_i_max = vm.getVelocity() + velocity_spread;
		
		// vehicle's min/max controllers
		Controller max_controller;
		Controller min_controller;
		std::set< std::pair<double, double>, Controller::ControlCompare >::iterator it;
		for ( it=vm.getController().getControlSequence().begin(); it!=vm.getController().getControlSequence().end(); ++it ) {
			double min_control = Utilities::truncateValue( it->second - acc_spread,
														  vm.getConstraints().getAMin(),
														  vm.getConstraints().getAMax() );
			double max_control = Utilities::truncateValue( it->second + acc_spread,
														  vm.getConstraints().getAMin(),
														  vm.getConstraints().getAMax() );
			min_controller.addControl( it->first, min_control );
			max_controller.addControl( it->first, max_control );
		}
		
		// build bounding vehicles
		this->getMinEstimateModel().setVelocity( v_i_min );
		this->getMinEstimateModel().setController( min_controller );
		
		this->getMaxEstimateModel().setVelocity( v_i_max );
		this->getMaxEstimateModel().setController( max_controller );
	}
	
	VehicleModelUncertain::VehicleModelUncertain ( const VehicleModelUncertain& vm ) {
		this->min_estimate_model = new VehicleModel( *vm.min_estimate_model );
		this->max_estimate_model = new VehicleModel( *vm.max_estimate_model );
		this->velocity_spread = vm.velocity_spread;
		this->acc_spread = vm.acc_spread;
	}
	
	VehicleModelUncertain::~VehicleModelUncertain ( ) {
		delete( this->min_estimate_model );
		delete( this->max_estimate_model );
	}
	
	VehicleModel * VehicleModelUncertain::getSweptRegion( VehicleModel& _min_estimate_model, VehicleModel& _max_estimate_model ) {
		VehicleModel min_estimate_model( _min_estimate_model );
		VehicleModel max_estimate_model( _max_estimate_model );
		
		// add vehicle model vertices to set of points for swept region computation
		boost::geometry::model::multi_point<point_xy<double> > swept_vertices;
		for ( size_t v_index=0; v_index<min_estimate_model.getVertices().size(); v_index++ ) {
			append( swept_vertices, make<point_xy<double> >(min_estimate_model.getVertices()[v_index].getX(), min_estimate_model.getVertices()[v_index].getY()) );
			append( swept_vertices, make<point_xy<double> >(max_estimate_model.getVertices()[v_index].getX(), max_estimate_model.getVertices()[v_index].getY()) );
		}
		
		// compute convex hull of rotating body at each path segment junction
		// assume angle of displacement is never greater than 90 degrees
		for ( ssize_t cur_seg_index=min_estimate_model.getPathPositionIndex(); cur_seg_index<max_estimate_model.getPathPositionIndex(); ++cur_seg_index ) {
			size_t next_seg_index = cur_seg_index + 1;
			
			// compute orientation of segments
			double slope = std::numeric_limits<double>::quiet_NaN();
			double cur_orientation;
			if ( min_estimate_model.getPath().segments[cur_seg_index].isVertical() ) {
				if ( min_estimate_model.getPath().segments[next_seg_index].getDirection() > 0 ) {
					cur_orientation = M_PI;
				} else {
					cur_orientation = -M_PI;
				}
			} else {
				if ( !min_estimate_model.getPath().segments[cur_seg_index].getSlopeAtArcLength(slope, 0., min_estimate_model.getConstraints()) ) {
					std::cerr << "ERROR IN VehicleModelUncertain::getSweptRegion: Failed finding segment arc length." << std::endl;
					return NULL;
				}
				if ( min_estimate_model.getPath().segments[cur_seg_index].getDirection() > 0 ) {
					cur_orientation = atan( slope );
				} else {
					cur_orientation = M_PI + atan( slope );
				}
			}
			
			double next_orientation;
			if ( min_estimate_model.getPath().segments[next_seg_index].isVertical() ) {
				if ( min_estimate_model.getPath().segments[next_seg_index].getDirection() > 0 ) {
					next_orientation = M_PI;
				} else {
					next_orientation = -M_PI;
				}
			} else {
				if ( !min_estimate_model.getPath().segments[next_seg_index].getSlopeAtArcLength(slope, 0., min_estimate_model.getConstraints()) ) {
					std::cerr << "ERROR IN VehicleModelUncertain::getSweptRegion: Failed finding segment arc length." << std::endl;
					return NULL;
				}
				if ( min_estimate_model.getPath().segments[next_seg_index].getDirection() > 0 ) {
					next_orientation = atan( slope );
				} else {
					next_orientation = M_PI + atan( slope );
				}
			}
			
			// set current vehicle model to junction with previous orientation
			min_estimate_model.moveToReferencePoint( min_estimate_model.getPath().segments[cur_seg_index].getEndPoint(), min_estimate_model.getConstraints() );
			min_estimate_model.setOrientation( cur_orientation, min_estimate_model.getConstraints() );
			
			// rotate at most 90 degrees at a time
			double _next_orientation = Maths::circleGeodesic( next_orientation );
			double angle_diff = next_orientation - cur_orientation;
			if ( fabs(angle_diff) > M_2PI ) {
				next_orientation = (angle_diff < 0.) ? -1. * M_2PI : M_2PI;
			}
			
			while ( true ) {
				
				// set next vehicle model to junction with next orientation
				max_estimate_model.moveToReferencePoint( max_estimate_model.getPath().segments[next_seg_index].getOrigin(), min_estimate_model.getConstraints() );
				max_estimate_model.setOrientation( next_orientation, min_estimate_model.getConstraints() );
				
				// add rotational vertices
				for ( size_t v_index=0; v_index<min_estimate_model.getVertices().size(); ++v_index ) {
					
					// slopes of lines connecting vertices to point of rotation
					double cur_slope = Maths::slope( min_estimate_model.getReferencePoint().getX(),
													min_estimate_model.getReferencePoint().getY(),
													min_estimate_model.getVertices()[v_index].getX(),
													min_estimate_model.getVertices()[v_index].getY() );
					
					double next_slope = Maths::slope( max_estimate_model.getReferencePoint().getX(),
													 max_estimate_model.getReferencePoint().getY(),
													 max_estimate_model.getVertices()[v_index].getX(),
													 max_estimate_model.getVertices()[v_index].getY() );
					
					// if the slope hasn't changed, no useful rotation has occurred
					if ( (Maths::isNaN(cur_slope) && Maths::isNaN(next_slope))
						|| Maths::approxEq(cur_slope, next_slope, min_estimate_model.getConstraints().getEpsilon()) ) {
						continue;
					}
					
					// slopes of perpendicular lines
					double cur_slope_perp;
					if ( Maths::approxEq(cur_slope, 0., min_estimate_model.getConstraints().getEpsilon()) ) {
						cur_slope_perp = std::numeric_limits<double>::quiet_NaN();
					} else if ( Maths::isNaN(cur_slope) ) {
						cur_slope_perp = 0.;
					} else {
						cur_slope_perp = -1. / cur_slope;
					}
					double next_slope_perp;
					if ( Maths::approxEq(next_slope, 0., min_estimate_model.getConstraints().getEpsilon()) ) {
						next_slope_perp = std::numeric_limits<double>::quiet_NaN();
					} else if ( Maths::isNaN(next_slope) ) {
						next_slope_perp = 0.;
					} else {
						next_slope_perp = -1. / next_slope;
					}
					
					// intercepts of perpendicular lines
					double cur_intercept_perp = Maths::yIntercept( min_estimate_model.getVertices()[v_index].getX(), min_estimate_model.getVertices()[v_index].getY(), cur_slope_perp );
					double next_intercept_perp = Maths::yIntercept( max_estimate_model.getVertices()[v_index].getX(), max_estimate_model.getVertices()[v_index].getY(), next_slope_perp );
					
					// compute intersection point
					double x_intersection;
					double y_intersection;
					if ( Maths::isNaN(cur_intercept_perp) ) {
						x_intersection = min_estimate_model.getVertices()[v_index].getX();
						y_intersection = Maths::yFromSlopeIntercept( next_slope_perp,
																	x_intersection,
																	next_intercept_perp,
																	min_estimate_model.getConstraints().getEpsilon() );
					} else if ( Maths::isNaN(next_intercept_perp) ) {
						x_intersection = max_estimate_model.getVertices()[v_index].getX();
						y_intersection = Maths::yFromSlopeIntercept( cur_slope_perp,
																	x_intersection,
																	cur_intercept_perp,
																	min_estimate_model.getConstraints().getEpsilon() );
					} else {
						Maths::lineIntersection( x_intersection,
												y_intersection,
												cur_slope_perp,
												cur_intercept_perp,
												next_slope_perp,
												next_intercept_perp,
												min_estimate_model.getConstraints().getEpsilon() );
					}
					
					// sanity check
					if ( Maths::isNaN(x_intersection) || Maths::isNaN(y_intersection) ) {
						std::cerr << "ERROR IN VehicleModelUncertain::getSweptRegion: Invalid line intersection." << std::endl;
						return NULL;
					}
					
					// add intersection point to vertex list
					append( swept_vertices, make<point_xy<double> >(x_intersection, y_intersection) );
				}
				
				// if we reach the target orientation, break out
				if ( Maths::approxEq(next_orientation, _next_orientation, min_estimate_model.getConstraints().getEpsilon()) ) {
					break;
				}
				
				double angle_diff = _next_orientation - next_orientation;
				if ( fabs(angle_diff) > M_2PI ) {
					next_orientation += (angle_diff < 0.) ? -1. * M_2PI : M_2PI;
				} else {
					next_orientation = _next_orientation;
				}
			}
		}
		
		
		// compute convex hull: the 'true' ensures points are ordered clockwise
		boost::geometry::model::polygon<point_xy<double>, true> hull;
		boost::geometry::convex_hull( swept_vertices, hull );
		
		// construct vehicle model points from convex hull
		std::vector<XY_Point> swept_vertex_offsets;
		std::vector<point_xy<double> > const& boost_points = hull.outer();
		for ( size_t points_index=0; points_index<boost_points.size(); points_index++ ) {
			swept_vertex_offsets.push_back( XY_Point(boost_points[points_index].x() - min_estimate_model.getReferencePoint().getX(),
													 boost_points[points_index].y() - min_estimate_model.getReferencePoint().getY()) );
		}
		
		// this constructed obstacle represents the swept region
		// between this time step and the next
		return new VehicleModel( swept_vertex_offsets,
							  min_estimate_model.getPath(),
							  min_estimate_model.getConstraints(),
							  min_estimate_model.getPathPosition(),
							  min_estimate_model.getVelocity(),
							  min_estimate_model.getAcceleration(),
							  0. );
	}
	
	VehicleModel * VehicleModelUncertain::getSweptRegion( VehicleModelUncertain& min_estimate_model, VehicleModelUncertain& max_estimate_model ) {
		VehicleModel * swept_region = VehicleModelUncertain::getSweptRegion( min_estimate_model.getMinEstimateModel(),
																			max_estimate_model.getMaxEstimateModel() );
		return swept_region;
	}
	
	VehicleModel * VehicleModelUncertain::getSweptRegion( void ) {
		return VehicleModelUncertain::getSweptRegion( this->getMinEstimateModel(), this->getMaxEstimateModel() );
	}

	std::ostream& operator<<(std::ostream& out, const VehicleModelUncertain& vm) {
		out << *vm.min_estimate_model << std::endl;
		out << "===" << std::endl;
		out << *vm.max_estimate_model;
		return out;
	}
	
} // end Scenario

#endif
#endif
