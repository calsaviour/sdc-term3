#include <PVTP/Utilities.hpp>
#include <PVTP/VehicleModel.hpp>

//#define DEBUG

using namespace PVTP;

namespace Scenario {
	
	VehicleModel::VehicleModel ( const std::vector<XY_Point>& vertex_offsets,
								const PathModel& path,
								const Constraints& c,
								double path_position,
								double velocity,
								double acc,
								double orientation ) {
		Controller controller;
		controller.addControl( 0., acc );
		this->init( vertex_offsets, path, c, controller, path_position, velocity, orientation );
	}
	
	VehicleModel::VehicleModel ( const std::vector<XY_Point>& vertex_offsets,
								const PathModel& path,
								const Constraints& c,
								const Controller& controller,
								double path_position,
								double velocity,
								double orientation ) {
		this->init( vertex_offsets, path, c, controller, path_position, velocity, orientation );
	}
	
	void VehicleModel::init( const std::vector<XY_Point>& vertex_offsets,
							const PathModel& path,
							const Constraints& c,
							const Controller& controller,
							double path_position,
							double velocity,
							double orientation ) {
		this->path_position_index = 0;
		this->orientation = std::numeric_limits<double>::quiet_NaN();
		this->c = new Constraints( c );
		this->reference_point = path.getInitialPoint();
		this->vertex_offsets = vertex_offsets;
		this->vertex_offsets_raw = vertex_offsets;
		this->vertices = vertex_offsets;
		this->path = new PathModel( path );
		this->moveToPathPosition( path_position, this->getConstraints() );
		this->setVelocity( velocity );
		this->controller = controller;
		this->setAcceleration( this->controller.getControl(0.) );
		
#ifdef DEBUG
		for ( size_t i=0; i<vertex_offsets.size(); ++i ) {
			if ( Maths::isNaN(vertex_offsets[i].getX()) || Maths::isNaN(vertex_offsets[i].getY()) ) {
				throw Constants::BAD_VERTEX;
			}
		}
#endif

		// if no orientation is explicitly set, orient to path
		if ( Maths::isNaN(orientation) ) {
			if ( !this->path->getOrientationAtArcLength(orientation, path_position, this->getConstraints()) ) {
				throw Constants::BAD_ORIENTATION;
			}
		}
		this->setOrientation( orientation, this->getConstraints() );
	}
	
	VehicleModel::VehicleModel ( const VehicleModel& vm ) {
		this->reference_point = vm.getReferencePoint();
		this->velocity = vm.getVelocity();
		this->acc = vm.getAcceleration();
		this->path = new PathModel( *vm.path );
		this->path_position = vm.getPathPosition();
		this->vertices = vm.vertices;
		this->vertex_offsets = vm.vertex_offsets;
		this->vertex_offsets_raw = vm.vertex_offsets_raw;
		this->orientation = vm.getOrientation();
		this->c = new Constraints( vm.getConstraints() );
		this->path_position_index = vm.path_position_index;
		this->controller = vm.controller;
	}
	
	VehicleModel::~VehicleModel( ) {
		delete( this->path );
		delete( this->c );
	}
	
	void VehicleModel::setVelocity( double velocity ) {
		this->velocity = Utilities::truncateValue( velocity, this->getConstraints().getVMin(), this->getConstraints().getVMax() );
	}
	
	void VehicleModel::setAcceleration( double acceleration ) {
		this->acc = Utilities::truncateValue( acceleration, this->getConstraints().getAMin(), this->getConstraints().getAMax() );
	}
	
	void VehicleModel::setReferencePoint( double x, double y ) {
		this->reference_point.setCoords( x, y );
		for ( size_t i=0; i<this->vertex_offsets.size(); i++ ) {
			this->vertices[i].setCoords( this->vertex_offsets[i].getX() + this->reference_point.getX(),
										this->vertex_offsets[i].getY() + this->reference_point.getY() );
		}
	}

	bool VehicleModel::moveToReferencePoint( double x, double y, const Constraints& c ) {
		
		// get new path position
		double path_position;
		ssize_t path_position_index = this->path->getArcLengthAtPositionVector( path_position, x, y, c );
		if ( path_position_index < 0 ) {
			return false;
		}
		
		// set new path position
		this->path_position = path_position;
		this->path_position_index = path_position_index;
		
		// compute orientation along path
		double orientation;
		if ( !this->path->getOrientationAtArcLength(orientation, this->getPathPosition(), c) ) {
			std::cerr << "ERROR IN VehicleModel::MoveToReferencePoint: Failed getting orientation at arc length: " << this->getPathPosition() << std::endl;
			return false;
		}
		
		// translate reference point
		this->setReferencePoint( x, y );
		
		// set new orientation
		this->setOrientation( orientation, c );
		
		return true;
	}
	
	bool VehicleModel::computeVertexAtPoint( XY_Point& vertex,
											size_t vertex_index,
											int direction,
											double x,
											double y,
											std::vector<double>& coefficients,
											const Constraints& c ) {
		
		double slope = coefficients[1];
		double intercept = coefficients[2];
		
		// handle vertical lines
		double orientation;
		if ( Maths::isNaN(slope) || Maths::isNaN(intercept) ) {

			if ( direction > 0 ) {
				orientation = M_PI / 2.;
			} else {
				orientation = -M_PI / 2.;
			}

		} else {
			
			double y_test = Maths::yFromSlopeIntercept( slope, x, intercept );
			if ( !Maths::approxEq(y, y_test, c.getEpsilon()) ) {
				std::cerr << "ERROR IN VehicleModel::computeVertexAtPoint: Given point (" << x << ", " << y << ") is invalid, expected y of " << y_test << std::endl;
				return false;
			}
			
			if ( direction > 0 ) {
				orientation = atan( slope );
			} else {
				orientation = M_PI + atan( slope );
			}
			
		}
		
		XY_Point vertex_offset( this->vertex_offsets_raw[vertex_index] );
		vertex_offset.rotate( orientation );
		
		vertex.setCoords( x + vertex_offset.getX(), y + vertex_offset.getY() );
		
		return true;
	}
	
	void VehicleModel::setOrientation( double new_orientation, const Constraints& c, bool orient_vertices ) {
		
		double orientation_delta = new_orientation - ( Maths::isNaN(this->getOrientation()) ? 0. : this->getOrientation() );
		
		// set new orientation
		this->orientation = new_orientation;
		if ( !orient_vertices || Maths::isNaN(this->getOrientation()) ) {
			return;
		}
		
		orientation_delta = Maths::clipToZero( orientation_delta, c.getEpsilon() );
		if ( orientation_delta == 0. ) {
			return;
		}
		
		// rotate offsets, set new absolute coordinates
		for ( size_t i=0; i<this->vertex_offsets.size(); i++ ) {
			this->vertex_offsets[i].rotate( orientation_delta );
			this->vertices[i].setCoords( this->vertex_offsets[i].getX() + this->reference_point.getX(),
										this->vertex_offsets[i].getY() + this->reference_point.getY() );
		}
	}
	
	bool VehicleModel::moveToPathPosition( double path_position, const Constraints& c, bool orient_to_prev ) {

		// compute world frame coordinates of new path position
		double x;
		double y;
		ssize_t path_position_index = this->path->getPositionVectorAtArcLength( x, y, path_position, c );
		if ( path_position_index < 0 ) {
			std::cerr << "ERROR IN VehicleModel::moveToPathPosition: Failed finding position vector at arc length " << path_position << std::endl;
			return false;
		}

		// set new path position
		this->path_position = path_position;
		this->path_position_index = path_position_index;

		// compute orientation along path
		double orientation;
		if ( !this->path->getOrientationAtArcLength(orientation, this->getPathPosition(), c, orient_to_prev) ) {
			std::cerr << "ERROR IN VehicleModel::MoveToReferencePoint: Failed getting orientation at arc length: " << this->getPathPosition() << std::endl;
			return false;
		}
		
		// translate reference point
		this->setReferencePoint( x, y );
		
		// set new orientation
		this->setOrientation( orientation, c );
		
		return true;
	}
	
	void VehicleModel::calculateControlEffects( double& new_velocity,
										  double& distance_travelled,
										  double current_velocity,
										  double control,
										  double time_step,
										  double a_min,
										  double a_max,
										  double min_v,
										  double max_v ) {
		new_velocity = std::numeric_limits<double>::quiet_NaN();
		distance_travelled = std::numeric_limits<double>::quiet_NaN();
		
		if ( time_step < 0. ) {
			std::cerr << "ERROR IN Vehicle::calculateControlEffects: Negative time step." << std::endl;
			return;
		}
		
		control = Utilities::truncateValue( control, a_min, a_max );
		
		new_velocity = Maths::V2_FromV1_T_A( current_velocity, time_step, control );
		
		// if the new velocity exceeds bounds, take this into account when
		// computing distance traversed
		if ( new_velocity > max_v ) {
			
			// time to reach max velocity
			double time_to_max_v = Maths::T_FromV1_V2_A( current_velocity,
														max_v,
														control );
			// time at max velocity
			double time_at_max_v = time_step - time_to_max_v;
			
			// compute distance piecewise
			double d1 = Maths::motionX_FromV1_T1_T2_A( current_velocity,
													  0.,
													  time_to_max_v,
													  control );
			double d2 = Maths::motionX_FromV1_T1_T2_A( max_v,
													  0.,
													  time_at_max_v,
													  0. );
			
			new_velocity = max_v;
			distance_travelled = d1 + d2;
		} else if ( new_velocity < min_v ) {
			
			// time to reach min velocity
			double time_to_min_v = Maths::T_FromV1_V2_A( current_velocity,
														min_v,
														control );
			
			// compute distance just for this segment
			double d1 = Maths::motionX_FromV1_T1_T2_A( current_velocity,
													  0.,
													  time_to_min_v,
													  control );
			
			new_velocity = min_v;
			distance_travelled = d1;
			
		} else {
			distance_travelled = Maths::motionX_FromV1_T1_T2_A( current_velocity,
															   0.,
															   time_step,
															   control );
		}
	}
	
	bool VehicleModel::translateInTime( double t ) {
		if ( this->path == NULL ) {
			return false;
		}
		if ( t < 0. ) {
			return false;
		}
		
		// if there's no more room, exit
		if ( Maths::approxEq(this->getPathPosition(), this->path->getLength(), this->getConstraints().getEpsilon()) ) {
			return false;
		}
		
		// compute arc length over given time
		double new_velocity;
		double new_position;
		double velocity = this->getVelocity();
		double control = this->controller.getControlFromTimeDelta(t);
		VehicleModel::calculateControlEffects( new_velocity,
										 new_position,
										 velocity,
										 control,
										 t,
										 this->getConstraints().getAMin(),
										 this->getConstraints().getAMax(),
										 this->getConstraints().getVMin(),
										 this->getConstraints().getVMax() );
		
		//if ( Maths::isNaN(new_position) ) {
		//	std::cerr << "NaN: " << velocity << ", " << control << ", " << new_position << ", " << new_velocity << std::endl;
		//}

		// update our velocity
		this->setVelocity( new_velocity );

		// update our position
		double new_path_position_raw = this->getPathPosition() + new_position;
		double new_path_position = Utilities::truncateValue( new_path_position_raw, 0., this->path->getLength() );
		
		if ( !this->moveToPathPosition(new_path_position, this->getConstraints()) ) {
			std::cerr << "ERROR IN VehicleModel::translateInTime: Invalid path position of " << new_path_position << std::endl;
			return false;
		}
		
		// compute position along path
		double x;
		double y;
		ssize_t path_position_index = this->path->getPositionVectorAtArcLength( x, y, this->getPathPosition(), this->getConstraints() );
		if ( path_position_index < 0 ) {
			std::cerr << "ERROR IN VehicleModel::translateInTime: Failed getting position vector." << std::endl;
			return false;
		}
		if ( Maths::approxEq(x, 0., this->getConstraints().getEpsilon()) && Maths::approxEq(y, 0., this->getConstraints().getEpsilon()) ) {
			std::cerr << "origin reference point" << std::endl;
		}
		
		// compute orientation along path
		double orientation;
		if ( !this->path->getOrientationAtArcLength(orientation, this->getPathPosition(), this->getConstraints()) ) {
			std::cerr << "ERROR IN VehicleModel::translateInTime: Failed getting orientation at arc length: " << this->getPathPosition() << std::endl;
			return false;
		}
		
		// translate reference point
		this->setReferencePoint( x, y );
		
		// set new orientation
		this->setOrientation( orientation, this->getConstraints() );
		
		// return true of there's room left for movement; false otherwise
		return true;
	}
	
	void VehicleModel::rotate90CW( void ) {
		
		// orientation
		this->orientation = this->orientation - .5 * M_PI;
		
		// reference point
		this->reference_point.rotate90CW();
		
		// vertices and offsets
		for ( size_t i=0; i<this->vertices.size(); i++ ) {
			this->vertices[i].rotate90CW();
			this->vertex_offsets[i].rotate90CW();
		}
		
	}
	
	void VehicleModel::rotate90CCW( void ) {
		
		// orientation
		this->orientation = this->orientation + .5 * M_PI;
		
		// reference point
		this->reference_point.rotate90CCW();
		
		// vertices and offsets
		for ( size_t i=0; i<this->vertices.size(); i++ ) {
			this->vertices[i].rotate90CCW();
			this->vertex_offsets[i].rotate90CCW();
		}
		
	}
	
	std::string VehicleModel::exceptionMessage( int e ) {
		std::stringstream ss;
		ss << "Vehicle construction failed: ";
		switch ( e ) {
			case Constants::BAD_ORIENTATION:
				ss << "Vehicle orientation could not be determined.";
				break;
			case Constants::BAD_VERTEX:
				ss << "Attempted to set NaN vertex.";
				break;
			default:
				return "";
		}
		return ss.str();
	}
	
	std::ostream& operator<<(std::ostream& out, const VehicleModel& vm) {
#ifdef DEBUG
		out << "Reference point: " << vm.getReferencePoint() << std::endl;
		out << "Vertex offsets:" << std::endl;
		for ( size_t i=0; i<vm.vertex_offsets.size(); i++ ) {
			out << vm.vertex_offsets[i] << std::endl;
		}
		out << "Vertices:" << std::endl;
		for ( size_t i=0; i<vm.vertices.size(); i++ ) {
			out << vm.vertices[i] << std::endl;
		}
#endif
		out << "Controller:" << std::endl;
		out << vm.controller;
		if ( !Maths::isNaN(vm.getVelocity()) ) {
			out << "Velocity: " << vm.getVelocity() << std::endl;
		}
		if ( !Maths::isNaN(vm.getAcceleration()) ) {
			out << "Acceleration: " << vm.getAcceleration() << std::endl;
		}
		out << *vm.path << std::endl;
		out << "Path position: " << vm.getPathPosition() << std::endl;
		out << "Path position index: " << vm.getPathPositionIndex() << std::endl;
		out << "Orientation: " << vm.getOrientation() << std::endl;
		return out;
	}
	
} // end Scenario