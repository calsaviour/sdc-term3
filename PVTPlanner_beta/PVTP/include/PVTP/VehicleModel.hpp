#ifndef PVTP_VEHICLE_MODEL_H
#define PVTP_VEHICLE_MODEL_H

#include <PVTP/Controller.hpp>
#include <PVTP/PathModel.hpp>
#include <PVTP/XY_Point.hpp>

using namespace PVTP;

namespace Scenario {
	
	/**
	 * Vehicle Model class. This class is for contructing and handling a model
	 * of the driver's vehicle, which is assumed to be polygonal in 2D Cartesian space.
	 */
	class VehicleModel {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const VehicleModel& vm);
		
	public:
		
		/**
		 * Vehicle Model constructor: polygonal model. The model is given as a
		 * reference point, and a set of offsets from that reference point that
		 * are the vertices of the model.
		 *
		 * WARNING: THE POINTS MUST BE GIVEN IN SEQUENCE SUCH THAT THEY MARCH
		 * ALONG THE CIRCUMFERENCE USING CLOCKWISE ORDERING. The
		 * start point is irrelevant.
		 */
		VehicleModel ( const std::vector<XY_Point>& vertex_offsets,
					  const PathModel& path,
					  const Constraints& c,
					  double path_position = 0.,
					  double velocity = 0.,
					  double acc = 0.,
					  double orientation = std::numeric_limits<double>::quiet_NaN() );
		
		/**
		 * Constructor with explicit controller
		 */
		VehicleModel ( const std::vector<XY_Point>& vertex_offsets,
					  const PathModel& path,
					  const Constraints& c,
					  const Controller& controller,
					  double path_position = 0.,
					  double velocity = 0.,
					  double orientation = std::numeric_limits<double>::quiet_NaN() );
		
		/**
		 * Copy constructor
		 */
		VehicleModel ( const VehicleModel& vm );
		
		/**
		 * Destructor
		 */
		~VehicleModel( );
		
		/**
		 * Assignment operator
		 */
		VehicleModel& operator=( const VehicleModel& vm ) {
			if ( this != &vm ) {
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
			return *this;
		}
		
		/**
		 * Calculate the effects of applying a control; because we have hard
		 * constraints on velocity, there is a piecewise, instead of linear,
		 * relationship, so this is a convenience method for dealing with it.
		 */
		static void calculateControlEffects( double& new_velocity,
											double& distance_travelled,
											double current_velocity,
											double control,
											double time_step,
											double a_min,
											double a_max,
											double min_v,
											double max_v );
		
		/**
		 * Object-specific version of above
		 */
		void calculateControlEffects( double& new_velocity,
									 double& distance_travelled,
									 double control,
									 double time_step ) {
			
			VehicleModel::calculateControlEffects( new_velocity,
												  distance_travelled,
												  this->getVelocity(),
												  control,
												  time_step,
												  this->getConstraints().getAMin(),
												  this->getConstraints().getAMax(),
												  this->getConstraints().getVMin(),
												  this->getConstraints().getVMax() );

		}
		
		/**
		 * Accessor for velocity
		 */
		double getVelocity( void ) const {
			return this->velocity;
		}

		/**
		 * Accessor for acceleration
		 */
		double getAcceleration( void ) const {
			return this->acc;
		}
		
		/**
		 * Accessor for orientation (radians)
		 */
		double getOrientation( void ) const {
			return this->orientation;
		}
		
		/**
		 * Accessor for path position
		 */
		double getPathPosition( void ) const {
			return this->path_position;
		}
		
		/**
		 * Accessor for path position index
		 */
		ssize_t getPathPositionIndex( void ) const {
			return this->path_position_index;
		}
		
		/**
		 * Mutator for path position
		 */
		bool moveToPathPosition( double path_position, const Constraints& c, bool orient_to_prev = false );
		
		/**
		 * Mutator for this vehicle's controller
		 */
		void setController( const Controller& controller ) {
			this->controller = controller;
		}
		
		/**
		 * Accessor for this vehicle's controller
		 */
		const Controller& getController( void ) const {
			return this->controller;
		}
		
		/**
		 * Mutator for orientation. When the orient_vertices flag is set to
		 * false, only the orientation member field is updated; the vehicle
		 * model is not rotated.
		 */
		void setOrientation( double new_orientation, const Constraints& c, bool orient_vertices = true );
		
		/**
		 * Mutator for reference point
		 */
		void setReferencePoint( const XY_Point& p ) {
			this->setReferencePoint( p.getX(), p.getY() );
		}
		
		/**
		 * Mutator for reference point
		 *
		 * NOTE: This does *not* re-orient the vehicle along the path!
		 */
		void setReferencePoint( double x, double y );
		
		/**
		 * Mutates reference point and re-orients vehicle to path
		 */
		bool moveToReferencePoint( const XY_Point& p, const Constraints& c ) {
			return this->moveToReferencePoint( p.getX(), p.getY(), c );
		}

		/**
		 * Mutates reference point and re-orients vehicle to path
		 */
		bool moveToReferencePoint( double x, double y, const Constraints& c );
		
		/**
		 * Given a vertex index, compute what the vertex would be if the vehicle
		 * were at the specified point along an arbitrary curve. Useful for
		 * PVT obstacle construction.
		 */
		bool computeVertexAtPoint( XY_Point& vertex,
								  size_t vertex_index,
								  int direction,
								  double x,
								  double y,
								  std::vector<double>& coefficients,
								  const Constraints& c );
		
		/**
		 * Get orientation at a specified arc length
		 */
		bool getSlopeAtArcLength( double& slope, double arc_length, const Constraints& c ) const {
			return this->path->getSlopeAtArcLength( slope, arc_length, c );
		}
		
		/**
		 * Get the current orientation of the driver vehicle
		 */
		bool getSlope( double& slope, const Constraints& c ) const {
			return this->getSlopeAtArcLength( slope, this->getPathPosition(), c );
		}
		
		/**
		 * Move the vehicle along the path according to its current state.
		 *
		 * Return true if the vehicle has path left for movement; false otherwise.
		 */
		bool translateInTime( double t );
		
		/**
		 * Convenience method for rotating system by 90 degrees clockwise.
		 *
		 * NOTE: This rotates *only* the vertices, not the path! And this
		 * rotates about the world frame origin, *not* the reference point!
		 */
		void rotate90CW( void );
		
		/**
		 * Convenience method for rotating system by 90 degrees counterclockwise.
		 *
		 * NOTE: This rotates *only* the vertices, not the path! And this
		 * rotates about the world frame origin, *not* the reference point!
		 */
		void rotate90CCW( void );
		
		/**
		 * Accessor for reference point
		 */
		const XY_Point& getReferencePoint( void ) const {
			return this->reference_point;
		}
		
		/**
		 * Accessor for path
		 */
		const PathModel& getPath( void ) const {
			return *this->path;
		}
		
		/**
		 * Accessor for vertices
		 */
		const std::vector<XY_Point>& getVertices( void ) const {
			return this->vertices;
		}
		
		/**
		 * Accessor for vertex offsets
		 */
		const std::vector<XY_Point>& getVertexOffsets( void ) const {
			return this->vertex_offsets;
		}
		
		/**
		 * Accessor for raw vertex offsets
		 */
		std::vector<XY_Point>& getRawVertexOffsets( void ) {
			return this->vertex_offsets_raw;
		}
		
		/**
		 * Get a reference to this vehicle's constraints
		 */
		const Constraints& getConstraints( void ) const {
			return *this->c;
		}
		
		/**
		 * Set velocity while being mindful of constraints
		 */
		void setVelocity( double velocity );
		
		/**
		 * Message handler for exceptions thrown during construction
		 */
		static std::string exceptionMessage( int e );
		
	private:
		
		/**
		 * Convenience method used by constructors
		 */
		void init( const std::vector<XY_Point>& vertex_offsets,
				  const PathModel& path,
				  const Constraints& c,
				  const Controller& controller,
				  double path_position = 0.,
				  double velocity = 0.,
				  double orientation = std::numeric_limits<double>::quiet_NaN() );
		
		/**
		 * Set acceleration while being mindful of constraints
		 */
		void setAcceleration( double acceleration );
		
		/**
		 * The controller sequence for this vehicle
		 */
		Controller controller;
		
		/**
		 * Storage for the vertices of this polygon
		 */
		std::vector<XY_Point> vertices;
		
		/**
		 * The offsets for each vertex from the reference point
		 */
		std::vector<XY_Point> vertex_offsets;
		
		/**
		 * The unoriented offsets for each vertex from the reference point
		 */
		std::vector<XY_Point> vertex_offsets_raw;
		
		/**
		 * Constraint set for this vehicle
		 */
		Constraints * c;
		
		/**
		 * Convenience function for computing vertices for a given reference point.
		 */
		void updateVertices( XY_Point& new_reference, double orientation );
		
		/**
		 * Orientation of the vehicle in the world frame
		 */
		double orientation;
		
		/**
		 * The path position is the arc length along the current path at which
		 * the vehicle finds itself
		 */
		double path_position;
		
		/**
		 * The index of the segment that corresponds to the current path position
		 */
		ssize_t path_position_index;
		
		/**
		 * The reference point for the driver model
		 */
		XY_Point reference_point;
		
		/**
		 * This is the path in the world frame of the obstacle
		 * defined as a piecewise parabolic path
		 */
		PathModel * path;
		
		/**
		 * Velocity of the vehicle along the path
		 *
		 * NOTE: This is *not* velocity in the world frame!
		 */
		double velocity;
		
		/**
		 * Acceleration of the vehicle along the path
		 *
		 * NOTE: This is *not* acceleration in the world frame!
		 */
		double acc;
		
	};
	
} // end Scenario namespace

#endif