#ifndef PVTP_EXCEPTION_HANDLER_H
#define PVTP_EXCEPTION_HANDLER_H

#include <PVTP/Constraints.hpp>
#include <PVTP/PVT_Obstacle.hpp>
#include <PVTP/VehicleModel.hpp>

using namespace Scenario;

namespace PVTP {
	
	/**
	 * This is a convenience namespace for handling thrown exceptions
	 */
	namespace ExceptionHandler {
		
		/**
		 * Message handler for exceptions thrown during construction. Exception
		 * codes are such that at most one exception method will be a non-empty
		 * string.
		 */
		static std::string exceptionMessageString( int e ) {
			
			// generate appropriate error
			std::stringstream ss;
			ss << Constraints::exceptionMessage( e );
			ss << PVT_Obstacle::exceptionMessage( e );
			ss << VehicleModel::exceptionMessage( e );
			ss << PathSegment::exceptionMessage( e );
			ss << PathModel::exceptionMessage( e );
			return ss.str().c_str();
			
		}
		
		static void exceptionMessage( int e ) {
			
			// print out appropriate error
			std::cerr << exceptionMessageString( e ) << std::endl;

		}
		
	} // end ExceptionHandler namespace

} // end PVTP namespace

#endif