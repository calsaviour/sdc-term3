/**
 * Global variables
 */
var origin = null;
var map, poly;
var markers = [];
var path = new google.maps.MVCArray;

/**
 * Initialize the map
 */
function initialize() {
	var mapOptions = {
		center: new google.maps.LatLng(39.165325, -86.52638569999999),
		zoom: 13,
		mapTypeId: google.maps.MapTypeId.ROADMAP
	};
	map = new google.maps.Map( document.getElementById("map_canvas"), mapOptions );

	poly = new google.maps.Polyline({
		strokeWeight: 3,
		fillColor: '#5555FF'
	});
	poly.setMap(map);
	poly.setPath(new google.maps.MVCArray([path]));
	
	google.maps.event.addListener(map, 'click', addPoint);
}

/**
 * Search for an address or lat/long coordinate, move the map to the search result
 */
function search() {
	var base_url = "http://maps.googleapis.com/maps/api/geocode/json";
	var sensor_url = "&sensor=false";
	var latitude = document.getElementById("lat").value.trim();
	var longitude = document.getElementById("long").value.trim();
	var zip = document.getElementById("zip").value.trim();

	if 	( !(latitude || longitude || zip) ) {
		alert( "Please enter either a lat and long, or a ZIP." );
		return;
	}

	if ( !zip && (!latitude || !longitude) ) {
		alert( "Both latitude and longitude are required if an address is not provided." );
		return;
	}

	if	( zip ) {
	
		// perform geocoding
		$.post(
			base_url + "?address=" + encodeURIComponent(zip) + "&sensor=false",
			{},
			function ( json, status ) {
				if ( json.results.length > 0 ) {				
					var center = new google.maps.LatLng(
						json.results[0].geometry.location.lat,
						json.results[0].geometry.location.lng
					);
					map.setCenter( center );
					map.setZoom( 13 );					
					document.getElementById("lat").value = json.results[0].geometry.location.lat;
					document.getElementById("long").value = json.results[0].geometry.location.lng;				
				} else {
					alert( "bad results" );
				}
			}
		);
		
		} else {
			var center = new google.maps.LatLng(
				latitude,
				longitude
			);
			map.setCenter( center );
			map.setZoom( 13 );	
	}
}
      
/**
 * Add a marker to the map when mouse clicked
 */
function addPoint(event) {
	path.insertAt(path.length, event.latLng);
	
	var marker = new google.maps.Marker({
		position: event.latLng,
		map: map,
		draggable: true
	});
	markers.push(marker);
	marker.setTitle("#" + path.length);
	
	google.maps.event.addListener(
		marker,
		'click',
		function() {
			marker.setMap(null);
			for (var i = 0, I = markers.length; i < I && markers[i] != marker; ++i);
			markers.splice(i, 1);
			path.removeAt(i);
		}
	);
	
	google.maps.event.addListener(
		marker,
		'dragend',
		function() {
			for (var i = 0, I = markers.length; i < I && markers[i] != marker; ++i);
			path.setAt(i, marker.getPosition());
		}
	);
}

/**
 * Generate a path from the sequence of markers placed on the map
 */
function getPath() {
	if ( origin == null ) {
		return;
	}

	var path = poly.getPath();
	if ( path.getArray().length == 0 ) {
		return;
	}
	
	var path_name_prefix = document.getElementById("path_name").value.trim();

	var prev_lat = origin.lat();
	var prev_lng = origin.lng();
	var cur_lat = path.getAt(0).lat();
	var cur_lng = path.getAt(0).lng();
	var d = HaversineDistance( prev_lat, prev_lng, cur_lat, cur_lng );
	var b = bearing(prev_lat, prev_lng, cur_lat, cur_lng);
	var delPos = deltaXdeltaY( d, b );
	var prev_x = delPos.delX;
	var prev_y = delPos.delY;
	prev_lat = cur_lat;
	prev_lng = cur_lng;

	var sum = 0;
	var path_string = "";
	for ( i=1; i<path.getArray().length; i++ ) {
		cur_lat = path.getAt(i).lat();
		cur_lng = path.getAt(i).lng();
		d = HaversineDistance( prev_lat, prev_lng, cur_lat, cur_lng );
		b = bearing(prev_lat, prev_lng, cur_lat, cur_lng);
		delPos = deltaXdeltaY( d, b );
		var cur_x = prev_x + delPos.delX;
		var cur_y = prev_y + delPos.delY;
		path_string += getPathSeg( {x:prev_x, y:prev_y}, {x:cur_x, y:cur_y}, d, path_name_prefix );
		sum += d;
		prev_lat = cur_lat;
		prev_lng = cur_lng;
		prev_x = cur_x;
		prev_y = cur_y;
	}

	var output = document.getElementById( "output" );
	output.value = path_string;
}

/**
 * Convenience method for converting degrees to radians
 */
function deg2rad( deg ) {
	return deg * Math.PI / 180;
}

/**
 * Compute distance between two lat/long coordinates with Haversine method
 *
 * http://www.movable-type.co.uk/scripts/latlong.html
 */
function HaversineDistance( lat1, lon1, lat2, lon2 ) {
	var R = 6371000; // meters
	var dLat = deg2rad( lat2 - lat1 );
	var dLon = deg2rad( lon2 - lon1 );
	lat1 = deg2rad( lat1 );
	lat2 = deg2rad( lat2 );
	
	var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
			Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(lat1) * Math.cos(lat2); 
	var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a)); 
	return R * c;
}

/**
 * Compute final bearing when traveling from one lat/long coordinate to another.
 * Note that for this purpose the distances are so short relative to the Earth that
 * we assume bearing does not change over course of path.
 *
 * http://www.movable-type.co.uk/scripts/latlong.html
 */
function bearing( lat1, lon1, lat2, lon2 ) {
	var dLat = deg2rad( lat2 - lat1 );
	var dLon = deg2rad( lon2 - lon1 );
	lat1 = deg2rad( lat1 );
	lat2 = deg2rad( lat2 );

	var y = Math.sin( dLon ) * Math.cos( lat2 );
	var x = Math.cos( lat1 ) * Math.sin( lat2 ) -
		Math.sin( lat1 ) * Math.cos( lat2 ) * Math.cos( dLon );
	return deg2rad( 90 ) - Math.atan2( y, x );
}

/**
 * Convenience method for computing legs of right triangle
 */
function deltaXdeltaY( d, b ) {
	var x = d * Math.cos( b );
	var y = d * Math.sin( b );
	return { delX : x, delY : y };
}

/**
 * Remove a path from the map
 */
function removePath() {
	for ( i=0; i<markers.length; i++ ) {
		markers[i].setMap( null );
		markers[i] = null;
	}
	markers = [];
	path = new google.maps.MVCArray;
	poly.setPath(new google.maps.MVCArray([path]));
	var output = document.getElementById( "output" );
	output.value = "";
}

/**
 * Convenience method for outputting C++ code to build path segment
 */
function getPathSeg( p1, p2, d, path_name_prefix ) {
	var constraints_object_name = "c";
	
	return "seg_start.setCoords( " + p1.x + ", " + p1.y + " );\n" +
		"seg_end.setCoords( " + p2.x + ", " + p2.y + " );\n" +
		path_name_prefix + "segments.push_back( PathSegment(seg_start, seg_end, " + constraints_object_name + ") );\n" +
		"\n";
}

/**
 * Set the map point that will be used as the origin for path generation
 */
function setOrigin() {
	if ( markers.length < 1 ) {
		return;
	}
	origin = markers[markers.length-1].getPosition();
	removePath();
	var origin_el = document.getElementById("origin");
	origin_el.innerHTML = "&nbsp;Lat: " + origin.lat().toFixed(6) + ", Long: " + origin.lng().toFixed(6) + "&nbsp;";
	origin_el.style.background = "#080";
}
      