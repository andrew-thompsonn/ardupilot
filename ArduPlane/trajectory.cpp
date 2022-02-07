#include "trajectory.h"
#include <stdio.h>
#include "plane.h"
void RalphieTrajectory::init(warioInput_t parameters) {

    // TODO: circle
	//


	// (x - xc)^2 + (y - yc)^2 = r^2


}


void RalphieTrajectory::update() {
    // TODO: wario
}


void RalphieTrajectory::setCurrentWind(Vector3f windEstimate) {

	// grab wind data
	Vector3f wind = AP::ahrs().wind_estimate();
	float angle;
	unsigned int startTime = 100; // 135sec when taking real data
	unsigned int endTime = 250; // 255sec when taking real data
	// statics just so they don't get overwritten each time - i think idk - it is working that's what matters - probably bad coding practice
	static std::vector<float> windVec;
	static std::vector<float> averageWindSpeedVec = {0};
	static std::vector<float> angleVec;
	static std::vector<float> averageAngleVec = {0};
	// kept getting divide by 0 errors - this seems to fix it
	wind.y = wind.y + 0.0000000000000000001;
	// find angle every time step
	angle = atan(wind.x / wind.y)*180/3.14; 	// if angle = 0 -> wind is blowing from the north
																					 // if angle = 90 -> wind is blowing FROM the east TO the west
	// print the x and y speeds to monitor
	printf("Wind: %.3f m/s, %.3f m/s   ", wind.x, wind.y); // if x = positive value -> wind is blowing TO the north FROM the south
																												// if y = positive value -> wind is blowing TO the east FROM the west

	// output the direciton based on sign of vectors components
	if (wind.x < 0 && wind.y < 0){ // wind from the NW --- no correction needed
	} else if (wind.x > 0 && wind.y < 0){ // wind from the SW
		angle = angle + 180;
	} else if (wind.x > 0 && wind.y > 0) { // wind from the SE
		angle = angle + 180;
	} else if (wind.x < 0 && wind.y > 0 ){ // wind from the NE
		angle = angle + 360;
	}
	// track number of seconds the plane has been ARMED
	static unsigned int callCount = 0;
	if (AP::arming().is_armed() == 1){
		callCount++;
	}
	// monitor the time
	printf("# of times called: %.4u \n", callCount);
	// initialize average data points
	static float averageWindSpeed = 0.0;
	static float averageAngle = 0.0;
	// assign an area to take data --- using the amount of times the whole functioned has been called
	if (callCount > startTime && callCount < endTime){ // 2:15 for max overshoot angle -> call count = 135 // 4:15 for max overcorrection angle -> call count = 255
		// attach each new wind and angle data point to a vector
		windVec.push_back(sqrtf(pow(wind.x,2)+pow(wind.y,2))); // taking magnitude of wind
		angleVec.push_back(angle);
		for (float j = 0; j < windVec.size(); j++) {
			// add each wind speed
			averageWindSpeed = averageWindSpeed + windVec[j];
			averageAngle = averageAngle + angleVec[j];
		}
		// calcaulte average wind speed over the above data collection time frame
		averageWindSpeed = averageWindSpeed / windVec.size();
		averageAngle = averageAngle / angleVec.size();
	}
	// once we reach the end of data period - reset and push the new average to the vector
	if (callCount > endTime){
		callCount = 0;
		averageWindSpeedVec.push_back(averageWindSpeed);
		averageAngleVec.push_back(averageAngle);
	}
	// average wind speed and direction for each data collection time frame - printing from the back where the new value is pushed
	printf("Average Wind Speed and Direction: %.3f m/s --- %.3f degrees CW from North ", averageWindSpeedVec.back(), averageAngleVec.back());
	currentWindSpeedEstimate = averageWindSpeedVec.back();
	currentWindDirectionEstimate = averageAngleVec.back();
    memcpy(&currentWindEstimate, &windEstimate, sizeof(Vector3f));

}
