//#include <Eigen/Dense>
#include <math.h>

#ifndef __PI__
#define __PI__

const double PI=3.14159265;

#endif //__PI__

#ifndef __FIX_ANGLE_HPP__
#define __FIX_ANGLE_HPP__


// to fix the angle to -pi to pi
float fix_angle(float angle){
	
	float angle_fix=((angle+PI)/(2*PI)-floorf((angle+PI)/(2*PI)))*2*PI-PI;
	
	return (angle_fix);
}

/*Eigen::ArrayXf fix_angle(Eigen::ArrayXf angle){
	
	Eigen::ArrayXf angle_fix;
	angle_fix.resizeLike(angle);
	
	for(int i=0; i<angle.size(); i++){
		angle_fix(i)=fix_angle(angle(i));
	}
	
	return (angle_fix);
}*/

#endif // __FIX_ANGLE_HPP__
