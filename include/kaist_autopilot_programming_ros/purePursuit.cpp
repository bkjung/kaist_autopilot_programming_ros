#include <kaist_autopilot_programming_ros/purePursuit.h>

purePursuit::purePursuit(){

}

double angle_difference(double targetAngle, double sourceAngle) {
	
	double tmp = targetAngle - sourceAngle;
	if (tmp > M_PI)
		tmp -= M_PI*2;

	else if(tmp <= -M_PI) 
		tmp += M_PI*2;
	
	return tmp;
}




control purePursuit::get_control(point x_robot, point x_goal){
	

    /* TO DO
     * implement purepursuit algorithm
    */

double max_linear_vel=1.2;
double max_angular_vel=300*M_PI/180;
double init_linear_vel=0.22;
double angular_vel1=50*M_PI/180;
double angular_vel2=30*M_PI/180;
double angular_vel3=20*M_PI/180;
double angular_vel4=15*M_PI/180;
double dx=x_goal.x-x_robot.x;
double dy=x_goal.y-x_robot.y;
double l = sqrt(dx*dx+dy*dy);



double path_th = atan2(x_goal.y-x_robot.y,x_goal.x-x_robot.x);
double dth=angle_difference(path_th,x_robot.th);

double delta = std::abs(l*sin(dth));
double r=(dx*dx+dy*dy)/(2*delta);




//if (x_goal.y-x_robot.y==0 && x_goal.x-x_robot.x==0)






double threshold1=90*M_PI/180;
double threshold2=70*M_PI/180;
double threshold3=40*M_PI/180;
double threshold4=20*M_PI/180;
double threshold5=10*M_PI/180;


if (std::abs(dth)>threshold1)
{
ctrl.v=0;
ctrl.w=-angular_vel1*dth/std::abs(dth);

}
else if (std::abs(dth)>threshold2)
{
ctrl.v=0;
ctrl.w=-angular_vel2*dth/std::abs(dth);

}


else if (std::abs(dth)>threshold3)
{
ctrl.v=0;
ctrl.w=-angular_vel3*dth/std::abs(dth);

}

/*
else if (std::abs(dth)>threshold4)
{
ctrl.v=0;
ctrl.w=-angular_vel4*dth/std::abs(dth);

}

else if (std::abs(dth)<threshold5) 
{
	ctrl.v=init_linear_vel;
	ctrl.w=-ctrl.v/r*dth/std::abs(dth);
	
}
*/

else 
{

	ctrl.v=init_linear_vel;
	ctrl.w=-ctrl.v/r*dth/std::abs(dth);

}



    return ctrl;
}
