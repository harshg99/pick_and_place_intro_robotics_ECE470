#include "lab4pkg/lab4.h"

/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{

	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;
  double yaw,phi1,phi2;

	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
	d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
  a6 = 0.055;
  d6 = (0.082+0.06);

  xgrip =xWgrip+0.15 ;
  ygrip =yWgrip-0.15 ;
  zgrip =zWgrip-0.015 ;

  yaw=yaw_WgripDegree*PI/180;
  xcen=xgrip-a6*cos(yaw);
  ycen=ygrip-a6*sin(yaw);
  zcen=zgrip;

  phi1=asin((d4+d2+d3)/sqrt(xcen*xcen+ycen*ycen));
  theta1 = atan2(ycen,xcen)-phi1; // Default value Need to Change
  theta6 = -yaw+theta1+PI/2; // Default value Need to Change
 
  x3end = xcen*cos(theta1)+ycen*sin(theta1)-d4;
  y3end = -xcen*sin(theta1)+ycen*cos(theta1)-d4-d2-d3;
  z3end = zcen+d6;
  cout<<y3end<<" x "<<x3end;;

  theta2=0 ; // Default value Need to Change
  theta3= acos((((z3end-d1)*(z3end-d1)+x3end*x3end-a3*a3-a2*a2)/(2.0*a2*a3))); // Default value Need to Change
  //phi1=atan(1/(a2+a3*cos(theta3))*(a3*sin(theta3)));
  //theta2=acos(x3end/sqrt(a2*a2+a3*a3+2*a2*a3*cos(theta3)))-phi1;
  phi2=atan2((z3end-d1),x3end);
  phi1=acos(((z3end-d1)*(z3end-d1)+x3end*x3end +a2*a2-a3*a3)/(2*sqrt((z3end-d1)*(z3end-d1)+x3end*x3end)*a2));
  theta2=-phi2-phi1;
  theta4= -1*(theta3+theta2+PI/2);// Default value Need to Change
	theta5=-PI/2;  // Default value Need to Change
	



	// View values
	//use cout
	cout<<"theta1: "<< theta1<<endl;
	cout<<"theta2: "<< theta2<<endl;
	cout<<"theta3: "<< theta3<<endl;
	cout<<"theta4: "<< theta4<<endl;
	cout<<"theta5: "<< theta5<<endl;
	cout<<"theta6: "<< theta6<<endl;

	// check that your values are good BEFORE sending commands to UR3
	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}
