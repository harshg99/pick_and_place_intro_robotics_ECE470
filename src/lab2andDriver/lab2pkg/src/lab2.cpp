#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

//arrays defining Waypoints
double home[]={158*PI/180,-95*PI/180,132*PI/180,-130*PI/180,-92*PI/180,68*PI/180};

double arr11[]={151*PI/180,-50*PI/180,107.1*PI/180,-146.7*PI/180,-86.6*PI/180,65*PI/180};
double arr12[]={151*PI/180,-55.8*PI/180,108.7*PI/180,-146.6*PI/180,-87.9*PI/180,61.6*PI/180};
double arr13[]={151*PI/180,-62.2*PI/180,105.33*PI/180,-134.5*PI/180,-88.5*PI/180,62.6*PI/180};

double arr21[]={162.2*PI/180,-50.3*PI/180,108.5*PI/180,-148.4*PI/180,-88.9*PI/180,69.3*PI/180};
double arr22[]={162.4*PI/180,-57.2*PI/180,110.9*PI/180,-147*PI/180,-89.1*PI/180,71.3*PI/180};
double arr23[]={162.4*PI/180,-63.4*PI/180,107.1*PI/180,-134.5*PI/180,-88.7*PI/180,72.4*PI/180};

double arr31[]={174.2*PI/180,-49.7*PI/180,107.4*PI/180,-148.6*PI/180,-89.3*PI/180,83.5*PI/180};
double arr32[]={175.3*PI/180,-56.3*PI/180,106.7*PI/180,-142*PI/180,-92*PI/180,84.6*PI/180};
double arr33[]={175.6*PI/180,-61.4*PI/180,104*PI/180,-134.5*PI/180,-92*PI/180,84.61*PI/180};

//structure to define one move form a position to other position

/*
 * 1-2 referenced by 0
 * 2-3 referenced by 1
 * 3-1 referenced by 2
 * 2-1 referenced by 3
 * 3-2 referenced by 4
 * 1-3 referenced by 5

 */

int moveseq12[6][7]={{1321,1231,2132,1121,3211,3122,1123},
                     {2331,2211,3112,2131,1221,1132,2133},
                     {3311,3221,1122,3111,2231,2112,3113},
                     {2311,2231,1132,2111,3221,3112,2113},
                     {3321,3211,2112,3121,1231,1122,3123},
                     {1331,1221,3122,1131,2211,2132,1133}};





// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};
bool pick_State;

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));

std::vector<double> Q21 (arr21,arr21+sizeof(arr21) / sizeof(arr21[0]));
std::vector<double> Q22 (arr22,arr22+sizeof(arr22) / sizeof(arr22[0]));
std::vector<double> Q23 (arr23,arr23+sizeof(arr23) / sizeof(arr23[0]));

std::vector<double> Q31 (arr31,arr31+sizeof(arr31) / sizeof(arr31[0]));
std::vector<double> Q32 (arr32,arr32+sizeof(arr32) / sizeof(arr32[0]));
std::vector<double> Q33 (arr33,arr33+sizeof(arr33) / sizeof(arr33[0]));


std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q [3][3] = {
    {Q11, Q12, Q13},
    {Q21, Q22, Q23},
    {Q31, Q32, Q33}
};


// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}

void inputState_callback(const ur_msgs::IOStates::ConstPtr& msg){
  pick_State=msg->digital_in_states[0].state;
}


int move_arm(	ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
    int error = 0;
    int spincount = 0;
    ece470_ur3_driver::command driver_msg;
    ROS_INFO("sending Goals");
    driver_msg.destination=dest;
    driver_msg.duration=duration;
    pub_command.publish(driver_msg);  // publish command, but note that is possible that
                          // the subscriber will not receive this message.
    spincount = 0;
    while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
      ros::spinOnce();  // Allow other ROS functionallity to run
      loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
      if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
        pub_command.publish(driver_msg);
        ROS_INFO("Just Published again driver_msg");
        spincount = 0;
      }
      spincount++;  // keep track of loop count
    }

    ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
    while(!isReady)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

}

//state 1.0 is on anmd 0.0 is off
bool setSuction(ros::ServiceClient srv_SetIO,ur_msgs::SetIO srv,double state){
  bool error = false;
  srv.request.fun = 1;
  srv.request.pin = 0;  //Digital Output 0
  srv.request.state = state; //Set DO0 on
  if (srv_SetIO.call(srv)) {
    ROS_INFO("True: Switched Suction ON");
  } else {
    ROS_INFO("False");
    error=true;
  }
  return error;
}



int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int start_loc,
                int start_height,
                int end_loc,
                int end_height)
{
    int ok=1;
    move_arm(pub_command,loop_rate,QH,1.0);
    move_arm(pub_command,loop_rate,Q[start_loc-1][start_height-1],1.0);
    setSuction(srv_SetIO,srv,1.0);
    move_arm(pub_command,loop_rate,QH,1.0);

    int spincount=0;
    while (spincount<6) { // Waiting for isReady to be false meaning that the driver has the new command
      ros::spinOnce();  // Allow other ROS functionallity to run
      loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
      spincount++;  // keep track of loop count
    }

    cout<<"Picked "<<pick_State;
    if(pick_State==false){
      ROS_INFO("BLOCK NOT PICKED UPP\n\n");
      ok=0;
      return ok;
    }
    move_arm(pub_command,loop_rate,Q[end_loc-1][end_height-1],1.0);
    setSuction(srv_SetIO,srv,0.0);
    return ok;

}

void solveHanoi(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int moveseqidx){
  for(int i=0;i<sizeof(moveseq12[0])/sizeof(moveseq12[0][0]);i++){
    int cmd=moveseq12[moveseqidx][i];
    int start_loc=cmd/1000;
    cmd=cmd%1000;
    int start_height=cmd/100;
    cmd=cmd%100;
    int end_loc=cmd/10;
    cmd=cmd%10;
    int end_height=cmd;
    cout<<"SL= "<<start_loc<<"SH= "<<start_height<<"EL= "<<end_loc<<"EH= "<<end_height<<"\n";
    if(!move_block(pub_command,loop_rate,srv_SetIO,srv,start_loc,start_height,end_loc,end_height)){
      ROS_INFO("EXECUTION HALTING\n");
      return;
    }
  }
}

int main(int argc, char **argv)
{
    
	int inputdone = 0;
	int Loopcnt = 0;
//initialization & variable definition
	ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
	ros::NodeHandle nh;				//handler for this node.
	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
  ros::Subscriber sub_IO=nh.subscribe("ur_driver/io_states",1,inputState_callback);
	ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
	ur_msgs::SetIO srv;

	ece470_ur3_driver::command driver_msg;

  int start;
  int end;
  int moveseqidx;
	while (!inputdone) {
    std::cout << "Enter Start <Either 1 2 or 3>";
    std::cin>>start;
    std::cout << "You entered " << start<<"\n";
    std::cout << "Enter Start <Either 1 2 or 3>";
    std::cin>>end;
    std::cout << "You entered " << end<<"\n";
    if (start==end) {
      inputdone = 0;
      moveseqidx=-1;
    } else if (start==1 && end==2 ) {
			inputdone = 1;
      moveseqidx=0;
    }else if (start==2 && end==3 ) {
      inputdone = 1;
      moveseqidx=1;
    }else if (start==3 && end==1 ) {
      inputdone = 1;
      moveseqidx=2;
    }else if (start==2 && end==1 ) {
      inputdone = 1;
      moveseqidx=3;
    }else if (start==3 && end==2 ) {
      inputdone = 1;
      moveseqidx=4;
    }else if (start==1 && end==3 ) {
      inputdone = 1;
      moveseqidx=5;
    }
    else {
      std:cout << "Please enter betwewen 1 and 3 for both positions\n\n";
		}
    std::cout << "Executing " << moveseqidx<<"\n";
	}

	while(!ros::ok()){};	//check if ros is ready for operation
		
	ROS_INFO("sending Goals");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
 
  move_arm(pub_command,loop_rate,QH,1.0);
  solveHanoi(pub_command,loop_rate,srv_SetIO,srv,moveseqidx);
  move_arm(pub_command,loop_rate,QH,1.0);
  setSuction(srv_SetIO,srv,0.0);
	return 0;
}
