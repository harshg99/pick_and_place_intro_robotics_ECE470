#include "lab56pkg/lab56.h"

extern ImageConverter* ic_ptr; //global pointer from the lab56.cpp

#define SPIN_RATE 20  /* Hz */

bool isReady=1;
bool pending=0;

float SuctionValue = 0.0;

bool leftclickdone = 1;
bool rightclickdone = 1;

/*****************************************************
* Functions in class:
* **************************************************/	

//constructor(don't modify) 
ImageConverter::ImageConverter():it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera_node/image_raw", 1, 
    	&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);   
    pub_command=nh_.advertise<ece470_ur3_driver::command>("ur3/command",10);
    sub_position=nh_.subscribe("ur3/position",1,&ImageConverter::position_callback,this); 

	sub_io_states=nh_.subscribe("ur_driver/io_states",1,&ImageConverter::suction_callback,this);
	
	srv_SetIO = nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");


    driver_msg.destination=lab_invk(-.3,-.3,0.2,-90);

	//publish the point to the robot
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	driver_msg.duration = 3.0;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
										  // the subscriber will not receive this message.
	spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO_STREAM("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}
	ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
	
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM("Ready for new point");

}

//destructor(don't modify)
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady;
	pending=msg->pending;
}

void ImageConverter::suction_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
	SuctionValue = msg->analog_in_states[0].state;
}


//subscriber callback function, will be called when there is a new image read by camera
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    } 
    // create an gray scale version of image
    Mat gray_image;
	cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );  
    // convert to black and white img, then associate objects:  

	Mat bw_image;
	adaptiveThreshold(gray_image,bw_image,255,0,0,151,5);
	//adaptiveThreshold(scr,dst,MAXVALUE,adaptiveMethod,thresholdType,blocksize,C);
	//adaptiveMethod = 0, ADAPTIVE_THRESH_MEAN_C
	//thresholdType = 0, BINARY
	//blocksize
	//C constant subtracted from tz.  
	
	
// FUNCTION you will be completing
    Mat associate_image = associateObjects(bw_image); // find associated objects

    // Update GUI Window
    imshow("Image window", cv_ptr->image);
    imshow("gray_scale", gray_image);
    imshow("black and white", bw_image);
    imshow("associate objects", associate_image);
    waitKey(3);
    // Output some video stream
    image_pub_.publish(cv_ptr->toImageMsg());
} 

/*****************************************************
	 * Function for Lab 5
* **************************************************/	
// Take a grayscale image as input and return an thresholded image.
// You will implement your algorithm for calculating threshold here.
Mat ImageConverter::thresholdImage(Mat gray_img)
{
		int   totalpixels;
		Mat bw_img  = gray_img.clone(); // copy input image to a new image
		totalpixels	  = gray_img.rows*gray_img.cols;			// total number of pixels in image
		uchar graylevel; // use this variable to read the value of a pixel
		int zt=0; // threshold grayscale value 
		
		

		zt = 100;  // you will be finding this automatically 


		//std::cout<<zt<<std::endl;
		// threshold the image
		for(int i=0; i<totalpixels; i++)
		{
			graylevel = gray_img.data[i];	
			if(graylevel>zt) bw_img.data[i]= 255; // set rgb to 255 (white)
			else             bw_img.data[i]= 0; // set rgb to 0   (black)
		}	
	return bw_img;	
}
/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take an black and white image and find the object it it, returns an associated image with different color for each image
// You will implement your algorithm for rastering here
Mat ImageConverter::associateObjects(Mat bw_img)
{
	//initiallize the variables you will use
	int height,width; // number of rows and colums of image
	int red, green, blue; //used to assign color of each objects
	uchar pixel; //used to read pixel value of input image 
	height = bw_img.rows;
	width = bw_img.cols;
	int num = 0;
	// initialize an array of labels, assigning a label number to each pixel in the image
	// this create a 2 dimensional array pixellabel[row][col]
	int ** pixellabel = new int*[height];
  int label[1000];
  int *equiv[1000];

  for (int i=0;i<1000;i++){
    label[i]=0;
    equiv[i]=&label[i];
  }

	for (int i=0;i<height;i++) {
		pixellabel[i] = new int[width];
	}


	// creating a demo image of colored lines
  // rastering algorithm

  //assigning -1 to entire image before calculating number of objects

  for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++) 
		{
      if(bw_img.data[row*width+col]==0){
        pixellabel[row][col] = -1;
      }
      else{
        pixellabel[row][col]=num;
      }

		}
	}

  num=1;


  for(int row=0; row<height; row++)
  {
    for(int col=0; col<width; col++)
    {
      int Up;
      int Left;
      int ThisPixel=pixellabel[row][col];

      if(row==0){
        Up=-1;
      }
      else{
        Up=pixellabel[row-1][col];
      }


      if(col==0){
        Left=-1;
      }
      else{
        Left=pixellabel[row][col-1];
      }

      if(ThisPixel==0){
        if(Up==-1 && Left==-1){
          pixellabel[row][col]=num;
          label[num]=num;
          num++;
        }
        else if(Left==-1 && Up>=0){
          pixellabel[row][col]=Up;
        }
        else if(Up==-1 && Left>=0){
          pixellabel[row][col]=Left;
        }
        else if(Up>=0 && Left>=0){
          if(*equiv[Up]>*equiv[Left]){
            pixellabel[row][col]=*equiv[Left];
            *equiv[Up]=*equiv[Left];
            equiv[Up]=equiv[Left];
          }
          else if((*equiv[Up]<*equiv[Left])){
            pixellabel[row][col]=*equiv[Up];
            *equiv[Left]=*equiv[Up];
            equiv[Left]=equiv[Up];
          }
          else{
            pixellabel[row][col]=*equiv[Up];
          }
        }
      }
    }
  }

  int size[2000];
  for(int i=0;i<2000;i++){
    size[i]=0;
  }


  for(int row=0; row<height; row++)
  {
    for(int col=0; col<width; col++)
    {
      if(pixellabel[row][col]>=0){
       pixellabel[row][col]=*equiv[pixellabel[row][col]];
       size[pixellabel[row][col]]++;
      }
    }
  }

  int obj_num=1;
  for(int i=0;i<2000;i++){
    if(size[i]>=150 && size[i]<=8000){
      size[i]=obj_num;
      obj_num++;
    }
    else{
      size[i]=0;
    }
  }

  for(int row=0; row<height; row++)
  {
    for(int col=0; col<width; col++)
    {
      if(pixellabel[row][col]>=0){
       pixellabel[row][col]=size[pixellabel[row][col]];
      }
      else{
        pixellabel[row][col]=0;
      }
    }
  }
  cout<<obj_num<<endl;

	// assign UNIQUE color to each object
	Mat associate_img = Mat::zeros( bw_img.size(), CV_8UC3 ); // function will return this image
	Vec3b color;
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			switch (  pixellabel[row][col] )
			{
				
				case 0:
					red    = 255; // you can change color of each objects here
					green = 255;
					blue   = 255;
					break;
				case 1:
					red    = 255; // you can change color of each objects here
					green  = 0;
					blue   = 0;
					break;
				case 2:
					red    = 0;
					green  = 255;
					blue   = 0;
					break;
				case 3:
					red    = 0;
					green  = 0;
					blue   = 255;
					break;
				case 4:
					red    = 255;
					green  = 255;
					blue   = 0;
					break;
				case 5:
					red    = 255;
					green  = 0;
					blue   = 255;
					break;
				case 6:
					red    = 0;
					green  = 255;
					blue   = 255;
					break;
                case 7:
                    red    = 128;
                    green  = 128;
                    blue   = 0;
                    break;
                case 8:
                    red    = 128;
                    green  = 0;
                    blue   = 128;
                    break;
                case 9:
                    red    = 0;
                    green  = 128;
                    blue   = 128;
                 	break;
				default:
					red    = 0;
					green = 0;
					blue   = 0;
					break;					
			}

			color[0] = blue;
			color[1] = green;
			color[2] = red;
			associate_img.at<Vec3b>(Point(col,row)) = color;
		}
	}
	
	return associate_img;
}

/*****************************************************
	*Function for Lab 6
 * **************************************************/
 //This is a call back function of mouse click, it will be called when there's a click on the video window.
 //You will write your coordinate transformation in onClick function.
 //By calling onClick, you can use the variables calculated in the class function directly and use publisher
 //initialized in constructor to control the robot.
 //lab4 and lab3 functions can be used since it is included in the "lab4.h" 
void onMouse(int event, int x, int y, int flags, void* userdata)
{
		ic_ptr->onClick(event,x,y,flags,userdata);
}
void ImageConverter::onClick(int event,int x, int y, int flags, void* userdata)
{
	// For use with Lab 6
	// If the robot is holding a block, place it at the designated row and column. 
	if  ( event == EVENT_LBUTTONDOWN ) //if left click, do nothing other than printing the clicked point
	{  
		if (leftclickdone == 1) {
			leftclickdone = 0;  // code started
			ROS_INFO_STREAM("left click:  (" << x << ", " << y << ")");  //the point you clicked

			// put your left click code here


			leftclickdone = 1; // code finished
		} else {
			ROS_INFO_STREAM("Previous Left Click not finshed, IGNORING this Click"); 
		}
	}
	else if  ( event == EVENT_RBUTTONDOWN )//if right click, find nearest centroid,
	{
		if (rightclickdone == 1) {  // if previous right click not finished ignore
			rightclickdone = 0;  // starting code
			ROS_INFO_STREAM("right click:  (" << x << ", " << y << ")");  //the point you clicked

			// put your right click code here



			rightclickdone = 1; // code finished
		} else {
			ROS_INFO_STREAM("Previous Right Click not finshed, IGNORING this Click"); 
		}
	}
}

