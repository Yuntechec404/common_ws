/*
*******************************************
*/
#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//创建一个serial类
serial::Serial sp;

#define to_rad  0.017453f  //角度转弧度


uint8_t FLAG_USART; //串口发送标志
uint16_t count_1,count_2;//计数器



int size;
int Voltage;


uint16_t a,b;
void send_data(void);//串口发送协议函数

int32_t S_H1;
int32_t S_L1;

uint8_t S_En1;
uint8_t S_En2;
uint8_t S_En3;



int32_t R_H1;
int32_t R_L1;

uint8_t R_En1;
uint8_t R_En2;
uint8_t R_En3;

float Servo_Angle_RE;


float Servo_Angle_SE;
short Saw_speed_SE;

short Move_speed_L ;
short Move_speed_H ;

void chatterCallback(const geometry_msgs::Twist &msg)//获取键盘控制的回调函数
{

    ROS_INFO("X_linear: [%g]", msg.linear.x);//
    ROS_INFO("Y_linear: [%g]", msg.linear.y);//
    ROS_INFO("Z_linear: [%g]", msg.linear.z);//
    ROS_INFO("X_angular: [%g]", msg.angular.x);//
    ROS_INFO("Y_angular: [%g]", msg.angular.y);//
    ROS_INFO("Z_angular: [%g]", msg.angular.z);//
    ROS_INFO("-------------");
	
            if(msg.linear.x>0 && msg.angular.z>0){//按下 U 键
							FLAG_USART=1;
			        }//开启发送指令
       else if(msg.linear.x>0 && msg.angular.z<0){//按下 O 键
              FLAG_USART=0;
			        }//停止发送指令
			 	
}


int main(int argc, char **argv){

    ros::init(argc, argv, "listener");

    ros::NodeHandle np;//为这个进程的节点创建一个句柄

    ros::Subscriber sub = np.subscribe("cmd_vel", 200, chatterCallback);//订阅键盘控制

    
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }  
	
	
   ros::Rate loop_rate(100);//设置循环间隔，即代码执行频率 100 HZ

   while(ros::ok())
   {	

       ros::spinOnce();//执行回调处理函数，完后继续往下执行

		 
       S_H1=-100;//手臂 的高度 高度范围-350~0(毫米)(初始位置0的时候是最高处),
       S_L1=-100;//手臂 的长度 长度范围 -400~0(毫米)(初始位置0的时候是最短处)
		 
       Servo_Angle_SE=0;//舵机的角度 范围±90°
       Saw_speed_SE=0;//锯片的转速	数值范围0~6000	，数越大转速越快 

			 Move_speed_H = 2000;//升降移动速度 范围500~5000
			 Move_speed_L = 3000;//伸缩移动速度 范围500~5000
		 
       S_En1=1;//电机1使能(控制手臂1的高度)
       S_En2=1;//电机2使能(控制手臂1的长度)
       S_En3=1;//电机3使能(控制手臂2的高度)

		 
       if(FLAG_USART==1)send_data(); //发送指令控制电机运行		  
		
       //连续获取下位机的数据				
       size_t n = sp.available();//获取缓冲区内的字节数
       a++;
         if(n>0)
               {		   
                 uint8_t buffer[24];uint8_t buf[24];
                 
                 if(n>=48){
                   while(n){n = sp.available();if(n>=48)sp.read(buf, 24);else {break;}}//砍掉旧缓存，获取最新数据                  
                 }                 
                 if(n>=24){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)sp.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到第一个帧头跳出
										n = sp.available(); 
                  }                    
                 if(buffer[0]==0XAA&&n>=23)//
                  {
                   sp.read(buffer, 23);//读出剩余的23个字节                   
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0;
											
	                     for(uint8_t j=0;j<22;j++)sum+=buffer[j];    //计算校验和	

                       if(buffer[2] == 19 && buffer[22] == (uint8_t)(sum + 0XAA))
                          {b++;	
 						                   R_H1  = (int32_t)((buffer[3]<<0) |(buffer[4]<<8) |(buffer[5]<<16) |(buffer[6]<<24));//单位毫米
														   R_L1  = (int32_t)((buffer[7]<<0) |(buffer[8]<<8) |(buffer[9]<<16) |(buffer[10]<<24));//单位毫米
									
										           R_En1 = buffer[11];
										           R_En2 = buffer[12]; 
										           R_En3 = buffer[13]; 

 										           Servo_Angle_RE  = (float)((int32_t)((buffer[14]<<0) |(buffer[15]<<8) |(buffer[16]<<16) |(buffer[17]<<24)))*0.01f;
														
                               Voltage =(int32_t)((buffer[18]<<0) |(buffer[19]<<8) |(buffer[20]<<16) |(buffer[21]<<24));														
	                        } 			  						
                       }
                       buffer[0]=0Xff;buffer[1]=0Xff;
                   }
		     
                } 
      /*<01>*///buffer[0]=0XAA;   //帧头
	    /*<02>*///buffer[1]=0XAA;   //帧头
      /*<03>*///buffer[2]=0XF1;   //功能字
      /*<04>*///buffer[3]=19;     //数据长度
								
			/*<05>*///buffer[4] ;//H1
			/*<06>*///buffer[5] ; 
			/*<07>*///buffer[6] ;
			/*<08>*///buffer[7] ; 
								
			/*<09>*///buffer[8] ;//L1
			/*<10>*///buffer[9] ;
			/*<11>*///buffer[10] ;    
			/*<12>*///buffer[11] ;  
																					
      /*<13>*///buffer[20];//EN1
      /*<14>*///buffer[21];//EN2					 
			/*<15>*///buffer[22];//EN3	
								
			/*<16>*///buffer[12] ;//Servo_Angle_RE 
			/*<17>*///buffer[13] ; 
			/*<18>*///buffer[14];
			/*<19>*///buffer[15];

			/*<20>*///buffer[26];//Voltage
			/*<21>*///buffer[27];// 
			/*<22>*///buffer[28];// 
			/*<23>*///buffer[29];// 			
		              count_1++;
                  if(count_1>10){//显示频率降低为10HZ
                      count_1=0;

                      ROS_INFO("[01] Current_Height_1: [%d  mm]", R_H1);

                      ROS_INFO("[02] Current_length_1: [%d  mm]", R_L1);


                      ROS_INFO("[03] En1: [%d  ]", R_En1);

                      ROS_INFO("[04] En2: [%d  ]", R_En2);

                      ROS_INFO("[05] En3: [%d  ]", R_En3);

                      ROS_INFO("[06] Servo_Angle_RE: [%.2f deg]", Servo_Angle_RE);										
								
                      ROS_INFO("[07] Voltage: [%.2f V]", (float)Voltage/100); // 电池电压
                      ROS_INFO("-----------------------"); 
                      ROS_INFO("a: [%d ]",   a);
                      ROS_INFO("b: [%d ]",   b);
											ROS_INFO("a/b: [%.2f ]",   (float)a/(float)b);
                      if(b>5000)b=b/10,a=a/10;														
                     }					 

        loop_rate.sleep();//循环延时时间
   }


   //关闭串口
   sp.close();  

    return 0;
}

//************************发送数据**************************// 

void send_data(void)
{
    uint8_t tbuf[26];

    tbuf[25]=0;  //校验位置零
    tbuf[0]=0XAA;//帧头
    tbuf[1]=0XAA;//帧头
    tbuf[2]=0XF1;//功能字
    tbuf[3]=21;  //数据长度
              tbuf[4]=	S_H1>>0; //    
              tbuf[5]=	S_H1>>8; //
              tbuf[6]=	S_H1>>16;//
              tbuf[7]=	S_H1>>24;//

              tbuf[8]=	S_L1>>0; //  
              tbuf[9]=	S_L1>>8; //
              tbuf[10]=	S_L1>>16;//
              tbuf[11]=	S_L1>>24;//

              tbuf[12]=	(int)(Servo_Angle_SE*100)>>0; // 
              tbuf[13]=	(int)(Servo_Angle_SE*100)>>8; //
              tbuf[14]=	(int)(Servo_Angle_SE*100)>>16;//
              tbuf[15]=	(int)(Servo_Angle_SE*100)>>24;//

              tbuf[16]=	Saw_speed_SE>>0;// 
              tbuf[17]=	Saw_speed_SE>>8;//

              tbuf[18]=	Move_speed_H>>0;// 
              tbuf[19]=	Move_speed_H>>8;//
              tbuf[20]=	Move_speed_L>>0;//
              tbuf[21]=	Move_speed_L>>8;//
							
              tbuf[22]=	S_En1;//
              tbuf[23]=	S_En2;//
							tbuf[24]=	S_En3;//
							
    for(uint8_t i=0;i<25;i++)tbuf[25]+=tbuf[i];//计算校验和 
  try
  {
    sp.write(tbuf, 26);//发送数据下位机(数组，字节数)

  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果发送数据失败，打印错误信息
  }
}  

