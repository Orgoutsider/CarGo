/*
Ping指令测试,测试总线上相应ID舵机是否就绪,广播指令只适用于总线只有一个舵机情况
*/

#include "ros/ros.h"
#include "my_hand_eye/SCServo.h"

SMS_STS sm_st; 
SCSCL sc;

int main(int argc, char **argv)
{
	if(argc<2){
        ROS_ERROR("argc error!");
        return 0;
	}
	ROS_INFO_STREAM("serial:"<<argv[1]);
    if(!sm_st.begin(115200, argv[1])||!sc.begin(115200, argv[1])){
        ROS_ERROR("Failed to init sms/sts motor!");
        return 0;
    }
    int ID = sc.Ping(1);
	if(ID!=-1){
		ROS_INFO_STREAM("ID:"<<ID);
	}else{
		ROS_WARN("Ping servo ID error!");
	}
	ID = sm_st.Ping(2);
	if(ID!=-1){
		ROS_INFO_STREAM("ID:"<<ID);
	}else{
		ROS_WARN("Ping servo ID error!");
	}	
    ID = sm_st.Ping(3);
	if(ID!=-1){
		ROS_INFO_STREAM("ID:"<<ID);
	}else{
		ROS_WARN("Ping servo ID error!");
	}
    ID = sm_st.Ping(4);
	if(ID!=-1){
		ROS_INFO_STREAM("ID:"<<ID);
	}else{
		ROS_WARN("Ping servo ID error!");
	}
	ID = sc.Ping(5);
	if(ID!=-1){
		ROS_INFO_STREAM("ID:"<<ID);
	}else{
		ROS_WARN("Ping servo ID error!");
	}
	ID = sm_st.Ping(6);
	if(ID!=-1){
		ROS_INFO_STREAM("ID:"<<ID);
	}else{
		ROS_WARN("Ping servo ID error!");
	}
	sm_st.end();
    sc.end();
	return 0;
}

