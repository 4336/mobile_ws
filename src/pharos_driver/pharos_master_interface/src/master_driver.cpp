//library
#include <math.h>
#include <stdint.h>
#include <iostream>

#include <ros/ros.h>
#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <pharos_msgs/CAN_GWAY_header.h>
#include <pharos_msgs/SCBADU.h>
#include <pharos_msgs/SCBADUheader.h>


pharos_msgs::SCBADUheader scbadu_header;
pharos_msgs::SCBADU scbadu;


serial::Serial ser;
std::string str="";
std::string strCompare1 = "SCBADU";
std::string setPortName("/dev/ttyACM0");
char c = ' ';
int BaudrateNum(115200);
double hzNum(100);
//std::vector<uint8_t> testHex;

float k_ = 0.0025;
float v_gain_ = 0.01;
float a_gain_ = 0.01;

double steering = 0;
double clutch = 0;
double brake = 0;
double accel = 0;
double geardown = 0;
double gearup = 0;

double kph = 0;
double vel = 0;
double vel_LPF = 0;

double acc = 0;
double acc_notch = 0;
double acc_notch_gain = 0.2;
double acc_LPF = 0;
double acc_LPF_gain = 0.1;

double vel_time = 0;
double vel_time_ = 0;

void SlaveStateCB(const std_msgs::String::ConstPtr& msg){
    ser.write(msg->data);
//    ser.write(testHex);
}

float CenterGuidance(const pharos_msgs::CAN_GWAY_headerConstPtr& CAN){

    float centerTorque = k_*(steering - CAN->GWAY2.Steering_Angle);
    return centerTorque;
}

float HandlePosFB(const pharos_msgs::CAN_GWAY_headerConstPtr& CAN, char side){
    int direction;
    if(side == 'l') direction = 1;
    else if(side == 'r') direction = -1;
    else direction = 0;

    float alpha = 0.01;
    vel_LPF = vel_LPF * (1-alpha) - kph/3.6 * alpha;

    if(CAN->GWAY4.Longitudinal_Accel_Speed - acc_notch > acc_notch_gain)
        acc_notch = acc_notch + acc_notch_gain;
    else if(CAN->GWAY4.Longitudinal_Accel_Speed - acc_notch < -acc_notch_gain)
        acc_notch = acc_notch - acc_notch_gain;
    else acc_notch = CAN->GWAY4.Longitudinal_Accel_Speed;

    acc_LPF = acc_LPF * (1-acc_LPF_gain) + acc_notch * acc_LPF_gain;

    std::cout<<v_gain_<<"\t"<<CAN->GWAY4.Longitudinal_Accel_Speed<<"\t"<<acc_notch<<"\t"<<acc_LPF<<std::endl;

    vel = vel_LPF;
    acc = -acc_LPF;
    return vel*v_gain_ + acc* a_gain_;
}

void CANinfoCB(const pharos_msgs::CAN_GWAY_headerConstPtr& CAN){
    kph = (CAN->GWAY1.Wheel_Velocity_FL+CAN->GWAY1.Wheel_Velocity_FR
        +CAN->GWAY1.Wheel_Velocity_RL+CAN->GWAY1.Wheel_Velocity_RR)/4.0;


    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss.str("");
    ss << CenterGuidance(CAN);
    std::string centerT_s = ss.str();

    ss.str("");
    ss << HandlePosFB(CAN, 'l');
    std::string leftP_s = ss.str();

    ss.str("");
    ss << HandlePosFB(CAN, 'r');
    std::string rightP_s = ss.str();

    ss << std::fixed << std::setprecision(2);

    ss.str("");
    ss << kph;
    std::string kph = ss.str();

    ss.str("");
    ss << CAN->GWAY4.Yaw_Rate_Sensor/5;
    std::string acc_lat = ss.str();

    ss.str("");
    ss << CAN->GWAY4.Longitudinal_Accel_Speed;
    std::string acc_lon = ss.str();

    ss.str("");
    ss << 0;
    std::string acc_alt = ss.str();


    std::string stringMsg = "$KPH,"+kph+"\n$ACC,"+acc_lat+","+acc_lon+","+acc_alt+"\n"
        +"$CLR,"+centerT_s+","+leftP_s+","+rightP_s+"\n";

    ser.write(stringMsg);
    std::cout<<stringMsg;
}

void SCBADUcalibration(){
    scbadu.steering = steering; //deg
    // clutch = (clutch - 890) / (240 - 890);
    scbadu.clutch = clutch>100?100:(clutch<0?0:clutch);
    // brake = (brake - 830) / (170 - 830);
    scbadu.brake = brake>100?100:(brake<0?0:brake);
    // accel = (accel - 870) / (210 - 870);
    scbadu.accel = accel>100?100:(accel<0?0:accel);
    scbadu.down = geardown;
    scbadu.up = gearup;
    scbadu_header.header.stamp = ros::Time::now();
    scbadu_header.scbadu = scbadu;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "master_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("setPort", setPortName, "/dev/ttyUSB0");
    pnh.param<int>("setBaudrate", BaudrateNum, 115200);
    pnh.param<double>("setHz", hzNum, 100);

    pnh.param<float>("k", k_, 0.0025);
    pnh.param<float>("v_gain", v_gain_, 0.00001);
    pnh.param<float>("a_gain", a_gain_, 0.00001);



    ros::Subscriber write_sub_ = nh.subscribe("/slave/interfaceFB", 1, SlaveStateCB);
    ros::Subscriber write_sub = nh.subscribe("/relay/CAN_Gateway", 1, CANinfoCB);
    ros::Publisher read_pub = nh.advertise<pharos_msgs::SCBADUheader>("/master/interface", 1);


    try
    {
        ser.setPort(setPortName); // ACM0 에서 USB0 으로 수정함. "/dev/ttyUSB0"
        ser.setBaudrate(BaudrateNum); // 비트레이트 수정함.
        serial::Timeout to = serial::Timeout::simpleTimeout(hzNum);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }


    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(hzNum);
//    ser.write("<sof1>"); // serial write 가능! 만세!
    while(ros::ok())
    {
        ros::spinOnce();

        while(ser.available())
        {
            std::string s = ser.read();
            c = *s.c_str();
            // str.append( ser.read() );
            if( c == '\n'){
                if(str.length() > 10){
                    if(strCompare1.compare( str.substr(1, 6) ) == 0 ){

                        int first = str.find(",");

                        int two = str.find(",", first+1);

                        int three = str.find(",", two+1);

                        int four = str.find(",", three+1);

                        int five = str.find(",", four+1);

                        int six = str.find(",", five+1);

                        std::string steering_s = str.substr(first+1, two-first);

                        std::string clutch_s = str.substr(two+1, three-two);

                        std::string brake_s = str.substr(three+1, four-three);

                        std::string accel_s = str.substr(four+1, five-four);

                        std::string geardown_s = str.substr(five+1, six-five);

                        std::string gearup_s = str.substr(six+1, str.length()-six);

                        steering = atof(steering_s.c_str());
                        clutch = atof(clutch_s.c_str());
                        brake = atof(brake_s.c_str());
                        accel = atof(accel_s.c_str());
                        geardown = atof(geardown_s.c_str());
                        gearup = atof(gearup_s.c_str());

                        // std::cout<<str<<std::endl;
                        // std::cout<<"S: "<<steering_s<<std::endl;
                        // std::cout<<"C: "<<clutch_s<<std::endl;
                        // std::cout<<"B: "<<brake_s<<std::endl;
                        // std::cout<<"A: "<<accel_s<<std::endl;
                        // std::cout<<"D: "<<geardown_s<<std::endl;
                        // std::cout<<"U: "<<gearup_s<<std::endl<<std::endl;

                        // steering = atoi(steering_s);

                        SCBADUcalibration();
                        read_pub.publish(scbadu_header);
                    }
                }
                str = "";
            }else{
                str.push_back(c);
            }
        }
    loop_rate.sleep();
    }
}

