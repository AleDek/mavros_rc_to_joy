
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "std_msgs/Int32.h"
#include "mavros_msgs/RCIn.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>

using namespace std;

class Rc_To_Joy {

	public:
		Rc_To_Joy();
        void run();
	
	private:
        void rc_cb( mavros_msgs::RCInConstPtr  data);
        void data_publisher();
        void load_params();
        double to_unit(int pwm);

		ros::NodeHandle _nh;
		ros::Subscriber _rc_sub;
        ros::Publisher _joy_pub;

        std::mutex _mtx;
        bool _first_rc;
        int _n_ch;
        //int _rssi;
        std::vector<int> _pwm;
        std::vector<int> _old_pwm;

        // ---parameters---
        int _N_axes;
        int _N_buttons;
        std::vector<int> _map_to_axes;
        std::vector<double> _axes_dir;
        std::vector<int> _map_to_button;
        std::vector<double> _button_dir;
        int _pwm_max;
        int _pwm_min;
        int _pwm_deadzone;
        double _pwm_mid;
        double _pwm_delta;

        
};

void Rc_To_Joy::load_params(){

    if( !_nh.getParam("/rc_to_joy/pwm_max", _pwm_max) ){
        ROS_ERROR("Failed to get parameter [pwm_max] from server.");
        _pwm_max = 2000;
    }
    if( !_nh.getParam("/rc_to_joy/pwm_min", _pwm_min) ){
        ROS_ERROR("Failed to get parameter[pwm_min] from server.");
        _pwm_min = 1000;
    }
    if( !_nh.getParam("/rc_to_joy/pwm_deadzone", _pwm_deadzone) ){
        ROS_ERROR("Failed to get parameter[pwm_deadzone] from server.");
        _pwm_deadzone = 2;
    }
    _pwm_mid = (_pwm_max + _pwm_min)/2;
    _pwm_delta = (_pwm_max - _pwm_min)/2;

    if( !_nh.getParam("/rc_to_joy/ch_to_axes", _map_to_axes) ){
        ROS_ERROR("Failed to get parameter[ch_to_axes] from server.");
        _map_to_axes = {1, 2, 3, 4};
    }
    _N_axes = _map_to_axes.size();
    if( !_nh.getParam("/rc_to_joy/reverse_axes", _axes_dir) ){
        ROS_ERROR("Failed to get parameter[reverse_axes] from server.");
        _axes_dir = {1, 1, 1, 1};
    }
    if(_axes_dir.size() != _N_axes){
        _axes_dir.resize(_N_axes);
        for(int i = 0;i< _N_axes; i++) _axes_dir.push_back(1);
    }
    std::stringstream ss;
    for(int i = 0;i< _map_to_axes.size(); i++){   
        ss <<_axes_dir[i] * _map_to_axes[i] <<" ";
    }
    ROS_INFO("axes are channels [ %s]",ss.str().c_str()); //DEBUG

    if( !_nh.getParam("/rc_to_joy/ch_to_buttons", _map_to_button) ){
        ROS_ERROR("Failed to get parameter[ch_to_buttons] from server.");
        _map_to_button = {5, 6};
    }
    _N_buttons = _map_to_button.size();
    if( !_nh.getParam("/rc_to_joy/reverse_buttons", _button_dir) ){
        ROS_ERROR("Failed to get parameter[reverse_buttons] from server.");
        _button_dir = {1, 1};
    }
    if(_button_dir.size() != _N_buttons){
        _button_dir.resize(_N_buttons);
        for(int i = 0;i< _N_buttons; i++) _button_dir.push_back(1);
    }
    std::stringstream ss2;
    for(int i = 0;i< _map_to_button.size(); i++){   
        ss2 << _button_dir[i]*_map_to_button[i] <<" ";
    }
    ROS_INFO("buttons are channels [ %s]",ss2.str().c_str()); //DEBUG
}

Rc_To_Joy::Rc_To_Joy() {
	_rc_sub = _nh.subscribe("/mavros/rc/in", 1, &Rc_To_Joy::rc_cb, this);
    _joy_pub = _nh.advertise<sensor_msgs::Joy>("/joy",10);
    _first_rc = false;
    _n_ch = 0;

    load_params();
}

void Rc_To_Joy::rc_cb( mavros_msgs::RCInConstPtr  data) {
    
    _first_rc = true;
    //_rssi = data->rssi;
    _n_ch = data->channels.size();
    _pwm.resize(_n_ch);
    //ROS_INFO("Listener: N_ch = %d",_n_ch); //DEBUG
    _mtx.lock();
    for(int i = 0;i< _n_ch; i++){
        _pwm[i] = (int)data->channels[i];
    }
    _mtx.unlock();

}

double Rc_To_Joy::to_unit(int pwm){
    if(pwm>_pwm_max) pwm = _pwm_max;
    else if(pwm < _pwm_min) pwm =_pwm_min;
    else if(pwm < _pwm_mid+_pwm_deadzone && pwm > _pwm_mid-_pwm_deadzone) pwm =_pwm_mid;

    return (pwm -_pwm_mid)/_pwm_delta;
}

void Rc_To_Joy::data_publisher(){
    ros::Rate rate(1.0);
    sensor_msgs::Joy joy_msg;
    joy_msg.header.frame_id = "/px4_radio";
    joy_msg.axes.resize(_N_axes);
    joy_msg.buttons.resize(_N_buttons);
    bool change = false;

    while(ros::ok()){
        change = false;
        _mtx.lock();
        for(int i = 0; i<_n_ch;i++){
            if(abs(_pwm[i] -_old_pwm[i]) > _pwm_deadzone){
                change = true;
                _old_pwm[i] = _pwm[i];
                //std::cout<<"input change"<<std::endl; //DEBUG
            }
        }
        //_old_pwm = _pwm;
        _mtx.unlock();
        if(change){
            joy_msg.header.stamp = ros::Time::now();
            for(int i = 0;i< _N_axes;i++){
               joy_msg.axes[i] = _axes_dir[i]*to_unit( _old_pwm[_map_to_axes[i]-1]);
            }
            for(int i = 0;i< _N_buttons;i++){
               joy_msg.buttons[i] = _button_dir[i]*to_unit( _old_pwm[_map_to_button[i]-1]);
            }
            _joy_pub.publish(joy_msg);
        }

        // std::cout<<"old    : ";  //DEBUG
        // for(int i = 0;i< _n_ch; i++){   
        //     std::cout << _old_pwm[i] <<" ";
        // }
        // std::cout<<endl;

        //rate.sleep();
    }

}

void Rc_To_Joy::run(){
    ros::Rate rate(1.0);
    ROS_INFO("Waiting first rc...");
    while(! _first_rc){
        ros::spinOnce();
        rate.sleep();
    }
    _old_pwm.resize(_n_ch);
    for(int i = 0; i<_n_ch;i++) _old_pwm[i] = 0;
    ROS_INFO("First rc arrived");

    boost::thread sp_pub_t( &Rc_To_Joy::data_publisher, this);
}

int main( int argc, char** argv ) {
	ros::init(argc, argv, "rc_to_joy");
	Rc_To_Joy rc;
	
    rc.run();
	ros::spin(); //ros::spin()
	//----This function will be never overcome

	return 0;
}

