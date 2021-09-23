#include    <ros/ros.h>
#include    <ros/console.h>
#include    <geometry_msgs/Twist.h>

#include    <ds4_driver/Feedback.h>
#include    <ds4_driver/Status.h>

#include    <string>


class Teleop_PS4
{
public:
    Teleop_PS4();
    void calc_teleop(uint teleop_state, float lin_x, float ang_z);
    void set_ctrlr_feedback(float led_r, float led_g, float led_b, float rumb_dur, float rumb_small, float rumb_big);
    
private:
    // Puplish loop time to prevent timing lags on ps4 controller
    ros::Time   _time_now, _time_old;
    double      _time_pub_loop = 0.03;

    std::string _sub_topic_prefix, _pub_topic_name;
    double      _speed_linear_max, _speed_angular_max;

    ds4_driver::Feedback _feedback;

    ros::NodeHandle _nh;

    void cb_ctrlr_status(const ds4_driver::Status::ConstPtr& _ctrlr);
    ros::Subscriber _sub_status;

    ros::Publisher  _pub_feedback;
    ros::Publisher  _pub_vel;

    struct _ps4_controller_status
    {
        uint    teleop_state_old = -1;
        double  speed_scale;
    } _ctrl_status;
    
    enum STATE
    {
        DISABLED = 0,
        NORMAL = 1,
        TURBO = 2
    };
};


Teleop_PS4::Teleop_PS4()
{
    // Parameters
    _nh.param<std::string>("topic_ps4",     _sub_topic_prefix,  "");
    _nh.param<std::string>("topic_cmd_vel", _pub_topic_name,    "");
    _nh.param<double>("speed_linear_max",   _speed_linear_max,  0.0);
    _nh.param<double>("speed_angular_max",  _speed_angular_max, 0.0);

    // ROS Stuff
    _sub_status     = _nh.subscribe<ds4_driver::Status>(_sub_topic_prefix + "status", 1, &Teleop_PS4::cb_ctrlr_status, this);
    _pub_feedback   = _nh.advertise<ds4_driver::Feedback>(_sub_topic_prefix + "set_feedback", 1);
    _pub_vel        = _nh.advertise<geometry_msgs::Twist>(_pub_topic_name, 1);

    // Endable LED and rumble motors for controller feedback
    _feedback.set_led       = true;
    _feedback.set_rumble    = true;
}


void Teleop_PS4::calc_teleop(uint teleop_state, float lin_x, float ang_z)
{
    if(_ctrl_status.teleop_state_old != teleop_state)
    {
        switch(teleop_state)
        {
        case DISABLED:
            set_ctrlr_feedback(0.0, 0.25, 0.0, 0.0, 0.0, 0.0);
            _ctrl_status.speed_scale = 0.0;
            break;

        case NORMAL:
            set_ctrlr_feedback(1.0, 1.0, 0.0, 0.2, 0.0, 0.5);
            _ctrl_status.speed_scale = 0.5;
            break;

        case TURBO:
            set_ctrlr_feedback(1.0, 0.0, 0.0, 0.2, 0.0, 1.0);
            _ctrl_status.speed_scale = 1.0;   
            break;                
        
        default:
            _ctrl_status.speed_scale = 0.0;
            break;
        }

        _pub_feedback.publish(_feedback);
    }

    _ctrl_status.teleop_state_old = teleop_state;

    geometry_msgs::Twist _twist;

    _twist.linear.x  = _speed_linear_max  * lin_x * _ctrl_status.speed_scale;
    _twist.angular.z = _speed_angular_max * ang_z * _ctrl_status.speed_scale;

    _pub_vel.publish(_twist);
}


void Teleop_PS4::cb_ctrlr_status(const ds4_driver::Status::ConstPtr& _ctrlr)
{
    _time_now = ros::Time::now();

    if((_time_now - _time_old).toSec() <= _time_pub_loop)
        return;
    else
    {
        // Buttons for enable/turbo mode
        bool _btn_enable = _ctrlr->button_r1;
        bool _btn_turbo  = _ctrlr->button_l1;

        float _axis_lin_x = _ctrlr->axis_right_y;
        float _axis_ang_z = _ctrlr->axis_left_x;

        if(_btn_enable == true && _btn_turbo == false)
        {
            calc_teleop(NORMAL, _axis_lin_x, _axis_ang_z);
        }
        else if(_btn_enable == true && _btn_turbo == true)
        {
            calc_teleop(TURBO, _axis_lin_x, _axis_ang_z);
        }
        else
        {
            calc_teleop(DISABLED, _axis_lin_x, _axis_ang_z);
        }

        _time_old = ros::Time::now();
    }
}


void Teleop_PS4::set_ctrlr_feedback(float led_r, float led_g, float led_b, float rumb_dur, float rumb_small, float rumb_big)
{
    _feedback.led_r             = led_r;
    _feedback.led_g             = led_g;
    _feedback.led_b             = led_b;
    _feedback.rumble_duration   = rumb_dur;
    _feedback.rumble_small      = rumb_small;
    _feedback.rumble_big        = rumb_big;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_ps4");

    Teleop_PS4 teleop_ps4;

    ros::spin();    
}