#include    <ros/ros.h>
#include    <ros/console.h>
#include    <geometry_msgs/Twist.h>

#include    <ds4_driver/Feedback.h>
#include    <ds4_driver/Status.h>

#include    <string>

// Local variables
// Variables for ROS params
std::string          _sub_topic_prefix, _pub_topic_name;
double               _speed_linear_max, _speed_angular_max;
ds4_driver::Feedback _feedback;
geometry_msgs::Twist _twist;

ros::Publisher       _pub_feedback;


struct _ps4_controller_status
{
    uint    teleop_state_old = -1;
    bool    feedback_send_state = true;
    double  speed_scale;
} _ctrl_status;

enum STATE
{
    DISABLED = 0,
    NORMAL   = 1,
    TURBO    = 2
};


// Enable or disable the gamepad feedback
void enable_ctrlr_feedback(bool enable)
{
    _feedback.set_led    = enable;
    _feedback.set_rumble = enable;
}

// Gamepad feedback subroutine, to set LEDs and rumble motors in the feedback msg
void set_ctrlr_feedback(float led_r, float led_g, float led_b, float rumb_dur, float rumb_small, float rumb_big)
{
    _feedback.led_r             = led_r;
    _feedback.led_g             = led_g;
    _feedback.led_b             = led_b;
    _feedback.rumble_duration   = rumb_dur;
    _feedback.rumble_small      = rumb_small;
    _feedback.rumble_big        = rumb_big;
}


void calc_joy(uint teleop_state, float lin_x, float ang_z)
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
            set_ctrlr_feedback(1.0, 1.0, 0.0, 0.5, 0.0, 0.5);
            _ctrl_status.speed_scale = 0.5;
            break;

        case TURBO:
            set_ctrlr_feedback(1.0, 0.0, 0.0, 0.5, 0.0, 1.0);
            _ctrl_status.speed_scale = 1.0;   
            break;                
        
        default:
            _ctrl_status.speed_scale = 0.0;
            break;
        }
        _ctrl_status.feedback_send_state = true;
    }

    _ctrl_status.teleop_state_old = teleop_state;

    _twist.linear.x  = _speed_linear_max  * lin_x * _ctrl_status.speed_scale;
    _twist.angular.z = _speed_angular_max * ang_z * _ctrl_status.speed_scale;
}


void cb_ctrlr_status(const ds4_driver::Status::ConstPtr& _ctrlr)
{
    // Buttons and axis
    bool  _btn_enable = _ctrlr->button_cross;
    bool  _btn_turbo  = _ctrlr->button_square;
    float _axis_lin_x = _ctrlr->axis_r2 - _ctrlr->axis_l2;
    float _axis_ang_z = _ctrlr->axis_left_x;

    if(_btn_enable == true && _btn_turbo == false)
    {
        calc_joy(NORMAL, _axis_lin_x, _axis_ang_z);
    }
    else if(_btn_enable == true && _btn_turbo == true)
    {
        calc_joy(TURBO, _axis_lin_x, _axis_ang_z);
    }
    else
    {
        calc_joy(DISABLED, _axis_lin_x, _axis_ang_z);
    }
}


int main(int argc, char **argv)
{
    // Init ROS, NodeHandle and gamepad feedback msg
    ros::init(argc, argv, "joy_ps4");
    ros::NodeHandle _nh;

    // #### ROS ####

    // ROS param    
    _nh.param<std::string>("topic_ps4",          _sub_topic_prefix,  "");
    _nh.param<std::string>("topic_cmd_vel",      _pub_topic_name,    "");
    _nh.param<double>     ("speed_linear_max",   _speed_linear_max,  0.0);
    _nh.param<double>     ("speed_angular_max",  _speed_angular_max, 0.0);

    // Subcriber and callbacks
    void cb_ctrlr_status(const ds4_driver::Status::ConstPtr& _ctrlr);
    ros::Subscriber _sub_status   = _nh.subscribe<ds4_driver::Status>(_sub_topic_prefix + "status", 1, &cb_ctrlr_status);

    // Publisher
    ros::Publisher  _pub_vel      = _nh.advertise<geometry_msgs::Twist>(_pub_topic_name, 1);
    ros::Publisher  _pub_feedback = _nh.advertise<ds4_driver::Feedback>(_sub_topic_prefix + "set_feedback", 1);
    
    // #### Gamepad ####
    // Enable LED and rumble motors of the gamepad
    enable_ctrlr_feedback(true);

    // #### ROS LOOP ####
    // ROS loop rate for ps4 gamepad feedback puplishing. If it's to fast, the feedback will struggle.
    ros::Rate _loop_rate(20);

    while(ros::ok())
    {
        _pub_vel.publish(_twist);

        // Prevends the feedback publisher to send all the time
        if(_ctrl_status.feedback_send_state == true)
        {
            _pub_feedback.publish(_feedback);
        }
        _ctrl_status.feedback_send_state = false;

        ros::spinOnce();
        _loop_rate.sleep();
    }

    return 0;
}