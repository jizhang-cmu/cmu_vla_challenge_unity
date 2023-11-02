#include "drive_widget.h"
#include "teleop_panel.h"

namespace teleop_rviz_plugin
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , mouse_pressed_( false )
  , mouse_pressed_sent_( false )
{
  QVBoxLayout* layout = new QVBoxLayout;
  push_button_1_ = new QPushButton( "Resume Navigation to Goal", this );
  layout->addWidget( push_button_1_ );
  drive_widget_ = new DriveWidget;
  layout->addWidget( drive_widget_ );
  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  connect( push_button_1_, SIGNAL( pressed() ), this, SLOT( pressButton1() ));
  connect( drive_widget_, SIGNAL( outputVelocity( float, float, bool )), this, SLOT( setVel( float, float, bool )));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  output_timer->start( 100 );

  velocity_publisher_ = nh_.advertise<sensor_msgs::Joy>( "/joy", 5 );
  drive_widget_->setEnabled( true );
}

void TeleopPanel::pressButton1()
{
  if ( ros::ok() && velocity_publisher_ )
  {
    sensor_msgs::Joy joy;

    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(-1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(0);

    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(1);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);

    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_.publish( joy );
  }
}

void TeleopPanel::setVel( float lin, float ang, bool pre )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
  mouse_pressed_ = pre;
}

void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ && ( mouse_pressed_ || mouse_pressed_sent_ ))
  {
    sensor_msgs::Joy joy;

    joy.axes.push_back( 0 );
    joy.axes.push_back( 0 );
    joy.axes.push_back( 1.0 );
    joy.axes.push_back( angular_velocity_ );
    joy.axes.push_back( linear_velocity_ );
    joy.axes.push_back( 1.0 );
    joy.axes.push_back( 0 );
    joy.axes.push_back( 0 );

    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 1 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );

    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_.publish( joy );

    mouse_pressed_sent_ = mouse_pressed_;
  }
}

} 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( teleop_rviz_plugin::TeleopPanel,rviz::Panel )
