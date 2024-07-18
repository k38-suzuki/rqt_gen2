/**
    @author Kenta Suzuki
*/

#include "rqt_gen2/my_widget.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#include <QBoxLayout>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTimer>
#include <QToolButton>

namespace rqt_gen2 {

class MyWidget::Impl
{
public:
    MyWidget* self;

    Impl(MyWidget* self);

    enum ControlMap { Joint, Twist };
    enum { NumJoints = 7 };
    enum { NumFingers = 3, FingerMaxTurn = 6800 };

    void joyCallback(const sensor_msgs::Joy& msg);

    void home();
    void grip();

    void on_mapCombo_currentIndexChanged(int index);
    void on_publishButton_toggled(bool checked);
    void on_toolButton_pressed(int arg1, int arg2);
    void on_toolButton_released(int arg1);
    void on_timer_timeout();

    ros::NodeHandle n;
    ros::Publisher joint_pub;
    ros::Publisher twist_pub;
    ros::Subscriber joy_sub;
    ros::ServiceClient client;

    sensor_msgs::Joy latestJoyState;

    QComboBox* mapCombo;
    QToolButton* publishButton;
    QToolButton* homeButton;
    QToolButton* gripButton;
    QStackedWidget* stackedWidget;

    QDoubleSpinBox* velSpins[NumJoints];
    QDoubleSpinBox* twistSpins[2];
    QSpinBox* fingerSpins[NumFingers];
    QTimer* timer;

    int control_mode;
    double joint_vel[NumJoints];
    double twist_linear[3];
    double twist_angular[3];
    bool prev_button_state;

    ControlMap currentMap;
};

MyWidget::MyWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}

MyWidget::Impl::Impl(MyWidget* self)
    : self(self)
    , control_mode(0)
    , prev_button_state(false)
    , currentMap(Joint)
{
    self->setWindowTitle("Development center");

    // home action
    client = n.serviceClient<kinova_msgs::HomeArm>("/j2s7s300_driver/in/home_arm");

    mapCombo = new QComboBox;
    mapCombo->addItems(QStringList() << "Joint" << "Twist");
    connect(mapCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [&](int index){ on_mapCombo_currentIndexChanged(index); });
    
    publishButton = new QToolButton;
    publishButton->setText("Publish");
    publishButton->setCheckable(true);
    connect(publishButton, &QToolButton::toggled, [&](bool checked){ on_publishButton_toggled(checked); });

    homeButton = new QToolButton;
    homeButton->setText("Home");
    homeButton->setEnabled(false);
    connect(homeButton, &QToolButton::clicked, [&](){ home(); });

    gripButton = new QToolButton;
    gripButton->setText("Grip");
    gripButton->setEnabled(false);
    connect(gripButton, &QToolButton::clicked, [&](){ grip(); });

    QWidget* jointWidget = new QWidget;
    {
        const QStringList list = {
            "joint 1 [deg/s]", "joint 2 [deg/s]", "joint 3 [deg/s]", "joint 4 [deg/s]", 
            "joint 5 [deg/s]", "joint 6 [deg/s]", "joint 7 [deg/s]"
        };

        auto gridLayout = new QGridLayout;
        for(int i = 0; i < NumJoints; ++i) {
            joint_vel[i] = 0.0;

            auto spin = new QDoubleSpinBox;
            velSpins[i] = spin;
            spin->setValue(30.0);
            spin->setRange(-90.0, 90.0);

            auto button1 = new QToolButton;
            button1->setText("+");
            button1->setFixedWidth(60);
            connect(button1, &QToolButton::pressed, [=](){ on_toolButton_pressed(i, 0); });
            connect(button1, &QToolButton::released, [=](){ on_toolButton_released(i); });

            auto button2 = new QToolButton;
            button2->setText("-");
            button2->setFixedWidth(60);
            connect(button2, &QToolButton::pressed, [=](){ on_toolButton_pressed(i, 1); });
            connect(button2, &QToolButton::released, [=](){ on_toolButton_released(i); });

            gridLayout->addWidget(new QLabel(list.at(i)), i, 0);
            gridLayout->addWidget(spin, i, 1);
            gridLayout->addWidget(button1, i, 2);
            gridLayout->addWidget(button2, i, 3);
        }

        auto layout = new QVBoxLayout;
        layout->addLayout(gridLayout);
        layout->addStretch();
        jointWidget->setLayout(layout);
    }

    QWidget* twistWidget = new QWidget;
    {
        const QStringList list = { "linear [m/s]", "angular [rad/s]" };
        const double upper[] = { 0.20, 1.07 };

        auto formLayout = new QFormLayout;
        for(int i = 0; i < 2; ++i) {
            twistSpins[i] = new QDoubleSpinBox;
            twistSpins[i]->setRange(0.0, upper[i]);
            twistSpins[i]->setValue(upper[i]);
            twistSpins[i]->setSingleStep(0.01);
            formLayout->addRow(list.at(i), twistSpins[i]);
        }

        auto layout = new QVBoxLayout;
        layout->addLayout(formLayout);
        layout->addStretch();
        twistWidget->setLayout(layout);
    }

    QWidget* fingerWidget = new QWidget;
    {
        auto formLayout = new QFormLayout;
        for(int i = 0; i < NumFingers; ++i) {
            fingerSpins[i] = new QSpinBox;
            fingerSpins[i]->setRange(0, 100);
            fingerSpins[i]->setSingleStep(10);
            formLayout->addRow(QString("finger %1 [%]").arg(i), fingerSpins[i]);
        }

        auto layout = new QVBoxLayout;
        layout->addLayout(formLayout);
        layout->addStretch();
        fingerWidget->setLayout(layout);
    }

    stackedWidget = new QStackedWidget;
    stackedWidget->addWidget(jointWidget);
    stackedWidget->addWidget(twistWidget);

    timer = new QTimer(self);
    timer->connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(new QLabel("Control map"));
    layout2->addWidget(mapCombo);
    layout2->addWidget(publishButton);
    layout2->addWidget(homeButton);
    layout2->addWidget(gripButton);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addWidget(stackedWidget);
    layout->addWidget(fingerWidget);
    layout->addStretch();
    self->setLayout(layout);
}

MyWidget::~MyWidget()
{
    delete impl;
}

void MyWidget::Impl::joyCallback(const sensor_msgs::Joy& msg)
{
    latestJoyState = msg;
}

void MyWidget::Impl::home()
{
    kinova_msgs::HomeArm srv;
    if(client.call(srv)) {
        ROS_INFO("Service call successed.");
    } else {
        ROS_INFO("Service call failed.");
    }
}

void MyWidget::Impl::grip()
{
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/j2s7s300_driver/fingers_action/finger_positions", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = ((double)fingerSpins[0]->value() / 100.0) * FingerMaxTurn;
    goal.fingers.finger2 = ((double)fingerSpins[1]->value() / 100.0) * FingerMaxTurn;
    goal.fingers.finger3 = ((double)fingerSpins[2]->value() / 100.0) * FingerMaxTurn;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }
    // exit
}

void MyWidget::Impl::on_mapCombo_currentIndexChanged(int index)
{
    stackedWidget->setCurrentIndex(index);

    switch(index) {
        case 0:
            currentMap = Joint;
            break;
        case 1:
            currentMap = Twist;
            break;
        default:
            break;
    }
}

void MyWidget::Impl::on_publishButton_toggled(bool checked)
{
    homeButton->setEnabled(checked);
    gripButton->setEnabled(checked);

    if(checked) {
        if(currentMap == Joint) {
            joint_pub = n.advertise<kinova_msgs::JointVelocity>("/j2s7s300_driver/in/joint_velocity", 2);
        } else if(currentMap == Twist) {
            twist_pub = n.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 2);
            joy_sub = n.subscribe("joy", 1, &MyWidget::Impl::joyCallback, this);
        }
        timer->start(1000 / 100);
    } else {
        timer->stop();
        if(currentMap == Joint) {
            joint_pub.shutdown();
        } else if(currentMap == Twist) {
            twist_pub.shutdown();
            joy_sub.shutdown();            
        }
    }
}

void MyWidget::Impl::on_toolButton_pressed(int arg1, int arg2)
{
    double value = arg2 == 0 ? 1.0 : -1.0;
    joint_vel[arg1] = value * velSpins[arg1]->value();
}

void MyWidget::Impl::on_toolButton_released(int arg1)
{
    joint_vel[arg1] = 0.0;
}

void MyWidget::Impl::on_timer_timeout()
{
    if(currentMap == Joint) {
        kinova_msgs::JointVelocity joint_msg;
        joint_msg.joint1 = joint_vel[0];
        joint_msg.joint2 = joint_vel[1];
        joint_msg.joint3 = joint_vel[2];
        joint_msg.joint4 = joint_vel[3];
        joint_msg.joint5 = joint_vel[4];
        joint_msg.joint6 = joint_vel[5];
        joint_msg.joint7 = joint_vel[6];
        joint_pub.publish(joint_msg);
    } else if(currentMap == Twist) {
        sensor_msgs::Joy joy = latestJoyState;
        twist_linear[0] = twist_linear[1] = twist_linear[2] = 0.0;
        twist_angular[0] = twist_angular[1] = twist_angular[2] = 0.0;
        if(joy.axes.size() && joy.buttons.size()) {
            bool current_state = joy.buttons[1];
            if(current_state && !prev_button_state) {
                control_mode = control_mode == 0 ? 1 : 0;
                ROS_INFO("Move to %s-mode.", control_mode == 0 ? "translation" : "wrist");
            }
            prev_button_state = current_state;

            const double linear_vel = twistSpins[0]->value();
            const double angular_vel = twistSpins[1]->value();
            twist_linear[0] = linear_vel * joy.axes[0];
            twist_linear[1] = linear_vel * joy.axes[1] * -1.0;
            twist_linear[2] = linear_vel * joy.axes[6];
            twist_angular[0] = angular_vel * joy.axes[0];
            twist_angular[1] = angular_vel * joy.axes[1] * -1.0;
            twist_angular[2] = angular_vel * joy.axes[6];
        }

        kinova_msgs::PoseVelocity twist_msg;
        if(control_mode == 0) {
            twist_msg.twist_linear_x = twist_linear[0];
            twist_msg.twist_linear_y = twist_linear[1];
            twist_msg.twist_linear_z = twist_linear[2];
            twist_msg.twist_angular_x = 0.0;
            twist_msg.twist_angular_y = 0.0;
            twist_msg.twist_angular_z = 0.0;
        } else if(control_mode == 1) {
            twist_msg.twist_linear_x = 0.0;
            twist_msg.twist_linear_y = 0.0;
            twist_msg.twist_linear_z = 0.0;
            twist_msg.twist_angular_x = twist_angular[0];
            twist_msg.twist_angular_y = twist_angular[1];
            twist_msg.twist_angular_z = twist_angular[2];
        }
        twist_pub.publish(twist_msg);
    }
}

}
