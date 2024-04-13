/**
    @author Kenta Suzuki
*/

#include "rqt_gen2/my_widget.h"

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#include <QBoxLayout>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>

namespace rqt_gen2 {

class JointWidget : public QWidget
{
public:
    JointWidget(QWidget* parent = nullptr);

    enum { NumJoints = 7 };

private:
    void on_toolButton_pressed(int arg1, int arg2);
    void on_toolButton_released(int arg1);
    void on_toolButton_toggled(bool checked);
    void on_timer_timeout();

    QDoubleSpinBox* velSpins[NumJoints];

    ros::NodeHandle n;
    ros::Publisher joint_pub;

    double joint_vel[NumJoints];
};

class TwistWidget : public QWidget
{
public:
    TwistWidget(QWidget* parent = nullptr);

    enum { TranslationMode, WristMode, FingerMode };

private:
    void joyCallback(const sensor_msgs::Joy& msg);

    void on_slider_valueChanged(int arg1, int arg2);
    void on_toolButton_toggled(bool checked);
    void on_timer_timeout();

    QDoubleSpinBox* twistSpins[2];

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber joy_sub;
    sensor_msgs::Joy latestJoyState;

    int control_mode;
    double twist_linear[3];
    double twist_angular[3];
    bool prevButtonState;
};

class HomeWidget : public QWidget
{
public:
    HomeWidget(QWidget* parent = nullptr);

private:
    void on_toolButton_clicked();

    ros::NodeHandle n;
    ros::ServiceClient client;
};

class FingerWidget : public QWidget
{
public:
    FingerWidget(QWidget* parent = nullptr);

    enum { NumFingers = 3, FingerMaxTurn = 6800 };

private:
    void on_toolButton_clicked();

    QSpinBox* fingerSpins[NumFingers];
};

class MyWidget::Impl
{
public:
    MyWidget* self;

    Impl(MyWidget* self);
};

MyWidget::MyWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}

MyWidget::Impl::Impl(MyWidget* self)
    : self(self)
{
    self->setWindowTitle("Development center");

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(new HomeWidget);
    layout2->addStretch();

    auto tabWidget = new QTabWidget;
    tabWidget->addTab(new JointWidget, "Joint speed");
    tabWidget->addTab(new TwistWidget, "Twist");
    tabWidget->addTab(new FingerWidget, "Finger");

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addWidget(tabWidget);
    layout->addStretch();
    self->setLayout(layout);
}

MyWidget::~MyWidget()
{
    delete impl;
}

JointWidget::JointWidget(QWidget* parent)
    : QWidget(parent)
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

    auto button = new QToolButton;
    button->setIcon(QIcon::fromTheme("network-wireless"));
    button->setCheckable(true);
    connect(button, &QToolButton::toggled,
        [&](bool checked){ on_toolButton_toggled(checked); });

    auto timer = new QTimer(this);
    timer->connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });
    timer->start(1000 / 100);

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(button);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(gridLayout);
    layout->addStretch();
    setLayout(layout);
}

void JointWidget::on_toolButton_pressed(int arg1, int arg2)
{
    double value = 1.0;
     if(arg2 == 0) {
         value *= 1.0;
     } else {
         value *= -1.0;
     }
    joint_vel[arg1] = value * velSpins[arg1]->value();
}

void JointWidget::on_toolButton_released(int arg1)
{
    joint_vel[arg1] = 0.0;
}

void JointWidget::on_toolButton_toggled(bool checked)
{
    if(checked) {
        joint_pub = n.advertise<kinova_msgs::JointVelocity>("/j2s7s300_driver/in/joint_velocity", 2);
    } else {
        joint_pub.shutdown();
    }
}

void JointWidget::on_timer_timeout()
{
    if(joint_pub) {
        kinova_msgs::JointVelocity joint_msg;
        joint_msg.joint1 = joint_vel[0];
        joint_msg.joint2 = joint_vel[1];
        joint_msg.joint3 = joint_vel[2];
        joint_msg.joint4 = joint_vel[3];
        joint_msg.joint5 = joint_vel[4];
        joint_msg.joint6 = joint_vel[5];
        joint_msg.joint7 = joint_vel[6];
        joint_pub.publish(joint_msg);
    }
}

TwistWidget::TwistWidget(QWidget* parent)
    : QWidget(parent)
    , control_mode(TranslationMode)
    , prevButtonState(false)
{
    joy_sub = n.subscribe("joy", 1, &TwistWidget::joyCallback, this);

    const QStringList list2 = { "linear [m/s]", "angular [rad/s]" };

    auto gridLayout2 = new QGridLayout;
    for(int i = 0; i < 2; ++i) {
        auto slider = new QSlider(Qt::Horizontal);
        connect(slider, &QSlider::valueChanged,
            [=](int value){ on_slider_valueChanged(i, value); });

        twistSpins[i] = new QDoubleSpinBox;
        twistSpins[i]->setEnabled(false);

        gridLayout2->addWidget(new QLabel(list2.at(i)), i, 0);
        gridLayout2->addWidget(slider, i, 1);
        gridLayout2->addWidget(twistSpins[i], i, 2);
    }

    auto button = new QToolButton;
    button->setIcon(QIcon::fromTheme("network-wireless"));
    button->setCheckable(true);
    connect(button, &QToolButton::toggled,
        [&](bool checked){ on_toolButton_toggled(checked); });

    auto timer = new QTimer(this);
    timer->connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });
    timer->start(1000 / 100);

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(button);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(gridLayout2);
    layout->addStretch();
    setLayout(layout);
}

void TwistWidget::joyCallback(const sensor_msgs::Joy& msg)
{
    latestJoyState = msg;
}

void TwistWidget::on_slider_valueChanged(int arg1, int arg2)
{
    double rate[] = { 0.002, 0.0107 };
    twistSpins[arg1]->setValue(rate[arg1] * (double)arg2);
}

void TwistWidget::on_toolButton_toggled(bool checked)
{
    if(checked) {
        twist_pub = n.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 2);
    } else {
        twist_pub.shutdown();
    }
}

void TwistWidget::on_timer_timeout()
{
    sensor_msgs::Joy joy = latestJoyState;
    twist_linear[0] = twist_linear[1] = twist_linear[2] = 0.0;
    twist_angular[0] = twist_angular[1] = twist_angular[2] = 0.0;
    if(joy.axes.size() && joy.buttons.size()) {
        bool currentState = joy.buttons[1];
        if(currentState && !prevButtonState) {
            control_mode = control_mode == TranslationMode ? WristMode : TranslationMode;
            ROS_INFO("Move to %s-mode.", control_mode == TranslationMode ? "translation" : "wrist");
        }
        prevButtonState = currentState;

        const double linear_vel = twistSpins[0]->value();
        const double angular_vel = twistSpins[1]->value();
        twist_linear[0] = linear_vel * joy.axes[0];
        twist_linear[1] = linear_vel * joy.axes[1] * -1.0;
        twist_linear[2] = linear_vel * joy.axes[6];
        twist_angular[0] = angular_vel * joy.axes[0];
        twist_angular[1] = angular_vel * joy.axes[1] * -1.0;
        twist_angular[2] = angular_vel * joy.axes[6];
    }

    if(twist_pub) {
        kinova_msgs::PoseVelocity twist_msg;
        if(control_mode == TranslationMode) {
            twist_msg.twist_linear_x = twist_linear[0];
            twist_msg.twist_linear_y = twist_linear[1];
            twist_msg.twist_linear_z = twist_linear[2];
            twist_msg.twist_angular_x = 0.0;
            twist_msg.twist_angular_y = 0.0;
            twist_msg.twist_angular_z = 0.0;
        } else if(control_mode == WristMode) {
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

HomeWidget::HomeWidget(QWidget* parent)
    : QWidget(parent)
{
    client = n.serviceClient<kinova_msgs::HomeArm>("/j2s7s300_driver/in/home_arm");

    auto button = new QToolButton;
    button->setIcon(QIcon::fromTheme("go-home"));
    connect(button, &QToolButton::clicked, [&](){ on_toolButton_clicked(); });

    auto layout = new QVBoxLayout;
    layout->addWidget(button);
    layout->addStretch();
    setLayout(layout);
}

void HomeWidget::on_toolButton_clicked()
{
    kinova_msgs::HomeArm srv;
    if(client.call(srv)) {
        ROS_INFO("Service call successed.");
    } else {
        ROS_INFO("Service call failed.");
    }
}

FingerWidget::FingerWidget(QWidget* parent)
    : QWidget(parent)
{
    auto formLayout = new QFormLayout;
    for(int i = 0; i < NumFingers; ++i) {
        fingerSpins[i] = new QSpinBox;
        fingerSpins[i]->setRange(0, 100);
        fingerSpins[i]->setSingleStep(10);
        formLayout->addRow(QString("finger %1 [%]").arg(i), fingerSpins[i]);
    }

    auto button = new QToolButton;
    button->setIcon(QIcon::fromTheme("media-playback-start"));
    connect(button, &QToolButton::clicked, [&](){ on_toolButton_clicked(); });

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(button);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(formLayout);
    layout->addStretch();
    setLayout(layout);
}

void FingerWidget::on_toolButton_clicked()
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
    goal.fingers.finger1 = fingerSpins[0]->value() * FingerMaxTurn;
    goal.fingers.finger2 = fingerSpins[1]->value() * FingerMaxTurn;
    goal.fingers.finger3 = fingerSpins[2]->value() * FingerMaxTurn;
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

}
