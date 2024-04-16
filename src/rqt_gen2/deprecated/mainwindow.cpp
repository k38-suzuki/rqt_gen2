/**
   @author Kenta Suzuki
*/

#include "rqt_gen2/mainwindow.h"

#include <ros/ros.h>

#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#include <QAction>
#include <QBoxLayout>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QTimer>
#include <QToolBar>
#include <QVector>

#include "rqt_gen2/joystick.h"

namespace {

struct ButtonInfo {
    const QString label;
    int row;
    int column;
};

ButtonInfo buttonInfo[] = {
    {    "Y-", 0, 2 },
    { "X+/Y-", 1, 1 },
    { "X-/Y-", 1, 3 },
    {    "X+", 2, 0 },
    {    "Z+", 2, 1 },
    {    "Z-", 2, 3 },
    {    "X-", 2, 4 },
    { "X+/Y+", 3, 1 },
    { "X-/Y+", 3, 3 },
    {    "Y+", 4, 2 }
};

ButtonInfo buttonInfo2[] = {
    {     "TY-", 0, 2 },
    {  "TX/TY-", 1, 1 },
    { "TX-/TY-", 1, 3 },
    {      "TX", 2, 0 },
    {      "TZ", 2, 1 },
    {     "TZ-", 2, 3 },
    {     "TX-", 2, 4 },
    {  "TX/TY+", 3, 1 },
    {  "TX-/TY", 3, 3 },
    {      "TY", 4, 2 }
};

}

namespace rqt_gen2 {

class CartesianWidget : public QWidget
{
public:
    CartesianWidget(QWidget* parent = nullptr);
    ~CartesianWidget();

private:
    void on_timer_timeout();
    void on_pushButton_pressed(int arg1);
    void on_pushButton_released(int arg1);
    void on_pushButton_2_pressed(int arg1);
    void on_pushButton_2_released(int arg1);
    void on_slider_valueChanged(int arg1, int arg2);

    void createGridGroupBox1();
    void createGridGroupBox2();
    void createGridGroupBox3();

    enum {
        YM,
        XPYM, XMYM,
        XP, ZP, ZM, XM,
        XPYP, XMYP,
        YP, NumButtons
    };

    QGroupBox* gridGroupBox;
    QGroupBox* gridGroupBox2;
    QGroupBox* gridGroupBox3;
    QDoubleSpinBox* twistSpins[2];
    QTimer* timer;

    ros::NodeHandle n;
    ros::Publisher cartesian_pub;

    double twist_linear[3];
    double twist_angular[3];
};

class MainWindow::Impl
{
public:
    MainWindow* self;

    Impl(MainWindow* self);

    void home();
    void play();

    void readState();
    void buttonDown(const int& id);

    void on_pushButton_pressed(int arg1, int arg2);
    void on_pushButton_released(int arg1);
    void on_slider_valueChanged(int arg1, int arg2);
    void on_timer_timeout();

    void createActions();
    void createToolBars();
    void createPage1Widget();
    void createPage2Widget();
    void createPage3Widget();

    enum {
        NumPages = 3,
        NumJoints = 7,
        NumFingers = 3,
        FingerMaxTurn = 6800
    };

    enum { TranslationMode, WristMode, FingerMode };

    QAction* homeAct;
    QAction* playAct;

    QTimer* timer;
    QTabWidget* tabWidget;
    QWidget* pageWidgets[NumPages];
    QDoubleSpinBox* velocitySpins[NumJoints];
    QSpinBox* fingerSpins[NumFingers];
    QDoubleSpinBox* twistSpins[2];
    QVector<bool> prevButtons;

    ros::NodeHandle n;
    ros::ServiceClient home_client;
    ros::Publisher joint_pub;
    ros::Publisher cartesian_pub;

    rqt_gen2::Joystick* joystick;

    int control_mode;
    double twist_linear[3];
    double twist_angular[3];
    double joint_velocity[NumJoints];
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    impl = new Impl(this);
}

MainWindow::Impl::Impl(MainWindow* self)
    : self(self)
{
    QWidget* widget = new QWidget;
    self->setCentralWidget(widget);

    createActions();
    createToolBars();
    createPage1Widget();
    createPage2Widget();
    createPage3Widget();

    self->setWindowTitle("Gen2");

    home_client = n.serviceClient<kinova_msgs::HomeArm>("/j2s7s300_driver/in/home_arm");
    joint_pub = n.advertise<kinova_msgs::JointVelocity>("/j2s7s300_driver/in/joint_velocity", 2);
    cartesian_pub = n.advertise<kinova_msgs::PoseVelocity>(
        "/j2s7s300_driver/in/cartesian_velocity", 2);

    control_mode = TranslationMode;

    for(int i = 0; i < 3; ++i) {
        twist_linear[i] = 0.0;
        twist_angular[i] = 0.0;
    }

    joystick = new rqt_gen2::Joystick("/dev/input/js0");
    if(!joystick->ready()) {

    }
    prevButtons.resize(joystick->numButtons());

    timer = new QTimer(self);
    timer->start(1000.0 / 100.0);
    self->connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    tabWidget = new QTabWidget;
    const QStringList list = { "Joint speed", "Twist", "Finger" };
    for(int i = 0; i < NumPages; ++i) {
        tabWidget->addTab(pageWidgets[i], list.at(i));
    }

    auto layout = new QVBoxLayout;
    layout->addWidget(tabWidget);
    widget->setLayout(layout);
}

MainWindow::~MainWindow()
{
    delete impl;
}

void MainWindow::Impl::home()
{
    kinova_msgs::HomeArm srv;
    if(home_client.call(srv)) {
        ROS_INFO("Service call successed.");
    } else {
        ROS_INFO("Service call failed.");
    }
}

void MainWindow::Impl::play()
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
    goal.fingers.finger1 = fingerSpins[0]->value();
    goal.fingers.finger2 = fingerSpins[1]->value();
    goal.fingers.finger3 = fingerSpins[2]->value();
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

void MainWindow::Impl::readState()
{
    joystick->readState();

    const double translation_velocity = twistSpins[0]->value();
    const double orientation_velocity = twistSpins[1]->value();

    for(int i = 0; i < joystick->numAxes(); ++i) {
        double pos = joystick->axis(i);
        if(fabs(pos) < 0.15) {
            pos = 0.0;
        }
        if(control_mode == TranslationMode) {
            if(i == 0) {
                twist_linear[0] = translation_velocity * pos;
            } else if(i == 1) {
                twist_linear[1] = translation_velocity * pos;
            } else if(i == 6) {
                twist_linear[2] = translation_velocity * pos;
            }
        } else if(control_mode == WristMode) {
            if(i == 0) {
                twist_angular[0] = orientation_velocity * pos;
            } else if(i == 1) {
                twist_angular[1] = orientation_velocity * pos;
            } else if(i == 6) {
                twist_angular[2] = orientation_velocity * pos;
            }
        } else if(control_mode == FingerMode) {
            // finger action
        }
    }

    for(int i = 0; i < joystick->numButtons(); ++i) {
        bool button_state = joystick->button(i);
        if(button_state && !prevButtons[i]) {
            buttonDown(i);
        }
        prevButtons[i] = button_state;
    } 
}

void MainWindow::Impl::buttonDown(const int& id)
{
    if(id == rqt_gen2::Joystick::Button_A) {
        control_mode = FingerMode;
        ROS_INFO("Move to finger-mode.");
    } else  if(id == rqt_gen2::Joystick::Button_B) {
        if(control_mode == TranslationMode) {
            control_mode = WristMode;
            ROS_INFO("Move to wrist-mode.");
        } else {
            control_mode = TranslationMode;
            ROS_INFO("Move to translation-mode.");
        }
    } else if(id == rqt_gen2::Joystick::Button_Guide) {
        home();
    }

    if(id == rqt_gen2::Joystick::Button_A || id == rqt_gen2::Joystick::Button_B) {
        for(int i = 0; i < 3; ++i) {
            twist_linear[i] = 0.0;
            twist_angular[i] = 0.0;
        }
    }
}

void MainWindow::Impl::on_pushButton_pressed(int arg1, int arg2)
{
    double value = 1.0;
     if(arg2 == 0) {
         value *= 1.0;
     } else {
         value *= -1.0;
     }
    joint_velocity[arg1] = value * velocitySpins[arg1]->value();
}

void MainWindow::Impl::on_pushButton_released(int arg1)
{
    joint_velocity[arg1] = 0.0;
}

void MainWindow::Impl::on_timer_timeout()
{
    if(tabWidget->currentIndex() == 0) {
        kinova_msgs::JointVelocity joint_msg;
        joint_msg.joint1 = joint_velocity[0];
        joint_msg.joint2 = joint_velocity[1];
        joint_msg.joint3 = joint_velocity[2];
        joint_msg.joint4 = joint_velocity[3];
        joint_msg.joint5 = joint_velocity[4];
        joint_msg.joint6 = joint_velocity[5];
        joint_msg.joint7 = joint_velocity[6];
        joint_pub.publish(joint_msg);
    } else if(tabWidget->currentIndex() == 1) {
        readState();

        kinova_msgs::PoseVelocity twist_msg;
        twist_msg.twist_linear_x = twist_linear[0];
        twist_msg.twist_linear_y = twist_linear[1];
        twist_msg.twist_linear_z = twist_linear[2];
        twist_msg.twist_angular_x = twist_angular[0];
        twist_msg.twist_angular_y = twist_angular[1];
        twist_msg.twist_angular_z = twist_angular[2];
        cartesian_pub.publish(twist_msg);
    }
}

void MainWindow::Impl::on_slider_valueChanged(int arg1, int arg2)
{
    double rate[] = { 0.002, 0.0107 };
    twistSpins[arg1]->setValue(rate[arg1] * (double)arg2);
}

void MainWindow::Impl::createActions()
{
    const QIcon homeIcon = QIcon::fromTheme("go-home");
    homeAct = new QAction(homeIcon, "&Home", self);
    homeAct->setStatusTip("");
    self->connect(homeAct, &QAction::triggered, [&](){ home(); });

    const QIcon playIcon = QIcon::fromTheme("media-playback-start");
    playAct = new QAction(playIcon, "&Play", self);
    playAct->setStatusTip("Play the finger action");
    self->connect(playAct, &QAction::triggered, [&](){ play(); });
}

void MainWindow::Impl::createToolBars()
{
    QToolBar* gen2ToolBar = self->addToolBar("Gen2");
    gen2ToolBar->addAction(homeAct);
    gen2ToolBar->addAction(playAct);
    gen2ToolBar->setObjectName("Gen2ToolBar");
}

void MainWindow::Impl::createPage1Widget()
{
    QWidget* page = new QWidget;
    pageWidgets[0] = page;

    const QStringList list = {
        "joint 1 [deg/s]", "joint 2 [deg/s]", "joint 3 [deg/s]", "joint 4 [deg/s]", 
        "joint 5 [deg/s]", "joint 6 [deg/s]", "joint 7 [deg/s]"
    };

    auto gridLayout = new QGridLayout;
    for(int i = 0; i < NumJoints; ++i) {
        joint_velocity[i] = 0.0;

        QDoubleSpinBox* spin = new QDoubleSpinBox;
        velocitySpins[i] = spin;
        spin->setValue(30.0);
        spin->setRange(-90.0, 90.0);

        QPushButton* button = new QPushButton;
        button->setText("+");
        button->setFixedWidth(60);
        self->connect(button, &QPushButton::pressed, [=](){ on_pushButton_pressed(i, 0); });
        self->connect(button, &QPushButton::released, [=](){ on_pushButton_released(i); });

        QPushButton* button2 = new QPushButton;
        button2->setText("-");
        button2->setFixedWidth(60);
        self->connect(button2, &QPushButton::pressed, [=](){ on_pushButton_pressed(i, 1); });
        self->connect(button2, &QPushButton::released, [=](){ on_pushButton_released(i); });

        gridLayout->addWidget(new QLabel(list.at(i)), i, 0);
        gridLayout->addWidget(spin, i, 1);
        gridLayout->addWidget(button, i, 2);
        gridLayout->addWidget(button2, i, 3);
    }

    auto layout = new QVBoxLayout;
    layout->addLayout(gridLayout);
    layout->addStretch();
    page->setLayout(layout);
}

void MainWindow::Impl::createPage2Widget()
{
    QWidget* page = new QWidget;
    pageWidgets[1] = page;

    const QStringList list = { "linear [m/s]", "angular [rad/s]" };

    auto gridLayout = new QGridLayout;
    for(int i = 0; i < 2; ++i) {
        QSlider* slider = new QSlider(Qt::Horizontal);
        self->connect(slider, &QSlider::valueChanged,
            [=](int value){ on_slider_valueChanged(i, value); });

        QDoubleSpinBox* spin = new QDoubleSpinBox;
        twistSpins[i] = spin;
        spin->setEnabled(false);

        gridLayout->addWidget(new QLabel(list.at(i)), i, 0);
        gridLayout->addWidget(slider, i, 1);
        gridLayout->addWidget(spin, i, 2);
    }

    auto layout = new QVBoxLayout;
    layout->addLayout(gridLayout);
    layout->addStretch();
    page->setLayout(layout);
}

void MainWindow::Impl::createPage3Widget()
{
    QWidget* page = new QWidget;
    pageWidgets[2] = page;

    auto formLayout = new QFormLayout;
    for(int i = 0; i < NumFingers; ++i) {
        QSpinBox* spin = new QSpinBox;
        fingerSpins[i] = spin;
        spin->setRange(0, FingerMaxTurn);
        QString label = QString("finger %1").arg(i);
        formLayout->addRow(label, spin);
    }

    auto layout = new QVBoxLayout;
    layout->addLayout(formLayout);
    layout->addStretch();
    page->setLayout(layout);
}

CartesianWidget::CartesianWidget(QWidget* parent)
    : QWidget(parent)
{
    createGridGroupBox1();
    createGridGroupBox2();
    createGridGroupBox3();

    cartesian_pub = n.advertise<kinova_msgs::PoseVelocity>(
        "/j2s7s300_driver/in/cartesian_velocity", 2);

    timer = new QTimer(this);
    timer->start(1000.0 / 100.0);
    connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    for(int i = 0; i < 3; ++i) {
        twist_linear[i] = 0.0;
        twist_angular[i] = 0.0;
    }

    auto hbox = new QHBoxLayout;
    hbox->addWidget(gridGroupBox);
    hbox->addWidget(gridGroupBox2);

    auto layout = new QVBoxLayout;
    layout->addLayout(hbox);
    layout->addStretch();
    layout->addWidget(gridGroupBox3);
    setLayout(layout);
}

CartesianWidget::~CartesianWidget()
{

}

void CartesianWidget::on_timer_timeout()
{
    double twist_linear[3];
    double twist_angular[3];
    for(int i = 0; i < 3; ++i) {
        twist_linear[i] = twist_linear[i] * twistSpins[0]->value();
        twist_angular[i] = twist_angular[i] * twistSpins[1]->value();
    }

    kinova_msgs::PoseVelocity twist_msg;
    twist_msg.twist_linear_x = twist_linear[0];
    twist_msg.twist_linear_y = twist_linear[1];
    twist_msg.twist_linear_z = twist_linear[2];
    twist_msg.twist_angular_x = twist_angular[0];
    twist_msg.twist_angular_y = twist_angular[1];
    twist_msg.twist_angular_z = twist_angular[2];
    cartesian_pub.publish(twist_msg);
}

void CartesianWidget::on_pushButton_pressed(int arg1)
{
    double value = 1.0;
    if(arg1 == YM || arg1 == XPYM || arg1 == XMYM) {
        twist_linear[1] -= value;
    }
    if(arg1 == XPYM || arg1 == XP || arg1 == XPYP) {
        twist_linear[0] += value;
    }
    if(arg1 == ZP) {
        twist_linear[2] += value;
    }
    if(arg1 == ZM) {
        twist_linear[2] -= value;
    }
    if(arg1 == XMYM || arg1 == XM || arg1 == XMYP) {
        twist_linear[0] -= value;
    }
    if(arg1 == YP || arg1 == XPYP || arg1 == XMYP) {
        twist_linear[1] += value;
    }
}

void CartesianWidget::on_pushButton_released(int arg1)
{
    if(arg1 == YM || arg1 == XPYM || arg1 == XMYM) {
        twist_linear[1] = 0.0;
    }
    if(arg1 == XPYM || arg1 == XP || arg1 == XPYP) {
        twist_linear[0] = 0.0;
    }
    if(arg1 == ZP) {
        twist_linear[2] = 0.0;
    }
    if(arg1 == ZM) {
        twist_linear[2] = 0.0;
    }
    if(arg1 == XMYM || arg1 == XM || arg1 == XMYP) {
        twist_linear[0] = 0.0;
    }
    if(arg1 == YP || arg1 == XPYP || arg1 == XMYP) {
        twist_linear[1] = 0.0;
    }
}

void CartesianWidget::on_pushButton_2_pressed(int arg1)
{
    double value = 1.0;
    if(arg1 == YM || arg1 == XPYM || arg1 == XMYM) {
        twist_angular[1] -= value;
    }
    if(arg1 == XPYM || arg1 == XP || arg1 == XPYP) {
        twist_angular[0] += value;
    }
    if(arg1 == ZP) {
        twist_angular[2] += value;
    }
    if(arg1 == ZM) {
        twist_angular[2] -= value;
    }
    if(arg1 == XMYM || arg1 == XM || arg1 == XMYP) {
        twist_angular[0] -= value;
    }
    if(arg1 == YP || arg1 == XPYP || arg1 == XMYP) {
        twist_angular[1] += value;
    }
}

void CartesianWidget::on_pushButton_2_released(int arg1)
{
    if(arg1 == YM || arg1 == XPYM || arg1 == XMYM) {
        twist_angular[1] = 0.0;
    }
    if(arg1 == XPYM || arg1 == XP || arg1 == XPYP) {
        twist_angular[0] = 0.0;
    }
    if(arg1 == ZP) {
        twist_angular[2] = 0.0;
    }
    if(arg1 == ZM) {
        twist_angular[2] = 0.0;
    }
    if(arg1 == XMYM || arg1 == XM || arg1 == XMYP) {
        twist_angular[0] = 0.0;
    }
    if(arg1 == YP || arg1 == XPYP || arg1 == XMYP) {
        twist_angular[1] = 0.0;
    }
}

void CartesianWidget::on_slider_valueChanged(int arg1, int arg2)
{
    double rate[] = { 0.002, 0.0107 };
    twistSpins[arg1]->setValue(rate[arg1] * (double)arg2);
}

void CartesianWidget::createGridGroupBox1()
{
    gridGroupBox = new QGroupBox("Translation");

    auto gridLayout = new QGridLayout;
    for(int i = 0; i < NumButtons; ++i) {
        ButtonInfo& info = buttonInfo[i];
        QPushButton* button = new QPushButton;
        button->setText(info.label);
        button->setFixedWidth(60);
        connect(button, &QPushButton::pressed, [=](){ on_pushButton_pressed(i); });
        connect(button, &QPushButton::released, [=](){ on_pushButton_released(i); });
        gridLayout->addWidget(button, info.row, info.column);
    }

    auto layout = new QVBoxLayout;
    layout->addLayout(gridLayout);
    gridGroupBox->setLayout(layout);
}

void CartesianWidget::createGridGroupBox2()
{
    gridGroupBox2 = new QGroupBox("Orientation");

    auto gridLayout = new QGridLayout;
    for(int i = 0; i < NumButtons; ++i) {
        ButtonInfo& info = buttonInfo2[i];
        QPushButton* button = new QPushButton;
        button->setText(info.label);
        button->setFixedWidth(60);
        connect(button, &QPushButton::pressed, [=](){ on_pushButton_2_pressed(i); });
        connect(button, &QPushButton::released, [=](){ on_pushButton_2_released(i); });
        gridLayout->addWidget(button, info.row, info.column);
    }

    auto layout = new QVBoxLayout;
    layout->addLayout(gridLayout);
    gridGroupBox2->setLayout(layout);
}

void CartesianWidget::createGridGroupBox3()
{
    gridGroupBox3 = new QGroupBox("Twist");

    const QStringList list = { "linear", "angular" };
    const QStringList list2 = { "m/s", "rad/s" };

    auto layout = new QGridLayout;
    for(int i = 0; i < 2; ++i) {
        QSlider* slider = new QSlider(Qt::Horizontal);
        connect(slider, &QSlider::valueChanged,
            [=](int value){ on_slider_valueChanged(i, value); });

        QDoubleSpinBox* spin = new QDoubleSpinBox;
        twistSpins[i] = spin;
        spin->setEnabled(false);

        layout->addWidget(new QLabel(list.at(i)), i, 0);
        layout->addWidget(slider, i, 1);
        layout->addWidget(spin, i, 2);
        layout->addWidget(new QLabel(list2.at(i)), i, 3);
    }

    gridGroupBox3->setLayout(layout);
}

}
