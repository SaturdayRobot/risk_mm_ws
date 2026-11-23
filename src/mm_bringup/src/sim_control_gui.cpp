#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tinyxml2.h>
#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QGridLayout>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <array>
#include <chrono>
#include <map>
#include <iostream>
#include <QCloseEvent>

class ControlPanel final : public QWidget {
  using JointState=sensor_msgs::msg::JointState;
  using Twist=geometry_msgs::msg::TwistStamped;
  using Clock=std::chrono::steady_clock;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<JointState>::SharedPtr arm_pub_;
  rclcpp::Publisher<Twist>::SharedPtr base_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::array<QDoubleSpinBox*,6> boxes_{};
  std::array<QLabel*,6> measured_{};
  std::array<double,6> values_{};
  std::array<double,3> velocity_{};
  QLabel *status_,*image_;
  QCheckBox *enabled_;
  QTimer timer_;
  bool initialized_=false,ready_=false;
  Clock::time_point feedback_=Clock::time_point::min(),status_time_=Clock::time_point::min();
  void send_base() {
    Twist message;message.header.stamp=node_->now();message.header.frame_id="base_link";
    message.twist.linear.x=velocity_[0];message.twist.linear.y=velocity_[1];message.twist.angular.z=velocity_[2];base_pub_->publish(message);
  }
  void stop() {velocity_.fill(0.);enabled_->setChecked(false);if(rclcpp::ok())send_base();}
  void sync() {enabled_->setChecked(false);for(size_t i=0;i<6;++i)boxes_[i]->setValue(values_[i]);}
  void tick() {
    if(!rclcpp::ok()){close();return;}
    executor_.spin_some(std::chrono::milliseconds(8));
    const auto now=Clock::now();
    if(!initialized_ || !ready_ || now-feedback_>std::chrono::seconds(1) || now-status_time_>std::chrono::seconds(1)) {
      velocity_.fill(0.);enabled_->setChecked(false);status_->setText("HOLD | waiting for fresh simulator feedback");
    }
    if(velocity_[0]!=0. || velocity_[1]!=0. || velocity_[2]!=0.)send_base();
    if(enabled_->isChecked()) {
      JointState message;message.header.stamp=node_->now();
      for(size_t i=0;i<6;++i){message.name.push_back("xarm6_joint"+std::to_string(i+1));message.position.push_back(boxes_[i]->value());}
      arm_pub_->publish(message);
    }
  }
protected:
  bool event(QEvent *event) override {
    if(event->type()==QEvent::WindowDeactivate && enabled_)stop();
    return QWidget::event(event);
  }
  void closeEvent(QCloseEvent *event) override {stop();timer_.stop();QWidget::closeEvent(event);}
public:
  explicit ControlPanel(rclcpp::Node::SharedPtr node):node_(std::move(node)),enabled_(nullptr) {
    executor_.add_node(node_);
    setWindowTitle("Rangermini + xArm6 | C++ Gazebo control");
    arm_pub_=node_->create_publisher<JointState>("/simulation/arm_command",10);
    base_pub_=node_->create_publisher<Twist>("/simulation/cmd_vel",10);
    auto *root=new QVBoxLayout(this);status_=new QLabel("Waiting for measured Gazebo state...");root->addWidget(status_);
    root->addWidget(new QLabel("Hold a base button to move; release to stop. Camera front = -base X."));
    auto *pad=new QGridLayout;
    auto button=[&](const char *label,int row,int col,std::array<double,3> command) {
      auto *b=new QPushButton(label);pad->addWidget(b,row,col);
      connect(b,&QPushButton::pressed,this,[this,command]{velocity_=command;});
      connect(b,&QPushButton::released,this,[this]{velocity_.fill(0.);send_base();});
    };
    button("Forward",0,1,{-.12,0.,0.});button("Back",2,1,{.12,0.,0.});
    button("Left",1,0,{0.,-.12,0.});button("Right",1,2,{0.,.12,0.});
    button("Rotate left",0,0,{0.,0.,.2});button("Rotate right",0,2,{0.,0.,-.2});
    auto *stop_button=new QPushButton("STOP / HOLD");pad->addWidget(stop_button,1,1);connect(stop_button,&QPushButton::clicked,this,[this]{stop();});root->addLayout(pad);
    auto *grid=new QGridLayout;grid->addWidget(new QLabel("Joint"),0,0);grid->addWidget(new QLabel("Target (rad)"),0,1);grid->addWidget(new QLabel("Measured (rad)"),0,2);
    tinyxml2::XMLDocument model;
    const auto file=ament_index_cpp::get_package_share_directory("rangermini_description")+"/urdf/rangermini_xarm6.urdf";
    if(model.LoadFile(file.c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("Missing robot URDF");
    for(size_t i=0;i<6;++i) {
      const std::string name="xarm6_joint"+std::to_string(i+1);boxes_[i]=new QDoubleSpinBox;boxes_[i]->setDecimals(4);boxes_[i]->setSingleStep(.02);
      for(auto *j=model.RootElement()->FirstChildElement("joint");j;j=j->NextSiblingElement("joint"))if(name==j->Attribute("name")) {
        auto *limit=j->FirstChildElement("limit");boxes_[i]->setRange(limit->DoubleAttribute("lower"),limit->DoubleAttribute("upper"));
      }
      measured_[i]=new QLabel("--");grid->addWidget(new QLabel(QString::fromStdString(name)),i+1,0);grid->addWidget(boxes_[i],i+1,1);grid->addWidget(measured_[i],i+1,2);
    }root->addLayout(grid);
    enabled_=new QCheckBox("Send arm targets (maximum 0.3 rad/s)");root->addWidget(enabled_);
    auto *copy=new QPushButton("Copy measured joints to targets");connect(copy,&QPushButton::clicked,this,[this]{if(initialized_)sync();});root->addWidget(copy);
    image_=new QLabel("Waiting for head camera image...");image_->setAlignment(Qt::AlignCenter);image_->setMinimumSize(400,280);root->addWidget(image_);
    root->addWidget(new QLabel("Manual simulator test; this panel does not validate planned paths."));
    subscriptions_.push_back(node_->create_subscription<JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](JointState::SharedPtr m){
      std::map<std::string,double> values;for(size_t i=0;i<std::min(m->name.size(),m->position.size());++i)values[m->name[i]]=m->position[i];
      for(size_t i=0;i<6;++i)if(!values.count("xarm6_joint"+std::to_string(i+1)))return;
      for(size_t i=0;i<6;++i){values_[i]=values.at("xarm6_joint"+std::to_string(i+1));measured_[i]->setText(QString::number(values_[i],'f',4));}
      feedback_=Clock::now();if(!initialized_){sync();initialized_=true;}
    }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>("/simulation/control_status",10,[this](std_msgs::msg::String::SharedPtr m){
      auto status=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();ready_=status["ready"].toBool();status_time_=Clock::now();
      status_->setText((ready_?"READY | ":"HOLD | ")+status["reason"].toString());
    }));
    subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::Image>("/camera/head/image_raw",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Image::SharedPtr m){
      if((m->encoding!="rgb8" && m->encoding!="bgr8") || m->data.size()<m->height*m->step)return;
      QImage image(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888);auto copy=image.copy();if(m->encoding=="bgr8")copy=copy.rgbSwapped();
      image_->setPixmap(QPixmap::fromImage(copy).scaled(400,300,Qt::KeepAspectRatio,Qt::SmoothTransformation));
    }));
    connect(&timer_,&QTimer::timeout,this,[this]{tick();});timer_.start(50);
  }
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);QApplication app(argc,argv);
  try {auto node=std::make_shared<rclcpp::Node>("sim_control_gui");ControlPanel panel(node);panel.show();app.exec();}
  catch(const std::exception &error){std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();
}
