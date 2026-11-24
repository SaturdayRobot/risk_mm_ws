#ifndef _MM_CONFIG_HPP_
#define _MM_CONFIG_HPP_

#include <cmath>
#include <Eigen/Eigen>
#include "mm_utils/ros2_utils.hpp"
#include <memory>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "plan_env/grid_map.h"

namespace remani_planner
{
    class MMConfig
    {
  rclcpp::Node::SharedPtr node_;

    
    public:
        MMConfig() {}
        ~MMConfig() {}

        int getManiDof() const {
            return manipulator_dof_;
        }

        int getBaseDof() const {
            return mobile_base_dof_;
        }

        double getBaseWheelBase() const {
            return mobile_base_wheel_base_;
        }

        Eigen::Matrix4d getTq0(){
            return T_q_0_;
        }

        std::vector<Eigen::Matrix4Xd> getLinkPoint(){
            return manipulator_link_pts_;
        }

        std::vector<Eigen::Vector3d> baseNeighbors(const Eigen::Vector3d &point,double radius,int arm_link,bool sorted=true) const;
        std::vector<int> armNeighbors(int link,const Eigen::Vector3d &point,double radius,bool sorted=true) const;
        // Conservative cost candidates in original point-index order. link=-1
        // selects the base. No SRDF/owner exclusions alter the source objective.
        std::vector<int> collisionCostNeighbors(int link,const Eigen::Vector3d &point,double radius) const;
        bool collisionCostBoxClear(int link,const Eigen::Matrix4d &transform,double threshold,double floor) const;
        bool hasSelfCollisionPair(int a, int b) const { return active_self_link_pairs_[a][b]; }
        bool hasSelfCollisionPointPair(int a, int point, int b) const {
            return active_owner_link_pairs_[collision_owners_.at(a).at(point)][b];
        }
        bool skipSelfPair(int a,int point_a,int b,int point_b) const {
            const int x=collision_owners_.at(a).at(point_a), y=collision_owners_.at(b).at(point_b);
            return x==y || disabled_self_pairs_[x][y];
        }
        void getAJointTran(int joint_num, double theta, Eigen::Matrix4d &T, Eigen::Matrix4d &T_grad) const;

        void getJointTrans(const Eigen::VectorXd &theta, std::vector<Eigen::Matrix4d> &T_joint, std::vector<Eigen::Matrix4d> &T_joint_grad){
            T_joint.clear();
            T_joint_grad.clear();

            for(int i = 0; i < manipulator_dof_; ++i)
            {
               Eigen::Matrix4d T_temp, T_temp_grad;
               getAJointTran(i, theta(i), T_temp, T_temp_grad);
               T_joint.push_back(T_temp);
               T_joint_grad.push_back(T_temp_grad);
            }
        }

        Eigen::VectorXd getManiConfig(){
            return manipulator_config_;
        }

        void setParam(rclcpp::Node::SharedPtr &nh);
        void setParam(rclcpp::Node::SharedPtr &nh, const std::shared_ptr<GridMap>& env);
        bool checkCarObsCollision(Eigen::Vector3d car_state, bool precise, bool safe, double &min_dist);
        bool checkManiObsCollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, double &min_dist, bool record=true);
        // Read-only collision checks for validators sharing a frozen map.
        bool planningEnvironmentCollision(const Eigen::Vector3d &base, const Eigen::VectorXd &q);
        bool planningSelfCollision(const Eigen::VectorXd &q) const;
        bool checkCarManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist);
        bool checkManiManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist);
        bool checkManicollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe);
        bool checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe);
        bool checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, int &coll_type /*0: car, 1: mani, 2: car-mani, 3: mani-mani*/);
        void visMM(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close);
        void visMMCheckBall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state);
        void getMMMarkerArray(visualization_msgs::msg::MarkerArray &marker_array, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close);
        double calYaw(const Eigen::Vector2d &vel_car, int traj_dir);
        Eigen::Matrix2d calR(const Eigen::Vector2d &vel_car, int traj_dir);
        Eigen::Matrix2d caldRldv(const Eigen::Vector2d &vel_car, const Eigen::Vector2d &delta_p, const int traj_dir);
        Eigen::Vector2d caldYawdV(const Eigen::Vector2d &vel_car);
        Eigen::Vector2d regularizeBaseVelocity(const Eigen::Vector2d &vel_car, int traj_dir) const;
        double getNonSingularVelocity() const { return mobile_base_non_singul_vel_; }
        void getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts);
        void getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts, const Eigen::Vector3d &inflate_size);
        void getCarPtsGrad(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                            std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Matrix2d> &dPtdv);
        void getCarPtsGradNew(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                    std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Vector2d> &dPtdYaw);
        void CarState2T(const Eigen::Vector3d &car_state, Eigen::Matrix4d &T_car);
        void getJointTMat(const Eigen::VectorXd &theta, std::vector<Eigen::Matrix4d> &T_joint);
        void setGripperPoint(const bool gripper_close);
        double getBaseMaxVel() const {
            return mobile_base_max_vel_;
        }
        double getBaseMaxAcc() const {
            return mobile_base_max_acc_;
        }
    private:
        std::vector<Eigen::Vector3d> color_set_;
        std::shared_ptr<GridMap> grid_map_;
        struct BodyIndex;
        std::shared_ptr<BodyIndex> body_index_;
        std::vector<std::shared_ptr<BodyIndex>> arm_indices_;
        std::vector<std::vector<int>> collision_owners_;
        bool active_self_link_pairs_[6][6]{};
        bool active_owner_link_pairs_[7][6]{};
        bool disabled_self_pairs_[7][7]{};
        std::vector<Eigen::Matrix4d> joint_origins_;
        std::vector<Eigen::Vector3d> joint_axes_, base_collision_points_;
        std::vector<std::string> arm_mesh_resources_;
        int mobile_base_dof_;
        int manipulator_dof_;
        double mobile_base_length_, mobile_base_width_, mobile_base_height_;
        double mobile_base_check_radius_;
        double mobile_base_wheel_base_, mobile_base_wheel_radius_, mobile_base_max_wheel_omega_, mobile_base_max_wheel_alpha_;
        double mobile_base_non_singul_vel_{0.05};
        
        Eigen::VectorXd manipulator_config_;
        double manipulator_thickness_;
        double map_resolution_;
        std::vector<Eigen::Matrix4Xd> manipulator_link_pts_;
        double car_safe_margin_;
        double mani_safe_margin_;
        double self_safe_margin_;
        double ground_safe_dis_;
        double mobile_base_max_vel_, mobile_base_max_acc_;

        Eigen::Matrix2d B_h_;
        Eigen::Matrix4d T_q_0_;
        int vis_idx_size_;

        Eigen::VectorXd manipulator_min_pos_, manipulator_max_pos_;

        visualization_msgs::msg::Marker sphere_free_, sphere_occ_;

        std::string mesh_resource_mobile_base_;

        void setColorSet();
        void visCarCheckBall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &state);
        void visManiCheckBall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state);
        void visMesh(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, int id, std::string ns, double alpha, Eigen::Vector3d color_rgb, const Eigen::Matrix4d &T, const std::string &mesh_file);
        
        visualization_msgs::msg::MarkerArray getCarMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &state);
        visualization_msgs::msg::MarkerArray getManiMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close);
        visualization_msgs::msg::Marker getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file);

    public:
        // typedef std::unique_ptr<MMConfig> Ptr;
        typedef std::shared_ptr<MMConfig> Ptr;
    };

    struct MMState{
        double time_stamp;
        Eigen::Vector2d car_pos;
        Eigen::Vector2d car_vel;
        double car_yaw;
        double car_v; // >0 foreward <0 backward
        double car_a; // >0 foreward <0 backward
        double car_omega;
        double car_domega;
        Eigen::Matrix2d car_R;
        double car_input_vel;
        double car_input_omega;
        Eigen::VectorXd mani_pos;
        Eigen::VectorXd mani_vel;

        Eigen::Vector3d getCarXYYaw(){
            return Eigen::Vector3d(car_pos(0), car_pos(1), car_yaw);
        }

        Eigen::Vector4d getCarXYYawV(){
            return Eigen::Vector4d(car_pos(0), car_pos(1), car_yaw, car_v);
        }
    };

} // namespace remani_planner

#endif
