#pragma once
#include <talos_clothes_wbc/arms_move.h>
#include <talos_clothes_wbc/torso_move.h>

class upperBodyMove
{
public:
    upperBodyMove(armsMove& arms, torsoMove& torso) : arms_(arms), torso_(torso) {}

    // totalYaw [rad], duration [s]. maxArmLag [m] and maxTorsoLag [rad] pause the
    // shared clock if the arms or torso fall behind their references.
    bool rotateTorsoWithArms(double totalYaw, double duration,
                             double maxArmLag = 0.03,
                             double maxTorsoLag = 0.05,
                             double settleTol = 0.02);

    // Remember the current torso_1 angle as the one the poses were designed at.
    // Call it once, with the torso in the pose where your offsets/quaternions make sense.
    bool captureReferenceYaw();
    void setReferenceYaw(double yaw) { refYaw_ = yaw; refSet_ = true; }

    // Rotation of the torso since the reference, read from /joint_states [rad]
    bool getCurrentYaw(double& yaw);

    // Pose [qx qy qz qw x y z] for the torso's current yaw.
    // q and offset are as designed (at the reference yaw), target is in odom.
    bool poseRelativeToTorso(const std::vector<double>& q,
                             const tf2::Vector3& target,
                             const tf2::Vector3& offset,
                             std::vector<double>& poseOut);

    // Same, with an explicit yaw
    static std::vector<double> poseRelativeToTorso(const std::vector<double>& q,
                                                   const tf2::Vector3& target,
                                                   const tf2::Vector3& offset,
                                                   double yaw);

private:
    armsMove& arms_;
    torsoMove& torso_;
    static double minJerk(double tau);
    static geometry_msgs::Pose rotateAboutZ(const geometry_msgs::Pose& p,
                                            const tf2::Vector3& pivot, double dyaw);
    static std::vector<double> toVec(const geometry_msgs::Pose& p);
    static double dist(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);
    double refYaw_ = 0.0;
    bool refSet_ = false;
};