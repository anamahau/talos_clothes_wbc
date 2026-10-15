#include <talos_clothes_wbc/upper_body_move.h>
#include <algorithm>
#include <cmath>

double upperBodyMove::minJerk(double tau)
{
    return tau*tau*tau * (10.0 - 15.0*tau + 6.0*tau*tau);
}

geometry_msgs::Pose upperBodyMove::rotateAboutZ(const geometry_msgs::Pose& p,
                                                const tf2::Vector3& pivot, double dyaw)
{
    tf2::Quaternion qz; qz.setRPY(0, 0, dyaw);
    tf2::Quaternion q;  tf2::fromMsg(p.orientation, q);
    tf2::Vector3 pos(p.position.x, p.position.y, p.position.z);

    tf2::Vector3 pn = pivot + tf2::quatRotate(qz, pos - pivot);
    tf2::Quaternion qn = qz * q; qn.normalize();

    geometry_msgs::Pose out;
    out.position.x = pn.x(); out.position.y = pn.y(); out.position.z = pn.z();
    out.orientation = tf2::toMsg(qn);
    return out;
}

std::vector<double> upperBodyMove::toVec(const geometry_msgs::Pose& p)
{
    return {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            p.position.x, p.position.y, p.position.z};
}

double upperBodyMove::dist(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool upperBodyMove::rotateTorsoWithArms(double totalYaw, double duration,
                                        double maxArmLag, double maxTorsoLag,
                                        double settleTol)
{
    if (!torso_.waitForState(2.0))
    {
        ROS_ERROR("No /joint_states received for torso");
        return false;
    }

    geometry_msgs::PoseStamped r0, l0, pivotPose;
    if (!arms_.getRightGripperPose(r0) ||
        !arms_.getLeftGripperPose(l0) ||
        !arms_.getPoseFromTF("odom", "torso_1_link", pivotPose))
    {
        ROS_ERROR("Cannot get initial poses / pivot");
        return false;
    }

    const tf2::Vector3 pivot(pivotPose.pose.position.x,
                             pivotPose.pose.position.y,
                             pivotPose.pose.position.z);
    const double t1_0 = torso_.getTorso1();
    const double t2_0 = torso_.getTorso2();   // pitch is held constant

    ros::Rate rate(50);
    ros::Time wallStart = ros::Time::now();
    ros::Time last = wallStart;
    double t = 0.0;                           // shared "motion clock"
    geometry_msgs::Pose pr = r0.pose, pl = l0.pose;
    double yaw = 0.0;

    while (ros::ok())
    {
        double tau = std::min(t / duration, 1.0);
        yaw = totalYaw * minJerk(tau);

        torso_.sendReference(t1_0 + yaw, t2_0);
        pr = rotateAboutZ(r0.pose, pivot, yaw);   // always from the INITIAL poses
        pl = rotateAboutZ(l0.pose, pivot, yaw);
        arms_.moveR(toVec(pr));
        arms_.moveL(toVec(pl));

        if (tau >= 1.0) break;

        ros::spinOnce();
        ros::Time now = ros::Time::now();
        double dt = (now - last).toSec();
        last = now;

        // Advance the shared clock only if everything keeps up
        geometry_msgs::PoseStamped cr, cl;
        bool haveTf = arms_.getRightGripperPose(cr) && arms_.getLeftGripperPose(cl);
        bool armsOk = !haveTf || (dist(cr.pose, pr) < maxArmLag &&
                                  dist(cl.pose, pl) < maxArmLag);
        bool torsoOk = std::abs(torso_.getTorso1() - (t1_0 + yaw)) < maxTorsoLag;
        if (armsOk && torsoOk) t += dt;

        if ((now - wallStart).toSec() > 5.0 * duration + 5.0)
        {
            ROS_WARN("rotateTorsoWithArms: stalled, aborting");
            return false;
        }
        rate.sleep();
    }

    // Settle: hold the final references until arms and torso converge
    torso_.sendReference(t1_0 + totalYaw, t2_0);
    bool armsDone = arms_.absoluteMoveBoth(toVec(pr), toVec(pl), settleTol);
    bool torsoDone = std::abs(torso_.getTorso1() - (t1_0 + totalYaw)) < settleTol;
    return armsDone && torsoDone;
}

bool upperBodyMove::captureReferenceYaw()
{
    if (!torso_.waitForState(2.0))
    {
        ROS_ERROR("No /joint_states received, cannot capture reference yaw");
        return false;
    }
    refYaw_ = torso_.getTorso1();
    refSet_ = true;
    ROS_INFO("Reference torso yaw set to %.3f rad", refYaw_);
    return true;
}

bool upperBodyMove::getCurrentYaw(double& yaw)
{
    if (!torso_.waitForState(2.0))
    {
        ROS_ERROR("No /joint_states received for torso");
        return false;
    }
    if (!refSet_)
    {
        ROS_WARN("Reference yaw not set, using 0.0 (design angle = torso_1_joint 0)");
    }
    yaw = torso_.getTorso1() - refYaw_;
    return true;
}

std::vector<double> upperBodyMove::poseRelativeToTorso(const std::vector<double>& q,
                                                       const tf2::Vector3& target,
                                                       const tf2::Vector3& offset,
                                                       double yaw)
{
    tf2::Quaternion qz; qz.setRPY(0, 0, yaw);

    tf2::Vector3 pos = target + tf2::quatRotate(qz, offset);

    tf2::Quaternion q0(q[0], q[1], q[2], q[3]);
    tf2::Quaternion qn = qz * q0;
    qn.normalize();

    return {qn.x(), qn.y(), qn.z(), qn.w(), pos.x(), pos.y(), pos.z()};
}

bool upperBodyMove::poseRelativeToTorso(const std::vector<double>& q,
                                        const tf2::Vector3& target,
                                        const tf2::Vector3& offset,
                                        std::vector<double>& poseOut)
{
    if (q.size() != 4)
    {
        ROS_ERROR("Quaternion must have 4 elements: qx qy qz qw");
        return false;
    }
    double yaw;
    if (!getCurrentYaw(yaw)) return false;

    poseOut = poseRelativeToTorso(q, target, offset, yaw);
    return true;
}