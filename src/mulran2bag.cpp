#include <ros/ros.h>
#include <ros/datatypes.h>
#include <rosbag/bag.h>


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <vector>
#include <map>
#include <boost/filesystem.hpp>


#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>


// msgs
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>



namespace bfs = boost::filesystem;

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  int ring;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (uint32_t, t, t) (int, ring, ring)
                                   )

#define PBSTR "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 80

void printProgress(const std::string& name, double percentage) {

    int pbwidth = PBWIDTH - name.length();

    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * pbwidth);
    int rpad = pbwidth - lpad;
    printf("\r%s: %3d%% [%.*s%*s]", name.c_str(), val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

Eigen::Affine3d vectorToAffine3d(const std::vector<double>& v)
{
    return Eigen::Translation<double, 3>(v[0], v[1], v[2]) *
        Eigen::AngleAxis<double>(v[3], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(v[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(v[5], Eigen::Vector3d::UnitZ());
}

geometry_msgs::TransformStamped eigToGeomStamped(
    const Eigen::Affine3d& T, 
    const ros::Time& t, 
    const std::string& parent, 
    const std::string& child)
{
  geometry_msgs::TransformStamped Tstamped;
  tf::Transform tf_T;
  tf::poseEigenToTF(T, tf_T);
  tf::transformStampedTFToMsg(
    tf::StampedTransform(tf_T, t, parent, child),
    Tstamped);
  return Tstamped;
}

int main(int argc, char** argv)
{
    std::cout << "mulran2bag" << std::endl;

    // Parameters

    // Set to tf to see results
    std::string gt_frame = "gt";
    std::string base_frame = "base_link";
    
    std::string ouster_topic = "os1_points";
    std::string ouster_frame = "ouster";

    std::string navtech_frame = "radar_polar";
    std::string navtech_topic = "Navtech/Polar";

    std::string xsens_frame = "imu";
    std::string xsens_topic = "imu/data_raw";
    std::string xsens_mag_topic = "imu/mag";

    std::string gps_frame = "gps";
    std::string gps_topic = "gps/fix";
    
    std::string tf_static_topic = "tf_static";
    std::string tf_topic = "tf";

    // start ground truth tf at zero
    // - disabling this leads to visualization issues
    //   in RViz because of floating point issues
    bool zero_gt_tf = true;

    // enable parts of export
    bool add_ouster = true;
    bool add_radar = true;
    bool add_imu = true;
    bool add_gps = true;
    bool add_dynamic_tf = true;
    bool add_static_tf = true;

    double imu_orientation_cov[3] = {0.01, 0.02, 0.04};
    double imu_ang_vel_cov[3]     = {0.002, 0.002, 0.002};
    double imu_lin_acc_cov[3]     = {0.1, 0.1, 0.1};

    // enable warning
    bool disable_warnings = true;

    if(argc < 2)
    {
        std::cout << "Usage: rosrun mulran2bag mulran2bag [mulran_folder]" << std::endl;
        return 1;
    }

    // in
    bfs::path mulran_root = argv[1];
    std::string bag_name = mulran_root.stem().string() + ".bag";


    if(argc > 2)
    {
        bag_name = argv[2];
    }

    if(bfs::exists(bag_name))
    {
        
        std::cout << "Override '" << bag_name << "'?: [Yn] ";
        while(true)
        {
            char c = std::cin.get();
            if(c == '\n' || c == 'Y' || c == 'y')
            {
                break;
                // continue
            } else if(c == 'n' || c == 'N') {
                // break
                std::cout << "EXIT" << std::endl;
                return 0;
            }
        }
    }

    // out
    rosbag::Bag bag;
    bag.open(bag_name,  rosbag::bagmode::Write);

    std::cout << "Converting " << mulran_root << " to \"" << bag_name << "\"" << std::endl;

    int stage = 1;
    ros::Time tf_static_stamp = ros::TIME_MAX;


    if(add_ouster)
    { // Ouster
        std::cout << stage << ". OUSTER" << std::endl;
        // sorted list of files
        const bfs::path ouster_stamps_csv = mulran_root / "sensor_data" / "ouster_front_stamp.csv";

        std::vector<uint64_t> stamps;

        std::fstream fin;
        fin.open(ouster_stamps_csv.string(), std::ios::in);
        if( fin.is_open() )
        {
            
            uint64_t stamp;
            while(fin >> stamp) 
            {
                stamps.push_back(stamp);
            }
        }
        fin.close();
        
        std::cout << "  - loaded " << stamps.size() << " Ouster stamps" << std::endl;

        long count = 0;
        for(auto stamp : stamps)
        {
            std::stringstream ss;
            ss << stamp << ".bin";

            const bfs::path ouster_data_path = mulran_root / "sensor_data" / "Ouster" / ss.str();
            if(!bfs::exists(ouster_data_path))
            {
                if(!disable_warnings)
                {
                    std::cout << "WARN: Cloud " << ouster_data_path << " doesnt exist! Skipping." << std::endl;
                }
                
                continue;
            }

            pcl::PointCloud<PointXYZIRT> cloud;
            cloud.clear();
            sensor_msgs::PointCloud2 publish_cloud;

            std::ifstream file;
            file.open(ouster_data_path.string(), std::ios::in | std::ios::binary);

            if(!file.is_open())
            {
                std::cout << "Couldnt open existing file " << ouster_data_path << std::endl;
                continue;
            }
            
            int k = 0;
            while(!file.eof())
            {
                PointXYZIRT point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                point.ring = (k%64) + 1;
                k = k+1;
                cloud.points.push_back(point);
            }

            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(stamp);
            publish_cloud.header.frame_id = ouster_frame;
            bag.write(ouster_topic, publish_cloud.header.stamp, publish_cloud);

            if(publish_cloud.header.stamp < tf_static_stamp)
            {
                tf_static_stamp = publish_cloud.header.stamp;
            }
            count++;

            double progress = static_cast<double>(count) / static_cast<double>(stamps.size());

            printProgress("  - add ouster clouds", progress);
        }
        std::cout << std::endl;
    
        stage++;
    }

    if(add_radar)
    { // Radar
        std::cout << stage << ". RADAR" << std::endl;
        // sorted list of files
        const bfs::path ouster_stamps_csv = mulran_root / "sensor_data" / "navtech_top_stamp.csv";

        std::vector<uint64_t> stamps;

        std::fstream fin;
        fin.open(ouster_stamps_csv.string(), std::ios::in);
        if( fin.is_open() )
        {
            uint64_t stamp;
            while(fin >> stamp) 
            {
                stamps.push_back(stamp);
            }
        }

        std::cout << "  - loaded " << stamps.size() << " Navtech stamps" << std::endl;

        long count = 0;
        for(auto stamp : stamps)
        {
            std::stringstream ss;
            ss << stamp << ".png";
            bfs::path radarpolar_path = mulran_root / "sensor_data" / "radar" / "polar" / ss.str();
        
            cv::Mat radarpolar_image = cv::imread(radarpolar_path.string(), 0);

            cv_bridge::CvImage radarpolar_out_msg;
            radarpolar_out_msg.header.stamp.fromNSec(stamp);
            radarpolar_out_msg.header.frame_id = navtech_frame;
            radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
            radarpolar_out_msg.image    = radarpolar_image;
            auto msg = radarpolar_out_msg.toImageMsg();
            bag.write(navtech_topic, msg->header.stamp, *msg);
            if(radarpolar_out_msg.header.stamp < tf_static_stamp)
            {
                tf_static_stamp = radarpolar_out_msg.header.stamp;
            }

            count++;

            double progress = static_cast<double>(count) / static_cast<double>(stamps.size());

            printProgress("  - add radar polar image", progress);
        }
        std::cout << std::endl;

        stage++;
    }

    if(add_imu)
    {
        std::cout << stage << ". IMU" << std::endl;

        const bfs::path imu_csv_path = mulran_root / "sensor_data" / "xsens_imu.csv";

        FILE *fp = fopen(imu_csv_path.string().c_str(), "r");
        uint64_t stamp;
        double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
        sensor_msgs::Imu imu_data;
        sensor_msgs::MagneticField mag_data;

        std::cout << "  - loading " << imu_csv_path << std::endl;
        long count = 0;

        while(1)
        {
            int length = fscanf(fp,
                "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                &stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
            
            if(length != 8 && length != 17)
            {
                break;
            }

            ros::Time stamp_ros;
            stamp_ros.fromNSec(stamp);

            if(stamp_ros < tf_static_stamp)
            {
                tf_static_stamp = stamp_ros;
            }

            if(length >= 8)
            {
                // Write IMU data
                imu_data.header.stamp = stamp_ros;
                imu_data.header.frame_id = xsens_frame;
                imu_data.orientation.x = q_x;
                imu_data.orientation.y = q_y;
                imu_data.orientation.z = q_z;
                imu_data.orientation.w = q_w;

                imu_data.orientation_covariance[0] = imu_orientation_cov[0];
                imu_data.orientation_covariance[4] = imu_orientation_cov[1];
                imu_data.orientation_covariance[8] = imu_orientation_cov[2];

                if(length >= 14)
                {
                    imu_data.angular_velocity.x = g_x;
                    imu_data.angular_velocity.y = g_y;
                    imu_data.angular_velocity.z = g_z;
                    
                    imu_data.angular_velocity_covariance[0] = imu_ang_vel_cov[0];
                    imu_data.angular_velocity_covariance[4] = imu_ang_vel_cov[1];
                    imu_data.angular_velocity_covariance[8] = imu_ang_vel_cov[2];
                } else {
                    imu_data.angular_velocity_covariance[0] = -1;
                }

                if(length >= 17)
                {
                    imu_data.linear_acceleration.x = a_x;
                    imu_data.linear_acceleration.y = a_y;
                    imu_data.linear_acceleration.z = a_z;
                    
                    imu_data.linear_acceleration_covariance[0] = imu_lin_acc_cov[0];
                    imu_data.linear_acceleration_covariance[4] = imu_lin_acc_cov[1];
                    imu_data.linear_acceleration_covariance[8] = imu_lin_acc_cov[2];
                } else {
                    imu_data.linear_acceleration_covariance[0] = -1;
                }

                bag.write(xsens_topic, stamp_ros, imu_data);
            }

            if(length == 17)
            {
                // Write magnetometer data
                mag_data.magnetic_field.x = m_x;
                mag_data.magnetic_field.y = m_y;
                mag_data.magnetic_field.z = m_z;
                bag.write(xsens_mag_topic, stamp_ros, mag_data);
            }
        }

        fclose(fp);

        std::cout << "  - added " << count << " imu messages" << std::endl;

        stage++;
    }

    if(add_gps)
    {
        std::cout << stage << ". GPS" << std::endl;

        const bfs::path gps_csv_path = mulran_root / "sensor_data" / "gps.csv";

        FILE *fp = fopen(gps_csv_path.string().c_str(), "r");
        uint64_t stamp;
        double latitude, longitude, altitude, altitude_orthometric;
        double cov[9];
        
        sensor_msgs::NavSatFix gps_data;
        long count = 0;
        std::cout << "  - loading " << gps_csv_path << std::endl;

        while( fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8])
                == 13)
        {
            ros::Time stamp_ros;
            stamp_ros.fromNSec(stamp);
            
            if(stamp_ros < tf_static_stamp)
            {
                tf_static_stamp = stamp_ros;
            }
            
            gps_data.header.stamp = stamp_ros;
            gps_data.header.frame_id = gps_frame;
            gps_data.latitude = latitude;
            gps_data.longitude = longitude;
            gps_data.altitude = altitude;

            for(int i = 0 ; i < 9 ; i ++) 
            {
                gps_data.position_covariance[i] = cov[i];
            }

            bag.write(gps_topic, stamp_ros, gps_data);
            count++;
        }

        fclose(fp);
        
        std::cout << "  - added " << count << " gps messages" << std::endl;

        stage++;
    }

    if(add_dynamic_tf)
    { // GT
        std::cout << stage << ". DYNAMIC TF (Ground Truth)" << std::endl;
        const bfs::path gt_csv_path = mulran_root / "global_pose.csv";
        // const std::string gt_csv_path = root_path + std::string("/global_pose.csv");

        std::fstream fin;
        fin.open(gt_csv_path.string(), std::ios::in);
        if(fin.is_open())
        {
            std::cout << "  - loading " << gt_csv_path << std::endl;

            std::string temp;
            int count  = 0;
            
            while(fin >> temp) 
            {
                Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Zero();
                T(3,3) = 1.0;

                std::vector<std::string> row;

                std::stringstream ss(temp);
                std::string str;
                while(getline(ss, str, ','))
                {
                    row.push_back(str);
                }

                if(row.size() != 13)
                {
                    break;
                }

                uint64_t stamp_int = stol(row[0]);
                for(int i=0; i<3; i++)
                {
                    for(int j=0; j<4; j++)
                    {
                        double d = boost::lexical_cast<double>(row[1+(4*i)+j]);
                        T(i,j) = d;
                    }
                }

                ros::Time t;
                t.fromNSec(stamp_int);

                if(t < tf_static_stamp)
                {
                    tf_static_stamp = t;
                }
                
                Eigen::Affine3d Tgt(T);            
                if(zero_gt_tf)
                {
                    static Eigen::Affine3d Tgt_init = Tgt; // nice trick
                    Tgt = Tgt_init.inverse() * Tgt; // Start at Zero \in SE(3)
                }
                
                // normally you put the world frame in top of base_link
                // however, in this case we put it below
                // in doing we can run standard localization algorithms without
                // destroying the tree
                geometry_msgs::TransformStamped tf_base_gt 
                    = eigToGeomStamped(Tgt.inverse(), t, base_frame, gt_frame);

                tf2_msgs::TFMessage tfmsg;
                tfmsg.transforms = {tf_base_gt};
                bag.write(tf_topic, t, tfmsg);
                count++;
            }

            std::cout << "  - added " << count << " ground truth poses to TF: '" 
                << base_frame << "' -> '" << gt_frame << "'" <<  std::endl;
        }
        stage++;
    }

    
    // Finally static tf to ealiest time stamp
    if(add_static_tf)
    {
        std::cout << stage << ". STATIC TF" << std::endl;
    
        Eigen::Affine3d Tbase2ouster = vectorToAffine3d(
            {1.7042, -0.021, 1.8047, 
            0.0001*M_PI/180.0, 0.0003*M_PI/180.0, 179.6654*M_PI/180.0 });
        geometry_msgs::TransformStamped tf_base_ouster 
            = eigToGeomStamped(Tbase2ouster, tf_static_stamp, base_frame, ouster_frame);

        Eigen::Affine3d Tbase2navtech = vectorToAffine3d(
            {1.50, -0.04, 1.97, 
            0.0000*M_PI/180.0, 0.0000*M_PI/180.0, 0.9*M_PI/180.0 });
        geometry_msgs::TransformStamped tf_base_navtech 
            = eigToGeomStamped(Tbase2navtech, tf_static_stamp, base_frame, navtech_frame);

        Eigen::Affine3d Tbase2imu = vectorToAffine3d(
            {-0.07, 0.0, 1.7, 
            0.0, 0.0, 0.0});
        geometry_msgs::TransformStamped tf_base_imu 
            = eigToGeomStamped(Tbase2imu, tf_static_stamp, base_frame, xsens_frame);

        Eigen::Affine3d Tbase2gps = vectorToAffine3d(
            {-0.32, 0.0, 1.7, 
            0.0, 0.0, 0.0});
        geometry_msgs::TransformStamped tf_base_gps 
            = eigToGeomStamped(Tbase2gps, tf_static_stamp, base_frame, gps_frame);
        
        tf2_msgs::TFMessage tfmsg;
        tfmsg.transforms = {tf_base_ouster, tf_base_navtech, tf_base_imu, tf_base_gps};

        ros::M_string connection_header = 
        {
            {"callerid", "amock"},
            {"topic", tf_static_topic},
            {"type", ros::message_traits::datatype(tfmsg)},
            {"message_definition", ros::message_traits::definition(tfmsg)},
            {"md5sum", ros::message_traits::md5sum(tfmsg)},
            {"latching", "1"}
        };

        bag.write(tf_static_topic, tf_static_stamp, tfmsg, boost::make_shared<ros::M_string>(connection_header) );

        std::cout << "- added static tf" << std::endl;
        stage++;
    }
    

    bag.close();

    return 0;
}
