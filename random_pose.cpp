#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <xidian_swarm/formation_msg.h>
#include <std_srvs/Trigger.h>
#include "xidian_swarm/takeOff_msg.h"
#include <nav_msgs/Path.h>

//2000 = 0.5 3333=0.3 1250=0.8 1000 = 1 20000 = tan0.3
double randomRange = 1;
double number = 4;
int max_epoch = 10;
int searchNum = 30;

std::vector<ros::Subscriber> mRealSubPose;
std::vector<ros::Publisher> mNoisePubPose;
std::vector<ros::Publisher> mRealPathPublisher;
std::vector<ros::Publisher> mNoisePathPublisher;
std::vector<ros::Publisher> mParticlePathPublisher;

std::vector<geometry_msgs::PoseStamped> mRealCurrentPose;
std::vector<geometry_msgs::PoseStamped> mRealLastPose;

// std::vector<geometry_msgs::PoseStamped> mNoiseCurPose;
std::vector<geometry_msgs::PoseStamped> mNoiseLastPose;

std::vector<nav_msgs::Path> particlePath;

std::vector<nav_msgs::Path> realPath;
std::vector<nav_msgs::Path> noisePath;

std::vector<std::vector<geometry_msgs::PoseStamped>> mParticle;
std::vector<geometry_msgs::PoseStamped> avgParticlePose;

void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr & msg, int i)
{
    mRealLastPose.at(i) = mRealCurrentPose.at(i);
    ROS_INFO("stamp_before: %d, %d",mRealCurrentPose.at(i).header.stamp, mRealLastPose.at(i).header.stamp);
    ROS_INFO("%f,%f,%f,------%f,%f,%f",mRealCurrentPose.at(i).pose.position.x,mRealCurrentPose.at(i).pose.position.y,mRealCurrentPose.at(i).pose.position.z,
    mRealLastPose.at(i).pose.position.x,mRealLastPose.at(i).pose.position.y,mRealLastPose.at(i).pose.position.z);
    mRealCurrentPose.at(i) = *msg;

}

void random_pose_init(int number, ros::NodeHandle nh){
    ros::Rate rate(200);
    mRealLastPose.clear();
    mNoiseLastPose.clear();
    mRealCurrentPose.resize(number);
    mRealLastPose.resize(number);
    mNoiseLastPose.resize(number);
    realPath.resize(number);
    noisePath.resize(number);
    mRealSubPose.resize(number);
    mNoisePubPose.resize(number);
    mParticle.resize(number);
    avgParticlePose.resize(number);
    particlePath.resize(searchNum);

    for(int i = 0; i < number;i++)
    {
        std::string poseSubTopicName = "/uav" + std::to_string(i) + "/mavros/local_position/pose";
        std::string posePubTopicName = "/uav" + std::to_string(i) + "/mavros/noise/pose";
        std::string noisePathPubTopicName = "/uav" + std::to_string(i) + "/mavros/noise/path";
        std::string realPathPubTopicName = "/uav" + std::to_string(i) + "/mavros/real/path";
        mRealSubPose.push_back(nh.subscribe<geometry_msgs::PoseStamped>(poseSubTopicName,1, boost::bind(&poseSubCB, _1,i)));
        mNoisePubPose.push_back(nh.advertise<geometry_msgs::PoseStamped>(posePubTopicName, 10));
        mNoisePathPublisher.push_back(nh.advertise<nav_msgs::Path>(noisePathPubTopicName, 10));
        mRealPathPublisher.push_back(nh.advertise<nav_msgs::Path>(realPathPubTopicName, 10));
    }
    for (int i=0;i<searchNum;i++){
        std::string particlePathPubTopicName = "/particle" + std::to_string(i) + "/mavros/path";
        mParticlePathPublisher.push_back(nh.advertise<nav_msgs::Path>(particlePathPubTopicName, 10));
    }
    int connect_id[number];
    int connected_nums = 0;
    std::cout << "waiting for connect" << std::endl;
    while(ros::ok())
    {
        for(int i = 0; i < number; i++){
            if(mRealCurrentPose.at(i).pose.position.x != 0 && connect_id[i] != 1){
                connect_id[i] = 1;
                connected_nums++;
                ROS_INFO("\n noise uav %d connected!! \n", i);
            }
        }
        if(connected_nums == number)
        {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("all uav has connectd!! \n");

}

double getDistance(int i, int j){

    double dis_x = mRealCurrentPose.at(i).pose.position.x - mRealCurrentPose.at(j).pose.position.x;
    double dis_y = mRealCurrentPose.at(i).pose.position.y - mRealCurrentPose.at(j).pose.position.y;
    double randNoist = (rand()%40 - 20)/100;
    if(randNoist>=0) randNoist = randNoist + 0.1;
    else randNoist = randNoist - 0.1;
    return sqrt(dis_x*dis_x+dis_y*dis_y)+ randNoist;
}

double getNoiseDistance(int i, int j){

    double dis_x = mNoiseLastPose.at(i).pose.position.x - mNoiseLastPose.at(j).pose.position.x;
    double dis_y = mNoiseLastPose.at(i).pose.position.y - mNoiseLastPose.at(j).pose.position.y;
    return sqrt(dis_x*dis_x+dis_y*dis_y);
}

double getRealDistance(int i, int j){

    double dis_x = mRealCurrentPose.at(i).pose.position.x - mRealCurrentPose.at(j).pose.position.x;
    double dis_y = mRealCurrentPose.at(i).pose.position.y - mRealCurrentPose.at(j).pose.position.y;
    return sqrt(dis_x*dis_x+dis_y*dis_y);
}

double getDistanceParticle(int uav_index, int search_index, int avg_index){
    double dis_x = mParticle.at(uav_index).at(search_index).pose.position.x - avgParticlePose.at(avg_index).pose.position.x;
    double dis_y = mParticle.at(uav_index).at(search_index).pose.position.y - avgParticlePose.at(avg_index).pose.position.y;
    return sqrt(dis_x*dis_x+dis_y*dis_y);
}

geometry_msgs::PoseStamped particlePtimize(int uav_index){
    int best_index=-1;
    double best_dis=10000.0;

    for(int particle_i=0;particle_i<searchNum;particle_i++){
        double uwb_dis = 0;
        double particle_dis = 0;
        for(int j=0;j<number;j++) {
            if(uav_index == j) continue;
            uwb_dis += getDistance(uav_index, j);
            if (uwb_dis == 0) exit(1);
            particle_dis += getDistanceParticle(uav_index, particle_i, j);
        }
        // ROS_INFO("uwb %f,part %f",uwb_dis,particle_dis);
        if(abs(uwb_dis-particle_dis) < best_dis) {
            best_index = particle_i;
            best_dis = abs(uwb_dis-particle_dis);
        }
    }
    return mParticle.at(uav_index).at(best_index);
}

void setAvgParticle(){
    for(int uav_index=0;uav_index<number;uav_index++){
        double sum_x=0,sum_y=0,sum_z=0;
        for(int i=0;i<searchNum;i++){
            sum_x += mParticle.at(uav_index).at(i).pose.position.x;
            sum_y += mParticle.at(uav_index).at(i).pose.position.y;
            sum_z += mParticle.at(uav_index).at(i).pose.position.z;
        }
        avgParticlePose.at(uav_index).pose.position.x = sum_x/(double)searchNum;
        avgParticlePose.at(uav_index).pose.position.y = sum_y/(double)searchNum;
        avgParticlePose.at(uav_index).pose.position.z = sum_z/(double)searchNum;
    }
}

void stepParticle(int epoch ,int uav_index, double dis_x, double dis_y){
    if(epoch == 0) {
        mParticle.at(uav_index).clear();
        mParticle.at(uav_index).resize(searchNum);
        for(int i=0;i<searchNum;i++){
            mParticle.at(uav_index).at(i) = mNoiseLastPose.at(uav_index);
        }
        return;
    }
    double randNoist;
    int kpx = 1;
    int kpy = 1;
    if(abs(dis_x)<0.01){
        kpx=0;
    }
    if(abs(dis_y)<0.01){
        kpy=0;
    }
    for(int i=0;i<searchNum;i++){
        ROS_INFO("dis %f,%f", dis_x,dis_y);
        randNoist = (rand()%100-50)/(double)50;
        ROS_INFO("the uav_index %d, particle index %d, noise %f", uav_index,i,randNoist);

        mParticle.at(uav_index).at(i).pose.position.x = mParticle.at(uav_index).at(i).pose.position.x + dis_x + randomRange*randNoist*dis_x;
        randNoist = (rand()%100-50)/(double)50;
        ROS_INFO("the uav_index %d, particle index %d, noise %f", uav_index,i,randNoist); 
        mParticle.at(uav_index).at(i).pose.position.y = mParticle.at(uav_index).at(i).pose.position.y + dis_y + randomRange*randNoist*dis_y;
        
        mParticle.at(uav_index).at(i).pose.position.z = mNoiseLastPose.at(uav_index).pose.position.z;
        ROS_INFO("the uav_index %d, particle_index %d,xpose %f, ypose %f", uav_index, i,mParticle.at(uav_index).at(i).pose.position.x, mParticle.at(uav_index).at(i).pose.position.y); 
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_pose_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    double noise;
    int noiseType = 45;
    int printMax = 1;
    srand(time(NULL));
    random_pose_init(number, nh);

    
    std::vector<int> mEpoch;
    std::vector<bool> mPoseInit;
    std::vector<int> mPrintInit;

    for(int tmp_epoch=0;tmp_epoch<searchNum;tmp_epoch++){
        mEpoch.push_back(0);
    }

    for(int k=0;k<number;k++){
        mPoseInit.push_back(true);
        mPrintInit.push_back(1);
    }

    while(ros::ok()){
        for(int i = 0; i < number; i++)
        {
            double dis_x = mRealCurrentPose.at(i).pose.position.x - mRealLastPose.at(i).pose.position.x; 
            double dis_y = mRealCurrentPose.at(i).pose.position.y - mRealLastPose.at(i).pose.position.y;
            double dis_z = mRealCurrentPose.at(i).pose.position.z - mRealLastPose.at(i).pose.position.z;
            int kp = 1;

            if(mPoseInit.at(i)) {
                mNoiseLastPose.at(i).pose.position.x = mRealCurrentPose.at(i).pose.position.x;
                mNoiseLastPose.at(i).pose.position.y = mRealCurrentPose.at(i).pose.position.y;
                mNoiseLastPose.at(i).pose.position.z = mRealCurrentPose.at(i).pose.position.z;
                mPoseInit.at(i) = false;
            } else {
                if(i%2 == 0) noiseType = 45;
                else noiseType = 55;
                double noise = (rand()%100-noiseType)/(double)50;

                mNoiseLastPose.at(i).pose.position.x = mNoiseLastPose.at(i).pose.position.x + dis_x + randomRange*dis_x*noise;
                noise = (rand()%100-noiseType)/(double)50;
                mNoiseLastPose.at(i).pose.position.y = mNoiseLastPose.at(i).pose.position.y + dis_y + randomRange*dis_y*noise;
                noise = (rand()%100-noiseType)/(double)50;
                mNoiseLastPose.at(i).pose.position.z = mNoiseLastPose.at(i).pose.position.z + dis_z;
            }
            realPath.at(i).header = mRealCurrentPose.at(i).header;
            double dis = getDistance(0,i);


            if(mEpoch.at(i) < max_epoch){
                stepParticle(mEpoch.at(i), i,dis_x,dis_y);
                mEpoch.at(i)++;
            }
            
            if(i == 0) {
                for(int k=0;k<searchNum;k++) {
                    particlePath.at(k).poses.push_back(mParticle.at(i).at(k));
                }
            }
            double resultx = mNoiseLastPose.at(i).pose.position.x - mRealCurrentPose.at(i).pose.position.x;
            double resulty = mNoiseLastPose.at(i).pose.position.y - mRealCurrentPose.at(i).pose.position.y;
            double resultz = mNoiseLastPose.at(i).pose.position.z - mRealCurrentPose.at(i).pose.position.z;

            mPrintInit.at(i)++;
            realPath.at(i).poses.push_back(mRealCurrentPose.at(i));
            noisePath.at(i).header = mRealCurrentPose.at(i).header;
            noisePath.at(i).poses.push_back(mNoiseLastPose.at(i));
            mNoisePubPose.at(i).publish(mRealCurrentPose.at(i));
            mRealPathPublisher.at(i).publish(realPath.at(i));
            mNoisePathPublisher.at(i).publish(noisePath.at(i));
        }
        for(int particle_index=0;particle_index<searchNum;particle_index++){
            particlePath.at(particle_index).header = mRealCurrentPose.at(0).header;
            mParticlePathPublisher.at(particle_index).publish(particlePath.at(particle_index));
        }
        mRealLastPose.clear();
        mRealLastPose.assign(mRealCurrentPose.begin(), mRealCurrentPose.end());
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}