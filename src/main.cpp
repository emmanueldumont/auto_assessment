#include <iostream>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>

#define MAX_TF_FRAME 5
#define KINECT1_DEPTH "/kinect1_depth_frame"
#define KINECT2_DEPTH "/kinect2_depth_frame"
#define NB_KINECT 2
#define NB_HUMAN_KINECT 3
#define NB_HUMAN (NB_KINECT*NB_HUMAN_KINECT)
#define KINECT_GENERIC "/kinect%d_depth_frame"
#define MAX_SIZE_STRING 64

typedef struct soignant{
    char ** id;

}t_soignant;

typedef struct patient{
    char ** id;

}t_patient;

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;


int main(int argc, char *argv[]) {

    // Init
    ros::init(argc, argv, "automaticAssessment");
    std::cout << "Oh hai there!" << std::endl;

    ros::NodeHandle node;

    tf::TransformListener listener;
    ros::Rate rate(10.0); // frequency of operation

    bool change2Print = true; // True if changement to print, else false

    // Main loop
    while (node.ok()) {

        // Human detection using number head + ID of each human
        t_patient humains;
        tf::StampedTransform transform[NB_HUMAN];
        int cpt = 0;

        // Create humans struct
        humains.id = (char**) malloc(NB_HUMAN*sizeof(char*));
        for(int loop = 0; loop <NB_HUMAN; loop++)
            humains.id[loop] = (char *) malloc(4* sizeof(char));

        // Create "head" tf name
//        char * head = NULL;
//        head = (char *) malloc(6* sizeof(char)); // 6 = '/''h''e''a''d'' + '\0'
//        snprintf(head, 6, "/head");
        // Create "right_hand1" tf name
        char * head = NULL;
        head = (char *) malloc(12* sizeof(char)); // 6 = 'r''i''g''h''t''_''h''a''n''d' + '\0'
        snprintf(head, 12, "/right_hand");

        int size = (int) strlen(KINECT1_DEPTH) +1;
        char frameK[size];
        char frameRef[MAX_SIZE_STRING];

        // Check every  head ID in the TF
        for (int nbK = 1; nbK < NB_KINECT+1; nbK++) {
            for (int nbH = 0; nbH < MAX_TF_FRAME; nbH++) {

                snprintf(frameK, size, KINECT_GENERIC, nbK);
                snprintf(frameRef, MAX_SIZE_STRING,"%s%d_%d", head, nbK, nbH);

                // Transforms declared for each joint
                try {
                    // each joint frame to reference frame transforms
                    listener.lookupTransform(frameRef, frameK, ros::Time(0), transform[cpt]);
                }
                catch (tf::TransformException &ex) {

//                    ROS_ERROR("%s", ex.what());
                    continue;
                }

                if( cpt < NB_HUMAN ){
                    snprintf(humains.id[cpt], 4, "%d_%d",nbK,nbH);
                    cpt++;
                    if (cpt >=NB_HUMAN) break;
                }
            }
            if (cpt >=NB_HUMAN) break;
        }

        // Differentiate Patient/caregiver
        if(cpt == NB_HUMAN ){

            // Display the ID
            std::cout << "Humains head: ";
            for(int loop =0; loop<NB_HUMAN; loop++){

                std::cout << humains.id[loop] << " ";
            }
            std::cout << std::endl;

            // Find who is the patient, who is the caregiver
            geometry_msgs::Point position[NB_HUMAN];
            for(int loop = 0; loop<NB_HUMAN; loop++) {

                position[loop].x = transform[loop].getOrigin().x();
                position[loop].y = transform[loop].getOrigin().y();
                position[loop].z = transform[loop].getOrigin().z();

                std::cout << "Position"<< loop <<": " << position[loop].x << " " << position[loop].y << " "
                          << position[loop].z << " " << std::endl;
                change2Print = true; // something has been printed
            }

        }
        else{
            // If something has already been printed
            if(change2Print == true){
                std::cout << '\r';
                std::cout << "Nothing found" << std::endl;
                change2Print = false;
            }

        }

        // Free memory
        for(int loop = 0; loop<NB_HUMAN; loop++) {
            free(humains.id[loop]);
        }
        free(humains.id);
        free(head);

        rate.sleep();
    }

    std::cout << "byebye my friend" << std::endl;


    return 0;
}



/*
 *
 * while(node.ok()){

        // Transforms declared for each joint
        tf::StampedTransform transform_hand1_1, transform_hand1_2, transform_hand2_1, transform_hand2_2;
        try
        {
            // each joint frame to reference frame transforms
            listener.lookupTransform("/left_hand1_1", "/right_hand1_1",ros::Time(0), transform_hand1_1);
            listener.lookupTransform("/left_hand2_1", "/right_hand2_1",ros::Time(0), transform_hand2_1);
            listener.lookupTransform("/left_hand1_2", "/right_hand1_2",ros::Time(0), transform_hand1_2);
            listener.lookupTransform("/left_hand2_2", "/right_hand2_2",ros::Time(0), transform_hand2_2);

        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.10).sleep();
            continue;
        }

        // geometry points declaration for storing 3D coordinates of joints and then published later
        geometry_msgs::Point hand1_1_pose, hand2_1_pose, hand1_2_pose, hand2_2_pose;

        // joint position extraction and store

        // First-first hand joint
        hand1_1_pose.x = transform_hand1_1.getOrigin().x();
        hand1_1_pose.y = transform_hand1_1.getOrigin().y();
        hand1_1_pose.z = transform_hand1_1.getOrigin().z();

        // Second-first hand joint
        hand2_1_pose.x = transform_hand2_1.getOrigin().x();
        hand2_1_pose.y = transform_hand2_1.getOrigin().y();
        hand2_1_pose.z = transform_hand2_1.getOrigin().z();

        // First-second hand joint
        hand1_2_pose.x = transform_hand1_2.getOrigin().x();
        hand1_2_pose.y = transform_hand1_2.getOrigin().y();
        hand1_2_pose.z = transform_hand1_2.getOrigin().z();

        // Second-second hand joint
        hand2_2_pose.x = transform_hand2_2.getOrigin().x();
        hand2_2_pose.y = transform_hand2_2.getOrigin().y();
        hand2_2_pose.z = transform_hand2_2.getOrigin().z();

        std::cout << "hand1_1_dis" << hand1_1_pose << "hand1_2_dis" << hand1_2_pose << std::endl;
        std::cout << "hand2_1_dis" << hand2_1_pose << "hand2_2_dis" << hand2_2_pose << std::endl;


        rate.sleep();
    }

    std::cout << "byebye my friend" << std::endl;
 */