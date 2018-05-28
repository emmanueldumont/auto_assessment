#include <iostream>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>

#include "tools.h"

using namespace std;

typedef struct human{
    char ** id;
    geometry_msgs::Point position;
}t_human;

typedef struct soignant{
    char ** id;
    geometry_msgs::Point position;

}t_soignant;

typedef struct patient{
    char ** id;
    geometry_msgs::Point position;

}t_patient;


int initialization(char * refBody, t_patient * humains);

int createPatient(t_patient * patient);

int createSoignant(t_soignant * soignant);

int createHuman(t_human * human);


int main(int argc, char *argv[]) {

    // ROS init
    ros::init(argc, argv, "automaticAssessment");
    log(LOG_DEBUG) << "Oh hai there!" << std::endl;

    ros::NodeHandle node;

    tf::TransformListener listener;
    ros::Rate rate(10.0); // frequency of operation

    // Ressources declaration:
    char * refBody = NULL; // Body part used as human detection
    t_human * humains = NULL;  // Human detection using number refBody + ID of each human
    tf::StampedTransform transform[NB_HUMAN];
    bool change2Print = true; // True if change to print, else false
    int cpt = 0;    // Generic counter

    size_t size = (int) strlen(KINECT1_DEPTH) +1;
    char frameK[size];
    char frameRef[MAX_SIZE_STRING];

    // Initialization
    int retVal = initialization(refBody, humains);
    if (retVal != OK){
        log(LOG_CRITICAL) << "-- Initialization errors, end of the skeleton program";
        exit(0);
    }

    log(LOG_DEBUG) << "-- Initialization done";

    // Main loop
    while (node.ok()) {

        // Check every  hand ID in the TF
        for (int nbK = 1; nbK < NB_KINECT+1; nbK++) {

            snprintf(frameK, size, KINECT_GENERIC, nbK);

            for (int nbH = 0; nbH < MAX_TF_FRAME; nbH++) {

                snprintf(frameRef, MAX_SIZE_STRING,"%s%d_%d", refBody, nbK, nbH);

                // Transforms declared for each joint
                try {
                    // each joint frame to reference frame transforms
                    listener.lookupTransform(frameRef, frameK, ros::Time(0), transform[cpt]);
                }
                catch (tf::TransformException &ex) {
                    continue;
                }

                log(LOG_DEBUG) << "-- Gotcha: " << frameRef << " |" << endl;

                if( cpt < NB_HUMAN ){
                    snprintf(humains->id[cpt], 4, "%d_%d",nbK,nbH);
                    cpt++;
                    if (cpt >=NB_HUMAN) break;
                }
            }
            if (cpt >=NB_HUMAN) break;
        }

        // Differentiate Patient/caregiver
        if(cpt > 0 ){



            // Display the ID
            if (LOG_DEBUG <= LOG_TRESH) {
                log(LOG_DEBUG) << "Humains Right_hand: ";
                for(int loop = 0; loop < cpt; loop++){

                    log(LOG_DEBUG)  << humains->id[loop] << " ";
                }
                log(LOG_DEBUG)  << std::endl;
            }

            // If there is at least 2 humans detected, find who is the patient, who is the caregiver
            if(cpt > 2) {

                geometry_msgs::Point position[NB_HUMAN];
                for (int loop = 0; loop < NB_HUMAN; loop++) {

                    position[loop].x = transform[loop].getOrigin().x();
                    position[loop].y = transform[loop].getOrigin().y();
                    position[loop].z = transform[loop].getOrigin().z();

                    log(LOG_DEBUG) << "Position" << loop << ": " << position[loop].x << " " << position[loop].y << " "
                                  << position[loop].z << " " << std::endl;
                    change2Print = true; // something has been printed - Avoid 9000 reprints
                }
            }

        }
        else{
            // If something has already been printed
            if(change2Print){
                log(LOG_DEBUG) << '\r';
                log(LOG_DEBUG) << "Nothing found" << std::endl;
                change2Print = false;
            }

        }

        // Free memory
        for (int loop = 0; loop < NB_HUMAN; loop++) {
            free(humains->id[loop]);
        }
        free(humains);
        free(refBody);


        rate.sleep();
    }

    log(LOG_DEBUG) << "-- Byebye my friend" << std::endl;


    return 0;
}



int initialization(char * refBody, t_human * humains) {

    int retval = 0;
    const int size = 12;

    // Create "right hand" tf name
    refBody = (char *) malloc(size* sizeof(char)); // 12 = '/''r''i''g''h''t''_''h''a''n''d' + '\0'
    retval = snprintf(refBody, size, "/right_hand");

    if(retval > size ||retval < 0){
        cout << "-- Error intialization"<< endl << "\t-- refBody malloc"<< endl;
        return NOK;
    }

    // Create humans struct
    retval = createHuman(humains);
    if(retval != OK ) return NOK;

    return OK;
}


int createPatient(t_patient * patient){

    return OK;
}

int createSoignant(t_soignant * soignant){

    return OK;
}

int createHuman(t_human * human){

    human->id = (char**) malloc(NB_HUMAN*sizeof(char*));
    for(int loop = 0; loop <NB_HUMAN; loop++){
        human->id[loop] = (char *) malloc(4* sizeof(char));
        memset(human->id[loop],0, 4);
    }

    return OK;
}
