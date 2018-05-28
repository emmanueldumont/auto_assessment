//
// Created by dumont on 28/05/18.
//

#ifndef AUTOASSESSMENT_TOOLS_H
#define AUTOASSESSMENT_TOOLS_H



/*
 *
 * Log definition
 *
 */

enum log_level_t {
    LOG_NOTHING,
    LOG_CRITICAL,
    LOG_ERROR,
    LOG_WARNING,
    LOG_INFO,
    LOG_DEBUG,
    LOG_TRESH
};

class mystreambuf: public std::streambuf
{
};

mystreambuf nostreambuf;
std::ostream nocout(&nostreambuf);
#define log(x) ((x >= LOG_TRESH)? std::cout : nocout)


/*
 *
 * Define macro
 *
 */
#define MAX_SIZE_STRING 64 // Global maximum size for a string
#define MAX_TF_FRAME 5

// Explicit
#define NOK 0
#define OK 1

#define KINECT1_DEPTH "/kinect1_depth_frame"
#define KINECT2_DEPTH "/kinect2_depth_frame"

#define NB_KINECT 2 // Number of inect used
#define NB_HUMAN_KINECT 3 // Maximum human we are looking for
#define NB_HUMAN (NB_KINECT*NB_HUMAN_KINECT) // Global max number of human we are looking for

#define KINECT_GENERIC "/kinect%d_depth_frame"


#endif //AUTOASSESSMENT_TOOLS_H
