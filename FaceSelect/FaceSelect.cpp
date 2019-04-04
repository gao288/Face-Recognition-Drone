#include "LandmarkCoreIncludes.h"

#include "VisualizationUtils.h"
#include "Visualizer.h"
#include "SequenceCapture.h"
#include "GimbalUtil.h"
#include <RecorderOpenFace.h>
#include <RecorderOpenFaceParameters.h>
#include <GazeEstimation.h>
#include <FaceAnalyser.h>

#define INFO_STREAM( stream ) \
    std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
    std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
    std::cout << "Error: " << stream << std::endl

static void printErrorAndAbort(const std::string & error)
{
    std::cout << error << std::endl;
    abort();
}

#define FATAL_STREAM( stream ) \
    printErrorAndAbort( std::string( "Fatal error: " ) + stream )

using namespace std;

vector<string> get_arguments(int argc, char **argv)
{

    vector<string> arguments;

    for (int i = 0; i < argc; ++i)
    {
        arguments.push_back(string(argv[i]));
    }
    return arguments;
}

void NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<float> >& face_detections)
{

    // Go over the model and eliminate detections that are not informative (there already is a tracker there)
    for (size_t model = 0; model < clnf_models.size(); ++model)
    {

        // See if the detections intersect
        cv::Rect_<float> model_rect = clnf_models[model].GetBoundingBox();

        for (int detection = face_detections.size() - 1; detection >= 0; --detection)
        {
            double intersection_area = (model_rect & face_detections[detection]).area();
            double union_area = model_rect.area() + face_detections[detection].area() - 2 * intersection_area;

            // If the model is already tracking what we're detecting ignore the detection, this is determined by amount of overlap
            if (intersection_area / union_area > 0.5)
            {
                face_detections.erase(face_detections.begin() + detection);
            }
        }
    }
}

int main(int argc, char **argv)
{

    vector<string> arguments = {"-device", "1"};

    int fd = gimbal_init(); // return the usb port
    int xoffset = 0;
    int yoffset = 0;
    int zoffset = 0;

    // no arguments: output usage
    LandmarkDetector::FaceModelParameters det_params(arguments);
    // This is so that the model would not try re-initialising itself
    det_params.reinit_video_every = -1;

    det_params.curr_face_detector = LandmarkDetector::FaceModelParameters::MTCNN_DETECTOR;

    vector<LandmarkDetector::FaceModelParameters> det_parameters;
    det_parameters.push_back(det_params);

    // The modules that are being used for tracking
    vector<LandmarkDetector::CLNF> face_models;
    vector<bool> active_models;

    int num_faces_max = 4;
    int selected_model = 0;

    LandmarkDetector::CLNF face_model(det_parameters[0].model_location);

    if (!face_model.loaded_successfully)
    {
        cout << "ERROR: Could not load the landmark detector" << endl;
        return 1;
    }

    // Loading the face detectors
    face_model.face_detector_HAAR.load(det_parameters[0].haar_face_detector_location);
    face_model.haar_face_detector_location = det_parameters[0].haar_face_detector_location;
    face_model.face_detector_MTCNN.Read(det_parameters[0].mtcnn_face_detector_location);
    face_model.mtcnn_face_detector_location = det_parameters[0].mtcnn_face_detector_location;

    // If can't find MTCNN face detector, default to HOG one
    if (det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::MTCNN_DETECTOR && face_model.face_detector_MTCNN.empty())
    {
        cout << "INFO: defaulting to HOG-SVM face detector" << endl;
        det_parameters[0].curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
    }

    face_models.reserve(num_faces_max);

    face_models.push_back(face_model);
    active_models.push_back(false);

    for (int i = 1; i < num_faces_max; ++i)
    {
        face_models.push_back(face_model);
        active_models.push_back(false);
        det_parameters.push_back(det_params);
    }

    // Load facial feature extractor and AU analyser (make sure it is static, as we don't reidentify faces)
    FaceAnalysis::FaceAnalyserParameters face_analysis_params(arguments);
    face_analysis_params.OptimizeForImages();
    FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);

    if (!face_model.eye_model)
    {
        cout << "WARNING: no eye model found" << endl;
    }

    if (face_analyser.GetAUClassNames().size() == 0 && face_analyser.GetAUClassNames().size() == 0)
    {
        cout << "WARNING: no Action Unit models found" << endl;
    }

    // Open a sequence
    Utilities::SequenceCapture sequence_reader;

    // A utility for visualizing the results (show just the tracks)
    Utilities::Visualizer visualizer(arguments);

    // Tracking FPS for visualization
    Utilities::FpsTracker fps_tracker;
    fps_tracker.AddFrame();



    // The sequence reader chooses what to open based on command line arguments provided
    if (!sequence_reader.Open(arguments)) return 1;

    INFO_STREAM("Device or file opened");

    cv::Mat rgb_image = sequence_reader.GetNextFrame();

    int frame_count = 0;

    INFO_STREAM("Using a webcam in feature extraction, forcing visualization of tracking to allow quitting the application (press q)");
    visualizer.vis_track = true;

    // flag for selecting mode / tracking mode
    bool selecting = true;

    INFO_STREAM("Starting tracking");

    // Main loop
    while (!rgb_image.empty())
    {

        // Reading the images
        cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();

        vector<cv::Rect_<float> > face_detections;

        if(selecting) {
            bool all_models_active = true;
            for (unsigned int model = 0; model < face_models.size(); ++model)
            {
                if (!active_models[model])
                {
                    all_models_active = false;
                }
            }

            // Get the detections (every 8th frame and when there are free models available for tracking)
            if (frame_count % 8 == 0 && !all_models_active)
            {
                if (det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR)
                {
                    vector<float> confidences;
                    LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, face_models[0].face_detector_HOG, confidences);
                }
                else if (det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HAAR_DETECTOR)
                {
                    LandmarkDetector::DetectFaces(face_detections, grayscale_image, face_models[0].face_detector_HAAR);
                }
                else
                {
                    vector<float> confidences;
                    LandmarkDetector::DetectFacesMTCNN(face_detections, rgb_image, face_models[0].face_detector_MTCNN, confidences);
                }

            }

            // Keep only non overlapping detections (so as not to start tracking where the face is already tracked)
            NonOverlapingDetections(face_models, face_detections);
            std::vector<bool> face_detections_used(face_detections.size(), false);

            // Go through every model and update the tracking
            for (unsigned int model = 0; model < face_models.size(); ++model)
            {

                bool detection_success = false;

                // If the current model has failed more than 4 times in a row, remove it
                if (face_models[model].failures_in_a_row > 4)
                {
                    active_models[model] = false;
                    face_models[model].Reset();
                }

                // If the model is inactive reactivate it with new detections
                if (!active_models[model])
                {

                    for (size_t detection_ind = 0; detection_ind < face_detections.size(); ++detection_ind)
                    {
                        // if it was not taken by another tracker take it
                        if (!face_detections_used[detection_ind])
                        {
                            face_detections_used[detection_ind] = true;

                            // Reinitialise the model
                            face_models[model].Reset();

                            // This ensures that a wider window is used for the initial landmark localisation
                            face_models[model].detection_success = false;
                            detection_success = LandmarkDetector::DetectLandmarksInVideo(rgb_image, face_detections[detection_ind], face_models[model], det_parameters[model], grayscale_image);

                            // This activates the model
                            active_models[model] = true;

                            // break out of the loop as the tracker has been reinitialised
                            break;
                        }

                    }
                }
                else
                {
                    // The actual facial landmark detection / tracking
                    detection_success = LandmarkDetector::DetectLandmarksInVideo(rgb_image, face_models[model], det_parameters[model], grayscale_image);
                }
            }
        }
        else {
			bool detection_success = LandmarkDetector::DetectLandmarksInVideo(rgb_image, face_models[selected_model], det_parameters[selected_model], grayscale_image);
        }
        // Keeping track of FPS
        fps_tracker.AddFrame();
        visualizer.SetImage(rgb_image, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

        // Estimate head pose				
        cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_models[selected_model], sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

        // Face analysis step
        cv::Mat sim_warped_img;
        cv::Mat_<double> hog_descriptor; int num_hog_rows = 0, num_hog_cols = 0;

        // Visualize the features
        visualizer.SetObservationFaceAlign(sim_warped_img);
        visualizer.SetObservationPose(LandmarkDetector::GetPose(face_models[selected_model], sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy), face_models[selected_model].detection_certainty);

        // if selected and usb successfully connected
        if(!selecting && fd > 0) {
            // send signal to micro
            cout << "writing to uart" << endl;
            writeToUART(fd, string("x") + to_string((int) pose_estimate[1]) + "\r");
            writeToUART(fd, string("z") + to_string((int) pose_estimate[0]) + "\r");
        }

        visualizer.SetFps(fps_tracker.GetFPS());

        // show visualization and detect key presses
        char character_press = visualizer.ShowObservation();

        // restart the trackers
        if (character_press == 'r')
        {
            for (size_t i = 0; i < face_models.size(); ++i)
            {
                face_models[i].Reset();
                active_models[i] = false;
            }
        }

        // quit the application
        else if (character_press == 'q')
        {
            return 0;
        }
        else if (selecting && character_press == 'n') {
            selected_model++;
            selected_model %= num_faces_max;
            for(int i = 0;i < num_faces_max-1 && !active_models[selected_model];i++){
                selected_model++;
                selected_model %= num_faces_max;
            }
        }
        else if (character_press == 'k') {
            selecting = !selecting;
            cout << (selecting?"selecting":"selected") << endl;
        }

        // Update the frame count
        frame_count++;

        // Grabbing the next frame in the sequence
        rgb_image = sequence_reader.GetNextFrame();
    }

    INFO_STREAM("Closing input reader");
    sequence_reader.Close();
    INFO_STREAM("Closed successfully");

    return 0;
}

