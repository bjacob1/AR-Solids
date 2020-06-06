//#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

vector<Point3f> house_coors() {
    vector<Point3f> ret;
    //walls
    for (int i = -1; i <= 1; i += 2) {
        for (int j = -1; j <= 1; j += 2) {
            for (int k = 0; k <= 2; k += 2) {
                ret.push_back(Point3f(i, j, k));
            }
        }
    }
    //roof
    ret.push_back(Point3f(0, 0, 3));
    //windows
    ret.push_back(Point3f(-.35, -1, .65));
    ret.push_back(Point3f(-.35, -1, 1.35));
    ret.push_back(Point3f(.35, -1, .65));
    ret.push_back(Point3f(.35, -1, 1.35));
    ret.push_back(Point3f(-.35, 1, .65));
    ret.push_back(Point3f(-.35, 1, 1.35));
    ret.push_back(Point3f(.35, 1, .65));
    ret.push_back(Point3f(.35, 1, 1.35));
    ret.push_back(Point3f(-1, -.35, .65));
    ret.push_back(Point3f(-1, -.35, 1.35));
    ret.push_back(Point3f(-1, .35, .65));
    ret.push_back(Point3f(-1, .35, 1.35));
    ret.push_back(Point3f(1, -.35, .65));
    ret.push_back(Point3f(1, -.35, 1.35));
    ret.push_back(Point3f(1, .35, .65));
    ret.push_back(Point3f(1, .35, 1.35));
    return ret;
}

//method to draw cube on video frame
void drawHouse(Mat& vid_frame, vector<Point2f>& projected) {
    Scalar black = Scalar(0, 0, 0);
    int size = 10;
    for (int i = 0; i < projected.size(); i++)
        circle(vid_frame, projected[i], 3, black, -1);
    line(vid_frame, projected[0], projected[1], black, size);
    line(vid_frame, projected[2], projected[3], black, size);
    line(vid_frame, projected[0], projected[2], black, size);
    line(vid_frame, projected[4], projected[6], black, size);
    line(vid_frame, projected[0], projected[4], black, size);
    line(vid_frame, projected[1], projected[3], black, size);
    line(vid_frame, projected[1], projected[5], black, size);
    line(vid_frame, projected[5], projected[7], black, size);
    line(vid_frame, projected[2], projected[6], black, size);
    line(vid_frame, projected[3], projected[7], black, size);
    line(vid_frame, projected[4], projected[5], black, size);
    line(vid_frame, projected[6], projected[7], black, size);
    line(vid_frame, projected[1], projected[8], black, size);
    line(vid_frame, projected[3], projected[8], black, size);
    line(vid_frame, projected[5], projected[8], black, size);
    line(vid_frame, projected[7], projected[8], black, size);
    //windows
    line(vid_frame, projected[9], projected[10], black, size);
    line(vid_frame, projected[9], projected[11], black, size);
    line(vid_frame, projected[10], projected[12], black, size);
    line(vid_frame, projected[11], projected[12], black, size);
    line(vid_frame, projected[13], projected[14], black, size);
    line(vid_frame, projected[13], projected[15], black, size);
    line(vid_frame, projected[14], projected[16], black, size);
    line(vid_frame, projected[15], projected[16], black, size);
    line(vid_frame, projected[17], projected[18], black, size);
    line(vid_frame, projected[17], projected[19], black, size);
    line(vid_frame, projected[18], projected[20], black, size);
    line(vid_frame, projected[19], projected[20], black, size);
    line(vid_frame, projected[21], projected[22], black, size);
    line(vid_frame, projected[21], projected[23], black, size);
    line(vid_frame, projected[22], projected[24], black, size);
    line(vid_frame, projected[23], projected[24], black, size);
}

vector<Point3f> cube_coors() {
    vector<Point3f> ret;
    for (int i = -1; i <= 1; i += 2) {
        for (int j = -1; j <= 1; j += 2) {
            for (int k = 0; k <= 2; k += 2) {
                ret.push_back(Point3f(i, j, k));
            }
        }
    }
    return ret;
}

//method to draw cube on video frame
void drawCube(Mat &vid_frame, vector<Point2f> &projected) {
    Scalar black = Scalar(0, 0, 0);
    int size = 10;
    for (int i = 0; i < projected.size(); i++)
        circle(vid_frame, projected[i], 3, black, -1);
    line(vid_frame, projected[0], projected[1], black, size);
    line(vid_frame, projected[2], projected[3], black, size);
    line(vid_frame, projected[0], projected[2], black, size);
    line(vid_frame, projected[4], projected[6], black, size);
    line(vid_frame, projected[0], projected[4], black, size);
    line(vid_frame, projected[1], projected[3], black, size);
    line(vid_frame, projected[1], projected[5], black, size);
    line(vid_frame, projected[5], projected[7], black, size);
    line(vid_frame, projected[2], projected[6], black, size);
    line(vid_frame, projected[3], projected[7], black, size);
    line(vid_frame, projected[4], projected[5], black, size);
    line(vid_frame, projected[6], projected[7], black, size);
}
//solves PnP problem to render cube on video of chessboard
void addCube()
{
    VideoCapture vid("chessboard.mov");
    Size size(7, 7);
    //preparing 3d object points in real world space (placeholder points)
    vector<Point3f> points;
    for (int i = -1; i < 2; i+=2) {
        for (int j = -1; j < 2; j+=2) {
            Point3f point = Point3f(i, j, 0);
            points.push_back(point);
        }
    }
    //setting up output video
    Size frame_size = Size(vid.get(CAP_PROP_FRAME_WIDTH), vid.get(CAP_PROP_FRAME_HEIGHT));
    vector<vector<Point2f>> calImgCorners; //will store 2d coordinates representing points in image space
    vector<vector<Point3f>> calObjCorners; //will store 3d coordinates representing points in real-world space
    int iter = 0;
    while (true) {
        //read frame of video into Mat object
        Mat init_frame;
        vid >> init_frame;
        //reads frame upside down, so making it right side up
        Mat vid_frame;
        flip(init_frame, vid_frame, 0);
        //end loop if frame is empty
        if (vid_frame.empty())
            break;
        vector<Point2f> corners; //will store 2d corner coordinates given by findChessBoardCorner
        Mat grayscale;
        //convert frame to grayscale
        cvtColor(vid_frame, grayscale, COLOR_BGR2GRAY);
        bool check = findChessboardCorners(grayscale, size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                                     + CALIB_CB_FAST_CHECK);
        if (check) { //only proceed if findChessBoardCorners found corners in frame
            vector<Point2f> central_corners; //stores 2d coordinates of four central corners on chessboard

            ////intended to increase accuracy of corners
            //cornerSubPix(grayscale, corners, Size(7, 7), Size(-1, -1),
            //	TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

            central_corners.push_back(corners[16]);
            central_corners.push_back(corners[18]);
            central_corners.push_back(corners[30]);
            central_corners.push_back(corners[32]);
            if (iter % 40 == 0) { //using all frames to calibrate camera is too slow, so only using every 40th frame for calibration
                calImgCorners.push_back(central_corners);
                calObjCorners.push_back(points);
            }
        }
        iter++;
    }
    Mat dist_coefs; //distortion coefficient matrix
    Mat cam_mat; //camera matrix
    vector<Mat> rot_vecs; //vector of rotation vectors
    vector<Mat> trans_vecs; //vector of translation vectors
    //calibrates camera based off of objCorners and imgCorners, calibrates cam_mat and dist_coefs
    calibrateCamera(calObjCorners, calImgCorners, frame_size, cam_mat, dist_coefs, rot_vecs, trans_vecs);
    //uses distortion coefficients to refine camera matrix and make it more accurate
    Mat newcam_mat = getOptimalNewCameraMatrix(cam_mat, dist_coefs, frame_size, 1, frame_size);
    Mat rot_vec; //translation vector
    Mat trans_vec; //rotation vector
    vector<Point3f> coors = house_coors();
    //loop through video again to draw cubes on every frame
    VideoCapture vid2("chessboard.mov");
    VideoWriter output = VideoWriter("result.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), vid.get(CAP_PROP_FPS), frame_size);
    while (true) {
        //read frame of video into Mat object
        Mat init_frame;
        vid2 >> init_frame;
        //frame read upside down, so flip frame
        Mat vid_frame;
        flip(init_frame, vid_frame, 0);
        //empty frame means video ended
        if (vid_frame.empty())
            break;
        vector<Point2f> corners; //stores 2d corners given by findChessBoardCorners
        bool check = findChessboardCorners(vid_frame, size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                                     + CALIB_CB_FAST_CHECK);
        if (check) { //only proceed if chessboardCorners found corners in frame
            vector<Point2f> central_corners;
            central_corners.push_back(corners[16]);
            central_corners.push_back(corners[18]);
            central_corners.push_back(corners[30]);
            central_corners.push_back(corners[32]);
            //solvePnPRansac outputs rotation vector and translation vector
            solvePnPRansac(points, central_corners, cam_mat, dist_coefs, rot_vec, trans_vec);
            vector<Point2f> projected; //stores points that will be projected from 3d to 2d
            projectPoints(coors, rot_vec, trans_vec, cam_mat, dist_coefs, projected);
            drawHouse(vid_frame, projected); //draw house on frame
        }
        output << vid_frame;
    }
    output.release();
}

int main()
{
    addCube();
    waitKey(0);
    return 0;
}
