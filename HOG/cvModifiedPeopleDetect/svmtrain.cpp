#include "cv.h"
#include <stdio.h>
#include <cxcore.h>
#include <cvaux.h>
#include "ml.h"
#include "highgui.h"
#define PI 3.14

char *cvtInt( char *str, int num)
{
        sprintf( str, "%d", num );
}

/*Histograms of Oriented Gradients or HOG features in combination with a
support vector machine have been successfully used for object Detection
 (most popularly pedestrian detection).
An Integral Histogram representation can be used for fast calculation of
Histograms of Oriented Gradients over arbitrary rectangular regions of the image.
The idea of an integral histogram is analogous to that of an integral image,
used by viola and jones for fast calculation of haar features for face detection.
 Mathematically,



where b represents the bin number of the histogram. This way the calculation of hog over any arbitrary rectangle in the image requires just 4*bins number of array references. For more details on integral histogram representation, please refer,

Integral Histogram The following demonstrates how such integral histogram
 can be calculated from an image and used for the calculation of hog features
 using the opencv computer vision library :
*/
/*Function to calculate the integral histogram*/
IplImage* doSobel(IplImage* src, int dx, int dy, int apertureSize)
{
    CvSize size = cvGetSize(src);

    IplImage* df = cvCreateImage(size, IPL_DEPTH_16S,1);
    cvSobel(src, df, dx, dy, apertureSize);

    IplImage* dest = cvCreateImage(size, IPL_DEPTH_8U,1);
    cvConvertScaleAbs(df,dest, 1, 0);

    cvReleaseImage(&df);
    return dest;
}
IplImage** calculateIntegralHOG(IplImage* in)
{

    /*Convert the input image to grayscale*/

    IplImage* img_gray = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U,1);
    cvCvtColor(in, img_gray, CV_BGR2GRAY);
    cvEqualizeHist(img_gray,img_gray);

    /* Calculate the derivates of the grayscale image in the x and y directions using a
    sobel operator and obtain 2 gradient images for the x and y directions*/

    IplImage *xsobel, *ysobel;
    xsobel = doSobel(img_gray, 1, 0, 3);
    ysobel = doSobel(img_gray, 0, 1, 3);
    cvReleaseImage(&img_gray);


    /* Create an array of 9 images (9 because I assume bin size 20 degrees and unsigned
    gradient ( 180/20 = 9), one for each bin which will have zeroes for all pixels,
    except for the pixels in the original image for which the gradient values correspond
    to the particular bin. These will be referred to as bin images. These bin images
    will be then used to calculate the integral histogram, which will quicken the
     calculation of HOG descriptors
    */

    IplImage** bins = (IplImage**) malloc(9 * sizeof(IplImage*));
    for (int i = 0; i < 9 ; i++) {
        bins[i] = cvCreateImage(cvGetSize(in), IPL_DEPTH_32F,1);
        cvSetZero(bins[i]);
    }


    /* Create an array of 9 images ( note the dimensions of the image, the cvIntegral()
    function requires the size to be that), to store the integral images calculated
    from the above bin images. These 9 integral images together constitute the integral
    histogram
    */

    IplImage** integrals = (IplImage**) malloc(9 * sizeof(IplImage*)); for (int i = 0; i < 9 ; i++) {
        integrals[i] = cvCreateImage(cvSize(in->width + 1, in->height + 1),
                                     IPL_DEPTH_64F,1);
    }

    /* Calculate the bin images. The magnitude and orientation of the gradient at each
    pixel is calculated using the xsobel and ysobel images.
    {Magnitude = sqrt(sq(xsobel) + sq(ysobel) ), gradient = itan (ysobel/xsobel) }.
    Then according to the orientation of the gradient, the value of the corresponding
    pixel in the corresponding image is set
    */


    int x, y;
    float temp_gradient, temp_magnitude;
    for (y = 0; y <in->height; y++) {

        /* ptr1 and ptr2 point to beginning of the current row in the xsobel and ysobel
        images respectively. ptrs[i] point to the beginning of the current rows in the
        bin images */

        float* ptr1 = (float*) (xsobel->imageData + y * (xsobel->widthStep));
        float* ptr2 = (float*) (ysobel->imageData + y * (ysobel->widthStep));
        float** ptrs = (float**) malloc(9 * sizeof(float*));
        for (int i = 0; i < 9 ;i++){
            ptrs[i] = (float*) (bins[i]->imageData + y * (bins[i]->widthStep));
        }

        /*For every pixel in a row gradient orientation and magnitude are calculated
        and corresponding values set for the bin images. */

        for (x = 0; x <in->width; x++) {

            /* if the xsobel derivative is zero for a pixel, a small value is added
            to it, to avoid division by zero. atan returns values in radians,
            which on being converted to degrees, correspond to values between -90 and 90
            degrees. 90 is added to each orientation, to shift the orientation values
            range from {-90-90} to {0-180}. This is just a matter of convention.
            {-90-90} values can also be used for the calculation. */

            if (ptr1[x] == 0){
                temp_gradient = ((atan(ptr2[x] / (ptr1[x] + 0.00001))) * (180/ PI)) + 90;
            }
            else{
                temp_gradient = ((atan(ptr2[x] / ptr1[x])) * (180 / PI)) + 90;
            }
            temp_magnitude = sqrt((ptr1[x] * ptr1[x]) + (ptr2[x] * ptr2[x]));

            /*The bin image is selected according to the gradient values.
           The corresponding pixel value is made equal to the gradient magnitude
           at that pixel in the corresponding bin image */

            if (temp_gradient <= 20) {
                ptrs[0][x] = temp_magnitude;
            }
            else if (temp_gradient <= 40) {
                ptrs[1][x] = temp_magnitude;
            }
            else if (temp_gradient <= 60) {
                ptrs[2][x] = temp_magnitude;
            }
            else if (temp_gradient <= 80) {
                ptrs[3][x] = temp_magnitude;
            }
            else if (temp_gradient <= 100) {
                ptrs[4][x] = temp_magnitude;
            }
            else if (temp_gradient <= 120) {
                ptrs[5][x] = temp_magnitude;
            }
            else if (temp_gradient <= 140) {
                ptrs[6][x] = temp_magnitude;
            }
            else if (temp_gradient <= 160) {
                ptrs[7][x] = temp_magnitude;
            }
            else {
                ptrs[8][x] = temp_magnitude;
            }
        }
    }

    cvReleaseImage(&xsobel);
    cvReleaseImage(&ysobel);

    /*Integral images for each of the bin images are calculated*/

    for (int i = 0; i <9 ; i++){
        cvIntegral(bins[i], integrals[i]);
    }

    for (int i = 0; i <9 ; i++){
        cvReleaseImage(&bins[i]);
    }

    /*The function returns an array of 9 images which consitute the integral histogram*/

    return (integrals);

}

/*The following demonstrates how the integral histogram calculated using the above
function can be used to calculate the histogram of oriented gradients for any
rectangular region in the image:

The following function takes as input the rectangular cell for which the
histogram of oriented gradients has to be calculated, a matrix hog_cell
of dimensions 1x9 to store the bin values for the histogram, the integral
histogram, and the normalization scheme to be used. No normalization is done
if normalization = -1 */


void calculateHOG_rect(CvRect cell, CvMat* hog_cell,IplImage** integrals, int normalization)
{


    /* Calculate the bin values for each of the bin of the histogram one by one */

    for (int i = 0; i < 9 ; i++)
    {

        float a =((double*)(integrals[i]->imageData + (cell.y)*(integrals[i]->widthStep)))[cell.x];
        float b = ((double*) (integrals[i]->imageData + (cell.y + cell.height)*(integrals[i]->widthStep)))[cell.x + cell.width];
        float c = ((double*) (integrals[i]->imageData + (cell.y)*(integrals[i]->widthStep)))[cell.x + cell.width];
        float d = ((double*) (integrals[i]->imageData + (cell.y + cell.height)*(integrals[i]->widthStep)))[cell.x];
        ((float*) hog_cell->data.fl)[i] = (a + b) - (c + d);

    }


    /*Normalize the matrix*/
    if (normalization != -1){
        cvNormalize(hog_cell, hog_cell, 1, 0, normalization);
    }

}


/* This function takes in a block as a rectangle and
calculates the hog features for the block by dividing
it into cells of size cell(the supplied parameter),
calculating the hog features for each cell using the
function calculateHOG_rect(...), concatenating the so
obtained vectors for each cell and then normalizing over
the concatenated vector to obtain the hog features for a
block */

void calculateHOG_block(CvRect block, CvMat* hog_block,
                        IplImage** integrals,CvSize cell, int normalization)
{
    int cell_start_x, cell_start_y;
    CvMat vector_cell;
    int startcol = 0;
    for (cell_start_y = block.y; cell_start_y <=
         block.y + block.height - cell.height;
    cell_start_y += cell.height)
    {
        for (cell_start_x = block.x; cell_start_x <=
             block.x + block.width - cell.width;
        cell_start_x += cell.width)
        {
            cvGetCols(hog_block, &vector_cell, startcol,
                      startcol + 9);

            calculateHOG_rect(cvRect(cell_start_x,
                                     cell_start_y, cell.width, cell.height),
                              &vector_cell, integrals, -1);

            startcol += 9;
        }
    }
    if (normalization != -1)
        cvNormalize(hog_block, hog_block, 1, 0,
                    normalization);
}

/* This function takes in a window(64x128 pixels,
but can be easily modified for other window sizes)
and calculates the hog features for the window. It
can be used to calculate the feature vector for a
64x128 pixel image as well. This window/image is the
training/detection window which is used for training
or on which the final detection is done. The hog
features are computed by dividing the window into
overlapping blocks, calculating the hog vectors for
each block using calculateHOG_block(...) and
concatenating the so obtained vectors to obtain the
hog feature vector for the window*/

CvMat* calculateHOG_window(IplImage** integrals,
                           CvRect window, int normalization)
{

    /*A cell size of 8x8 pixels is considered and each
block is divided into 2x2 such cells (i.e. the block
is 16x16 pixels). So a 64x128 pixels window would be
divided into 7x15 overlapping blocks*/

    int block_start_x, block_start_y, cell_width = 8;
    int cell_height = 8;
    int block_width = 2, block_height = 2;

    /* The length of the feature vector for a cell is
9(since no. of bins is 9), for block it would  be
9*(no. of cells in the block) = 9*4 = 36. And the
length of the feature vector for a window would be
36*(no. of blocks in the window */

    CvMat* window_feature_vector = cvCreateMat(1,
   ((((window.width - cell_width * block_width)/ cell_width) + 1) * (((window.height -
    cell_height * block_height) / cell_height) + 1)) * 36, CV_32FC1);

    CvMat vector_block;
    int startcol = 0;
    for (block_start_y = window.y; block_start_y <= window.y + window.height - cell_height * block_height;
         block_start_y += cell_height)
    {
        for (block_start_x = window.x; block_start_x<= window.x + window.width - cell_width * block_width;
             block_start_x += cell_width)
        {
            cvGetCols(window_feature_vector, &vector_block, startcol, startcol + 36);

            calculateHOG_block(cvRect(block_start_x,
             block_start_y, cell_width * block_width, cell_height * block_height),
                               &vector_block, integrals,
                               cvSize(cell_width, cell_height), normalization);

            startcol += 36;
        }
    }
    return (window_feature_vector);
}
/*This function takes in a the path and names of 64x128 pixel images, the size of the cell to be
used for calculation of hog features(which should be 8x8 pixels, some modifications will have to be
done in the code for a different cell size, which could be easily done once the reader understands
how the code works), a default block size of 2x2 cells has been considered and the window size
parameter should be 64x128 pixels (appropriate modifications can be easily done for other say
64x80 pixel window size). All the training images are expected to be stored at the same location and
the names of all the images are expected to be in sequential order like a1.jpg, a2.jpg, a3.jpg ..
and so on or a(1).jpg, a(2).jpg, a(3).jpg ... The explanation of all the parameters below will make
clear the usage of the function. The synopsis of the function is as follows :

prefix : it should be the path of the images, along with the prefix in the image name for
example if the present working directory is /home/saurabh/hog/ and the images are in
/home/saurabh/hog/images/positive/ and are named like pos1.jpg, pos2.jpg, pos3.jpg ....,
then the prefix parameter would be "images/positive/pos" or if the images are
named like pos(1).jpg, pos(2).jpg, pos(3).jpg ... instead, the prefix parameter
would be "images/positive/pos("suffix : it is the part of the name of the image
files after the number for example for the above examples it would be ".jpg" or ").jpg"

cell   : it should be CvSize(8,8), appropriate changes need to be made for other cell sizes
window : it should be CvSize(64,128), appropriate changes need to be made for other window sizes
number_samples : it should be equal to the number of training images, for example if the
training images are pos1.jpg, pos2.jpg ..... pos1216.jpg, then it should be 1216

start_index : it should be the start index of the images' names for example for the above case it
should be 1 or if the images were named like pos1000.jpg, pos1001.jpg, pos1002.jpg
.... pos2216.jpg, then it should be 1000

end_index : it should be the end index of the images' name for example for the above cases it should be 1216 or 2216
savexml   : if you want to store the extracted features, then you can pass to it the name of an xml
file to which they should be saved

normalization : the normalization scheme to be used for computing the hog features, any of the
opencv schemes could be passed or -1 could be passed if no normalization is to be done
*/

CvMat* train_64x128(char *prefix, char *suffix, CvSize cell,CvSize window, int number_samples, int start_index,
int end_index, char *savexml = NULL, int canny = 0,int block = 1, int normalization = 4)
{

        char filename[50] = "\0", number[8];
        int prefix_length;
        prefix_length = strlen(prefix);
        int bins = 9;

        /* A default block size of 2x2 cells is considered */

        int block_width = 2, block_height = 2;

        /* Calculation of the length of a feature vector for
        an image (64x128 pixels)*/

        int feature_vector_length;
        feature_vector_length = (((window.width - cell.width * block_width)/ cell.width) + 1) *(((window.height - cell.height * block_height)/cell.height) + 1) * 36;

        /* Matrix to store the feature vectors for
        all(number_samples) the training samples */

        CvMat* training = cvCreateMat(number_samples,feature_vector_length, CV_32FC1);

        CvMat row;
        CvMat* img_feature_vector;
        IplImage** integrals;
        int i = 0, j = 0;

        printf("Beginning to extract HoG features from positive images\n");

        strcat(filename, prefix);

        /* Loop to calculate hog features for each
        image one by one */

        for (i = start_index; i <= end_index; i++)
        {
                cvtInt(number, i);
                strcat(filename, number);
                strcat(filename, suffix);
                IplImage* img = cvLoadImage(filename);

                /* Calculation of the integral histogram for
                fast calculation of hog features*/

                integrals = calculateIntegralHOG(img);
                cvGetRow(training, &row, j);
                img_feature_vector = calculateHOG_window(integrals, cvRect(0, 0, window.width, window.height), normalization);
                cvCopy(img_feature_vector, &row);
                j++;
                printf("%s\n", filename);
                filename[prefix_length] = '\0';
                for (int k = 0; k < 9; k++)
                {
                        cvReleaseImage(&integrals[k]);
                }
        }
        if (savexml != NULL)
        {
                cvSave(savexml, training);
        }

        return training;
}

/* This function is almost the same as train_64x128(...), except the fact that it can
take as input images of bigger sizes and generate multiple samples out of a single
image.

It takes 2 more parameters than train_64x128(...), horizontal_scans and
vertical_scans to determine how many samples are to be generated from the image. It
generates horizontal_scans x vertical_scans number of samples. The meaning of rest of the
parameters is same.

For example for a window size of 64x128 pixels, if a 320x240 pixel image is
given input with horizontal_scans = 5 and vertical scans = 2, then it will generate to
samples by considering windows in the image with (x,y,width,height) as
(0,0,64,128),
(64,0,64,128), (128,0,64,128), .....,
(0,112,64,128), (64,112,64,128) .....
(256,112,64,128)

The function takes non-overlapping windows from the image except the last row and last
column, which could overlap with the second last row or second last column. So the values
of horizontal_scans and vertical_scans passed should be such that it is possible to perform
that many scans in a non-overlapping fashion on the given image. For example horizontal_scans
= 5 and vertical_scans = 3 cannot be passed for a 320x240 pixel image as that many vertical scans
are not possible for an image of height 240 pixels and window of height 128 pixels.
*/

CvMat* train_large(char *prefix, char *suffix,CvSize cell, CvSize window, int number_images,
int horizontal_scans, int vertical_scans,int start_index, int end_index,
char *savexml = NULL, int normalization = 4)
{
        char filename[50] = "\0", number[8];
        int prefix_length;
        prefix_length = strlen(prefix);
        int bins = 9;

        /* A default block size of 2x2 cells is considered */

        int block_width = 2, block_height = 2;

        /* Calculation of the length of a feature vector for
        an image (64x128 pixels)*/

        int feature_vector_length;
        feature_vector_length = (((window.width -
        cell.width * block_width) / cell.width) + 1) *
        (((window.height - cell.height * block_height)
        / cell.height) + 1) * 36;

        /* Matrix to store the feature vectors for
        all(number_samples) the training samples */

        CvMat* training = cvCreateMat(number_images
        * horizontal_scans * vertical_scans,
        feature_vector_length, CV_32FC1);

        CvMat row;
        CvMat* img_feature_vector;
        IplImage** integrals;
        int i = 0, j = 0;
        strcat(filename, prefix);

        printf("Beginning to extract HoG features from negative images\n");

        /* Loop to calculate hog features for each
        image one by one */

        for (i = start_index; i <= end_index; i++)
        {
                cvtInt(number, i);
                strcat(filename, number);
                strcat(filename, suffix);
                IplImage* img = cvLoadImage(filename);
                integrals = calculateIntegralHOG(img);
                for (int l = 0; l < vertical_scans - 1; l++)
                {
                        for (int k = 0; k < horizontal_scans - 1; k++)
                        {
                                cvGetRow(training, &row, j);
                                img_feature_vector = calculateHOG_window(
                                integrals, cvRect(window.width * k,
                                window.height * l, window.width,
                                window.height), normalization);

                                cvCopy(img_feature_vector, &row);
                                j++;
                        }

                        cvGetRow(training, &row, j);

                        img_feature_vector = calculateHOG_window(
                        integrals, cvRect(img->width - window.width,
                        window.height * l, window.width,
                        window.height), normalization);

                        cvCopy(img_feature_vector, &row);
                        j++;
                }

                for (int k = 0; k < horizontal_scans - 1; k++)
                {
                        cvGetRow(training, &row, j);

                        img_feature_vector = calculateHOG_window(
                        integrals, cvRect(window.width * k,
                        img->height - window.height, window.width,
                        window.height), normalization);

                        cvCopy(img_feature_vector, &row);
                        j++;
                }
                cvGetRow(training, &row, j);

                img_feature_vector = calculateHOG_window(integrals,
                cvRect(img->width - window.width, img->height -
                window.height, window.width, window.height),
                normalization);

                cvCopy(img_feature_vector, &row);
                j++;

                printf("%s\n", filename);
                filename[prefix_length] = '\0';
                for (int k = 0; k < 9; k++)
                {
                        cvReleaseImage(&integrals[k]);
                }

                cvReleaseImage(&img);

        }

        printf("%d negative samples created \n",
        training->rows);

        if (savexml != NULL)
        {
        cvSave(savexml, training);
        printf("Negative samples saved as %s\n",
        savexml);
        }

        return training;

}


/* This function trains a linear support vector machine for object classification. The synopsis is
as follows :
pos_mat : pointer to CvMat containing hog feature vectors for positive samples. This may be
NULL if the feature vectors are to be read from an xml file

neg_mat : pointer to CvMat containing hog feature vectors for negative samples. This may be
NULL if the feature vectors are to be read from an xml file

savexml : The name of the xml file to which the learnt svm model should be saved

pos_file: The name of the xml file from which feature vectors for positive samples are to be read.
It may be NULL if feature vectors are passed as pos_mat

neg_file: The name of the xml file from which feature vectors for negative samples are to be read.
It may be NULL if feature vectors are passed as neg_mat
*/


void trainSVM(CvMat* pos_mat, CvMat* neg_mat, char *savexml,char *pos_file = NULL, char *neg_file = NULL)
{


        /* Read the feature vectors for positive samples */
        if (pos_file != NULL)
        {
                printf("positive loading...\n");
                pos_mat = (CvMat*) cvLoad(pos_file);
                printf("positive loaded\n");
        }

        /* Read the feature vectors for negative samples */
        if (neg_file != NULL)
        {
                neg_mat = (CvMat*) cvLoad(neg_file);
                printf("negative loaded\n");
        }

        int n_positive, n_negative;
        n_positive = pos_mat->rows;
        n_negative = neg_mat->rows;
        int feature_vector_length = pos_mat->cols;
        int total_samples;
        total_samples = n_positive + n_negative;

        CvMat* trainData = cvCreateMat(total_samples,
        feature_vector_length, CV_32FC1);

        CvMat* trainClasses = cvCreateMat(total_samples,
        1, CV_32FC1 );

        CvMat trainData1, trainData2, trainClasses1,
        trainClasses2;

        printf("Number of positive Samples : %d\n",
        pos_mat->rows);

        /*Copy the positive feature vectors to training
        data*/

        cvGetRows(trainData, &trainData1, 0, n_positive);
        cvCopy(pos_mat, &trainData1);
        cvReleaseMat(&pos_mat);

        /*Copy the negative feature vectors to training
        data*/

        cvGetRows(trainData, &trainData2, n_positive,
        total_samples);

        cvCopy(neg_mat, &trainData2);
        cvReleaseMat(&neg_mat);

        printf("Number of negative Samples : %d\n",
        trainData2.rows);

        /*Form the training classes for positive and
        negative samples. Positive samples belong to class
        1 and negative samples belong to class 2 */

        cvGetRows(trainClasses, &trainClasses1, 0, n_positive);
        cvSet(&trainClasses1, cvScalar(1));

        cvGetRows(trainClasses, &trainClasses2, n_positive,
        total_samples);

        cvSet(&trainClasses2, cvScalar(2));


        /* Train a linear support vector machine to learn from
        the training data. The parameters may played and
        experimented with to see their effects*/

        CvSVM svm(trainData, trainClasses, 0, 0,
        CvSVMParams(CvSVM::C_SVC, CvSVM::LINEAR, 0, 0, 0, 2,
        0, 0, 0, cvTermCriteria(CV_TERMCRIT_EPS,0, 0.01)));

        printf("SVM Training Complete!!\n");

        /*Save the learnt model*/

        if (savexml != NULL)
        {
                svm.save(savexml);
        }
        cvReleaseMat(&trainClasses);
        cvReleaseMat(&trainData);

}
