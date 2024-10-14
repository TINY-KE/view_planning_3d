#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<std::vector<std::vector<double>>> bwdist3D(const std::vector<std::vector<std::vector<double>>>& map) {
    // Convert map to binary volume
    cv::Mat binaryVolume(map[0].size(), map[0][0].size(), map.size(), CV_8UC1);

    std::cout<<"szie: "<< binaryVolume.size()
    <<std::endl;

    for (int i = 0; i < map[0].size(); i++) {
        for (int j = 0; j < map[0][0].size(); j++) {
            for (int k = 0; k < map.size(); k++) {
                std::cout<<"grid, i:"<<i<<", j:"<<j<<", k:"<<k;
                std::cout<<",  = "<<map[k][i][j];
                std::cout<<", = "<<(map[k][i][j] > 0 ? 255 : 0)<<std::endl;
                binaryVolume.at<uchar>(i, j, k) = map[k][i][j] > 0 ? 255 : 0;
                std::cout<<"end"<<std::endl;
            }
        }
    }

    // Perform 3D distance transform
    cv::Mat distanceTransform;
    cv::distanceTransform(binaryVolume, distanceTransform, CV_DIST_L2, CV_DIST_MASK_3);

    // Convert distance transform result back to triple nested vector
    std::vector<std::vector<std::vector<double>>> result(distanceTransform.size[0], std::vector<std::vector<double>>(distanceTransform.size[1], std::vector<double>(distanceTransform.size[2], 0.0)));

    for (int i = 0; i < distanceTransform.size[0]; i++) {
        for (int j = 0; j < distanceTransform.size[1]; j++) {
            for (int k = 0; k < distanceTransform.size[2]; k++) {
                result[i][j][k] = distanceTransform.at<float>(i, j, k);
            }
        }
    }

    return result;
}

int main() {
    // Example usage
    std::vector<std::vector<std::vector<double>>> map = {
        {
            {0, 1, 0},
            {0, 0, 1},
            {1, 0, 0}
        },
        {
            {1, 0, 1},
            {0, 1, 0},
            {1, 0, 1}
        }
    };

    std::vector<std::vector<std::vector<double>>> map_dist = bwdist3D(map);

    // Print the resulting distance transform
    for (int i = 0; i < map_dist.size(); i++) {
        for (int j = 0; j < map_dist[0].size(); j++) {
            for (int k = 0; k < map_dist[0][0].size(); k++) {
                std::cout << map_dist[i][j][k] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}