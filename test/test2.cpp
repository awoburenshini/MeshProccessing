#include <iostream>
#include <tinytiffreader.h>
#include <memory>
#include <common.h>

using namespace std;

typedef Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXR16u;

int main()
{

    std::string file = "D:/Mesh_process/data/terrain.tiff";

    // TinyTIFFReaderFile *tiffr = NULL;
    // tiffr = TinyTIFFReader_open(file.c_str());
    TinyTIFFReaderFile *tiffr = TinyTIFFReader_open(file.c_str());

    if (!tiffr)
    {
        std::cout << "    ERROR reading (not existent, not accessible or no TIFF file)\n";
    }
    else
    {
        const uint32_t width = TinyTIFFReader_getWidth(tiffr);
        const uint32_t height = TinyTIFFReader_getHeight(tiffr);
        const uint16_t samples = TinyTIFFReader_getSamplesPerPixel(tiffr);
        const uint16_t bitsPerSample = TinyTIFFReader_getBitsPerSample(tiffr, 0);
        std::cout << width << " " << height << " " << samples << " " << bitsPerSample << std::endl;
        uint16_t *image = (uint16_t *)calloc(width * height, bitsPerSample / 8);
        MatrixXR16u hmap;
        hmap.resize(height, width);
        // Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> heightMap;
        // heightMap.resize(height, width);
        // TinyTIFFReader_getSampleData(tiffr, heightMap.data(), 0);
        TinyTIFFReader_getSampleData(tiffr, hmap.data(), 0);

        const char* descrption = TinyTIFFReader_getImageDescription(tiffr);

        std::cout << descrption << std::endl;
        // for(int h = 0; h < height; ++h){

        //     for(int w = 0; w < width; ++w){
        //         std::cout<< image[w + h * width]<< " ";
        //     }
        //     std::cout<< std::endl;
        // }
    }

    TinyTIFFReader_close(tiffr);
    std::cout << "hello" << std::endl;

    return 0;
}