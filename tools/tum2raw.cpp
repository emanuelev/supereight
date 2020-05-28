#include "lodepng.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <experimental/filesystem>
#include <fstream>
#include <unistd.h>
#include <vector>

namespace fs = std::experimental::filesystem;

struct uint2 {
    unsigned x, y;
};

void get_images(std::vector<std::string>& depth_images,
    std::vector<std::string>& rgb_images,
    const std::string& associations_file) {
    auto associations = std::ifstream(associations_file);

    if (!associations) {
        std::fprintf(stderr, "Error: could not open associations file \"%s\"\n",
            associations_file.c_str());
        std::exit(1);
    }

    std::string depth_ts, depth_file, rgb_ts, rgb_file;
    while (associations >> depth_ts >> depth_file >> rgb_ts >> rgb_file) {
        depth_images.push_back(depth_file);
        rgb_images.push_back(rgb_file);
    }

    std::printf("Found %lu image pairs:\n", depth_images.size());
    for (auto i = 0ul; i < std::min(depth_images.size(), 3ul); ++i) {
        std::printf(
            "    %s - %s\n", depth_images[i].c_str(), rgb_images[i].c_str());
    }

    if (depth_images.size() > 3) {
        std::printf("    ...\n");
        std::printf("    %s - %s\n", depth_images.back().c_str(),
            rgb_images.back().c_str());
    };
}

void scale_depth(std::vector<unsigned char>& depth) {
    auto* data = reinterpret_cast<uint16_t*>(depth.data());
    for (auto i = 0ul; i < depth.size() / 2; ++i) {
        data[i] = __builtin_bswap16(data[i]) / 5;
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "Usage: %s <TUM dataset path>\n", argv[0]);
        std::exit(1);
    }

    if (chdir(argv[1]) != 0) {
        std::fprintf(stderr, "Error: could not find directory %s\n", argv[1]);
        std::exit(1);
    };

    auto depth_images = std::vector<std::string>();
    auto rgb_images   = std::vector<std::string>();
    get_images(depth_images, rgb_images, "associations.txt");

    if (depth_images.size() != rgb_images.size()) {
        std::fprintf(stderr, "Error: depth/rgb images size mismatch\n");
        std::exit(1);
    }

    if (depth_images.size() == 0) {
        std::fprintf(
            stderr, "Error: no depth/rgb images in associations file\n");
        std::exit(1);
    }

    std::printf("\n");

    auto depth_image = std::vector<unsigned char>();
    auto rgb_image   = std::vector<unsigned char>();

    auto dim = uint2();

    auto output = std::ofstream("scene.raw", std::ios::binary);
    for (auto i = 0ul; i < depth_images.size(); ++i) {
        std::printf("\r[%4lu/%4lu] writing %s - %s to scene.raw", i + 1,
            depth_images.size(), depth_images[i].c_str(),
            rgb_images[i].c_str());
        std::fflush(stdout);

        depth_image.clear();
        lodepng::decode(
            depth_image, dim.x, dim.y, depth_images[i], LCT_GREY, 16);

        scale_depth(depth_image);

        output.write(reinterpret_cast<char*>(&dim), sizeof(dim));
        output.write(reinterpret_cast<char*>(depth_image.data()),
            dim.x * dim.y * sizeof(uint16_t));

        rgb_image.clear();
        lodepng::decode(rgb_image, dim.x, dim.y, rgb_images[i], LCT_RGB, 8);

        output.write(reinterpret_cast<char*>(&dim), sizeof(dim));
        output.write(
            reinterpret_cast<char*>(rgb_image.data()), dim.x * dim.y * 3);
    }

    std::printf("\n");
}
