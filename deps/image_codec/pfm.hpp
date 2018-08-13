#ifndef PFM_HPP_
#define PFM_HPP_

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <cstdio>

namespace pfm {
class PFM{
    template <typename T>
    using Grid2D = std::vector< std::vector<T> >;

public:
    Grid2D<float> Data_;
private:
    int height_ = 0;
    int width_ = 0;
    float endianness_ = -1.0;

public:
    PFM(std::string path) {
        ReadPfmFile(path);
    }
    
    // TODO: unfinished
    PFM(float* data, int height, int width)
        : height_(height), width_(width) {

    }

    Grid2D<float>* ReadPfmFile(std::string path) {
        Data_.clear();
        std::ifstream file(path, std::ios::in | std::ios::binary);
        if (!file.is_open()) {
            std::cout << "FAILED to open the file!!!" <<std::endl;
        } else {
            
            // get information
            std::string line;
            getline(file, line);
            if (!((line == "Pf") || (line == "PF"))) {
                std::cout << "NOT pfm format!!!" <<std::endl;
                file.close();
            } else {
                getline(file, line);
                std::istringstream iss_wd(line);
                iss_wd >> width_ >> height_;
                getline(file, line);
                std::istringstream iss_e(line);
                iss_e >> endianness_;

                // get data buffer
                file.seekg(0, file.end);
                int length = file.tellg();
                int pos = length - width_ * height_ * sizeof(float);
                file.seekg(pos);
                std::vector<char> buffer((std::istreambuf_iterator<char>(file)),
                    (std::istreambuf_iterator<char>()));
                file.close();

                //get chars back into float
                union {
                    float f;
                    char u8[sizeof(float)];
                } pixel;
                std::vector<float> temp;
                if (endianness_ != -1.0f) {
                    // TODO: deal with big endianness
                    std::cout << "TODO: NOT little endian!!!" << std::endl;
                } else {
                    for (int h = 0; h < height_; h++) {
                        Data_.push_back(temp);
                        for (int w = 0; w < width_; w++) {
                            for (int s = 0; s < sizeof(float); s++) {
                                pixel.u8[s] = buffer[(h * width_ + w) * sizeof(float) + s];
                            }
                            Data_[h].push_back(pixel.f);
                        }
                    }

                }
            }
        }
        return &Data_;
    }

    void SaveAsPpm(std::string path, float max_dist = 255.f) {
        if (RemoveIfExists(path + ".ppm")) {
            std::cout << "File" + path + ".ppm exists already! New data will be appended." << std::endl;
        }
        std::ofstream file(path + ".ppm", std::ios::app);
        int max_pix_val = 255;
        file << "P3\n" << width_ << " " << height_ << "\n" << max_pix_val << "\n";
        //R,G,B
        uint8_t pixel[3];
        Clamp(max_dist);
        // pfm is up-side-down, while ppm is not
        for (int h = height_ - 1; h >= 0; h--) {
            for (int w = 0; w < width_; w++) {
                pixel[0] = static_cast<int>(Data_[h][w] * max_pix_val / max_dist);
                pixel[1] = pixel[2] = pixel[0];
                file << std::setw(3) << std::to_string(pixel[0]) << " "
                    << std::setw(3) << std::to_string(pixel[1]) << " "
                    << std::setw(3) << std::to_string(pixel[2]) << " ";
	        }
            file << "\n";
        }
        file.close();
    }
    
#ifdef INCLUDE_BITMAP_IMAGE_HPP
    // TODO: unfinished
    void SaveAsBmp(std::string path, float max_dist = 255.f) {
        
    }

#endif // INCLUDE_BITMAP_IMAGE_HPP
    
    // TODO: unfinished
    void SaveAsPfm(std::string path) {

    }

    void Clamp(float max = 255.f) {
        for (int h = 0; h < height_; h++) {
            for (int w = 0; w < width_; w++) {
                if (Data_[h][w] > max) {
                    Data_[h][w] = max;
                }
            }
        }
    }

    float GetPixel(int y_pos, int x_pos) {
        if((y_pos >= height_) || (x_pos >=  width_)) {
            std::cout << "NO such a pixel in a (" << width_ << ", " 
                << height_ << ") map!" << std::endl;
            return 0;
        }
        return Data_[y_pos][x_pos];
    }

    std::vector<float> GetRow(int y_pos) {
        std::vector<float> row;
        if (y_pos >= height_) {
            std::cout << "No such a row, height = " << height_
                << ", return the last row(top) instead!" << std::endl;
            y_pos = height_;
            std::cout << y_pos << std::endl;
        }
        return row = Data_[y_pos];
    }

    std::vector<float> GetCol(int x_pos) {
        std::vector<float> col;
        if (x_pos >= width_) {
            std::cout << "No such a column, width = " << width_
                << ", return the last column(right) instead!" << std::endl;
            x_pos = width_;
        }
        for (int i = 0; i < height_; i++) {
            col.push_back(Data_[i][x_pos]);
        }
        return col;
    }

    Grid2D<float> GetCols(int x_pos, int d) {
        Grid2D<float> cols;
        if (x_pos >= width_) {
            std::cout << "No such a column, width = " << width_
                << ", return the last column(right) instead!" << std::endl;
                cols.push_back(GetCol(width_));
                return cols;
        } else if (x_pos + d > width_) {
            std::cout << "x_pos + d has exceeded the width of the image data. "
                << "Return (width - x_pos) = " << width_ - x_pos << " columns instead." << std::endl;
            d = width_ - x_pos;
        }
        for (int i = 0; i < d; i++) {
            cols.push_back(GetCol(x_pos + i));
        }
        return cols;
    }

    int GetHeight() {
        return height_;
    }
    int GetWidth() {
        return width_;
    }
    


private:
    bool isLittleEndian() {
        int intval = 1;
        unsigned char *uval = reinterpret_cast<unsigned char *>(&intval);
        return uval[0] == 1;
    }
    
    bool RemoveIfExists(std::string filename) {
        std::fstream file(filename);
        if(file.is_open()) {
            file.close();
            std::cout << "File" + filename + "exists already! All of its existing data will be overwritten!" << std::endl;
            return std::remove(filename.c_str());
        } else {
            file.close();
            return 0;
        }
    }
}; // class PFM
} // namespace pfm

#endif //PFM_HPP_
