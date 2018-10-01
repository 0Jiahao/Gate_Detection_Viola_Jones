#include "Image.h"


Image::Image(std::vector<uint8_t> rgba, int width, int height){
    this->data = rgba;
    this->width = width;
    this->height = height;
}

std::vector<uint8_t > Image::getData() {
    return this->data;
}

int Image::getHeight() {
    return this->height;
}

int Image::getWidth() {
    return this->width;
}

bool Image::empty(){
    return data.empty();
}