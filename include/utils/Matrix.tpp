//
// Created by fogoz on 23/12/2024.
//

#include <stdexcept>

#include "utils/Matrix.h"

template<typename data_type>
const data_type &Matrix<data_type>::getData(uint16_t row, uint16_t col) const {
    if (row >= rows || col >= cols) {
        throw std::out_of_range("Matrix index out of bounds");
    }
    return data->at(row * cols + col);
}

template<typename data_type>
size_t Matrix<data_type>::printTo(Print &p) const {
    size_t total = 0;
    for (uint16_t row = 0; row < rows; row++) {
        for (uint16_t col = 0; col < cols - 1; col++) {
            total+= p.print(getData(row, col));
            total += p.print(" ,");
        }
        total += p.print(getData(row, cols-1));
        if (row != rows-1) {
            total += p.print("\n");
        }
    }
    return total;
}


template<typename data_type>
Matrix<data_type>::Matrix(Matrix<data_type> &copy) {
    this->rows = copy.rows;
    this->cols = copy.cols;
    this->data = std::make_shared<std::vector<data_type>>(*copy.data.get());
}

template<typename data_type>
std::vector<data_type> &Matrix<data_type>::getVectorEditable() {
    return *(data.get());
}

template<typename data_type>
const std::vector<data_type> &Matrix<data_type>::getVector() const {
    return *(data.get());
}

template<typename data_type>
data_type &Matrix<data_type>::getDataEditable(uint16_t row, uint16_t col) {
    return data.get()->at(row * cols + col);
}

template<typename data_type>
Matrix<data_type> Matrix<data_type>::getEye(uint16_t size) {
    Matrix<data_type> matrix(size, size);
    for(int i = 0; i < size; i++){
        matrix.getDataEditable(i, i) = 1;
    }
    return matrix;
}

template<typename data_type>
Matrix<data_type>::Matrix(uint16_t rows, uint16_t cols) : rows(rows), cols(cols) {
    this->data = std::make_shared<std::vector<data_type>>(rows * cols, 0);
}

template<typename data_type>
Matrix<data_type>::Matrix(Matrix<data_type>&& copy) {
    this->rows = copy.rows;
    this->cols = copy.cols;
    this->data = copy.data;
}

template<typename data_type>
Matrix<data_type> Matrix<data_type>::operator*(const Matrix<data_type> &other) const {
    if(this->cols == other.rows){
        Matrix<data_type> result(this->rows, other.cols);
        for(int i = 0; i < result.rows; i++){
            for(int j = 0; j < result.cols; j++){
                for(int k = 0; k < this->cols; k++){
                    result.getDataEditable(i, j) += this->getData(i, k) * other.getData(k, j);
                }
            }
        }
        return result;
    }
    throw std::invalid_argument("Matrix::operator*");
}


template<typename data_type>
Matrix<data_type> Matrix<data_type>::operator*(const data_type &number) const {
    Matrix<data_type> result(this->rows, this->cols);
    for(int i = 0; i < this->rows * this->cols; i++)
        result.getVectorEditable().at(i) = this->getVector().at(i) * number;

    return result;
}


template<typename data_type>
Matrix<data_type> Matrix<data_type>::operator+(const Matrix<data_type> &other) const {
    if(this->rows == other.rows && this->cols == other.cols){
        Matrix<data_type> result(this->rows, this->cols);
        for(int i = 0; i < this->rows * this->cols; i++){
            result.getVectorEditable().at(i) = this->getVector().at(i) + other.getVector().at(i);
        }
        return result;
    }
    throw std::invalid_argument("Matrix::operator+");
}

template<typename data_type>
Matrix<data_type> Matrix<data_type>::operator+(const data_type &number) const {
    Matrix<data_type> result(this->rows, this->cols);
    for(int i = 0; i < this->rows * this->cols; i++){
        result.getVectorEditable().at(i) = this->getVector().at(i) + number;
    }
    return result;
}

template<typename data_type>
Matrix<data_type> &Matrix<data_type>::operator+=(const data_type &number) {
    for(int i = 0; i<getVector().size(); i++)
        getVectorEditable().at(i) += number;
    return *this;
}


template<typename data_type>
Matrix<data_type> Matrix<data_type>::getYRot(double angle) {
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    return Matrix(4,4,{cos_a, 0, sin_a, 0, 0, 1, 0, 0, -sin_a, 0, cos_a, 0, 0, 0, 0, 1});
}

template<typename data_type>
Matrix<data_type> Matrix<data_type>::getZRot(double angle) {
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    return Matrix(4,4,{cos_a, -sin_a, 0, 0, sin_a, cos_a, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template<typename data_type>
Matrix<data_type>::Matrix(uint16_t rows, uint16_t cols, std::vector<data_type> data) {
    if (data.size() != rows * cols) {
        throw std::invalid_argument("Size of data vector does not match matrix dimensions");
    }
    this->data = std::make_shared<std::vector<data_type>>(data);
}

template<typename data_type>
Matrix<data_type> Matrix<data_type>::getXRot(double angle) {
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    return Matrix(4,4,{1, 0, 0, 0, cos_a, -sin_a, 0, 0, 0, 0, sin_a, cos_a, 0, 0, 0, 1});
}

template<typename data_type>
Matrix<data_type> &Matrix<data_type>::operator+=(const Matrix<data_type> &other) {
    if(other.rows == rows && other.cols == cols){
        for(int i = 0; i < getVector().size();i++){
            getVectorEditable().at(i) += other.getVector().at(i);
        }
    }
    return *this;
}

template<typename data_type>
Matrix<data_type> Matrix<data_type>::getTranslation(double x, double y, double z) {
    return Matrix<data_type>(4,4, std::vector<double>({1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1}));
}
template<typename data_type>
Matrix<data_type> & Matrix<data_type>::operator*=(const Matrix<data_type> &other) {
    if(this->cols == other.rows){
        Matrix<data_type> temp(*this);
        this->cols = other.cols;
        this->data = std::make_shared<std::vector<data_type>>(this->cols * this->rows);
        for(int i = 0; i < this->rows; i++){
            for(int j = 0; j < this->cols; j++){
                for(int k = 0; k < temp.cols; k++){
                    this->getDataEditable(i, j) += temp.getData(i, k) * other.getData(k, j);
                }
            }
        }
    }
    return *this;
}

template<typename data_type>
Matrix<data_type> & Matrix<data_type>::operator*=(const data_type &number) {
    for(int i = 0; i < this->rows * this->cols; i++)
        this->getVectorEditable().at(i) *= number;
    return *this;
}
