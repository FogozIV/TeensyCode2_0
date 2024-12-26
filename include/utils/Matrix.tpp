//
// Created by fogoz on 23/12/2024.
//

#include "utils/Matrix.h"

template<typename data_type>
const data_type &Matrix<data_type>::getData(uint16_t row, uint16_t col) const {
    return data->at(row * cols + col);
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
    copy.data = nullptr;
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
    return Matrix<data_type>(0, 0);
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
    return Matrix<data_type>(0, 0);
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
