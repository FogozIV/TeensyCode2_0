//
// Created by fogoz on 23/12/2024.
//

#ifndef TEENSYCODE2_0_MATRIX_H
#define TEENSYCODE2_0_MATRIX_H

#include <memory>
#include "cstdint"
#include "vector"

template<typename data_type>
class Matrix {
protected:
    uint16_t rows;
    uint16_t cols;
    std::shared_ptr<std::vector<data_type>> data;
    data_type& getDataEditable(uint16_t row, uint16_t col);

    std::vector<data_type>& getVectorEditable();

public:
    Matrix(uint16_t rows, uint16_t cols);

    Matrix(Matrix<data_type>&& copy);

    static Matrix<data_type> getEye(uint16_t size);

    const std::vector<data_type>& getVector() const;

    Matrix<data_type> operator*(const Matrix<data_type>& other) const;

    Matrix<data_type> operator*(const data_type& number) const;

    Matrix<data_type> operator+(const Matrix<data_type>& other) const;

    Matrix<data_type> operator+(const data_type& number) const;

    Matrix<data_type>& operator+=(const data_type& number);

    Matrix<data_type>& operator+=(const Matrix<data_type>& other);

    const data_type& getData(uint16_t row, uint16_t col) const;
};

template<typename data_type>
Matrix<data_type> &Matrix<data_type>::operator+=(const Matrix<data_type> &other) {
    if(other.rows == rows && other.cols == cols){
        for(int i = 0; i < getVector().size();i++){
            getVectorEditable().at(i) += other.getVector().at(i);
        }
    }
    return *this;
}


#include "Matrix.tpp"

#endif //TEENSYCODE2_0_MATRIX_H
