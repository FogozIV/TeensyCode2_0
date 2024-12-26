//
// Created by fogoz on 22/12/2024.
//

#ifndef TEENSYCODE2_0_ABSTRACTENCODER_H
#define TEENSYCODE2_0_ABSTRACTENCODER_H
#include "cstdint"

class AbstractEncoder {
public:
    /**
     * @return the current encoder count
     */
    virtual int64_t getEncoderCount() const = 0;
    /**
     * This method can be use to get the delta count from the last call of this function
     * Its behavior should be that if the setEncoderCount is used then the delta should continue to work
     * @return the number of count since last call
     */
    virtual int64_t getDeltaCount() = 0;

    /**
     * This method allows to set the count
     * @param count the count we want to set
     * @return the previous count
     */
    virtual int64_t setEncoderCount(int64_t count) = 0;
};


#endif //TEENSYCODE2_0_ABSTRACTENCODER_H
