//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_QUADENCODERIMPL_H
#define TEENSYCODE2_0_QUADENCODERIMPL_H

#include <QuadEncoder.h>
#include "AbstractEncoder.h"
class QuadEncoderImpl: public AbstractEncoder{
    QuadEncoder* quadEncoder;
    int32_t previous_count;
public:
    QuadEncoderImpl(uint8_t pinA, uint8_t pinB, uint8_t channel);

    virtual ~QuadEncoderImpl();

    int32_t getEncoderCount() const override;

    int32_t getDeltaCount() override;

    int32_t setEncoderCount(int32_t count) override;
};


#endif //TEENSYCODE2_0_QUADENCODERIMPL_H
