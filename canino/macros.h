
#pragma once

#define OR_BITS2(b1, b2) (_BV(b1) | _BV(b2))
#define OR_BITS3(b1, b2, b3) (_BV(b1) | _BV(b2) | _BV(b3))
#define OR_BITS4(b1, b2, b3, b4) (_BV(b1) | _BV(b2) | _BV(b3) | _BV(b4))

#define RESET(x)        _XRS(x)
#define SET(x)          _XS(x)
#define TOGGLE(x)       _XT(x)
#define SET_OUTPUT(x)   _XSO(x)
#define SET_INPUT(x)    _XSI(x)
#define IS_SET(x)       _XR(x)

#define PORT(x)         _port2(x)
#define DDR(x)          _ddr2(x)
#define PIN(x)          _pin2(x)

#define _XRS(x, y)      PORT(x) &= ~(1 << y)
#define _XS(x, y)       PORT(x) |= (1 << y)
#define _XT(x, y)       PORT(x) ^= (1 << y)

#define _XSO(x, y)      DDR(x) |= (1 << y)
#define _XSI(x, y)      DDR(x) &= ~(1 << y)

#define _XR(x, y)       ((PIN(x) & (1 << y)) != 0)

#define _port2(x)       PORT ## x
#define _ddr2(x)        DDR ## x
#define _pin2(x)        PIN ## x

