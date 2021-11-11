#include <d0_reader.h>

D0Reader::D0Reader(ObisResultFunc result_func) {
    _result_func = result_func;
    state = 0;
    }

void D0Reader::process(char c) {
    switch(state) {
        /* wait LF */
        case 0:
            if (c == '\n') {
                state = 1;
            }
            break;
        /* wait for : */
        case 1:
            if (c == ':') {
                state = 2;
                currentObisCode[0] = 0;
                currentObisCode[1] = 0;
                currentObisCode[2] = 0;
            } else if (c != '1' && c != '0' && c != '-') {
                state = 0;
            }
            break;
        /* read obis number */
        case 2:
        case 3:
        case 4:
            if (c == '.') {
                state++;
            } else if (state == 4 && c == '*') {
                state = 5;
            } else if (c < '0' || c > '9') {
                state = 0;
            } else {
                currentObisCode[state-2] *= 10;
                currentObisCode[state-2] += c - '0';
            }
            
            break;
        /* read 255 */
        case 5:
            if (c == '(') {
                state = 6;
                value = 0;
            } else if (c != '2' && c != '5') {
                state = 0;
            }
            break;
        /* read value */
        case 6:
            if (c == '*') {
                state = 0;
                _result_func(currentObisCode[0], currentObisCode[1], currentObisCode[2], value);
            } else if (c >= '0' && c <= '9') {
                value *= 10;
                value += c - '0';
            } else if (c == '.') {
                /* skip dot */
            } else {
                state = 0;
            }
    }
}