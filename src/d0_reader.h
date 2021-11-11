#include <stdint.h>
#include <functional>

typedef std::function<void(uint8_t, uint8_t, uint8_t, uint64_t)> ObisResultFunc;

//typedef uint8_t ObisResultFunc;

class D0Reader {
    public:
        D0Reader(ObisResultFunc result_func);
        void process(char c);

    private:
        ObisResultFunc _result_func;
        uint8_t state;
        uint8_t currentObisCode[3];
        uint64_t value;
};