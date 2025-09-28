#include <map>

namespace baja {
namespace mag {

const int MAG_COUNT = 4;

// typedef enum MagId {
//     Mag0 = 30,
//     Mag1 = 33,
//     Mag2 = 36,
//     Mag3 = 39,
// } MagId;

const std::map<int, uint8_t> MAG_ADDR_MAP = {
    {0, 0x18},
    {1, 0x19},
    {2, 0x1A},
    {3, 0x1B},
};

}
}