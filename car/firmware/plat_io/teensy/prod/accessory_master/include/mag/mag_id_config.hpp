#include <map>
#include "TLx493D_inc.hpp"

namespace baja {
namespace mag {

const int MAG_COUNT = 4;

// typedef enum MagId {
//     Mag0 = 30,
//     Mag1 = 33,
//     Mag2 = 36,
//     Mag3 = 39,
// } MagId;

const std::map<int, TLx493D_IICAddressType_t> MAG_ADDR_MAP = {
    {0, TLx493D_IIC_ADDR_A0_e},
    {1, TLx493D_IIC_ADDR_A1_e},
    {2, TLx493D_IIC_ADDR_A2_e},
    {3, TLx493D_IIC_ADDR_A3_e},
};

}
}