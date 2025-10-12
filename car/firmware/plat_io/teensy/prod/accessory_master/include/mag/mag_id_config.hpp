#include <map>
#include "tlv493D.hpp"

namespace baja {
namespace mag {

const int MAG_COUNT = 2;

// typedef enum MagId {
//     Mag0 = 30,
//     Mag1 = 33,
//     Mag2 = 36,
//     Mag3 = 39,
// } MagId;

const std::map<int, Tlv493d_Address> MAG_ADDR_MAP = {
    {0, TLV493D_ADDRESS1},
    {1, TLV493D_ADDRESS2},
    // {2, TLV493D_ADDRESS1},
    // {3, TLV493D_ADDRESS1},
};

}
}