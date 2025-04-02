#pragma once

#include <pistache/http.h>
#include <pistache/endpoint.h>
#include <nlohmann/json.hpp>
#include "aggregator.hpp"
#include <memory>

namespace baja {
namespace server {

class AggregationHttpHandler : public Pistache::Http::Handler {
public:
    HTTP_PROTOTYPE(AggregationHttpHandler)

    AggregationHttpHandler(std::shared_ptr<baja::aggregator::Aggregator> aggregator)
        : aggregator_(aggregator) {}

    void onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response) override;

private:
    std::shared_ptr<baja::aggregator::Aggregator> aggregator_;
};

} // namespace server
} // namespace baja
