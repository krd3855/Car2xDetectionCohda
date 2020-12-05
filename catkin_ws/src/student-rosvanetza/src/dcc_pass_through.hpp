#ifndef DCC_PASS_THROUGH_HPP_CE2GBHIP
#define DCC_PASS_THROUGH_HPP_CE2GBHIP

#include <vanetza/access/interface.hpp>
#include <vanetza/dcc/interface.hpp>

class DccPassThrough : public vanetza::dcc::RequestInterface
{
public:
    DccPassThrough(vanetza::access::Interface& ifc);

    void request(const vanetza::dcc::DataRequest& req, std::unique_ptr<vanetza::ChunkPacket> packet) override;

private:
    vanetza::access::Interface& access_;
};

#endif /* DCC_PASS_THROUGH_HPP_CE2GBHIP */

