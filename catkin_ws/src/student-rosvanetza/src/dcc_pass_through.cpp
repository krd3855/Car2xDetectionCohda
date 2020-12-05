#include "dcc_pass_through.hpp"
#include <vanetza/access/data_request.hpp>
#include <vanetza/dcc/data_request.hpp>
#include <vanetza/dcc/mapping.hpp>

using namespace vanetza;

DccPassThrough::DccPassThrough(access::Interface& ifc) :
    access_(ifc)
{
}

void DccPassThrough::request(const dcc::DataRequest& dcc_req, std::unique_ptr<ChunkPacket> packet)
{
        access::DataRequest access_req;
        access_req.source_addr = dcc_req.source;
        access_req.destination_addr = dcc_req.destination;
        access_req.ether_type = dcc_req.ether_type;
        access_req.access_category = map_profile_onto_ac(dcc_req.dcc_profile);
        access_.request(access_req, std::move(packet));
}
