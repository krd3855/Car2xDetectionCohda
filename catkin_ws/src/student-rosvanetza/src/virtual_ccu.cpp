#include "cmc-ccu.pb.h"
#include "virtual_ccu.hpp"
#include <ros/ros.h>
#include <fstream>

class Radio;
class RadioMedium;

static const char this_node[] = "virtual_ccu";
static const std::string broadcast_address { -1, -1, -1, -1, -1, -1 };

Radio::Radio(RadioMedium* medium, unsigned id, boost::asio::io_service& io,
    boost::asio::ip::address addr, unsigned short cmd_port, unsigned short fwd_port) :
        radio_medium_(medium), id_(id),
        cmd_socket_(io, boost::asio::ip::udp::endpoint(addr, cmd_port)),
        fwd_socket_(io, boost::asio::ip::udp::v4()),
        fwd_endpoint_(boost::asio::ip::address_v4::from_string("239.67.77.67"), fwd_port)
    {
        cmd_socket_.async_receive_from(boost::asio::buffer(cmd_buffer_), cmd_sender_,
                boost::bind(&Radio::handleCommandReception, this,
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

void Radio::receive(const LinkLayerReception& reception)
{
    if (!filter_unicast_destination_ || reception.destination() == broadcast_address || reception.destination() == mac_address)
    {
        GossipMessage gossip;
        gossip.mutable_linklayer_rx()->CopyFrom(reception);
        gossip.SerializeToString(&fwd_gossip_);
        fwd_socket_.send_to(boost::asio::buffer(fwd_gossip_), fwd_endpoint_);
    }
}

void Radio::handleCommandReception(const boost::system::error_code& ec, std::size_t bytes)
{
    if (!ec && bytes > 0)
    {
        CommandRequest request;
        if (request.ParseFromArray(cmd_buffer_.data(), bytes))
        {
            CommandResponse response;
            response.set_status(CommandResponse::UNKNOWN);

            if (request.has_linklayer_tx())
            {
                ROS_INFO_NAMED(this_node, "LinkLayerTransmission received by CCU %u", id_);
                radio_medium_->disseminate(request.linklayer_tx(), this);
                response.set_status(CommandResponse::SUCCESS);
            }
            else if (request.has_radio_cfg())
            {
                ROS_INFO_NAMED(this_node, "RadioConfiguration received by CCU %u", id_);
                if (request.radio_cfg().has_address())
                {
                    mac_address = request.radio_cfg().address();
                }
                if (request.radio_cfg().has_filter_unicast_destination())
                {
                    filter_unicast_destination_ = request.radio_cfg().filter_unicast_destination();
                }

                RadioConfiguration* radio_cfg = response.mutable_data()->mutable_radio_cfg();
                radio_cfg->set_address(mac_address.data(), mac_address.size());
                radio_cfg->set_filter_unicast_destination(filter_unicast_destination_);
                response.set_status(CommandResponse::SUCCESS);
            }
            else
            {
                response.set_status(CommandResponse::NOT_IMPLEMENTED);
            }

            response.SerializeToString(&cmd_response_);
            cmd_socket_.send_to(boost::asio::buffer(cmd_response_), cmd_sender_);
        }
    }

    cmd_socket_.async_receive_from(boost::asio::buffer(cmd_buffer_), cmd_sender_,
            boost::bind(&Radio::handleCommandReception, this,
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

RadioMedium::RadioMedium(boost::asio::io_service& io) : io_(io)
{
}

void RadioMedium::add(boost::asio::ip::address addr, unsigned short cmd_port, unsigned short fwd_port)
{
    radios_.emplace_front(this, ++idx_, io_, addr, cmd_port, fwd_port);
}

void RadioMedium::disseminate(const LinkLayerTransmission& transmission, const Radio* self)
{
    LinkLayerReception reception;
    reception.set_source(transmission.source());
    reception.set_destination(transmission.destination());
    reception.set_payload(transmission.payload());

    unsigned counter = 0;
    for (Radio& radio : radios_)
    {
        if (&radio != self) {
            radio.receive(reception);
            ++counter;
        }
    }

    ROS_INFO_NAMED(this_node, "frame from CCU %u disseminated to %u other CCUs", self->getId(), counter);

    // optionally record to PCAP stream
    if (record_ && transmission.source().size() == 6 && transmission.destination().size() == 6) {
        const auto now = ros::Time::now();
        struct {
            std::uint32_t ts_sec = 0;
            std::uint32_t ts_usec = 0;
            std::uint32_t incl_len = 0;
            std::uint32_t orig_len = 0;
        } pcap_packet_header;
        pcap_packet_header.ts_sec = now.sec;
        pcap_packet_header.ts_usec = now.nsec / 1000;
        pcap_packet_header.incl_len = transmission.payload().size() + 6 + 6 + 2;
        pcap_packet_header.orig_len = pcap_packet_header.incl_len;

        static_assert(sizeof(pcap_packet_header) == 16, "size of PCAP packet header has to be 16 bytes");
        auto pcap_packet_header_start = reinterpret_cast<const char*>(&pcap_packet_header);
        record_->write(pcap_packet_header_start, sizeof(pcap_packet_header));

        record_->write(transmission.destination().data(), transmission.destination().size());
        record_->write(transmission.source().data(), transmission.source().size());
        record_->put(0x89); record_->put(0x47);
        record_->write(transmission.payload().data(), transmission.payload().size());
        record_->flush();
    }
}

void RadioMedium::record(std::ostream* os)
{
    record_ = os;
    if (record_) {
        // write global header first
        struct {
            std::uint32_t magic_number = 0xa1b2c3d4;
            std::uint16_t version_major = 2;
            std::uint16_t version_minor = 4;
            std::int32_t this_zone = 0;
            std::uint32_t ts_accuracy = 0;
            std::uint32_t snap_len = 4096;
            std::uint32_t network = 1;
        } pcap_global_header;
        static_assert(sizeof(pcap_global_header) == 24, "size of PCAP global header has to be 24 bytes");
        auto pcap_global_header_start = reinterpret_cast<const char*>(&pcap_global_header);
        record_->write(pcap_global_header_start, sizeof(pcap_global_header));
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, this_node);
    ros::start();
    int nodes = 2;
    if (ros::param::get("~nodes", nodes)) {
        if (nodes < 1) {
            ROS_FATAL("Less than 1 node requested. Revise the nodes parameter!");
            return 1;
        } else if (nodes > 254) {
            ROS_ERROR("Limiting to 254 nodes, %u have been requested", nodes);
            nodes = 254;
        }
    }

    boost::asio::io_service io_service;
    auto signal_handler = [&](const boost::system::error_code& ec, int signal_number) {
        if (!ec) {
            io_service.stop();
            ros::shutdown();
        }
    };
    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait(signal_handler);

    std::string pcap_file;
    std::ofstream pcap_stream;
    if (ros::param::get("~pcap_output", pcap_file) && !pcap_file.empty()) {
        pcap_stream.open(pcap_file, std::ios_base::binary | std::ios_base::out | std::ios_base::trunc);
    }

    try {
        RadioMedium radio_medium(io_service);
        if (pcap_stream.is_open()) {
            radio_medium.record(&pcap_stream);
        }

        for (int i = 0; i < nodes; ++i) {
            boost::asio::ip::address_v4::bytes_type addr { 127, 0, 2, static_cast<unsigned char>(i + 1) };
            radio_medium.add(boost::asio::ip::address_v4 { addr }, 33001 + i, 33601 + i);
        }
        ROS_INFO_NAMED(this_node, "providing virtual radio medium for %i nodes", nodes);
        io_service.run();
    } catch (std::exception& ex) {
        ROS_FATAL_NAMED(this_node, "Exception caught: %s", ex.what());
        return 1;
    }

    return 0;
}

